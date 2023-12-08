/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * locklessHash.c
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This software is licensed under the 3-Clause BSD License.
 *
 * Copyright (C) 2018-2023 - KIOXIA Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#undef NDEBUG
#include <assert.h>
#if !defined(_WIN32)
#include <stdatomic.h>
#include <unistd.h>
#else
#include "stdatomic.h"
#include "usleep.h"
#endif

#include "condcounter.h"
#include "locklessHash.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

static const int reservedEntries = 2;
static const int defaultSize = 8;
static const uint64_t LHTsignature = 0x64696c617654484c;    // 'LHTvalid'

static const uint32_t engraved = 0x21504952;    // 'RIP!'
static void* tombstone = (void*)&engraved;      // a deleted key
static const uint32_t wildcard = 0x21594e41;    // 'ANY!'
static void* matchAny = (void*)&wildcard;
static const uint32_t headstone = 0xffffffff;    // an empty slot in a table being resized
static const uint64_t PRIME_MARK = 1;

enum copyState { NOT_COPYING, COPYING, PREVIOUSLY_COPIED };

#ifdef UNIT_TEST
#define UT_SLEEP_PRE_RESIZE \
    (0 << 1)    // randomly sleep for 1 sec 1 out of 10 times just before resize
uint32_t lhtConfig;
static int maxDepth = 0;
_Atomic uint32_t resizeCnt = 0;
#endif

// atomics must be aligned naturally... do not pack table
typedef struct
{
    _Atomic(uint32_t) key;
    _Atomic(void*) value;
} LHTentry;

typedef LHTentry* LHTptr;

// Condition states for instance structure refCnt
static const int32_t INSTANCE_NORMAL = 0;
static const int32_t INSTANCE_PROMOTING = 1;

// LHTinstance contains data that is entirely private
// and should not be modified or accessd directly by the user
typedef struct
{
    _Atomic(LHTptr) ptr;
    _Atomic(LHTptr) deletePtr;
    uint64_t sig;
    condCounter32_t refCnt;
    _Atomic(int32_t) deleting;
} LHTinstance;

typedef LHTinstance* locklessHashTable;

// Condition states for stats structure refCnt
static const int32_t TABLE_NORMAL = 0;
static const int32_t TABLE_ZOMBIE = 1;
typedef struct
{
    _Atomic(uint32_t) inUseCnt;
    _Atomic(uint32_t) deletedCnt;
    _Atomic(uint32_t) probeMax;
    condCounter32_t refCnt;
    _Atomic(uint32_t) refMax;
    _Atomic(uint64_t) probeSum;
    _Atomic(uint64_t) probeNum;
} LHTstats;

const uint32_t TABLE_STATE_NORMAL = 0;
const uint32_t TABLE_STATE_RESIZE = 1;
const uint32_t TABLE_STATE_PROMOTE = 2;
const uint32_t TABLE_STATE_DELETE = 3;
const uint32_t TABLE_STATE_DEAD = 4;

typedef struct
{
    _Atomic(LHTptr) next;
    _Atomic(uint32_t) copyIndex;
    _Atomic(uint32_t) copied;
    _Atomic(uint32_t) state;
} resizeInfo;

#define getSize(x)          ((x)[0].key)
#define getStatPtr(x)       ((LHTstats*)(x)[1].value)
#define getResizeInfoPtr(x) ((resizeInfo*)(x)[0].value)

static void dump(LHTptr table, int printData);

static inline bool incReference(LHTptr table)
{
    uint32_t curRefCnt, curMax;
    LHTstats* stats = getStatPtr(table);

    if (!condcounter_cond_add(&(stats->refCnt), TABLE_NORMAL, 1))
    {
        return false;
    }

    curRefCnt = condcounter_counter_get(&(stats->refCnt));
    curMax = stats->refMax;
    while (curRefCnt > curMax)
    {
        curMax = atomic_exchange(&(stats->refMax), curRefCnt);
        if (curMax > curRefCnt)
        {
            atomic_exchange(&(stats->refMax), curMax);
        }
    }
    return true;
}

static inline void decReference(LHTptr table)
{
    LHTstats* stats = getStatPtr(table);

    condcounter_sub(&(stats->refCnt), 1);
    stats = NULL;    // may have been deleted by now
}

// returns a pointer to the top table with a guarantee that the table is referenced and not about to be deleted...
static inline LHTptr getTopTable(locklessHashTable instance)
{
    LHTptr table;
    bool locked;

    if (!condcounter_cond_add(&(instance->refCnt), INSTANCE_NORMAL, 1))
    {
        return NULL;
    }

    // if we get here, the instance is not currently promoting an obsolete table and
    // we have incremented the instance reference so a table cannot be promoted. Since
    // a table must be promoted before the old table can be deleted, this prevents races
    // between acquiring  a reference on a table while it is in the process of being deleted.

    table = (LHTptr)instance->ptr;
    locked = incReference(table);    // will fail if table is zombie, which should be impossible
    assert(locked);

    // allow promotes to process; but we may have incremented the table which will delay a delete
    condcounter_sub(&(instance->refCnt), 1);

    // we can only get here if we got the reference for the table and we were not deleting a table
    return table;
}

static LHTptr create(uint32_t size)
{
    // note first two entries are reserved for internal use, so adjust size to account for this.
    // Entry 0 records hash table size (must be a power of 2) and pointer to next table in resize
    // chain Entry 1 has stats pointer

    LHTptr newTable = (LHTptr)calloc((uint64_t)size + reservedEntries, sizeof(LHTentry));
    assert(newTable);

    if (newTable)
    {
        newTable[0].key = size;
        newTable[0].value = calloc(1, sizeof(resizeInfo));
        assert(newTable[0].value);
        newTable[1].value = calloc(1, sizeof(LHTstats));
        assert(newTable[1].value);
        getStatPtr(newTable)->refCnt = 1;    // mark as referenced
    }
    return newTable;
}

// from code.google.com/p/smhasher/wiki/MurmurHash3
inline static uint32_t murmurHash3(uint32_t h)
{
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}

inline static uint32_t getIndex(LHTptr table, uint32_t key)
{
    uint32_t index = murmurHash3(key);
    return index & (getSize(table) - 1);
}

inline static uint32_t getKey(LHTptr table, uint32_t index)
{
    return table[index + reservedEntries].key;
}

inline static void* getValue(LHTptr table, uint32_t index)
{
    return (void*)table[index + reservedEntries].value;
}

// this is a pretty arbitrary limit...
// if you make it too small, you will resize a lot...
// possibly endlessly without forward progress!!!
// useful for stress testing resize!
// e.g. to stress: return (len >> 2) + 4;
inline static uint32_t reprobeLimit(uint32_t len)
{
    return ((len >> 1) + 4) > 100 ? 100 : (len >> 1) + 4;
}

inline static void probeStats(LHTptr table, uint32_t probeLen)
{
    LHTstats* stats = getStatPtr(table);
    uint32_t oldMax = stats->probeMax;
    atomic_fetch_add(&(stats->probeNum), 1);
    atomic_fetch_add(&(stats->probeSum), (uint64_t)probeLen);
    while (oldMax < probeLen)
    {
        atomic_compare_exchange_strong(&(stats->probeMax), &oldMax, probeLen);
    }
}

inline static void promoteTable(locklessHashTable instance, LHTptr table)
{
    LHTptr oldHead;
    LHTptr newTable;
    uint32_t oldState = TABLE_STATE_PROMOTE;
    resizeInfo* info = getResizeInfoPtr(table);
    assert(info);

    // Only promote top level table
    if ((LHTptr)instance->ptr != table)
    {
        return;
    }
    // mark that a promote process has begun... this interlock with getTopTable
    // ensures that we aquire a lock on the top table before the table is
    // promoted. Since a table must be promoted before it can be deleted, this
    // provides protection against races with getting a reference count on the top table
    // and the table being processed for deletion.
    if (condcounter_condition_exchange(&(instance->refCnt), INSTANCE_PROMOTING))
    {
        return;    // another thread is already working on it...
    }

    if (((LHTptr)instance->ptr != table) || (info->state != TABLE_STATE_PROMOTE))
    {
        // between the time we checked that promotion was needed and the time we set the
        // promoting flag, another thread already promoted this table.
        condcounter_condition_exchange(&(instance->refCnt), INSTANCE_NORMAL);
        return;
    }

#ifdef UNIT_TEST
    {
        resizeInfo* info = getResizeInfoPtr(table);
        LHTptr nextTable = table;
        int depth = 1;
        while (info->next)
        {
            depth++;
            nextTable = info->next;
            info = getResizeInfoPtr(nextTable);
        }
        if (depth > maxDepth)
        {
            maxDepth = depth;
            printf("Max depth = %d\n", depth);
        }
    }
#endif
    newTable = (LHTptr)info->next;
    assert(newTable);

    assert(condcounter_counter_get(&(getStatPtr(newTable)->refCnt)) != 0);
    assert(condcounter_counter_get(&(getStatPtr(table)->refCnt)) > 1);

    // we have blocked incrementing the instance reference counter - now we wait until
    // all inflight calls to getTopTable() exit.. this should be a really short wait
    while (condcounter_counter_get(&(instance->refCnt)))
    {
        usleep(1);
    }

    oldHead = table;
    atomic_compare_exchange_strong((_Atomic(uintptr_t)*)&instance->ptr, (uintptr_t*)&oldHead,
                                   (uintptr_t)newTable);
    assert(oldHead == table);

    oldState = atomic_exchange(&info->state, TABLE_STATE_DELETE);
    assert(oldState == TABLE_STATE_PROMOTE);

    condcounter_condition_exchange(&(instance->refCnt), INSTANCE_NORMAL);
}

inline static void reapTable(locklessHashTable instance)
{
    LHTptr table;
    resizeInfo* info;
    LHTstats* stats;
    int32_t zero = 0;
    int32_t one = 1;
    uint32_t oldState;

    assert(instance);

    // Deletes only possible after new table has been promoted
    if ((LHTptr)instance->ptr == instance->deletePtr)
    {
        return;
    }

    // only allow 1 thread to attempt delete
    if (!atomic_compare_exchange_strong(&instance->deleting, &zero, 1))
    {
        return;
    }

    // all other threads blocked from attempting deletes

    // get delete table pointer AFTER we have set deleting flag...
    table = instance->deletePtr;
    assert(table);

    info = getResizeInfoPtr(table);

    assert(info);

    oldState = info->state;
    if (oldState != TABLE_STATE_DELETE)
    {
        // not ready for delete.
        // either we caught a promote in mid flight, or we
        // lost a race with another thread between the check for ptr != deletePtr
        // and setting the deleting flag and the delete has already happened
        instance->deleting = 0;
        return;
    }

    stats = getStatPtr(table);

    assert(stats);

    // avoid atomic operation for most likely case
    if (condcounter_counter_get(&(stats->refCnt)) > 1)
    {
        // others still using table, abort delete attempt
        instance->deleting = 0;
        return;
    }

    // table has been promoted, so no one should be attempting new references
    // set the zombie flag to catch any logic errors
    condcounter_condition_exchange(&(stats->refCnt), TABLE_ZOMBIE);
    // From this point on incReference on table will fail

    // Make sure all other threads have released the table and no one references it
    if (!condcounter_compare_exchange(&(stats->refCnt), (int32_t*)&one, 0))
    {
        // abort the delete attempt
        instance->deleting = 0;
        return;
    }

    // No one is referencing the table and we have blocked anyone else from trying to delete
    // the table

    // set flag to detect logic errors... they will trigger assert above that table state is
    // TABLE_STATE_DELETE when attempting a delete
    oldState = atomic_exchange(&info->state, TABLE_STATE_DEAD);

    assert(oldState == TABLE_STATE_DELETE);

    assert(info->next);

    // unlink from order of delete list
    atomic_exchange((_Atomic(uintptr_t)*)&instance->deletePtr, (uintptr_t)info->next);
    info->next = NULL;

    //	dump(table, 0);

    //	printf("Deleting table %p, size %u---------------\n", table, getSize(table)); // debug aid
    free(stats);
    free(info);
    table[0].value = NULL;
    table[1].value = NULL;
    free(table);

    instance->deleting = 0;    // we are no longer in the process of deleting!
}

static LHTptr copySlotAndCheck(LHTptr oldTable, uint32_t index, int shouldHelp);
static void helpCopy(LHTptr oldTable);

static void* get(LHTptr table, uint32_t lookupKey)
{
    uint32_t index;
    uint32_t reprobes = 0;
    uint32_t key;
    void* value;
    bool locked;

    index = getIndex(table, lookupKey);
    while (1)
    {
        LHTptr newTable;
        key = getKey(table, index);
        value = getValue(table, index);

        newTable = (LHTptr)getResizeInfoPtr(table)->next;    // volatile read

        if (key == 0)    // not in table
        {
            probeStats(table, reprobes + 1);
            return NULL;
        }
        if (key == lookupKey)
        {
            if (!((uintptr_t)value & PRIME_MARK))
            {
                probeStats(table, reprobes + 1);
                return value == tombstone ? NULL : value;
            }
            // Found it; but it is being copied currently
            copySlotAndCheck(table, index, 1);    // guarantee copy is complete

            assert(newTable);
            // locks should only fail at top level
            locked = incReference(newTable);
            assert(locked);

            value = get(newTable, lookupKey);    // retry in new table

            decReference(newTable);

            return value;
        }
        if (++reprobes >= reprobeLimit(getSize(table)) || key == headstone)
        {
            // This is a miss; however we are going to help with any copy if possible...
            if (newTable)
            {
                helpCopy(table);

                assert(newTable);

                // locks should only fail at top level
                locked = incReference(newTable);
                assert(locked);

                value = get(newTable, lookupKey);    // retry in new table
                decReference(newTable);

                return value;
            }
            return NULL;    // no need to retry in new table
        }
        index = (index + 1) & (getSize(table) - 1);
    }
}

inline static uint32_t CASkey(LHTptr table, uint32_t index, uint32_t key)
{
    uint32_t expected = 0;
    atomic_compare_exchange_strong(&(table[index + reservedEntries].key), &expected, key);
    return expected;
}

inline static void* CASvalue(LHTptr table, uint32_t index, void* oldValue, void* newValue)
{
    void* curValue = oldValue;

    assert(newValue != matchAny);

    if (oldValue == matchAny)
    {
        // Note: A primed value can only be replaced by a primed tombstone
        curValue = getValue(table, index);
        while (!((uintptr_t)curValue & PRIME_MARK))
        {
            if (atomic_compare_exchange_strong(
                    (_Atomic(uintptr_t)*)&(table[index + reservedEntries].value),
                    (uintptr_t*)&curValue, (uintptr_t)newValue))
            {
                break;
            }
        }
        if ((uintptr_t)curValue & PRIME_MARK)
        {
            if ((uintptr_t)newValue & PRIME_MARK)    // should only be a primed tombstone
            {
                assert((uintptr_t)newValue == ((uintptr_t)tombstone | PRIME_MARK));
                curValue = (void*)atomic_exchange(
                    (_Atomic(uintptr_t)*)&(table[index + reservedEntries].value), (uintptr_t)newValue);
            }
        }
    }
    else
    {
        while (!atomic_compare_exchange_strong(
            (_Atomic(uintptr_t)*)&(table[index + reservedEntries].value), (uintptr_t*)&curValue,
            (uintptr_t)newValue))
        {
            if (oldValue != 0 || (curValue != 0 && curValue != tombstone))
            {
                break;    // only retry if trying to replace an "empty" entry
            }
        }
    }
    return curValue;
}

inline static void adjustInUse(LHTptr table, uint32_t value)
{
    // Note: due to race conditions, these may briefly dip negative
    // or be slightly high... they will be correct once the table is idle
    atomic_fetch_add(&(getStatPtr(table)->inUseCnt), value);
}

inline static void adjustDeleted(LHTptr table, uint32_t value)
{
    // Note: due to race conditions, these may briefly dip negative
    // or be slightly high... they will be correct once the table is idle
    atomic_fetch_add(&(getStatPtr(table)->deletedCnt), value);
}

// returns true if resize complete
static int resize(LHTptr oldTable)
{
    resizeInfo* info = getResizeInfoPtr(oldTable);
    LHTptr newTable, oldNext;
    uint32_t newSize;
    LHTstats* stats = getStatPtr(oldTable);
    uint32_t oldState = TABLE_STATE_NORMAL;

    // someone beat us
    if (info->next)
    {
        return 1;
    }

#ifdef UNIT_TEST
    if (lhtConfig & UT_SLEEP_PRE_RESIZE)
    {
        if (rand() % 100 == 0)
        {
            usleep(1);
        }
    }
#endif

    // Allow this only once per level
    if (!atomic_compare_exchange_strong(&info->state, &oldState, TABLE_STATE_RESIZE))
    {
        return 0;
    }

    // if we get here, we are the first to advance to state resize...
#ifdef UNIT_TEST
    atomic_fetch_add(&resizeCnt, 1);
#endif

    // This can rehash forever if probes too long - don't load the table too heavily
    newSize = getSize(oldTable);    // assume just reclaiming space
    //	if (stats->inUseCnt > ((getSize(oldTable) >> 2) * 3)) //75% loading min for table increase
    if (stats->inUseCnt > ((getSize(oldTable) >> 2) * 2))    // 50% loading min for table increase
    {
        newSize <<= 1;
    }

    newTable = create(newSize);
    assert(newTable);
    assert(info->next == NULL);
    oldNext = (LHTptr)atomic_exchange((_Atomic(uintptr_t)*)&info->next, (uintptr_t)newTable);
    assert(oldNext == NULL);
    return 1;
}

static void* put(LHTptr table, uint32_t lookupKey, void* oldValue, void* value, enum copyState* cState)
{
    uint32_t index;
    uint32_t reprobes = 0;
    uint32_t key;
    bool locked;

    assert(lookupKey);
    assert(!((uintptr_t)value & PRIME_MARK));

    index = getIndex(table, lookupKey);
    while (1)
    {
        void* retValue;

        // Dont waste space deleting a key that is not present
        if (value == tombstone && (getValue(table, index) == 0))
        {
            return NULL;
        }

        key = CASkey(table, index, lookupKey);
        if ((key == 0) || (key == lookupKey))
        {
            retValue = CASvalue(table, index, oldValue, value);
            if ((uintptr_t)retValue & PRIME_MARK)
            {
                LHTptr newTable;
                resizeInfo* info;

                info = getResizeInfoPtr(table);
                assert(info->next);
                newTable = (LHTptr)info->next;
                copySlotAndCheck(table, index, 0);

                assert(newTable);

                // If we are copying and see that a destination has a primed value,
                // it means this slot has already been copied. We will try to copy at
                // the next level up, which may have a empty slot, in which case it will
                // look like we copied succesfully for the first time, when in fact it
                // was previously copied.
                if (*cState == COPYING)
                {
                    if ((uintptr_t)retValue & ~PRIME_MARK)
                    {
                        *cState = PREVIOUSLY_COPIED;
                    }
                }

                // locks should only fail at top level
                locked = incReference(newTable);
                assert(locked);

                retValue = put(newTable, lookupKey, oldValue, value, cState);    // retry in new table
                decReference(newTable);

                // We want near constant execution time, if multiple resizes are happening,
                // only help copy if not already copying
                if (*cState == NOT_COPYING)
                {
                    helpCopy(table);
                }

                return retValue;
            }

            // old value was not primed...
            if (value != tombstone)
            {
                // we updated with a real value...
                if (key == 0)    // Empty to used transition
                {
                    adjustInUse(table, 1);
                }
                else if (retValue == tombstone)    // Deleted to reused transition
                {
                    adjustInUse(table, 1);
                    adjustDeleted(table, -1);
                }
            }
            else if (retValue != tombstone)    // Used to deleted transition
            {
                adjustInUse(table, -1);
                adjustDeleted(table, 1);
            }
            probeStats(table, reprobes + 1);

            return retValue;
        }
        if (++reprobes >= reprobeLimit(getSize(table)) || key == headstone)
        {
            if (!getResizeInfoPtr(table)->next)
            {
                while (!resize(table)) {}
            }
            assert(getResizeInfoPtr(table)->next);
            // locks should only fail at top level
            locked = incReference((LHTptr)getResizeInfoPtr(table)->next);
            assert(locked);
            retValue = put((LHTptr)getResizeInfoPtr(table)->next, lookupKey, oldValue, value, cState);
            decReference((LHTptr)getResizeInfoPtr(table)->next);

            // We want near constant execution time, if multiple resizes are happening,
            // only help copy if not already copying
            if (*cState == NOT_COPYING)
            {
                helpCopy(table);
            }

            return retValue;
        }
        index = (index + 1) & (getSize(table) - 1);
    }
}

// --- copy_slot ---------------------------------------------------------
// Copy one K/V pair from old table to new table.  Returns true if we can
// confirm that the new table guaranteed has a value for this old-table
// slot.  We need an accurate confirmed-copy count so that we know when we
// can promote (if we promote the new table too soon, other threads may
// 'miss' on values not-yet-copied from the old table).  We don't allow
// any direct updates on the new table, unless they first happened to the
// old table - so that any transition in the new table from null to
// not-null must have been from a copy_slot (or other old-table overwrite)
// and not from a thread directly writing in the new table.  Thus we can
// count null-to-not-null transitions in the new table.
static int copySlot(int index, LHTptr oldTable, LHTptr newTable)
{
    // Blindly set the key slot from null to HEADSTONE, to eagerly stop
    // fresh put's from inserting new values in the old table when the old
    // table is mid-resize.  We don't need to act on the results here,
    // because our correctness stems from prime'ing the Value field.  Slamming
    // the Key field is a minor speed optimization.
    uint32_t key;
    int copied_into_new;
    bool oldWasPrimed;
    uintptr_t oldVal;
    uintptr_t newVal;
    enum copyState cState = COPYING;
    int empty = 0;

    if ((key = getKey(oldTable, index)) == 0)
    {
        key = CASkey(oldTable, index, headstone);
    }

    // ---
    // Prevent new values from appearing in the old table.
    // Prime what we see in the old table, to prevent further updates.
    oldVal = (uintptr_t)getValue(oldTable, index);    // Read OLD table
    while (!(oldVal & PRIME_MARK))
    {
        newVal = oldVal | PRIME_MARK;
        if (oldVal == (uintptr_t)CASvalue(oldTable, index, (void*)oldVal,
                                          (void*)newVal))    // CAS down a primed'd version of oldval
        {
            // If we made the Value slot hold a TOMBPRIME, then we both
            // prevented further updates here but also the (absent)
            // oldval is vacuously available in the new table.  We
            // return with true here: any thread looking for a value for
            // this key can correctly go straight to the new table and
            // skip looking in the old table.
            if (oldVal == (uintptr_t)tombstone)
            {
                return 1;
            }
            // Otherwise we primed something, but it still needs to be
            // copied into the new table and tombstoned.
            empty = !oldVal;
            break;    // Break loop; oldval is now primed by us
        }
        oldVal = (uintptr_t)getValue(oldTable, index);    // Else try, try again
    }

    oldWasPrimed = (oldVal & PRIME_MARK);
    // someone else may be trying to copy this slot too...
    // but we MUST guarantee copy is complete before returning
    // so we copy it too. Only one thread will succeed since
    // we compare_exchange with an expected value of zero (empty)
    if (oldWasPrimed)
    {
        oldVal &= ~PRIME_MARK;

        // This slot is guaranteed to have been fully copied already
        // since it was already a primed tombstone or empty
        if (!oldVal || (oldVal == (uintptr_t)tombstone))
        {
            return 0;
        }
    }

    // assumption on who will get credit...
    copied_into_new = !oldWasPrimed;    // assume we are going to copy it if it wasn't primed

    // ---
    // Copy the value into the new table, but only if we overwrite a null.
    // If another value is already in the new table, then somebody else
    // wrote something there and that write is happens-after any value that
    // appears in the old table.  If putIfMatch does not find a null in the
    // new table - somebody else should have recorded the null-not_null
    // transition in this copy.
    //
    // Note: it is not necessary to lock the new table here since it cannot
    // be promoted or deleted while a copy is happening.
    if (!empty && key && (key != headstone))
    {
        void* curVal;
        curVal = put(newTable, key, 0, (void*)oldVal, &cState);
        copied_into_new = !curVal;
        if (curVal && !oldWasPrimed && (curVal != (void*)oldVal && (curVal != tombstone)))
        {
            // Copy lost a race... after we primed the old value and before we copied
            // to the next table, an put of a new value for this key passed us. This
            // caused our put() to return the new value, but we should still get the credit
            copied_into_new = 1;
            assert(cState != PREVIOUSLY_COPIED);
        }
    }
    // ---
    // Finally, now that any old value is exposed in the new table, we can
    // forever hide the old-table value by slapping a TOMBPRIME down.  This
    // will stop other threads from uselessly attempting to copy this slot
    // (i.e., it's a speed optimization not a correctness issue).
    newVal = (uintptr_t)tombstone;
    newVal |= PRIME_MARK;
    CASvalue(oldTable, index, matchAny, (void*)newVal);

    return copied_into_new && (cState != PREVIOUSLY_COPIED);
}    // end copy_slot

// --- copy_check_and_promote --------------------------------------------
static void copyCheckAndUpdate(LHTptr oldTable, uint32_t workDone)
{
    uint32_t oldLen;
    resizeInfo* info = getResizeInfoPtr(oldTable);
    uint32_t newVal;
    uint32_t copyProgress;

    assert(workDone);
    assert(info);
    assert(info->next);

    oldLen = getSize(oldTable);

    // We made a slot unusable and so did some of the needed copy work
    copyProgress = info->copied;
    newVal = copyProgress + workDone;

    assert(newVal <= oldLen);

    while (!atomic_compare_exchange_strong(&info->copied, &copyProgress, newVal))
    {
        newVal = copyProgress + workDone;
        assert((copyProgress + workDone) <= oldLen);
    }
    if (newVal == oldLen)
    {
        // We have finished copying and the old table is obsolete. Promote it
        // and then delete it!
        uint32_t oldState;
        oldState = atomic_exchange(&info->state, TABLE_STATE_PROMOTE);
        assert(oldState == TABLE_STATE_RESIZE);
    }
}

// --- copy_slot_and_check -----------------------------------------------
// Copy slot 'index' from the old table to the new table.  If this thread
// confirmed the copy, update the counters and check for promotion.
//
// Returns the result of reading the volatile next, mostly as a
// convenience to callers.  We come here with 1-shot copy requests
// typically because the caller has found a Prime, and has not yet read
// the next volatile - which must have changed from null-to-not-null
// before any Prime appears.  So the caller needs to read the next
// field to retry his operation in the new table, but probably has not
// read it yet.
static LHTptr copySlotAndCheck(LHTptr oldTable, uint32_t index, int shouldHelp)
{
    resizeInfo* info;
    assert(oldTable);
    info = getResizeInfoPtr(oldTable);
    assert(info);

    LHTptr newTable = (LHTptr)info->next;    // VOLATILE READ
    // We're only here because the caller saw a Prime, which implies a
    // table-copy is in progress.
    assert(newTable);
    if (copySlot(index, oldTable, newTable))    // Copy the desired slot
    {
        copyCheckAndUpdate(oldTable, 1);    // Record the slot copied
    }
    // Generically help along any copy (except if called recursively from a helper)
    if (shouldHelp)
    {
        helpCopy(oldTable);
    }
    return newTable;
}

// --- copyHelp ----------------------------------------------------
// Help along an existing resize operation.  We hope its the top-level
// copy (it was when we started) but this table might have been promoted out
// of the top position.
static void helpCopy(LHTptr oldTable)
{
    LHTptr newTable;
    resizeInfo* info;
    uint32_t oldLen;

    assert(oldTable);
    info = getResizeInfoPtr(oldTable);
    assert(info);

    // another thread may have finished the copy and the current table is no longer valid
    if (info->state != TABLE_STATE_RESIZE)
    {
        return;
    }

    newTable = (LHTptr)info->next;
    assert(newTable);

    oldLen = getSize(oldTable);                    // Total amount to copy
    uint32_t helpCount = MIN(oldLen >> 2, 128);    // Limit per-thread work

    // ---
    int panicStart = -1;
    uint32_t copyIndex = info->copyIndex;    // Fool compile to think it's initialized
    while (info->copied < oldLen)            // Still needing to copy?
    {
        // Carve out a chunk of work.  The counter wraps around so every
        // thread eventually tries to copy every slot repeatedly.

        // We "panic" if we have tried TWICE to copy every slot - and it still
        // has not happened.  i.e., twice some thread somewhere claimed they
        // would copy 'slot X' (by bumping _copyIdx) but they never claimed to
        // have finished (by bumping _copyDone).  Our choices become limited:
        // we can wait for the work-claimers to finish (and become a blocking
        // algorithm) or do the copy work ourselves.  Tiny tables with huge
        // thread counts trying to copy the table often 'panic'.
        if (panicStart == -1)
        {    // No panic?
            copyIndex = info->copyIndex;
            while (copyIndex < (oldLen << 1) &&    // 'panic' check
                   !atomic_compare_exchange_strong(&info->copyIndex, &copyIndex, copyIndex + helpCount))
            {
            }
            if (!(copyIndex < (oldLen << 1)))    // Panic!
            {
                panicStart = copyIndex;    // Record where we started to panic-copy
            }
        }

        // We now know what to copy.  Try to copy.
        int workdone = 0;
        for (uint32_t i = 0; i < helpCount; i++)
        {
            if (copySlot((copyIndex + i) & (oldLen - 1), oldTable,
                         newTable))    // Made an oldtable slot go dead?
            {
                workdone++;    // Batch up normal case
            }
        }
        if (workdone > 0)    // Report work-done occasionally
        {
            copyCheckAndUpdate(oldTable, workdone);    // credit to this table
        }

        copyIndex += helpCount;
        // Uncomment these next 2 lines to turn on incremental table-copy.
        // Otherwise this thread continues to copy until it is all done.
        if (panicStart == -1)    // No panic?
        {
            return;    // Then done copying after doing MIN_COPY_WORK
        }
    }
    return;
}

static void foreach (LHTptr table, void (*func)(void*, void*), void* param)
{
    for (LHTptr curTable = table; curTable; curTable = getResizeInfoPtr(curTable)->next)
    {
        for (uint32_t index = 0; index < getSize(curTable); index++)
        {
            uint32_t key = getKey(table, index);
            if (key)
            {
                void* value = getValue(curTable, index);
                if ((uintptr_t)value & PRIME_MARK)
                {
                    printf("Primed_");
                }
                if (((uintptr_t)value & ~PRIME_MARK) == (uintptr_t)tombstone)
                {
                    value = (void*)((uintptr_t)value & ~PRIME_MARK);
                }
                else
                {
                    func(param, value);
                }
            }
        }
    }
}

static void dump(LHTptr table, int printData)
{
    int level = 0;
    resizeInfo* info;

    for (LHTptr curTable = table; curTable; curTable = (LHTptr)getResizeInfoPtr(curTable)->next)
    {
        float average;
        LHTstats* stats;
        info = getResizeInfoPtr(curTable);

        stats = getStatPtr(curTable);
        printf("Level %d stats ------------------------------------\n", level++);
        printf("\tSize = %u\n", getSize(curTable));
        printf("\tIn use = %u\n", stats->inUseCnt);
        printf("\tDeleted = %u\n", stats->deletedCnt);
        printf("\tEmpty = %u\n", getSize(curTable) - (stats->inUseCnt + stats->deletedCnt));
        printf("\tReference count = %d\n", (int32_t)condcounter_counter_get(&(stats->refCnt)));
        printf("\tMax reference count= %u\n", stats->refMax);

        printf("\tLookups = %" PRIu64 "\n", stats->probeNum);
        printf("\tMax probe length = %u\n", stats->probeMax);
        average = (float)stats->probeSum;
        average /= stats->probeNum;
        printf("\tAverage probe length = %.6f\n", average);
        if (info)
        {
            printf("\tCopied count = %d\n", info->copied);
            printf("\tResize state = %d\n", info->state);
        }
        if (printData)
        {
            printf("Data:\n");
            for (uint32_t index = 0; index < getSize(curTable); index++)
            {
                uint32_t key;
                void* value;
                key = getKey(curTable, index);
                if (key)
                {
                    printf("\tIndex %u, key %u ", index, key);
                    value = getValue(curTable, index);
                    if ((uintptr_t)value & PRIME_MARK)
                    {
                        printf("Primed_");
                    }
                    if (((uintptr_t)value & ~PRIME_MARK) == (uintptr_t)tombstone)
                    {
                        printf("deleted\n");
                        // value = (void*)((uintptr_t)value & ~PRIME_MARK);
                    }
                    else
                    {
                        printf("%p\n", value);
                    }
                }
            }
        }
    }
}

//-------------------------------- PUBLIC FUNCTIONS --------------------------------------

LHThandle LHTcreate(void)
{
    locklessHashTable newInstance = (locklessHashTable)malloc(sizeof(LHTinstance));
    assert(newInstance);
    newInstance->ptr = create(defaultSize);
    newInstance->deletePtr = newInstance->ptr;
    newInstance->sig = LHTsignature;
    newInstance->refCnt = 0;
    newInstance->deleting = 0;
    return (LHThandle)newInstance;
}

void* LHTget(LHThandle handle, uint32_t lookupKey)
{
    locklessHashTable instance = (locklessHashTable)handle;
    void* ret;
    LHTptr table;

    assert(instance);
    assert(instance->sig == LHTsignature);
    assert(instance->ptr);

    if (!lookupKey || (lookupKey == headstone))
    {
        return NULL;
    }

    while (!(table = getTopTable(instance)))
    {
        usleep(1);
    }

    ret = get(table, lookupKey);

    if (getResizeInfoPtr(table)->state == TABLE_STATE_PROMOTE)
    {
        promoteTable(instance, table);
    }

    reapTable(instance);

    decReference(table);

    return ret;
}

void* LHTputIfEmpty(LHThandle handle, uint32_t lookupKey, void* value)
{
    locklessHashTable instance = (locklessHashTable)handle;
    LHTptr table;
    void* ret;
    enum copyState cState = NOT_COPYING;

    assert(instance);
    assert(instance->sig == LHTsignature);
    assert(instance->ptr);
    assert(lookupKey);
    assert(lookupKey != headstone);
    assert(value);
    assert(!((uint64_t)value & PRIME_MARK));

    while (!(table = getTopTable(instance)))
    {
        usleep(1);
    }

    ret = put(table, lookupKey, 0, value, &cState);
    // Clean up if put traversed more than two tables during resize
    if (ret == tombstone)
    {
        ret = NULL;
    }

    if (getResizeInfoPtr(table)->state == TABLE_STATE_PROMOTE)
    {
        promoteTable(instance, table);
    }

    reapTable(instance);

    decReference(table);

    return ret;
}

void* LHTput(LHThandle handle, uint32_t lookupKey, void* value)
{
    locklessHashTable instance = (locklessHashTable)handle;
    LHTptr table;
    void* ret;
    enum copyState cState = NOT_COPYING;

    assert(instance);
    assert(instance->sig == LHTsignature);
    assert(instance->ptr);
    assert(lookupKey);
    assert(lookupKey != headstone);
    assert(value);
    assert(!((uint64_t)value & PRIME_MARK));

    while (!(table = getTopTable(instance)))
    {
        usleep(1);
    }

    ret = put(table, lookupKey, matchAny, value, &cState);
    // Clean up if put traversed more than two tables during resize
    if (ret == tombstone)
    {
        ret = NULL;
    }

    if (getResizeInfoPtr(table)->state == TABLE_STATE_PROMOTE)
    {
        promoteTable(instance, table);
    }

    reapTable(instance);

    decReference(table);

    return ret;
}

void* LHTdelete(LHThandle handle, uint32_t lookupKey)
{
    return LHTput(handle, lookupKey, tombstone);
}

void LHTdump(LHThandle handle, int printData)
{
    locklessHashTable instance = (locklessHashTable)handle;
    LHTptr table;

    assert(instance);
    assert(instance->sig == LHTsignature);
    assert(instance->ptr);
    table = instance->ptr;

    dump(table, printData);
}

void LHTforeach(LHThandle handle, void (*func)(void*, void*), void* param)
{
    LHTptr table;
    locklessHashTable ht = handle;

    assert(ht);
    assert(ht->sig == LHTsignature);
    assert(ht->ptr);
    table = ht->ptr;

    foreach (table, func, param)
        ;
}

void LHTdestroy(LHThandle handle)
{
    locklessHashTable instance = (locklessHashTable)handle;
    LHTptr table;

    assert(instance);
    assert(instance->sig == LHTsignature);
    assert(instance->ptr);
    table = instance->ptr;

#ifdef UNIT_TEST
    printf("Number of resize operations %d\n", resizeCnt);
    dump(table, 0);
#endif

    // fail all new calls
    instance->sig = 0;
    instance->deletePtr = NULL;
    instance->ptr = NULL;

    while (table)
    {
        LHTptr nextTable;
        resizeInfo* info;
        LHTstats* stats;

        info = getResizeInfoPtr(table);
        stats = getStatPtr(table);
        nextTable = info->next;

        free(info);
        free(stats);
        table[0].value = NULL;
        table[1].value = NULL;
        free(table);

        table = nextTable;
    }
    free(instance);
}
