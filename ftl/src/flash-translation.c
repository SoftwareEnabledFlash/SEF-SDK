/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * flash-translation.c
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
#include "flash-translation.h"

#include <assert.h>
#include <errno.h>
#include <inttypes.h>    // PRIxxx
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "data-tree-object.h"
#include "log-manager.h"
#include "sef-block-module.h"
#include "sef-utils.h"
#include "superblock.h"
#include "utils/iov-helper.h"

#define SFT_LUT_PERSISTENCE_KEY "LookupTable"

#define sftForEachBlock(C, I, S)                                                              \
    for (int i = 0; i < ((I).filter.dstate = (C)->ssbState->dstate + i, (I).filter.index = 0, \
                        (I).filter.placement = -1, (I).filter.state = (S), ctxt->numDomains); \
         i++)                                                                                 \
        while (SSBEnumBlocks((C), &(I)))

static struct SEFStatus sftReadLut(FTLContext *ctxt)
{
    struct SFTState *sftState = ctxt->sftState;
    struct SEFStatus status;
    struct PDLData pdlData;
    struct PDLKey pdlKey = {SFT_LUT_PERSISTENCE_KEY, 0};
    int64_t numADUs = sftState->lutSize;
    size_t lutLen = numADUs * sizeof(atomic_uint_least64_t);

    // read lookup table from flash
    status = PDLReadFlash(ctxt->pdlHandle, pdlKey, &pdlData);
    if (!status.error)
    {
        assert(lutLen <= pdlData.ObjSize);
        memcpy(sftState->lut, pdlData.Obj, lutLen);
        SUfree(pdlData.Obj);
    }
    else if (status.error == -ENOENT)
    {
        // initializing the new lookup table
        LogInfo(ctxt->logHandle, "Lookup table size is %" PRId64, lutLen);
        memset(sftState->lut, 0, lutLen);
        status = SUMakeStatusOk();
    }

    return status;
}

void SFTQueueToWriteLut(FTLContext *ctxt)
{
    // store the lookup table and valid map using root pointers
    struct PDLData lutData = {};

    if (ctxt->pdlHandle == NULL || ctxt->sftState == NULL)
    {
        return;
    }
    strcpy(lutData.Key.Name, SFT_LUT_PERSISTENCE_KEY);
    lutData.Key.Index = 0;
    lutData.EncodingType = kPDLNoEncoding;
    lutData.Obj = ctxt->sftState->lut;
    lutData.ObjSize = ctxt->sftState->lutSize * sizeof(uint64_t);
    // lutData.arg = ctxt;
    // lutData.FlushCompleteFunc = ftlLookupTableCleanup;

    PDLQueueData(ctxt->pdlHandle, &lutData);
}

static void sftRegisterCounters(FTLContext *ctxt)
{
    struct SFTState *sftState = ctxt->sftState;

#if CC_DEBUG_COUNTERS
    struct INSCounter c[] = {
        // ftl
        InstructionCounterDef(lateUpdate, "ADU was rewritten before GC update", struct SFTState, sftState),
        InstructionCounterDef(lutRetry, "LUT read was retried", struct SFTState, sftState),
    };

    sftState->sftIoCntId = INSRegisterIoCounters(ctxt->hInst, c, NELEM(c), NULL, NULL);
#else
    sftState->sftIoCntId = -1;
#endif
}

struct SEFStatus SFTInitialize(FTLContext *ctxt, int64_t numADUs)
{
    struct SEFStatus status = SUMakeStatusOk();
    size_t stateSize;

    assert(ctxt->sftState == NULL);
    stateSize = sizeof(*ctxt->sftState) + numADUs * sizeof(ctxt->sftState->lut[0]);
    ctxt->sftState = SUzalloc(stateSize);
    ctxt->sftState->lutSize = numADUs;
    if (ctxt->pdlHandle)
    {
        status = sftReadLut(ctxt);
    }
    if (status.error == 0)
    {
        sftRegisterCounters(ctxt);
    }
    else
    {
        SFTCleanup(ctxt);
    }
    return status;
}

int SFTCleanup(FTLContext *ctxt)
{
    struct SFTState *sftState = ctxt->sftState;

    if (sftState == NULL)
    {
        return 0;
    }

    INSUnRegisterIoCounters(ctxt->hInst, sftState->sftIoCntId);
    SUfree(sftState);
    ctxt->sftState = NULL;
    return 0;
}

int SFTSet(FTLContext *ctxt,
           int64_t userLBA,
           struct SEFFlashAddress flashAddress,
           struct SEFPlacementID placementId)
{
    struct SFTState *sftState = ctxt->sftState;
    struct SEFFlashAddress oldAddress;
    int returnVal = 0;

    if (userLBA >= sftState->lutSize)
    {
        return -EINVAL;
    }

    // Mark new flash address as used in superblock map
    // Update lut[] with the new flash address
    // Mark old flash address as free in superblock map
    if (!SEFIsNullFlashAddress(flashAddress) &&
        (returnVal = SSBSetAduValid(ctxt, flashAddress, placementId)))
    {
        return returnVal;
    }

    oldAddress.bits = atomic_exchange(&sftState->lut[userLBA], flashAddress.bits);
    if (oldAddress.bits)
    {
        returnVal = SSBClearAduValid(ctxt, oldAddress);
    }
    assert(returnVal == 0);

    // mark lut and root pointer as dirty
    sftState->lutDirty = true;

    return 0;
}

int SFTUpdate(FTLContext *ctxt,
              uint64_t userLBA,
              struct SEFFlashAddress oldFlashAddress,
              struct SEFFlashAddress newFlashAddress)
{
    struct SFTState *sftState = ctxt->sftState;
    bool lutUpdated;
    uint64_t expectedBits = oldFlashAddress.bits;

    if (userLBA >= sftState->lutSize)
    {
        LogError(ctxt->logHandle, "Ignoring bad lba 0x%lx update", userLBA);
        return -EINVAL;
    }
    // Mark new flash address as used in superblock map
    // Update lut[] with the new flash address as long as the old was there
    // Mark old flash address as free in superblock map, if lut[] updated
    // Mark new flash address as free in superblock map, if lut[] failed update

    SSBSetAduValid(ctxt, newFlashAddress,
                   (struct SEFPlacementID){UINT8_MAX});    // In case exchange works, must be set
    lutUpdated =
        atomic_compare_exchange_strong(&sftState->lut[userLBA], &expectedBits, newFlashAddress.bits);

    if (lutUpdated && expectedBits == 0)
    {
        LogTrace(ctxt->logHandle, "Reverting trimmed address move - UA:0x%lx NFA:0x%lx", userLBA,
                 newFlashAddress.bits);
        lutUpdated = false;
    }

    if (lutUpdated)
    {
        assert(oldFlashAddress.bits);
        SSBClearAduValid(ctxt, oldFlashAddress);

        // mark lut and root pointer as dirty
        sftState->lutDirty = true;
    }
    else
    {
        SSBClearAduValid(ctxt, newFlashAddress);    // Undo it if it didn't
#if CC_DEBUG_COUNTERS
        atomic_fetch_add(&sftState->lateUpdate, 1);
#endif
    }
    return 0;
}

static bool sftAcquireForRead(FTLContext *ctxt, struct SEFFlashAddress flashAddress)
{
    struct SSBParsedFLA pfa = {};
    struct SEFStatus status;

    status = SSBParseFlashAddress(ctxt, flashAddress, &pfa);
    if (status.error)
    {
        return false;
    }

    return !!SSBMarkBusy(ctxt, &pfa);
}

void SFTReleaseForRead(FTLContext *ctxt, struct SEFFlashAddress flashAddress)
{
    struct SSBParsedFLA pfa = {};
    struct SEFStatus status;

    status = SSBParseFlashAddress(ctxt, flashAddress, &pfa);
    if (status.error)
    {
        return;
    }
    SSBMarkNotBusy(ctxt, &pfa);
}

struct SEFFlashAddress SFTLookupForRead(FTLContext *ctxt, int64_t lba)
{
    struct SFTState *sftState = ctxt->sftState;
    struct SEFFlashAddress addr = SEFNullFlashAddress;
    struct SEFFlashAddress hintAddr;

    hintAddr = SFTLookup(ctxt, lba);
    while (sftAcquireForRead(ctxt, hintAddr))
    {
        addr = SFTLookup(ctxt, lba);
        if (SEFIsEqualFlashAddress(addr, hintAddr))
        {
            break;
        }
        SFTReleaseForRead(ctxt, hintAddr);
        hintAddr = addr;
        addr.bits = 0;
#if CC_DEBUG_COUNTERS
        atomic_fetch_add(&sftState->lutRetry, 1);
#endif
    }
    return addr;
}

struct SEFFlashAddress SFTLookup(FTLContext *ctxt, int64_t userLBA)
{
    struct SEFFlashAddress flashAddress = SEFNullFlashAddress;
    struct SFTState *sftState = ctxt->sftState;

    // check for FTL/ user LBA validity
    if (!sftState->lut || userLBA >= sftState->lutSize)
    {
        return flashAddress;
    }

    flashAddress.bits = atomic_load(&sftState->lut[userLBA]);
    return flashAddress;
}

bool SFTLutChanged(FTLContext *ctxt,
                   struct SEFUserAddress userAddress,
                   struct SEFFlashAddress flashAddress,
                   uint32_t numADU)
{
    uint64_t lba = SEFGetUserAddressLba(userAddress);
    struct SFTState *sftState = ctxt->sftState;
    uint64_t end;
    uint32_t i;

    if (__builtin_add_overflow(lba, numADU, &end))
    {
        LogError(ctxt->logHandle, "Invalid lba/numAdu %" PRId16 "/%u\n", lba, numADU);
        return false;
    }

    for (i = 0; i < numADU; i++, lba++)
    {
        if (atomic_load(&sftState->lut[lba]) != flashAddress.bits)
        {
            return true;
        }
        flashAddress = SEFNextFlashAddress(ctxt->qosHandles[0], flashAddress);
    }
    return false;
}

//
// Update:
//   Get list of all sb's across all domains.
//   order sb's by serial number/eraseNumber in a heap
//     SB serial number is the serial number of the lowest ADU processed.
//   while(heap ! empty)
//     process top heap address,
//     move to next - refresh address list if empty
//     if no more address, remove from heap
//     else update sb's serial number with next address serial number
//
// Roughly twice the number of calls to read user addresses, but ~ same amount
// of data
// # "open" lists is # of placement ids
//
struct sbKey
{
    uint64_t sn;     // lowest unprocessed wsn in list (heap key)
    uint32_t bsn;    // Super block s/n
};

struct sbInfo
{
    int index;    // current entry in list
    int listSize;
    int metaSize;
    struct sbKey key;
    struct SEFFlashAddress addr;    // flash address of list->userAddressesRecovery[0];
    uint8_t *metadata;
    struct SEFUserAddressList *list;
};

struct sbHeap
{
    int size;
    int capacity;
    struct sbInfo data[/* sb's across all domains */];
};

static struct sbKey heapKey(struct sbHeap *heap, int index)
{
    if (index < heap->size)
    {
        return heap->data[index].key;
    }
    return (struct sbKey){.sn = UINT64_MAX, .bsn = UINT32_MAX};
}

static int heapCmp(struct sbKey a, struct sbKey b)
{
    // order by write sn then block sn - source gc block get processed
    // 1st, then dest gc block second.  If lba was rewritten during gc,
    // then re-written data is process last as it has the highest wsn.
    // last processed wins.
    if (a.sn < b.sn)
    {
        return -1;
    }
    if (b.sn < a.sn)
    {
        return 1;
    }
    if (a.bsn < b.bsn)
    {
        return -1;
    }
    if (b.bsn < a.bsn)
    {
        return 1;
    }
    return 0;
}

static int heapInsert(struct sbHeap *heap, struct sbInfo *entry)
{
    if (heap->size >= heap->capacity)
    {
        return -1;
    }
    int index = heap->size++;
    struct sbKey key = entry->key;
    while (index)
    {
        int parent = (index - 1) / 2;

        if (heapCmp(key, heapKey(heap, parent)) >= 0)
        {
            break;
        }
        heap->data[index] = heap->data[parent];
        index = parent;
    }
    heap->data[index] = *entry;
    return 0;
}

static void heapUpdateRootKey(struct sbHeap *heap, struct sbKey key)
{
    int index = 0;
    heap->data[0].key = key;
    struct sbInfo temp = heap->data[0];
    for (;;)
    {
        int swapIndex = 1 + index * 2;
        struct sbKey left = heapKey(heap, swapIndex);
        struct sbKey right = heapKey(heap, swapIndex + 1);

        if (heapCmp(key, left) <= 0 && heapCmp(key, right) <= 0)
        {
            break;
        }
        if (heapCmp(right, left) < 0)
        {
            swapIndex++;
        }
        heap->data[index] = heap->data[swapIndex];
        index = swapIndex;
    }
    if (index)
    {
        heap->data[index] = temp;
    }
}

static int heapPop(struct sbHeap *heap)
{
    if (heap->size == 0)
    {
        return -1;
    }
    heap->data[0] = heap->data[--heap->size];
    heapUpdateRootKey(heap, heap->data[0].key);
    return 0;
}

static struct SEFStatus sbInfoLoadList(FTLContext *ctxt, uint32_t bsn, struct sbInfo *info)
{
    struct SSBParsedFLA pfa;
    struct SEFStatus status;
    int nElems = ctxt->superBlockCapacity;
    int listSize = sizeof(*info->list) + nElems * sizeof(info->list->userAddressesRecovery[0]);
    int metaSize = nElems * ctxt->qosInfo.ADUsize.meta;

    status = SSBParseFlashAddress(ctxt, info->addr, &pfa);
    if (status.error)
    {
        return status;
    }
    if (pfa.adu != 0)
    {
        status.error = -EINVAL;
        status.info = 2;
        return status;
    }

    if (listSize > info->listSize)
    {
        SUfree(info->list);
        info->list = SUmalloc(listSize);
        info->listSize = listSize;
    }

    // get user addresses
    status = SEFGetUserAddressList(pfa.dstate->qosHandle, info->addr, info->list, listSize);
    if (status.error)
    {
        SUfree(info->list);
        info->list = NULL;
        SUfree(info->metadata);
        info->metadata = NULL;

        return status;
    }

    if (metaSize > info->metaSize)
    {
        SUfree(info->metadata);
        info->metadata = SUmalloc(metaSize);
        info->metaSize = metaSize;
    }

    if (info->metaSize)
    {
        // get meta data
        status = SEFReadWithPhysicalAddress(pfa.dstate->qosHandle, info->addr, nElems, NULL, 0, 0,
                                            SEFUserAddressIgnore, info->metadata, NULL);
        if (status.error)
        {
            SUfree(info->list);
            info->list = NULL;
            SUfree(info->metadata);
            info->metadata = NULL;

            return status;
        }
    }

    // populate the sbInfo
    info->index = 0;
    info->key.sn = UINT64_MAX;
    info->key.bsn = bsn;
    if (info->list->numADUs)
    {
        uint32_t userAddressMeta;
        int i;
        int ua = 0;

        // find a valid user address to type the block with
        for (i = 0; i < info->list->numADUs; i++)
        {
            if (info->list->userAddressesRecovery[i].unformatted != SEFUserAddressIgnore.unformatted)
            {
                ua = i;
                break;
            }
        }

        userAddressMeta = SEFGetUserAddressMeta(info->list->userAddressesRecovery[ua]);

        // mark LUT blocks
        if (userAddressMeta == SEF_LUT_META_TAG)
        {
            struct SSBParsedFLA pfa;

            SSBParseFlashAddress(ctxt, info->addr, &pfa);
            SSBMarkAsMeta(ctxt, &pfa);
        }

        // Only process user data blocks
        if (userAddressMeta == SEF_USR_META_TAG)
        {
            uint64_t wsn = 0;
            uint32_t meta_size = ctxt->qosInfo.ADUsize.meta;

            if (meta_size >= sizeof(wsn))
            {
                memcpy(&wsn, info->metadata + ua * meta_size, sizeof(wsn));
            }
            info->key.sn = wsn;
        }
        else
        {
            info->list->numADUs = 0;
        }
    }

    return status;
}

static struct SEFStatus rebuildLoadDomain(FTLContext *ctxt,
                                          struct sbHeap *heap,
                                          struct SSBDomainState *dstate,
                                          uint32_t minEraseOrder)
{
    struct SEFSuperBlockList *list = SSBGetSuperBlockList(ctxt, dstate);
    struct SEFStatus status = {};
    int i;

    if (!list)
    {
        return SUMakeStatusError(-ENOMEM);
    }

    for (i = 0; i < list->numSuperBlocks; i++)
    {
        struct SEFSuperBlockInfo sbInfo;
        struct sbInfo tmp = {.addr = list->superBlockRecords[i].flashAddress};
        uint32_t bsn;

        status = SEFGetSuperBlockInfo(dstate->qosHandle, list->superBlockRecords[i].flashAddress, 0,
                                      &sbInfo);
        bsn = sbInfo.eraseOrder;
        if (tmp.key.bsn < minEraseOrder)
        {
            continue;
        }

        status = sbInfoLoadList(ctxt, bsn, &tmp);
        if (status.error)
        {
            break;
        }
        if (tmp.list->numADUs > 0)
        {
            heapInsert(heap, &tmp);
        }
        else
        {
            SUfree(tmp.list);
            SUfree(tmp.metadata);
        }
    }
    SUfree(list);
    return status;
}

struct SEFStatus SFTRebuildLut(FTLContext *ctxt, struct SEFFlashAddress baseBlock)
{
    uint64_t wsn = atomic_load(&ctxt->wsn);    // tracks maximum wsn seen
    uint32_t minEraseOrder = 0;
    struct SEFStatus status;
    struct sbHeap *heap;
    struct SEFSuperBlockList sbList = {};
    struct SSBDomainState *dstate = ctxt->ssbState->dstate;
    struct SFTState *sftState = ctxt->sftState;
    int heapCapacity = 0;
    bool lbaUpdated = false;
    uint32_t metaSize = 0;

    // Load into heap sb info across all domains sorted by the wsn of the
    // first ADU then by the block sn
    int i;
    for (i = 0; i < ctxt->numDomains; i++)
    {
        SEFGetSuperBlockList(dstate[i].qosHandle, &sbList, sizeof(sbList));
        heapCapacity += sbList.numSuperBlocks;
    }

    heap = SUmalloc(sizeof(*heap) + heapCapacity * sizeof(heap->data[0]));
    if (heap == NULL)
    {
        LogError(ctxt->logHandle, "Failed to allocate %zd bytes for recovery",
                 sizeof(*heap) + heapCapacity * sizeof(heap->data[0]));
        return SUMakeStatusError(-ENOMEM);
    }
    heap->size = 0;
    heap->capacity = heapCapacity;
    if (baseBlock.bits)
    {
        struct SSBParsedFLA pfa;
        struct SEFSuperBlockInfo sbInfo;

        status = SSBParseFlashAddress(ctxt, baseBlock, &pfa);
        if (status.error)
        {
            goto Exit;
        }
        status = SEFGetSuperBlockInfo(pfa.dstate->qosHandle, baseBlock, 0, &sbInfo);
        if (status.error)
        {
            goto Exit;
        }
        // if eraseOrder is 0, it's not implemented; load everything
        minEraseOrder = sbInfo.eraseOrder ? sbInfo.eraseOrder + 1 : 0;
    }

    for (i = 0; i < ctxt->numDomains; i++)
    {
        status = rebuildLoadDomain(ctxt, heap, &dstate[i], minEraseOrder);
    }

    // Process all the sb's in the heap, always processing the sb that's at the
    // root of the heap (lowest wsn).  After the sb is processed, it's key is
    // update with the next ADU's wsn and "reinserted".  Once all it's ADUs have
    // been processed the sb is popped off the heap.
    struct sbInfo *root = &heap->data[0];
    metaSize = ctxt->qosInfo.ADUsize.meta;
    while (heap->size)
    {
        uint64_t lba;
        uint32_t meta;
        uint64_t sn = 0;

        if (metaSize)
        {
            memcpy(&sn, root->metadata + root->index * metaSize, sizeof(sn));
        }

        SEFParseUserAddress(root->list->userAddressesRecovery[root->index], &lba, &meta);
        if (lba < sftState->lutSize && meta == SEF_USR_META_TAG)
        {
            if (metaSize == 0)
            {
                lbaUpdated |= !SEFIsNullFlashAddress(SFTLookup(ctxt, lba));
            }
            SFTSet(ctxt, lba, root->addr, (struct SEFPlacementID){0});
            if (sn != UINT64_MAX && sn > wsn)
            {
                wsn = sn;
            }
        }

        // find the next valid ADU (user address != 0xffh)
        for (;;)
        {
            struct SEFUserAddress ua;

            root->addr = SEFNextFlashAddress(dstate->qosHandle, root->addr);
            if (++root->index == root->list->numADUs)
            {
                break;
            }
            ua = root->list->userAddressesRecovery[root->index];
            if (ua.unformatted != SEFUserAddressIgnore.unformatted)
            {
                break;
            }
        }

        if (root->index == root->list->numADUs)
        {
            SUfree(root->list);
            SUfree(root->metadata);
            heapPop(heap);
        }
        else
        {
            struct sbKey key = {.bsn = root->key.bsn};

            if (metaSize)
            {
                memcpy(&key.sn, root->metadata + root->index * metaSize, sizeof(key.sn));
            }

            heapUpdateRootKey(heap, key);
        }
    }
    struct SSBIterator itr;

    // SBB state and LUT have been rebuilt - mark all super blocks as closed
    sftForEachBlock(ctxt, itr, kSSBStateOpen)
    {
        struct SEFSuperBlockInfo sbInfo;
        struct SSBParsedFLA pfa;

        status = SSBParseFlashAddress(ctxt, itr.info.flashAddress, &pfa);
        if (status.error)
        {
            goto Exit;
        }
        status = SEFGetSuperBlockInfo(pfa.dstate->qosHandle, itr.info.flashAddress, 0, &sbInfo);
        if (status.error)
        {
            goto Exit;
        }
        SSBClosed(ctxt, &pfa, sbInfo.writtenADUs, sbInfo.writableADUs);
    }
Exit:
    if (metaSize == 0 && lbaUpdated)
    {
        LogError(ctxt->logHandle,
                 "Recovered LUT may reference old data.  User data may be corrupt");
    }
    if (heap)
    {
        while (heap->size)
        {
            SUfree(heap->data[--heap->size].list);
        }

        SUfree(heap);
    }
    atomic_store(&ctxt->wsn, wsn);

    return status;
}
