/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * superblock.h
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
/** @defgroup  SefSdkSsb Super Block State Management API */

#pragma once

#ifndef SEF_SDK_SUPERBLOCK_H
#define SEF_SDK_SUPERBLOCK_H

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <inttypes.h>
#include <stdatomic.h>

#include "flash-translation.h"
#include "log-manager.h"
#include "sef-utils.h"

struct gcContext;

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Denotes the Super Block current state
 *
 *  The maximum value must be less than or equal to kSSBStateMask.
 *
 *  @note       Code relies on kSSBStateDeleted to be 0 (init).
 *
 *  @see        SSBQdState
 */
enum SSBStateEnum {
    kSSBStateDeleted = 0x0, /**< Deleted (ptr to map should be NULL) */
    kSSBStateOpen = 0x1,    /**< Available for read and write */
    kSSBStateClosed = 0x2,  /**< Available for read and collection */
    kSSBStateBusy = 0x3,    /**< Empty but busy (has in-flight read I/O) */
    kSSBStateEmpty = 0x4,   /**< Empty (being released) */
    kSSBStateMeta = 0x5,    /**< Used for FTL metadata */
    kSSBStateAlloc = 0x6,   /**< Allocated - not online (e.g., copy) */
    kSSBStateMax = 0x6,     /**< For declaring arrays indexed by state */
    /* 0x7 available */
};

#define kSSBBusyShift 3                    /**< Number of bit shifts to get busy count */
#define kSSBBusyInc   (1 << kSSBBusyShift) /**< Value to add to busy count */
#define kSSBStateMask (kSSBBusyInc - 1)    /**< Bitmask to isolate state */
#define kSSBBusyMask  (~kSSBStateMask)     /**< Bitmask to isolate busy count */

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Active super block state (open/closed)
 *  @note       A map used to keep track of valid data in a super block
 */
struct SSBValidMap
{
#if CC_DEMO_DP
    atomic_uint_least32_t writableADUs; /**< Proxy for SB info written (open) and writableADUs (closed) */
#else
    uint32_t writableADUs; /**< Cached copy of writableADUs from super block info; 0 if not set */
#endif
    atomic_uint_least32_t refCnt; /**< Number of valid ADUs (bits set to 1 in map) +1 if state is open */
    atomic_uint_least64_t map[]; /**< Bitmap of which ADUs are valid in this block.  Stored LE
                                      because it's passed to the device */
};

/**
 *  @ingroup    SefSdkSsb
 *  @brief      State data for each super block.
 */
struct SSBBlockData
{
    atomic_uint state; /**< The SSBStateEnum are stored in the 3 bottom bits of the 32 bit value.
                        * The kSSBStateMask can be used to retrieve the state.
                        * The upper 29 bits are used as a busy counter and can only be incremented
                        * when "Available for read". The kSSBBusyMask can be used to retrieve the
                        * counters. */
    struct SEFPlacementID placementID; /**< Placement ID (set by GC or copied from super block info) */
    uint16_t GCRequired : 1; /**< The super block should be selected for garbage collect */
    uint16_t id : 15;        /**< Instance id, incremented every time it's allocated */
#if CC_PER_SB_LATENCY
    atomic_uint_fast64_t readADU;
    atomic_uint_fast64_t readLatency_ns;
    atomic_uint_fast64_t readMaxLatency_ns;
    atomic_uint_fast64_t readMinLatency_ns;
#endif
    struct SSBValidMap *vData; /**< Null if deleted or meta block */
};

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Flash address info
 */
struct SSBParsedFLA
{
    struct SEFFlashAddress flashAddress; /**< Unparsed address */
    uint32_t block;                      /**< SEF's super block number */
    uint32_t adu;                        /**< SEF's ADU number */
    struct SEFQoSDomainID domain;        /**< SEF's QoS Domain ID */
    struct SSBDomainState *dstate;       /**< Pointer to domain state for the address */
    struct SSBBlockData *bstate;         /**< Pointer to block state for the address */
};

/**
 *  @ingroup    SefSdkSft
 *  @brief      Per domain state for each placement ID
 */
struct SSBPlacementState
{
    uint32_t openBlock;             /**< Guess of block number last opened  */
    atomic_uint_least64_t writeADU; /**< Number of written ADUs */
};

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Super block iterator
 */
struct SSBIterator
{
    /**
     * @brief Filter for an SSBIterator
     */
    struct SSBItrFilter
    {
        struct SSBDomainState *dstate; /**< Domain to iterate */
        uint32_t index;                /**< 0 to start enumeration */
        int16_t placement;             /**< -1 for all placement IDs */
        enum SSBStateEnum state;       /**< The expected state */
    } filter;                          /**< Filter to select which blocks to return */
    /**
     * @brief Information about a super block in an SSBIterator
     */
    struct SSBItrInfo
    {
        uint32_t block;                      /**< Block ID */
        struct SEFPlacementID placementID;   /**< Placement ID */
        uint16_t id;                         /**< Instance ID */
        uint32_t nValid;                     /**< Number of valid ADUs in the block */
        uint32_t nInvalid;                   /**< Number of invalid ADUs in the block */
        struct SEFFlashAddress flashAddress; /**< Flash address for the block */
        uint8_t GCRequired;                  /**< !0 if marked to force collection */
    } info;                                  /**< Super block information */
};

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Per domain state for the super block layer
 */
struct SSBDomainState
{
    /* qosHandle of 0 terminates dstate list so keep it at the
       top of the struct (to reduce how much extra memory is allocated) */
    SEFQoSHandle qosHandle;           /**< Handle to the QoS Domain */
    struct SEFQoSDomainID domainId;   /**< QoS Domain ID */
    atomic_uchar isQueued;            /**< If non-zero, queued for GC */
    struct sefSlistNode link;         /**< Link for gcContext.gcQueue */
    struct gcContext *gctx;           /**< GC context for this domain */
    struct SEFDieList *dieList;       /**< List of dies that are used by this domain */
    uint64_t maxAdus;                 /**< Limit on adusUsed before write I/O will block */
    atomic_uint_least64_t adusQueued; /**< Number of adus in fsWaiters list */
    atomic_uint_least64_t adusUsed;   /**< Number of adus allocated against the
                                           domain quota + adusQueues */
    atomic_uint_least64_t blocksUsed; /**< Number of closed super blocks in valid array */
    pthread_mutex_t fsLock;           /**< fsWaiters lock */
    TmaDList fsWaiters;               /**< I/Os waiting for a free adus */
    pthread_mutex_t validLock;        /**< Serializes the creation of
                                           new entries in the validMap */

    struct PendingWorkCounter blkRelease; /**< So we can wait for releases to finish */
    struct SSBPlacementState *pState;     /**< Per-placement ID state */
    uint32_t blockCount;                  /**< Maximum super block number (sizeof valid array) */
    int mapSize;                          /**< Size of SSBValidMap.map */
    struct SSBBlockData *valid;           /**< Super block and page valid state array */
    bool inGC;                            /**< True if GC is running for this domain */
    bool needPatrol;                      /**< True to cause a patrol check after a GC run */
    bool blocksOpen;                      /**< True if shutdown couldn't close open super blocks*/
};

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Super block layer state
 */
// SSB data
struct SSBState
{
    void (*blockReleased)(struct FTLContext_ *ctxt,
                          struct SSBDomainState *dstate); /**< Called by SSB when a block is released */
    atomic_uint_least64_t openBlocks;                     /**< Number of open super blocks */
    int ssbIoCntId;                                       /**< SSB io counter registration id */
    int ssbStateCntId;                                    /**< SSB state counter registration id */
    atomic_uint_least64_t releasedBlockCount;             /**< Number of blocks released */
    atomic_uint_least64_t openedBlockCount;               /**< Number of blocks opened */
    atomic_uint_least64_t maintBlocks; /**< Number of blocks released for maintenance */
#if CC_DEMO_DP
    atomic_uint_least64_t validAdus; /**< Number of valid ADUs */
#endif
#if CC_DEBUG_COUNTERS
    atomic_uint_least64_t openRetry;  /**< Moving block to open retry cnt */
    atomic_uint_least64_t emptyRetry; /**< Moving block to empty retry cnt */
    atomic_uint_least64_t closeRetry; /**< Moving block to closed retry cnt */
    atomic_uint_least64_t staleRead;  /**< LUT entry used to read was stale */
    atomic_uint_least64_t busyOverflow; /**< Busy count at max (will never happen without coding mistake) */
    atomic_uint_least64_t busyRetry;      /**< Moving block to busy retry cnt */
    atomic_uint_least64_t emptyNotClosed; /**< Block became empty before closed */
#endif
    int num_qos;                    /**< Number of dstate array */
    struct SSBDomainState dstate[]; /**< Array of domain states for super block
                                     manager, also NULL terminated */
};

/**
 *  @ingroup    SefSdkSsb
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *  @param      flashAddress         The flash address that needs to be parsed
 *  @param[out] parsedAddr           Flash address's information
 *
 *  @retval     -EINVAL:0 if the domain ID in the flash address is bad; info is 0
 *  @retval     -EINVAL:1 if the block ID is too large; info is 1
 *  @retval     -EINVAL:2 if the ADU offset is too large; info is 2
 */
static inline struct SEFStatus SSBParseFlashAddress(FTLContext *ctxt,
                                                    struct SEFFlashAddress flashAddress,
                                                    struct SSBParsedFLA *parsedAddr)
{
    struct SSBDomainState *dstate = ctxt->ssbState->dstate;

    if (SEFIsNullFlashAddress(flashAddress))
    {
        return SUMakeStatus(-EINVAL, 2);
    }

    SEFParseFlashAddress(dstate->qosHandle, flashAddress, &parsedAddr->domain, &parsedAddr->block,
                         &parsedAddr->adu);

    int domainIndex = parsedAddr->domain.id - dstate->domainId.id;
    assert(domainIndex >= 0 && domainIndex < ctxt->ssbState->num_qos);
    if (domainIndex >= ctxt->ssbState->num_qos)
    {
        LogError(ctxt->logHandle, "Bad domain id %d (%d - %d)", parsedAddr->domain.id,
                 dstate->domainId.id, dstate->domainId.id + ctxt->ssbState->num_qos - 1);
        return SUMakeStatus(-EINVAL, 0);
    }

    dstate += domainIndex;

    assert(parsedAddr->block < dstate->blockCount);
    if (parsedAddr->block >= dstate->blockCount)
    {
        LogError(ctxt->logHandle, "Bad flash address 0x%lx, block %d too large (%d)",
                 flashAddress.bits, parsedAddr->block, dstate->blockCount);
        return SUMakeStatus(-EINVAL, 1);
    }

    if (parsedAddr->adu > ctxt->superBlockCapacity)
    {    // allow one past for adus used as non-inclusive end
        LogError(ctxt->logHandle, "Bad flash address 0x%lx, ADU %d too large (%" PRIu64 ")",
                 flashAddress.bits, parsedAddr->adu, ctxt->superBlockCapacity);
        return SUMakeStatus(-EINVAL, 2);
    }

    parsedAddr->dstate = dstate;
    parsedAddr->bstate = &dstate->valid[parsedAddr->block];
    parsedAddr->flashAddress = flashAddress;

    return SUMakeStatusOk();
}

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Remove a reference to a block. If it hits zero, it will move to empty.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *  @param      pfa                  The flash address for the super block
 *
 *  @returns    What the refcount was before being decremented.  If 1, the block
 *              was moved to the empty state.
 */
uint32_t SSBRemoveRef(FTLContext *ctxt, struct SSBParsedFLA *pfa);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Mark a block as a meta-data block
 *
 *  Blocks that are managed outside the FTL, like the persisted LUT, need
 *  to be marked as meta blocks or risk being deleted as leaked data blocks.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *  @param      pfa                  Block address to mark as a meta block
 */
void SSBMarkAsMeta(FTLContext *ctxt, struct SSBParsedFLA *pfa);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Inform super block layer a block is now closed.
 *
 *  To be called when a QoS notification indicates a block has closed.  If this
 *  is not called, it can never be the source of a garbage collection.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *  @param      pfa                  Block address that was closed
 *  @param      writtenADUs          Number of ADUs written to the super block.
 *  @param      numADUs              The size of the block in ADUs (how much quota
 *                                   it's consuming).  Subtracting writtenADUs
 *                                   is how much padding the device added.
 */
void SSBClosed(FTLContext *ctxt, struct SSBParsedFLA *pfa, uint32_t writtenADUs, uint32_t numADUs);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Inform super block layer a block has been deleted
 *
 *  Called by SEFBlockCheck() when cleaning up post repair.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *  @param      pfa                  Block address that was deleted
 */
void SSBDeleted(FTLContext *ctxt, struct SSBParsedFLA *pfa);

/**
 * @ingroup    SefSdkSsb
 * @brief Marks a block for maintenance
 *
 * GC will collect it moving the data to a new super block and then release
 * it back to the virtual device free pool.  GC decides when this will happen.
 *
 * @param     ctxt          A pointer to the instance of FTLContext
 * @param     flashAddress  The flash address that requires maintenance
 */
void SSBMarkForMaintenance(FTLContext *ctxt, struct SEFFlashAddress flashAddress);

/**
 * @ingroup    SefSdkSsb
 * @brief Returns the number of writable ADUs in a super block
 *
 * Returns the cached number of writable ADUs.  When not cached, the cached
 * value is set after getting the current value from the device.  When the
 * super block is open or cannot be marked as busy, 0 is returned.
 *
 * @param     ctxt          A pointer to the instance of FTLContext
 * @param     pfa           The parsed flash address of the super block
 *
 * @returns         Number of writable ADUs for a super block
 *
 * @retval    0     The block is open or failed to be marked as busy.
 */
uint32_t SSBGetWritableADUs(FTLContext *ctxt, struct SSBParsedFLA *pfa);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Iterate through all the blocks owned by a domain
 *
 *  To start an iteration, set filter members. Call to fill in info member
 *  for each block until false is returned.
 *
 *  To verify a block is still in-use by a domain, mark busy with SSBMarkBusy()
 *  and then release with SSBMarkNotBusy().  This will prevent the block from
 *  being deleted.  It is possible a block will be deleted and re-allocated
 *  between iteration and being marked as busy.  The instance ID can be used
 *  to verify this has not occurred.
 *
 *  @param          ctxt    A pointer to an instance of FTLContext
 *  @param[in,out]  itr     Iterator defining filter and holding the result
 *
 *  @retval         true    itr->info fill with info for the next block
 *  @retval         false   No more blocks to iterate
 */
bool SSBEnumBlocks(FTLContext *ctxt, struct SSBIterator *itr);

/**
 * @ingroup    SefSdkSsb
 * @brief      Get/refresh info for a block in a domain
 *
 * @param       ctxt    A pointer to an instance of FTLContext
 * @param       dstate  Domain state to get info from
 * @param       block   Block ID to get info for
 * @param[out]  info    Info to fill in
 */
void SSBGetInfo(FTLContext *ctxt, struct SSBDomainState *dstate, uint32_t block, struct SSBItrInfo *info);

/**
 * @ingroup    SefSdkSsb
 * @brief Moves a block to the allocated state
 *
 * The block must be in the deleted state. After this function returns, it will
 * be in the allocated state with a reference count of 2: one for the caller to
 * decrement with SSBRemoveRef() to allow the block to move to empty, and one for
 * the open state to decrement once the close event is processed.
 *
 * @param ctxt          FTL context to operate on
 * @param pfa           Super block flash address to place in the alloc state
 * @param placementID   Placement ID for the block (used by GC to keep data
 *                      together)
 * @param writableADUs  Writable ADUs returned from SEFAllocateSuperBlock()
 */
void SSBAllocated(FTLContext *ctxt,
                  struct SSBParsedFLA *pfa,
                  struct SEFPlacementID placementID,
                  uint32_t writableADUs);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Adds ADUs to the accounting of used ADUs for a domain
 *
 *  As writes consume ADUs, the SSB and FTL track their use and FTL will block a
 *  write when it exceeds the max ADUs allowed for a domain.  When a super block
 *  is filled by other means (i.e., GC), this function must be called to keep
 *  the SSB's accounting correct.
 *
 *  @param  dstate      Domain to add used ADUs to
 *  @param  adusUsed    Number of ADUs to add as used
 */
void SSBAddAdusUsed(struct SSBDomainState *dstate, uint32_t adusUsed);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Copy the valid bits from a block
 *
 *  The returned status.info is an estimate of the number of valid bits.  It is
 *  number of valid adus after the bitmap is copied so may be too small, however
 *  SSBClearAduValid() clears the ADU valid bit before decrementing the valid
 *  count so the estimate may also be too large.
 *
 *  @param      pfa         Super block flash address to copy valid bits for
 *  @param      buffer      Destination buffer for the bits
 *  @param      size        Size of the destination buffer in bytes
 *
 *  @retval     0           Bits copied; info estimate of number of valid bits set
 *  @retval     -ENOBUFS    Destination buffer size is too small to hold the valid bits
 */
struct SEFStatus SSBCopyValidBits(struct SSBParsedFLA *pfa, void *buffer, ssize_t size);

/**
 * @ingroup    SefSdkSsb
 * @brief Closes all the domain's open blocks that are referenced by the LUT.
 *
 * After this function returns, this domain will not be responsible for any
 * open super blocks.  This may result in notifications that a tentative
 * address has been moved to a new flash address as well as super block close, so
 * the system should be functional enough to handle the notifications.
 *
 * @param ctxt      FTL context to operate on
 * @param ssbState  Super block state to close open blocks
 */
void SSBCloseOpenBlocks(FTLContext *ctxt, struct SSBState *ssbState);

/**
 * @ingroup    SefSdkSsb
 * @brief   Waits for empty blocks to be released
 *
 * @param dstate     Domain state of QoS domain to flush
 *
 */
void SSBFlushQdEmptyBlocks(struct SSBDomainState *dstate);

/**
 * @ingroup    SefSdkSsb
 * @brief   Waits for empty blocks to be released
 *
 * @param ssbState     Super block state to flush
 *
 */
void SSBFlushEmptyBlocks(struct SSBState *ssbState);

/**
 * @ingroup    SefSdkSsb
 * @brief Returns the max size in bytes of persisted SBS state
 *
 * Used during block configuration to reserved space in a domain for saved
 * super block state.
 *
 * @param       sefInfo     SefInfo for qdInfo
 * @param       qdInfo      Domain info being configured
 * @param       numADUs     Size of user data space in ADUs
 */
uint64_t SBSStateSize(const struct SEFInfo *sefInfo, struct SEFQoSDomainInfo *qdInfo, int64_t numADUs);

/**
 * @ingroup    SefSdkSsb
 * @brief Queues to a PDLHandle a request to write the state for a domain.
 *
 * The per-block state will be written when pdlHandle is flushed with
 * PDLFlushToFlash().
 *
 * @param       ctxt        A pointer to an instance of FTLContext
 * @param       pdlHandle   Handle for queuing write state
 */
void SSBQueueToWriteState(FTLContext *ctxt, PDLHandle pdlHandle);

/**
 * @ingroup    SefSdkSsb
 * @brief      Restore state from flash if present
 *
 * @param      ctxt        A pointer to an instance of FTLContext
 * @param      hPdl        Persistence handle to restore from
 *
 * @retval     0           Success; info = state restored from flash
 * @retval     -ENOMEM     Not enough memory to initialize
 */
struct SEFStatus SSBRestore(FTLContext *ctxt, PDLHandle hPdl);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      This function is used to initialize the super block layer.
 *
 *  @param      ctxt          A pointer to an instance of FTLContext
 *  @param      hPdl          Persistence handle, when non-null is used to
 *                            restore SSB state from FLASH.
 *  @param      blockReleased Called after a super block is released
 *
 *  @retval     0           Success
 *  @retval     -ENOMEM     Not enough memory to initialize
 */
struct SEFStatus SSBInitialize(FTLContext *ctxt,
                               PDLHandle hPdl,
                               void (*blockReleased)(struct FTLContext_ *ctxt,
                                                     struct SSBDomainState *dstate));

/**
 *  @ingroup    SefSdkSsb
 *  @brief      This function is used to clean up the address translation components.
 *
 *  @param      ctxt             A pointer to an instance of FTLContext
 */
void SSBCleanup(FTLContext *ctxt);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Marks an ADU as in-use for a block.
 *
 *  When this is the first ADU for a block, the block will be moved into the
 *  open state.  The placementId parameter will be recorded as the placement ID
 *  for the block.
 *
 *  @param      ctxt             A pointer to an instance of FTLContext
 *  @param      flashAddress     Flash address of ADU to mark in-use
 *  @param      placementId      Placement ID of the block
 *
 *  @retval     0           Success
 *  @retval     -ENOMEM     Not enough memory to move the block to the open state
 */
int SSBSetAduValid(FTLContext *ctxt, struct SEFFlashAddress flashAddress, struct SEFPlacementID placementId);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Unmarks an ADU as in-use for a block.
 *
 *  @param      ctxt             A pointer to an instance of FTLContext
 *  @param      flashAddress     Flash address of ADU to unmark as in-use
 *
 *  @retval     0       Success
 *  @retval     -EINVAL The flash address is bad
 */
int SSBClearAduValid(FTLContext *ctxt, struct SEFFlashAddress flashAddress);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Decrements a super block's busy count.
 *
 *  If the block is empty and the busy count becomes 0, the block will be queued
 *  for delete by the GC thread.
 *
 *  @todo   Remove ctxt once instrumentation counters are local
 *
 *  @param      ctxt    A pointer to an instance of FTLContext
 *  @param      pfa     Parsed flash address of block to mark not busy
 */
void SSBMarkNotBusy(FTLContext *ctxt, struct SSBParsedFLA *pfa);

/**
 *  @ingroup    SefSdkSsb
 *
 *  @param      ctxt    A pointer to an instance of FTLContext
 *  @param      pfa     Parsed flash address of block to mark busy
 *
 *  @retval     0       Failed to mark busy, block is not readable
 *  @retval     !0      Marked busy, value is block instance id
 */
int SSBMarkBusy(FTLContext *ctxt, struct SSBParsedFLA *pfa);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Finds blocks owned by the domain that are not referenced.
 *
 *  Compares what is allocated to the domain from the device with the super
 *  block layer's list of allocated blocks.  The leaked block count is returned
 *  in the info portion of a good status (error == 0).
 *
 *  When release is true, blocks open for copy are closed.
 *
 *  @param      ctxt        A pointer to an instance of FTLContext
 *  @param      ssbState    Super block state to scan
 *  @param      release     If true, leaked blocks are released
 *
 *  @retval     -ENOMEM     Not enough memory or device failed to return the
 *                          list of blocks
 *  @retval     0           Compare completed; info is the number of leaked blocks
 */
struct SEFStatus SSBFindLeakedBlocks(FTLContext *ctxt, struct SSBState *ssbState, bool release);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Returns the list of super blocks in a domain
 *
 *  Allocates the required memory and returns the list of super blocks in
 *  a domain.  Caller is responsible for freeing the memory of the returned
 *  super block list.
 *
 *  @returns    The list of super blocks in a domain
 */
struct SEFSuperBlockList *SSBGetSuperBlockList(FTLContext *ctxt, struct SSBDomainState *dstate);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Returns the QoS handle for a domain state
 *
 *  @param      dstate  Domain state to retrieve handle from
 *  @returns    The QoS Domain Handle for dstate
 */
SEFQoSHandle SSBGetQoSHandle(struct SSBDomainState *dstate);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Set patrol state for a domain
 *
 *  @param      dstate  Domain to set the patrol state
 *  @param      patrol  Patrol state to set, if true will trigger GC
 */
void SSBSetPatrol(struct SSBDomainState *dstate, bool patrol);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Get patrol state for a domain
 *
 *  @param      dstate  Domain to get the patrol state
 *  @returns    The patrol state of dstate
 */
bool SSBGetPatrol(struct SSBDomainState *dstate);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Set GC context for a domain
 *
 *  @param      dstate  Domain to set the GC context
 *  @param      gctx    GC contgext to set
 */
void SSBSetGC(struct SSBDomainState *dstate, struct gcContext *gctx);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Returns the QoS Domain id of a Domain State
 *
 *  @param      dstate  Domain state to get ID from
 *  @returns    QoS Domain ID of dstate
 */
struct SEFQoSDomainID SSBGetQoSID(struct SSBDomainState *dstate);

/**
 *  @ingroup    SefSdkSsb
 *  @brief      Returns number of available super blocks
 *
 *  @param      ctxt    Used to get highwater value
 *  @param      dstate  Domain state to get available block count from
 *  @returns    Number of available super blocks
 */
uint32_t SSBGetNumAvailable(FTLContext *ctxt, struct SSBDomainState *dstate);

#endif /* ADDRESS_TRANSLATION_H */
