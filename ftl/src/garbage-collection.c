/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * garbage-collection.c
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
#include "garbage-collection.h"

#include <SEFAPI.h>
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>    // INT_MAX...
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "config.h"
#include "flash-translation.h"
#include "log-manager.h"
#include "sef-utils.h"
#include "superblock.h"
#include "utils/sef-event.h"

// notification included to init MCP

#include "manual-copy.h"
#include "sef-notification.h"

#define GC_SB_LIST_SIZE 8 /**< Number of SB's to consider at a time to collect */

struct GCCounter
{
    atomic_uint_least64_t gcActive;          /**< GC state (0, idle) */
    atomic_uint_least64_t gcSrcWriteWAF;     /**< GC source block WAF * 1000 */
    atomic_uint_least64_t gcLateSBLock;      /**< SB lock failed post enum */
    atomic_uint_least64_t gcMaxLoops;        /**< Max gc loops req to free a super block */
    atomic_uint_least64_t gcNoProgress;      /**< GC made no progress while I/O is blocked */
    atomic_uint_least64_t GCRunCount;        /**< Number of times GC ran */
    atomic_uint_least64_t skippedADU;        /**< ADUs skipped in a partial block */
    atomic_uint_least64_t GCValidADUCopied;  /**< Number of ADUs copied as a result of GC */
    atomic_uint_least64_t namelessCopyCount; /**< Number of nameless copies */
    atomic_uint_least64_t partialBlockCopy;  /**< Partial GC blocks */
#if CC_DEBUG_COUNTERS
    atomic_uint_least64_t lateEmpty; /**< Block became empty after GC selection */
#endif
};

/**
 * @ingroup    SefSdkGc
 * @brief Context data passed to GC functions while GCing a domain
 */
struct gcContext
{
    FTLContext* ctxt;
    LogHandle hLog;        /**< Logger to use */
    SEFQoSHandle hQoS;     /**< Domain handle for info */
    INSHandle hInst;       /**< Instrumentation Handle */
    SEFHandle hSef;        /**< Unit handle */
    uint64_t gcAduTrigger; /**< Adu usage that triggers GC to run */

    int nQoS;                       /**< Number of domains */
    struct sefSlist gcQueue;        /**< List of domains to GC */
    struct SSBDomainState* dstate;  /**< Domain being collected */
    struct SEFFlashAddress dstAddr; /**< Destination of GC */
    uint64_t superBlockCapacity;    /**< Super block capacity */
    int32_t dstSize;                /**< ADUs in dstAddr */

    bool gcInit;             /**< true if GCInit() has been called */
    bool gcStarted;          /**< GC started (may not be running) */
    atomic_bool gcStop;      /**< true to cause thread to exit */
    struct SEFStatus status; /**< exit status for GC */
    void* notifyData;
    void (*gcNotify)(struct GCNotify*); /**< Event callback (e.g. when gc thread exits) */
    pthread_t FTLThread;                /**< Gc thread handle, only valid if gcStarted */
    struct sefEvent gcEvent;            /**< Event to trigger a GC */
    struct sefEvent canRun;             /**< set if gc not suspended */
    atomic_int suspend_count;           /**< Suspend count, if 0, not suspended */

    struct GCCounter counter;
    int stateCntId; /**< State instrumentation counter instance id */
    int ioCntId;    /**< IO instrumentation counter instance id */

    // Filled in by GcQosDomain to avoid having to pass them as individual
    // parameters.
    uint64_t* srcBitmap;   /**< bitmap to track which blocks have been collected */
    ssize_t srcBitmapSize; /**< size of srcBitmap in bytes */
    int pid;               /**< placement ID being collected, -1 if not set */
#if CC_MANUAL_NLC
    bool gcManual; /**< copy of ctxt->gcManual at the start of GC */
    MCPHandle mcpHandle;
#endif
    /**
     * @ingroup    SefSdkGc
     * @brief Heap per placement id of super blocks in rank order to collect
     */
    struct gcPidStats
    {
        int64_t numInvalid; /**< # of collectable ADUs per placement id */
        int64_t numValid;   /**< # of valid ADUs per placement id */
        int numBlocks;      /**< # of colletable blocks for a placement id */
        int size;           /**< # of items in the sbList */
        int head;           /**< head of the sbList */
        struct
        {
            int gcRank; /**< The priority of getting collected, lower collected first */
            int id;
            struct SEFFlashAddress flashAddress;
        } sbList[GC_SB_LIST_SIZE];
    }* pidCnts; /**< collectable stats per placement ID  */

    // Used during copy
    struct SEFCopyOverrides overrides; /**< Weights to use NLC if ctxt->wfqGc is true */
    struct sefEvent copyComplete;      /**< set when copyInFlight hits zero */
    bool destClosed;                /**< Set when NCL completes with kCopyClosedDestination flag */
    bool pSLC;                      /**< True if GC to pSLC */
    atomic_int copyInFlight;        /**< count # of copies inflight */
    atomic_int adusCopied;          /**< count of adus copied to detect padding */
    struct sefSlist completedIOCBs; /**< List of completed copy IOCBs to post process */
    // At the end because ends in a flex-array
    struct SEFQoSDomainInfo qosInfo;
};

void GCSetCopyIOWeights(struct gcContext* gctx, struct SEFCopyOverrides* overrides)
{
    gctx->overrides = *overrides;
}

/**
 * @brief Frees the resources allocated by GcContextInit()
 **/
static void GcContextCleanup(struct gcContext* gctx)
{
    assert(SefSlistPopAll(&gctx->completedIOCBs) == NULL);
    assert(atomic_load(&gctx->copyInFlight) == 0);

    SefEventFree(&gctx->gcEvent);
    SefEventFree(&gctx->copyComplete);
    if (gctx->srcBitmap)
    {
        SUfree(gctx->srcBitmap);
    }
    if (gctx->pidCnts)
    {
        SUfree(gctx->pidCnts);
    }
    gctx->srcBitmap = NULL;
    gctx->gcInit = false;
}

static uint64_t gcCalculateAduTrigger(struct gcContext* gctx, struct GCConfig* config)
{
    uint64_t maxLba = config->maxLba / config->nQoS;
    uint64_t aduTrigger;
    uint64_t op_adus;
    uint64_t min_op_adus;

    min_op_adus = 2 * gctx->superBlockCapacity;
#if CC_GC_SINGLE_PLID
    min_op_adus += gctx->qosInfo->numPlacementIDs * gctx->superBlockCapacity;
#endif
    assert(maxLba < gctx->dstate->maxAdus);
    op_adus = gctx->dstate->maxAdus - maxLba;
    assert(op_adus >= min_op_adus);
#if CC_WFQ_ZENO
    // Trigger is halfway through the non-min OP adus
    aduTrigger = gctx->dstate->maxAdus - (op_adus - min_op_adus) / 2;
#else
    // Trigger is at the end of OP
    aduTrigger = gctx->dstate->maxAdus - min_op_adus;
#endif
    return aduTrigger;
}

/**
 * @brief Initializes a GC Context
 *
 * The GC context is used to hold information about the domain being GC'd
 * This functions allocates the necessary memory for a blank context that
 * will be used by the GcThread and the functions it calls.
 **/
static struct SEFStatus GcContextInit(struct gcContext* gctx, struct GCConfig* config)
{
    struct SEFStatus status = {0};
    struct SEFQoSDomainID qosID;

    gctx->ctxt = config->ctxt;
    gctx->hLog = config->hLog;
    gctx->hInst = config->hInst;
    gctx->hQoS = SSBGetQoSHandle(config->dstate);
    gctx->dstate = config->dstate;
    gctx->hSef = SUGetHandle(gctx->hQoS);
    gctx->gcNotify = config->notify;
    gctx->notifyData = config->notifyData;
    gctx->pSLC = config->ctxt->pSLC;
    gctx->superBlockCapacity = config->ctxt->superBlockCapacity;

    qosID = SEFGetQoSHandleProperty(gctx->hQoS, kSefPropertyQoSDomainID).qosID;
    status = SEFGetQoSDomainInformation(gctx->hSef, qosID, &gctx->qosInfo);
    if (status.error)
    {
        LogError(gctx->hLog, "Get QoS Info failed (%d/%d)", status.error, status.info);
        return status;
    }

    gctx->gcAduTrigger = gcCalculateAduTrigger(gctx, config);
    LogInfo(gctx->hLog, "GC triggers at %" PRIu64 " ADUs used", gctx->gcAduTrigger);

    int ret = SefEventInit(&gctx->copyComplete, kSefEventFlagAutoReset);

    if (ret)
    {
        LogError(gctx->hLog, "Copy event failed to Init %d (%s)", ret, strerror(ret));
        status.error = ret;
        return status;
    }
    ret = SefEventInit(&gctx->gcEvent, kSefEventFlagAutoReset);
    if (ret)
    {
        LogError(gctx->hLog, "GC event failed to Init %d (%s)", ret, strerror(ret));
        status.error = ret;
        return status;
    }
    gctx->suspend_count = 0;
    SefEventInit(&gctx->canRun, kSefEventFlagSet);
    atomic_store(&gctx->copyInFlight, 0);
    SefSlistInitialize(&gctx->completedIOCBs);
    gctx->nQoS = config->nQoS;
    SefSlistInitialize(&gctx->gcQueue);
    gctx->srcBitmapSize = sizeof(gctx->srcBitmap[0]) * DIV_ROUND_UP(gctx->dstate->blockCount, 64);
    gctx->srcBitmap = SUmalloc(gctx->srcBitmapSize);
    if (!gctx->srcBitmap)
    {
        LogError(gctx->hLog, "Not enough memory for GC");
        GcContextCleanup(gctx);
        status.error = -ENOMEM;
        return status;
    }

    gctx->pidCnts = SUmalloc(gctx->qosInfo.numPlacementIDs * sizeof(gctx->pidCnts[0]));
    if (gctx->pidCnts == NULL)
    {
        LogError(gctx->hLog, "Not enough memory for GC");
        GcContextCleanup(gctx);
        status.error = -ENOMEM;
        return status;
    }

    // Use the default I/O weights
    memset(&gctx->overrides, 0xff, sizeof(gctx->overrides));

    return status;
}

/**
 * @brief Pushes a candidate flash address onto super block list for a placement id
 *
 * The address is only pushed if there is room or has fewer valid ADUs than
 * all the other blocks on the list, in which case, the candidate on the list
 * with the most valid ADUs is dropped from the list to make room.
 *
 * @param stats         Placement id stats structure to push the address onto
 * @param id            Instance id of the block
 * @param gcRank        The priority of getting collected, lower collected first
 * @param flashAddress  Candidate flash address
 *
 **/
static void gcPushCandidate(struct gcPidStats* stats, int id, int32_t gcRank, struct SEFFlashAddress flashAddress)
{
    assert(stats->head == 0);

    if (stats->size < NELEM(stats->sbList))
    {
        stats->sbList[stats->size++].gcRank = INT32_MAX;
    }
    if (gcRank < stats->sbList[stats->size - 1].gcRank)
    {
        int i;
        for (i = stats->size - 1; i >= 0; i--)
        {
            if (i == 0 || gcRank >= stats->sbList[i - 1].gcRank)
            {
                stats->sbList[i].id = id;
                stats->sbList[i].gcRank = gcRank;
                stats->sbList[i].flashAddress = flashAddress;
                break;
            }
            stats->sbList[i] = stats->sbList[i - 1];
        }
    }
}

/**
 * @brief Returns the next candidate with the fewest valid adus
 *
 * @param       stats   Placement ID stats to select a source block from
 * @param[out]  id      Instance id of block
 *
 * @returns     Source block flash address with the fewest valid ADUs or
 *              SEFAutoAllocate if no more candidates to return
 **/
static struct SEFFlashAddress gcPopCandidate(struct gcPidStats* stats, int* id)
{
    if (stats->head == stats->size)
    {
        return SEFAutoAllocate;
    }
    if (id)
    {
        *id = stats->sbList[stats->head].id;
    }
    return stats->sbList[stats->head++].flashAddress;
}

/**
 * @ingroup SefSdkGc
 * @brief Populates gctx->pidCnts with candidate source blocks for gctx->pid
 *
 * Expected that gcSelectPlacementIdToCollect() has been called first to
 * populate gctx->pid.  gctx->srcBitMap is used to skip blocks already
 * selected.
 */
static void gcBuildListOfCandidates(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    FTLContext* ctxt = gctx->ctxt;
    int pid = gctx->pid;

    memset(gctx->pidCnts, 0, gctx->qosInfo.numPlacementIDs * sizeof(gctx->pidCnts[0]));
    struct SSBIterator itr = {.filter = {.dstate = dstate, .placement = pid, .state = kSSBStateClosed}};

    while (SSBEnumBlocks(ctxt, &itr))
    {
        uint32_t block = itr.info.block;
        int pid = itr.info.placementID.id;

        if (itr.info.nInvalid == 0 && !itr.info.GCRequired)
        {
            continue;    // will make no progress to copy
        }

        // skip previously selected blocks so they aren't reselected
        if (gctx->srcBitmap[block / 64] & ((uint64_t)1 << (block % 64)))
        {
            continue;
        }

        gctx->pidCnts[pid].numBlocks += !!itr.info.nInvalid;
        gctx->pidCnts[pid].numInvalid += itr.info.nInvalid;
        gctx->pidCnts[pid].numValid += itr.info.nValid;

        if (itr.info.GCRequired)
        {
            // set the gcRank to 0 if it should be collected
            gcPushCandidate(&gctx->pidCnts[pid], itr.info.id, 0, itr.info.flashAddress);
        }
        else
        {
            // use the number of valid ADUs to rank the super block
            gcPushCandidate(&gctx->pidCnts[pid], itr.info.id, itr.info.nValid, itr.info.flashAddress);
        }
    }
}

#if !CC_GC_SINGLE_PLID
static uint64_t gcNumInvalid(struct SEFQoSDomainInfo* qosInfo, const struct gcPidStats* stats)
{
    int i;
    uint64_t numInvalid = 0;

    for (i = 0; i < qosInfo->numPlacementIDs; i++)
    {
        if (stats[i].size != 0 && stats[i].sbList[0].gcRank == 0)
        {
            return qosInfo->superBlockCapacity;
        }

        numInvalid += stats[i].numInvalid;
    }
    return numInvalid;
}
#endif

/**
 * @brief pick the placement id that has the most invalid ADUs by %.
 *
 * Only blocks that are collectable are considered when calculating the %.
 * The total across all placement IDs must have at least one superblock's worth
 * of ADUs to reclaim.  Those with enough valid blocks to fill the destination
 * are selected before those that do not.
 *
 * Placement id's that contain a block marked for wear leveling are returned
 * 1st.
 *
 * @returns The placement id to collect
 */
static int gcSelectPlacementIdToCollect(struct SEFQoSDomainInfo* qosInfo, const struct gcPidStats* stats)
{
    int i;
    int maxRatio = -1;
    int maxId = -1;
    int maxFull = 0;

    for (i = 0; i < qosInfo->numPlacementIDs; i++)
    {
        int64_t ratio = stats[i].numInvalid * INT64_C(1000) / (stats[i].numBlocks ?: 1);
        int full = (stats[i].numValid >= qosInfo->superBlockCapacity);

        if (stats[i].size != 0 && stats[i].sbList[0].gcRank == 0)
        {
            return i;
        }

        if (stats[i].numInvalid == 0)
        {
            continue;
        }
        if (full > maxFull || ratio > maxRatio)
        {
            maxRatio = ratio;
            maxFull = full;
            maxId = i;
        }
    }
    return maxId;
}

/**
 * @brief Scans the full blocks and selects a superblock to collect
 *
 * On first call (gctx->pid == -1), selects a placement ID to collect and
 * on subsequent calls only returns blocks from that placement ID until it
 * runs out of blocks to collect and switches to another placement ID.  It may
 * select a placement ID even though the number of invalid ADUs for the whole
 * domain is less than a super block. If the caller is starting a collection,
 * and not block maintenance, it should verify with gcNumInvalid() that a
 * collection will make progress.
 *
 * The current strategy is O(n) for each pass to find the least utilized
 * superblock.  A small list is kept to avoid a full scan every call.
 *
 * Note, as an optimization the loop skips blocks with zero valid as they will
 * be in the empty list already and processed there.  It's possible that by
 * the time the nameless copy is done, there will be no more valid entries left
 * and can be skipped as well.
 *
 * @param       gctx          GC context
 * @param       dstate        Domain being collected
 * @param[out]  id            Instance id of block to collect
 *
 * @returns     flash address deemed best to collect (SEFAutoAllocate if none)
 */
static struct SEFFlashAddress gcSelectBlockToCollect(struct gcContext* gctx,
                                                     struct SSBDomainState* dstate,
                                                     int* id)
{
    struct SEFFlashAddress sbAddress;

    // No PID selected, build a cache of potential blocks and pick a PID to collect
    if (gctx->pid == -1)
    {
        gcBuildListOfCandidates(gctx, dstate);
        gctx->pid = gcSelectPlacementIdToCollect(&gctx->qosInfo, gctx->pidCnts);
        if (gctx->pid == -1)
        {
            return SEFAutoAllocate;
        }
    }

    // If depleted the cache of blocks, find more
    if (gctx->pidCnts[gctx->pid].size == gctx->pidCnts[gctx->pid].head)
    {
        gcBuildListOfCandidates(gctx, dstate);
    }
    sbAddress = gcPopCandidate(&gctx->pidCnts[gctx->pid], id);

#if !CC_GC_SINGLE_PLID
    // If PID ran out of blocks, find the next best PID to collect from
    if (SEFIsEqualFlashAddress(sbAddress, SEFAutoAllocate))
    {
        LogTrace(gctx->hLog, "PLID %d ran out of blocks to collect", gctx->pid);
        gctx->pid = -1;
        gcBuildListOfCandidates(gctx, dstate);
        gctx->pid = gcSelectPlacementIdToCollect(&gctx->qosInfo, gctx->pidCnts);
        if (gctx->pid == -1)
        {
            return SEFAutoAllocate;
        }
        LogTrace(gctx->hLog, "Switching to PLID %d for blocks to collect", gctx->pid);
        sbAddress = gcPopCandidate(&gctx->pidCnts[gctx->pid], id);
    }
#endif

    return sbAddress;
}

#if CC_WFQ_ZENO
static uint32_t gcCalcCycleValid(struct gcContext* gctx, struct SSBDomainState* dstate, uint32_t valid)
{
    struct gcPidStats* stats = &gctx->pidCnts[gctx->pid];
    int sbIndex = stats->head;
    int total = valid;
    int nBlocks = 1;

    while (valid < gctx->superBlockCapacity && sbIndex < stats->size)
    {
        struct SSBItrInfo info = {};
        uint32_t block;

        SEFParseFlashAddress(dstate->qosHandle, stats->sbList[sbIndex].flashAddress, NULL, &block, NULL);
        SSBGetInfo(gctx->ctxt, dstate, block, &info);
        total += info.nValid;
        sbIndex++;
        nBlocks++;
    }
    return total / nBlocks;
}
#endif

/**
 * @ingroup SefSdkGc
 * @brief Completion routine of nameless copy for gcToASuperBlock()
 *
 * Queues the nameless copy's IOCB for processing after the
 * destination super block is full.
 */
static void GcCopyComplete(struct SEFCommonIOCB* common)
{
    struct SEFNamelessCopyIOCB* iocb = (void*)common;
    struct gcContext* gctx = iocb->common.param1;
    struct SSBParsedFLA pfa;

    SSBParseFlashAddress(gctx->ctxt, iocb->copySource.srcFlashAddress, &pfa);
    // Simulator may return ENXIO (-6), for one of the partially copied blocks
    if (iocb->common.status.error)
    {
        LogError(gctx->hLog, "nameless copy for 0x%lx completed with error %d/%d",
                 iocb->copySource.srcFlashAddress.bits, iocb->common.status.error,
                 iocb->common.status.info);
    }
    else
    {
        LogTrace(gctx->hLog, "Nameless copy of 0x%lx complete of %d ADUs",
                 iocb->copySource.srcFlashAddress.bits, iocb->addressChangeInfo->numProcessedADUs);
        if (iocb->common.status.info & kCopyClosedDestination)
        {
            gctx->destClosed = true;
        }
        atomic_fetch_add(&gctx->adusCopied, iocb->addressChangeInfo->numProcessedADUs);
    }

    SefSlistPush(&gctx->completedIOCBs, (struct sefSlistNode*)&iocb->common.param1);
    if (atomic_fetch_sub(&gctx->copyInFlight, 1) == 1)
    {
        SefEventSet(&gctx->copyComplete);
    }
    atomic_fetch_add(&gctx->counter.namelessCopyCount, 1);
    SSBMarkNotBusy(gctx->ctxt, &pfa);
}

/**
 * @brief Sends GC notification of estimated WAF
 *
 * Calls the gc notification registered when GC was initialized with an
 * estimate of the current WAF.  This is the caller's opportunity to adjust
 * i/o weights just before the nameless copy is submitted to the device.
 *
 * @param gctx          GC Context for state
 * @param dstate        Domain being garbage collected
 * @param nValid        How many ADUs are valid in the source block being copied
 *
 */
static void gcNotifyWaf(struct gcContext* gctx, struct SSBDomainState* dstate, uint32_t nValid)
{
    float WAF = 20.0;    // infinite WAF capped at 20

    if (nValid < gctx->superBlockCapacity)
    {
        WAF = 1.0 + ((float)nValid) / (gctx->superBlockCapacity - nValid);
    }
    gctx->counter.gcSrcWriteWAF = WAF * 1000;

    if (gctx->gcNotify)
    {
        struct GCNotify gcEvent = {
            .type = kGcCopy, .dstate = dstate, .WAF = WAF, .data = gctx->notifyData};
        uint32_t numBlocksFree = SSBGetNumAvailable(gctx->ctxt, dstate);

        // remove one for GC to use as a destination.
        gcEvent.numBlocksFree = numBlocksFree ? numBlocksFree - 1 : 0;
        gctx->gcNotify(&gcEvent);
    }
}

/**
 * @brief Builds nameless copy iocb
 *
 * Builds a nameless copy IOCB.  It's required the source block be locked for
 * read before calling this function to ensure the block is not released before
 * the copy can be submitted.  The bitmap and ACR are allocated on the end of
 * the IOCB and will be freed when the iocb is freed.
 *
 * @param gctx             GC Context for state (validMap)
 * @param dstate           Domain to GC
 * @param pfa              Parsed flash address of the source block
 * @param adusToCopy       Number of ADUs to copy
 *
 * @returns                NLC iocb or NULL if source is empty
 */
static struct SEFNamelessCopyIOCB* gcCreateCopyIocb(struct gcContext* gctx,
                                                    struct SSBDomainState* dstate,
                                                    struct SSBParsedFLA* pfa,
                                                    uint32_t adusToCopy)
{
    struct SEFAddressChangeRequest* addressChangeInfo;
    struct SEFNamelessCopyIOCB* iocb = NULL;
    FTLContext* ctxt = gctx->ctxt;
    struct SEFStatus status;
    uint64_t* bitmap;
    size_t acrSize = struct_size(addressChangeInfo, addressUpdate, adusToCopy);
    size_t bmElems = DIV_ROUND_UP(ctxt->superBlockCapacity, 64);
    size_t bmSize = bmElems * sizeof(*bitmap);

    // allocate and initialize IOCB
    iocb = SUzalloc(sizeof(*iocb) + bmSize + acrSize);
    bitmap = (void*)(iocb + 1);
    addressChangeInfo = (void*)(bitmap + bmElems);
    iocb->common.param1 = gctx;
    iocb->common.complete_func = GcCopyComplete;
    iocb->dstQosHandle = SSBGetQoSHandle(dstate);
    iocb->copyDestination = gctx->dstAddr;
    iocb->copySource.format = kBitmap;
    iocb->copySource.srcFlashAddress = pfa->flashAddress;
    iocb->copySource.validBitmap = bitmap;
    iocb->copySource.arraySize = bmElems;
    iocb->filter = NULL;
    iocb->overrides = gctx->overrides;
    iocb->common.flags = kSefIoFlagOverride;

    iocb->numAddressChangeRecords = adusToCopy;
    iocb->addressChangeInfo = addressChangeInfo;
    status = SSBCopyValidBits(pfa, bitmap, bmSize);
    assert(status.error == 0);    // means the lock above didn't work
    if (status.info == 0)         // no valid adus
    {
        SUfree(iocb);
        iocb = NULL;
    }
#if 0
    int i;
    int cnt = 0;
    for ( i = 0 ; i < iocb->copySource.arraySize ; i++ )
    {
        cnt += __builtin_popcount((uint32_t) iocb->copySource.validBitmap[i]);
        cnt += __builtin_popcount((uint32_t) (iocb->copySource.validBitmap[i] >> 32));
    }
    assert( cnt >= status.info );
#endif
    return iocb;
}

/**
 * @brief Submits nameless copy
 *
 * Builds and submits an i/o to copy the valid contents of the block at
 * sAddress to the block at gctx->dstAddr using a nameless copy.  The validMap is
 * used to only copy blocks that are valid at the time the IOCB is created.
 *
 * @param gctx             GC Context for state (validMap)
 * @param dstate           Domain to GC
 * @param id               Expected block id for the source block
 * @param sAddress         Source superblock flash address
 * @param numAdus          Number of ADUs to copy
 *
 * @returns     Number of blocks to be copied(ish) - may be smaller than actual
 *              If 0 is returned, no copy was issued
 */
static int gcSubmitNamelessCopy(struct gcContext* gctx,
                                struct SSBDomainState* dstate,
                                int id,
                                struct SEFFlashAddress sAddress,
                                uint32_t numAdus)
{
    struct SEFNamelessCopyIOCB* iocb = NULL;
    FTLContext* ctxt = gctx->ctxt;
    struct SSBParsedFLA pfa;
    struct SEFStatus status;
    bool markNotBusy = true;
    uint32_t adusToCopy = 0;
    uint32_t nValid = 0;
    int lid;

    status = SSBParseFlashAddress(ctxt, sAddress, &pfa);
    assert(status.error == 0);
    assert(pfa.dstate == dstate);
    assert(numAdus > 0);

    //  Add our selves as a reader to keep it from being deleted, verify id
    //  hasn't changed.
    if ((lid = SSBMarkBusy(ctxt, &pfa)) == id)
    {
        // get an upper limit estimate on how many change records, allocate buffer,
        status = SSBCopyValidBits(&pfa, NULL, 0);
        nValid = status.info;
        adusToCopy = nValid;
    }
    else
    {
        atomic_fetch_add(&gctx->counter.gcLateSBLock, 1);
        markNotBusy = !!lid;
    }
    if (adusToCopy)
    {
        iocb = gcCreateCopyIocb(gctx, dstate, &pfa, adusToCopy);
    }
    if (iocb)
    {
#ifndef CC_WFQ_ZENO
        gcNotifyWaf(gctx, dstate, nValid);
#endif
        markNotBusy = false;    // completion routine will do it
#if CC_MANUAL_NLC
        if (gctx->gcManual)
        {
            MCPCopyAsync(gctx->mcpHandle, SSBGetQoSHandle(dstate), iocb);
        }
        else
        {
            SEFNamelessCopyAsync(SSBGetQoSHandle(dstate), iocb);
        }
#else
        SEFNamelessCopyAsync(SSBGetQoSHandle(dstate), iocb);
#endif
    }
    else
    {
        adusToCopy = 0;
        LogTrace(gctx->hLog, "Nameless copy not performed for 0x%lx", sAddress.bits);
#if CC_DEBUG_COUNTERS
        atomic_fetch_add(&gctx->counter.lateEmpty, 1);
#endif
    }
    if (markNotBusy)
    {
        SSBMarkNotBusy(ctxt, &pfa);
    }
    return adusToCopy;    // Guess as to how many will be moved (it may move less)
}

static void gcProcessAddressChangeRequests(struct gcContext* gctx, const struct SEFNamelessCopyIOCB* iocb)
{
    void (*notifyFnc)(void*, struct SEFQoSNotification);
    SEFQoSHandle hQos = gctx->dstate->qosHandle;

    notifyFnc = SEFGetQoSHandleProperty(hQos, kSefPropertyQoSNotify).qosNotify;
    if (notifyFnc != NULL)
    {
        const struct SEFAddressChangeRequest* addressChangeInfo;
        struct SEFQoSDomainID qosId;

        addressChangeInfo = iocb->addressChangeInfo;
        qosId = SEFGetQoSHandleProperty(hQos, kSefPropertyQoSDomainID).qosID;
        // Create Notification
        for (uint32_t i = 0; i < addressChangeInfo->numProcessedADUs; i++)
        {
            struct SEFQoSNotification notify;

            // Common notification content
            notify.QoSDomainID = qosId;

            // Type-specific notification content
            if (addressChangeInfo->addressUpdate[i].userAddress.unformatted ==
                SEFUserAddressIgnore.unformatted)
            {
                notify.type = kUnreadableData;
                notify.unreadableFlashAddress = addressChangeInfo->addressUpdate[i].oldFlashAddress;
            }
            else
            {
                notify.type = kAddressUpdate;
                notify.changedUserAddress = addressChangeInfo->addressUpdate[i].userAddress;
                notify.oldFlashAddress = addressChangeInfo->addressUpdate[i].oldFlashAddress;
                notify.newFlashAddress = addressChangeInfo->addressUpdate[i].newFlashAddress;
            }

            notifyFnc(gctx->ctxt, notify);
        }
    }
}

/**
 * @brief Processes address change change requests post nameless copy
 *
 * Updating the LUT with the new addresses after a nameless copy is held off
 * until the destination superblock is closed.  The IOCBs are held in a list
 * and walked and processed by calling SEFProcessAddressChangeRequests()
 *
 * Once an IOCB is processed, the source super block is checked to see that
 * it's entirely invalid as it should be.  If not, its invalid count is
 * updated to ensure GC will release the block.
 *
 * @returns Number of blocks released
 */
static int gcPostProcessCopyIOCB(struct gcContext* gctx)
{
    struct SEFNamelessCopyIOCB* iocb;
    FTLContext* ctxt = gctx->ctxt;
    int numReleased = 0;

    void* node = SefSlistPopAll(&gctx->completedIOCBs);
    iocb = SefSlistNodeAs(node, struct SEFNamelessCopyIOCB, common.param1);

    while (iocb != NULL)
    {
        struct SEFNamelessCopyIOCB* next;
        struct SEFStatus ret;
        uint32_t numChanged = iocb->addressChangeInfo->numProcessedADUs;
        struct SSBParsedFLA pfa;

        if (iocb->common.status.info & kCopyConsumedSource)
        {
            numReleased++;
        }

        ret = SSBParseFlashAddress(ctxt, iocb->copySource.srcFlashAddress, &pfa);
        assert(ret.error == 0);

        gcProcessAddressChangeRequests(gctx, iocb);
        LogTrace(ctxt->logHandle, "Flags %d - 0x%lx %d/%u valid ADUs copied", iocb->common.status.info,
                 iocb->copySource.srcFlashAddress.bits, iocb->numAddressChangeRecords, numChanged);
        atomic_fetch_add(&gctx->counter.GCValidADUCopied, iocb->numAddressChangeRecords);

        node = ((struct sefSlistNode*)&iocb->common.param1)->next;
        next = SefSlistNodeAs(node, struct SEFNamelessCopyIOCB, common.param1);

        // free buffer
        free(iocb);
        iocb = next;
    }
    return numReleased;
}

static struct SEFStatus gcAllocSuperBlock(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    struct SEFAllocateOverrides overrides = {gctx->overrides.programWeight};
    struct SSBParsedFLA pfa;
    FTLContext* ctxt = gctx->ctxt;
    struct SEFStatus status;

    // allocate a destination
    status = SEFAllocateSuperBlock(dstate->qosHandle, &gctx->dstAddr,
                                   gctx->pSLC ? kForPSLCWrite : kForWrite, NULL, &overrides);
    if (status.error)
    {
        LogError(gctx->hLog, "Handle %p: allocateSuperblock returned error %d", gctx->hSef,
                 status.error);
        return status;
    }
    gctx->dstSize = status.info;
    SSBParseFlashAddress(ctxt, gctx->dstAddr, &pfa);
    SSBAllocated(ctxt, &pfa, (struct SEFPlacementID){gctx->pid}, status.info);
    LogTrace(gctx->hLog, "Block 0x%lx opened with %u writable ADUs", gctx->dstAddr.bits, gctx->dstSize);
    gctx->destClosed = false;
    return status;
}

/**
 * @ingroup SefSdkGc
 * @brief Collects the least used super blocks into a single superblock.
 *
 * Given an allocated superblock address and a size, loops through the
 * currently full super blocks, copying the lowest utilized super blocks until
 * the destination super block is full and closed.
 *
 * @retval 0    Success ; info is number of blocks freed
 */
static struct SEFStatus gcToASuperBlock(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    FTLContext* ctxt = gctx->ctxt;
    struct SEFStatus status = {0};
    int32_t leftToCopy = -1;
    struct SSBParsedFLA pfa;
    uint32_t refCnt;

    atomic_store(&gctx->copyInFlight, 1);    // 1 for us
    atomic_store(&gctx->adusCopied, 0);

    if (gctx->gcNotify)
    {
        struct GCNotify notify = {.type = kGcCycleStart, .data = gctx->notifyData, .dstate = dstate};
        gctx->gcNotify(&notify);
    }

    gctx->dstAddr = SEFAutoAllocate;
    // loop through allocated blocks for minimum valid data
    do
    {
        int id;
        struct SEFFlashAddress flashAddress = gcSelectBlockToCollect(gctx, dstate, &id);
        int numADUs;
        uint32_t block;

        if (SEFIsEqualFlashAddress(flashAddress, SEFAutoAllocate))
        {    // Nothing worth collecting
            LogInfo(gctx->hLog, "No more blocks to GC, but destination not full (%d left)", leftToCopy);
            break;
        }
        SEFParseFlashAddress(dstate->qosHandle, flashAddress, NULL, &block, NULL);
        if (SEFIsEqualFlashAddress(gctx->dstAddr, SEFAutoAllocate))
        {    // alloc sb just in time to consider estimated WAF for erase weight
            struct SSBItrInfo info;

            gctx->counter.gcActive = kGCStateAllocating;
            SSBGetInfo(ctxt, dstate, block, &info);
#if CC_WFQ_ZENO
            gcNotifyWaf(gctx, dstate, gcCalcCycleValid(gctx, dstate, info.nValid));
#else
            gcNotifyWaf(gctx, dstate, info.nValid);
#endif
            status = gcAllocSuperBlock(gctx, dstate);
            gctx->counter.gcActive = kGCStateCopying;
            if (status.error)
            {
                break;
            }
            leftToCopy = gctx->dstSize;
        }
        // Mark block as selected as a source
        assert((gctx->srcBitmap[block / 64] & ((uint64_t)1 << (block % 64))) == 0);
        gctx->srcBitmap[block / 64] |= (uint64_t)1 << (block % 64);
        atomic_fetch_add(&gctx->copyInFlight, 1);
        numADUs = gcSubmitNamelessCopy(gctx, dstate, id, flashAddress, leftToCopy);
        if (numADUs == 0)
        {
            atomic_fetch_sub(&gctx->copyInFlight, 1);
        }
        LogTrace(gctx->hLog, "copy source 0x%lx submitted with %d ADUs", flashAddress.bits, numADUs);

        if (numADUs > leftToCopy)
        {
            numADUs = leftToCopy;
        }
        leftToCopy -= numADUs;
    } while (leftToCopy);

    if (!gctx->destClosed)
    {
        // If an LBA in source is rewritten during the copy, the destination
        // block may not be filled - issue a flush so all copies will complete
        SEFFlushSuperBlock(gctx->dstate->qosHandle, gctx->dstAddr, NULL);
    }
    if (atomic_fetch_sub(&gctx->copyInFlight, 1) == 1)
    {
        SefEventSet(&gctx->copyComplete);
    }
    SefEventWait(&gctx->copyComplete);

    if (gctx->gcNotify)
    {
        struct GCNotify notify = {
            .type = kGcCycleEnd, .data = gctx->notifyData, .dstate = dstate, .adusUsed = gctx->dstSize};
        gctx->gcNotify(&notify);
    }

    int adusCopied = atomic_load(&gctx->adusCopied);
    if (adusCopied == 0)
    {    // close block - will delete when reference removed
        LogTrace(gctx->hLog, "Nothing copied for Block 0x%lx, closing", gctx->dstAddr.bits);
        SEFCloseSuperBlock(dstate->qosHandle, gctx->dstAddr);
    }
    else
    {
        if (!gctx->destClosed)
        {
            LogTrace(gctx->hLog, "Block 0x%lx was only partially filled (%d), forcing closed",
                     gctx->dstAddr.bits, adusCopied);
            SEFCloseSuperBlock(dstate->qosHandle, gctx->dstAddr);
            atomic_fetch_add(&gctx->counter.partialBlockCopy, 1);
            atomic_fetch_add(&gctx->counter.skippedADU, leftToCopy);
        }
    }

    SSBAddAdusUsed(dstate, adusCopied);
    LogTrace(gctx->hLog, "Block 0x%lx closed with %d valid", gctx->dstAddr.bits, adusCopied);

    // Now we'll update LUT and count how much is valid
    status.error = 0;
    status.info = gcPostProcessCopyIOCB(gctx);

    // Release the reference from gcAllocSuperBlock/SSBAllocated
    SSBParseFlashAddress(ctxt, gctx->dstAddr, &pfa);
    refCnt = pfa.bstate->vData->refCnt;
    SSBRemoveRef(ctxt, &pfa);
    LogTrace(gctx->hLog, "Block 0x%lx is in state 0x%x ref count %u", gctx->dstAddr.bits,
             pfa.bstate->state, refCnt - 1);

    return status;
}

bool GCShouldRun(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    if (gctx == NULL)
    {
        return false;
    }
    return (atomic_load(&dstate->adusUsed) > gctx->gcAduTrigger);
}

static bool gcWearLevelRequired(struct gcContext* gctx)
{
    if (gctx->pid == -1)
    {
        return false;
    }
    if (gctx->pidCnts[gctx->pid].size == 0)
    {
        return false;
    }
    return (gctx->pidCnts[gctx->pid].sbList[0].gcRank == 0);
}

/**
 * @ingroup SefSdkGc
 * @brief   Setup/reset GC state to start a collection cycle
 *
 * Clears the bitmap used to track what was used as a source block.
 * Builds a short list of candidate source blocks and selects PID to collect
 *
 * @param gctx      State variables for this instance of GC
 * @param dstate    Domain to collect
 * @return true     If collection is required
 * @return false    Nothing to collect/isn't required
 */
static bool gcSetupCollection(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    memset(gctx->srcBitmap, 0, gctx->srcBitmapSize);
    gctx->pid = -1;
    gcBuildListOfCandidates(gctx, dstate);
    gctx->pid = gcSelectPlacementIdToCollect(&gctx->qosInfo, gctx->pidCnts);
    if (gctx->pid == -1)
    {
        return false;
    }

#if CC_GC_SINGLE_PLID
    if (gctx->pidCnts[gctx->pid].numInvalid < gctx->superBlockCapacity)
    {
        return gcWearLevelRequired(gctx);
    }
#else
    if (gcNumInvalid(&gctx->qosInfo, gctx->pidCnts) < gctx->superBlockCapacity)
    {
        return gcWearLevelRequired(gctx);
    }
#endif

    return gcWearLevelRequired(gctx) || GCShouldRun(gctx, dstate);
}

/**
 * @ingroup    SefSdkGc
 * @brief      garbage collects a QoSDomain
 *
 * Loops collecting the lowest utilized blocks together until the number of
 * used blocks for the domain is below the highWater mark.  It then releases
 * any waiting writers.
 */
static int gcQoSDomain(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    FTLContext* ctxt = gctx->ctxt;
    int loops = 0;
    int progress = atomic_load(&ctxt->ssbState->releasedBlockCount);

    LogTrace(gctx->hLog, "Handle %p: Starting GC scan domain %d...", gctx->hSef, dstate->domainId.id);
    LogTrace(gctx->hLog, "Handle %p: %" PRId64 " ADUs used, blocks: %" PRIu64 " in use, %d open",
             gctx->hSef, dstate->adusUsed, dstate->blocksUsed, ctxt->ssbState->openBlocks);

    dstate->inGC = true;

    while (!atomic_load(&gctx->gcStop))
    {
        struct SEFStatus ret;

        gctx->counter.gcActive = kGCStateSelecting;
        if (!gcSetupCollection(gctx, dstate))
        {
            LogTrace(gctx->hLog, "Nothing worth collecting");
            break;
        }
        if (gcWearLevelRequired(gctx))
        {
            LogTrace(gctx->hLog, "Block maintenance required");
            loops = 0;
        }
        LogTrace(gctx->hLog, "New garbage collect!");
#if CC_MANUAL_NLC
        // lock method at per super block (can't copy & write to a super block)
        gctx->gcManual = ctxt->gcManual;
        gctx->mcpHandle = ctxt->mcpHandle;
#endif
        ret = gcToASuperBlock(gctx, dstate);

        SSBFlushQdEmptyBlocks(dstate);    // wait for empty blocks to be released

        assert(ret.error == 0);
        if (ret.info > 1)    // gc is making progress if it released more than 1
        {
            loops = 0;
        }
        loops++;
        if (loops > gctx->counter.gcMaxLoops)
        {
            gctx->counter.gcMaxLoops = loops;
        }
        assert(loops < gctx->superBlockCapacity);
    }
    dstate->inGC = false;
    gctx->counter.gcActive = kGCStateReleasing;
    LogTrace(gctx->hLog, "Handle %p: GC domain %d complete", gctx->hSef, dstate->domainId.id);
    LogTrace(gctx->hLog, "Handle %p: %u in use %" PRIu64 " open", gctx->hSef, dstate->blocksUsed,
             ctxt->ssbState->openBlocks);

    return (atomic_load(&ctxt->ssbState->releasedBlockCount) - progress);
}

static void gcQoSDomainPatrol(struct gcContext* gctx, struct SSBDomainState* dState)
{
    int checkInfoSize, i, numSuperBlocks = 0;
    struct SEFStatus status;
    struct SEFCheckInfo* checkInfo = NULL;

    // mark gc as patrolling
    gctx->counter.gcActive = kGCStatePatrol;
    // reset the flag
    SSBSetPatrol(dState, false);

    do
    {
        if (checkInfo != NULL)
        {
            numSuperBlocks = checkInfo->numSuperBlocks;
            SUfree(checkInfo);
            checkInfo = NULL;
        }

        // get number superblocks that need patrol
        checkInfoSize =
            sizeof(struct SEFCheckInfo) + sizeof(struct SEFSuperBlockRecord) * numSuperBlocks;
        checkInfo = (struct SEFCheckInfo*)SUzalloc(checkInfoSize);
        status = SEFGetCheckList(dState->qosHandle, checkInfo, checkInfoSize);
        if (status.error)
        {
            LogError(gctx->hLog, "was unable to get superblock check list");
            SUfree(checkInfo);
            return;
        }

    } while (checkInfo->numSuperBlocks != numSuperBlocks);    // ensures no new superblock was added

    // perform checkPage for super blocks
    LogInfo(gctx->hLog, "%d Super blocks need patrol", checkInfo->numSuperBlocks);
    for (i = 0; i < checkInfo->numSuperBlocks; i++)
    {
        LogDebug(gctx->hLog, "Performing check page for superblock 0x%lx",
                 checkInfo->superBlockRecords[i].flashAddress.bits);
        status = SEFCheckSuperBlock(dState->qosHandle, checkInfo->superBlockRecords[i].flashAddress);
        switch (status.error)
        {
            case -EBADF:
                SSBMarkForMaintenance(gctx->ctxt, checkInfo->superBlockRecords[i].flashAddress);
                break;

            default:
                break;
        }
    }

    // free local variables
    SUfree(checkInfo);
}

/**
 * @ingroup    SefSdkGc
 * @brief       Processes a domain for GC or Patrol
 *
 * Calls gcQoSDomain() and gcQoSDomainPatrol() if required.  Detects if the
 * super blocks become overcommitted and shuts down GC.
 *
 * @param gctx      State variables for this instance of GC
 * @param dstate    The domain to process for gc or patrol
 */
static void gcProcessDomain(struct gcContext* gctx, struct SSBDomainState* dstate)
{
    FTLContext* ctxt = gctx->ctxt;

    if (!atomic_load(&gctx->gcStop) && gcQoSDomain(gctx, dstate) == 0)
    {
        // GC made no progress.  We can wait for empty blocks to release but we
        // can't wait for i/o to complete.  If we're still out of blocks, sleep
        // to wait for i/o, try again and see if progress can be made again.  If
        // not, we're overcommitted on super blocks (writes have consumed the
        // blocks reserved for GC).
        SSBFlushQdEmptyBlocks(dstate);
        if (atomic_load(&dstate->adusUsed) > dstate->maxAdus)
        {
            LogTrace(gctx->hLog,
                     "Domain %d at high water with nothing to collect, sleeping and retrying",
                     dstate->domainId.id);
            gctx->counter.gcNoProgress++;
            sleep(1);
            gctx->counter.gcActive = kGCStateReleasing;
            uint64_t progress = atomic_load(&ctxt->ssbState->releasedBlockCount);
            gcQoSDomain(gctx, dstate);
            SSBFlushQdEmptyBlocks(dstate);
            progress = atomic_load(&ctxt->ssbState->releasedBlockCount) - progress;
            if (!atomic_load(&gctx->gcStop) && !progress &&
                atomic_load(&dstate->adusUsed) > dstate->maxAdus)
            {
                LogTrace(gctx->hLog, "No progress GC shutting down");
                GCSetFatalError(gctx, SUMakeStatusError(-ENOSPC));
                return;
            }
        }
    }

    if (SSBGetPatrol(dstate))
    {
        gcQoSDomainPatrol(gctx, dstate);
    }
}

static void gcProcessDomainList(struct gcContext* gctx, struct sefSlistNode* list)
{
    struct SSBDomainState* dstate;

    dstate = SefSlistNodeAs(list, struct SSBDomainState, link);
    while (dstate && !atomic_load(&gctx->gcStop))
    {
        struct SSBDomainState* next;

        next = SefSlistNextAs(&dstate->link, struct SSBDomainState, link);
        dstate->link.next = NULL;
        atomic_store(&dstate->isQueued, 0);
        gcProcessDomain(gctx, dstate);
        dstate = next;
    }
}

void GcSuspend(struct gcContext* gctx)
{
    if (atomic_fetch_add(&gctx->suspend_count, 1) == 0)
    {
        SefEventReset(&gctx->canRun);
    }
}

void GcResume(struct gcContext* gctx)
{
    if (atomic_fetch_sub(&gctx->suspend_count, 1) == 1)
    {
        SefEventSet(&gctx->canRun);
    }
}

static void* GcThread(void* arg)
{
    struct gcContext* gctx = arg;

    int ret;

    while (!atomic_load(&gctx->gcStop))
    {
        if ((ret = SefEventWait(&gctx->gcEvent)) != 0)
        {
            LogError(gctx->hLog, "GC event failed to wait, exiting GC thread");
            GCSetFatalError(gctx, SUMakeStatusError(ret));
            break;
        }
        SefEventWait(&gctx->canRun);
        atomic_fetch_add(&gctx->counter.GCRunCount, 1);
        gctx->counter.gcActive = kGCStateReleasing;
        gcProcessDomainList(gctx, SefSlistReverse(SefSlistPopAll(&gctx->gcQueue)));
        gctx->counter.gcActive = kGCStateIdle;
    }
    gctx->counter.gcActive = kGCStateDown;
    if (gctx->gcNotify)
    {
        struct GCNotify notify = {
            .type = kGcNotifyDown, .status = gctx->status, .data = gctx->notifyData};
        gctx->gcNotify(&notify);
    }
    return NULL;
}

void GCSetFatalError(struct gcContext* gctx, struct SEFStatus status)
{
    if (!atomic_load(&gctx->gcStop))
    {
        gctx->status = status;
        atomic_store(&gctx->gcStop, true);
    }
}

enum GCActiveStates GCActiveState(struct gcContext* gctx)
{
    if (gctx)
    {
        return gctx->counter.gcActive;
    }
    return kGCStateDown;
}

// returns true if GC was triggered to run
bool GCTrigger(struct gcContext* gctx)
{
    // because gc thread is used to get info on open blocks, which are closed
    // on shutdown, this is called before gc thread is up and after it's down
    if (!gctx)
    {
        return false;
    }

    bool set = SefEventIsSet(&gctx->gcEvent);
    if (SefEventSet(&gctx->gcEvent))
    {
        assert(gctx->gcStarted == true);    // Should know if GC is running or not
                                            // not so good to trigger just-in-case
        LogTrace(gctx->hLog, "GC trigger failed - likely shutdown %d/%d", gctx->gcStarted,
                 atomic_load(&gctx->gcStop));
    }
    return !set;
}

bool GCTriggerDomain(struct gcContext* gctx, struct SSBDomainState* domain)
{
    bool triggered = false;

    if (!gctx)
    {
        return false;
    }
    if (atomic_exchange(&domain->isQueued, 1) == 0)
    {
        assert(domain->link.next == NULL);
        if (SefSlistPush(&gctx->gcQueue, &domain->link))
        {
            triggered = GCTrigger(gctx);
        }
    }
    return triggered;
}

static void gcAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    struct gcContext* gctx = arg;
    FTLContext* ctxt = gctx->ctxt;

    if (strcmp(verb, "help") == 0)
    {
#if CC_MANUAL_NLC
        snprintf(out, size,
                 "* gc [low water]|[sef|manual]\n"
                 "     [low water] : status/configure garbage collect - triggers GC\n"
                 "     sef|manual  : switch from SEF to manual to GC blocks\n");
#else
        snprintf(out, size, "* gc [low water] : status/configure garbage collect - triggers GC\n");
#endif
        return;
    }
    char* arg1 = strtok_r(NULL, " \t", savePtr) ?: "";
    // char* end = NULL;
    int i;

    if (arg1)
    {
#if CC_MANUAL_NLC
        if (strcasecmp("sef", arg1) == 0)
        {
            ctxt->gcManual = false;
            snprintf(out, size,
                     "SEF nameless copy will be used for the collection of a source block ADUs\n");
            return;
        }
        if (strcasecmp("manual", arg1) == 0)
        {
            ctxt->gcManual = true;
            snprintf(out, size,
                     "Manual copy will be used for the collection of a source block ADUs\n");
            return;
        }
#endif

        char* end = NULL;
        int64_t lowWater = strtol(arg1, &end, 10);
        if (end == NULL || lowWater > ctxt->ssbState->dstate->maxAdus ||
            lowWater < ctxt->sftState->lutSize)
        {
            snprintf(out, size,
                     "Bad low water argument '%s'\n"
                     "Valid range is %" PRId64 " through %" PRId64 "\n",
                     arg1, ctxt->sftState->lutSize, ctxt->ssbState->dstate->maxAdus);
        }
        else
        {
            ssize_t used = 0;
            gctx->gcAduTrigger = lowWater;
#if CC_MANUAL_NLC
            // used = snprintf(out, size, "HighWater: %d lowWater: %d using %s\n", ctxt->highWater, ctxt->lowWater,
            //                 ctxt->gcManual ? "manual copy" : "sef");
#else
            used = snprintf(out, size,
                            "Triggered with adus used: %" PRIu64 ", highWater: %" PRIu64
                            " lowWater: %" PRIu64 "\n",
                            ctxt->ssbState->dstate->adusUsed, ctxt->ssbState->dstate->maxAdus,
                            gctx->gcAduTrigger);
#endif
            for (i = 0; i < ctxt->ssbState->num_qos; i++)
            {
                if (used >= size)
                {
                    break;
                }
                used += snprintf(
                    out + used, size - used, "Triggering domain %d with blocks used: %" PRIu64 "\n",
                    ctxt->ssbState->dstate[i].domainId.id, ctxt->ssbState->dstate[i].blocksUsed);
                GCTriggerDomain(gctx, ctxt->ssbState->dstate + i);
            }
        }
    }
    else
    {
        ssize_t used = 0;
#if CC_MANUAL_NLC
        // used = snprintf(out, size, "HighWater: %d lowWater: %d using %s copy\n", ctxt->highWater, ctxt->lowWater,
        //                 ctxt->gcManual ? "manual" : "sef");
#else
        used = snprintf(
            out, size, "Adus used: %" PRIu64 ", highWater: %" PRIu64 " lowWater: %" PRIu64 "\n",
            ctxt->ssbState->dstate->adusUsed, ctxt->ssbState->dstate->maxAdus, gctx->gcAduTrigger);
#endif
        for (i = 0; i < ctxt->ssbState->num_qos; i++)
        {
            if (used >= size)
            {
                break;
            }
            used += snprintf(out + used, size - used, "Domain %d with blocks used: %" PRIu64 "\n",
                             ctxt->ssbState->dstate[i].domainId.id,
                             ctxt->ssbState->dstate[i].blocksUsed);
        }
    }
}

static void gcRegisterCounters(struct gcContext* gctx)
{
    struct INSCounter counter[] = {
        InstructionCounterDef(gcActive, "GC state (0, idle)", GCCounter, &gctx->counter),
        InstructionCounterDef(gcSrcWriteWAF, "GC source block WAF * 1000", GCCounter, &gctx->counter),
        InstructionCounterDef(gcMaxLoops, "Maximum GC loops to release a super block", GCCounter,
                              &gctx->counter),
    };

    gctx->stateCntId = INSRegisterStateCounters(gctx->hInst, counter, NELEM(counter), NULL, NULL, NULL);

    struct INSCounter ioCounter[] = {
        InstructionCounterDef(GCRunCount, "Number of times GC ran", GCCounter, &gctx->counter),
        InstructionCounterDef(GCValidADUCopied, "Number of ADUs copied as a result of GC",
                              GCCounter, &gctx->counter),
        InstructionCounterDef(namelessCopyCount, "Number of nameless copies", GCCounter, &gctx->counter),
        InstructionCounterDef(skippedADU, "ADUs skipped in a partial block", GCCounter, &gctx->counter),
        InstructionCounterDef(partialBlockCopy, "Partial GC blocks", GCCounter, &gctx->counter),
        InstructionCounterDef(gcLateSBLock,
                              "Enumerated block's instance id changed before it could be locked",
                              GCCounter, &gctx->counter),
        InstructionCounterDef(gcNoProgress, "GC has made no progress while I/O is blocked",
                              GCCounter, &gctx->counter),
#if CC_DEBUG_COUNTERS
        InstructionCounterDef(lateEmpty, "Block became empty after GC selection", GCCounter,
                              &gctx->counter),
#endif
    };

    gctx->ioCntId = INSRegisterIoCounters(gctx->hInst, ioCounter, NELEM(ioCounter), NULL, NULL);
}

struct gcContext* GCInit(struct GCConfig* config)
{
    struct gcContext* gctx = SUzalloc(sizeof(*gctx));
    struct SEFStatus status;

    status = GcContextInit(gctx, config);
    if (status.error)
    {
        struct GCNotify notify = {.type = kGcNotifyDown, .status = status, .data = config->notifyData};
        if (config->notify)
        {
            config->notify(&notify);
        }
        SUfree(gctx);
        return NULL;
    }
    gctx->gcInit = false;
    gctx->gcStarted = true;    // Assume it will start
    atomic_store(&gctx->gcStop, false);

    // Startup GC thread, init will complete in there
    if (pthread_create(&gctx->FTLThread, NULL, GcThread, gctx))
    {
        struct SEFStatus status = {0};

        LogFatal(gctx->hLog, "Error creating FTL thread (%d) - %s", errno, strerror(errno));
        status.error = -errno;
        gctx->gcStarted = false;
        if (config->notify)
        {
            struct GCNotify notify = {
                .type = kGcNotifyDown, .status = status, .data = config->notifyData};
            config->notify(&notify);
        }
    }
    else
    {
        // register inst action
        INSRegisterAction(gctx->hInst, "gc", gcAction, gctx);
        LogDebug(gctx->hLog, "garbage collection thread was init");
        gcRegisterCounters(gctx);
    }

#if CC_MANUAL_NLC
    MCPInit(&gctx->ctxt->mcpHandle, gctx->hInst, gctx->hLog);
    gctx->mcpHandle = gctx->ctxt->mcpHandle;
#endif
    gctx->gcInit = true;
    return gctx;
}

void GCCleanup(struct gcContext* gctx)
{
    if (!gctx->gcInit)
    {
        return;
    }

    INSUnRegisterAction(gctx->hInst, "gc");
    INSUnRegisterIoCounters(gctx->hInst, gctx->ioCntId);
    INSUnRegisterStateCounters(gctx->hInst, gctx->stateCntId);

    if (gctx->gcStarted)
    {
        int ret;

        atomic_store(&gctx->gcStop, true);
        while (gctx->suspend_count > 0)
        {
            GcResume(gctx);
        }
        LogTrace(gctx->hLog, "Kicking off GC to shutdown");
        GCTrigger(gctx);
        if ((ret = pthread_join(gctx->FTLThread, NULL)))
        {
            LogError(gctx->hLog, "Unable to join thread while shutting down GC; (%d) - %s", ret,
                     strerror(ret));
        }
        gctx->gcStarted = false;
        LogTrace(gctx->hLog, "GC thread shut down");
    }
    gctx->gcNotify = NULL;

#if CC_MANUAL_NLC
    MCPCleanup(gctx->mcpHandle);
#endif

    GcContextCleanup(gctx);
    SUfree(gctx);
}
