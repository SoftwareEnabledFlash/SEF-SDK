/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-block-module.c
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
#include "sef-block-module.h"

#include <SEFAPI.h>
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "data-tree-object.h"
#include "flash-translation.h"
#include "garbage-collection.h"
#include "log-manager.h"
#include "persistence.h"
#include "sef-notification.h"
#include "sef-utils.h"
#include "superblock.h"
#include "utils/dlist.h"
#include "utils/file-logger.h"
#include "utils/instrumentation.h"
#include "utils/iov-helper.h"

#ifndef SEFSDKVersion
#define SEFSDKVersion "unavailable"
#endif

static __attribute((used)) char SdkVersion[] = "@(#) SEF_SDK version " SEFSDKVersion;

#define SBM_BLOCK_META_PERSISTENCE_KEY "BlockMeta"


struct SefBlockWriteIocb
{
    struct SEFWriteWithoutPhysicalAddressIOCB iocb;
    struct timespec start;    // time the i/o spends in the sef library
    bool canceled;
    bool completed;
    int32_t numLeft;    // #of of ADUs left to write
    struct SSBDomainState *dstate;
    struct iovec *iov;
    TmaDListEntry link;
    struct SEFFlashAddress tentative[];
    // uint64_t metadata[2][];  // metadata is after the tentative array
};

struct SefBlockReadIocb
{
    struct SEFReadWithPhysicalAddressIOCB iocb;
    struct timespec start;    // time the i/o spends in the sef library
};

// given a WAF estimate, adjust GC to keep up with writes
//
// To have gc rate equal to the estimated WAF
//   new program weight = program weight * waf
//   nlc weight = program weight * (waf-1)/waf * cost of NLW/cost of NLC
//
// todo: For now C_c assumed the same as C_w ignoring the read portion of NLC
//       which gives copy a slight advantage. UPDATE: in practice, NLC doesn't
//       always keep up - empirically derived cost_nw adjusts bias against
//       NLW.
//
// todo: When WAF is < 2, the trigger point needs to be at a point such that
//       there are enough free blocks to cover the slowed down gc.  For example,
//       when blocks are 1/4 full (WAF of 1.25), a cycle will release 3 blocks
//       so there needs to be that many free before the cycle starts.
//
static void sbmAdjustIOWeights(FTLContext *ctxt, float WAF_f, uint32_t numBlocksFree)
{
    struct SEFCopyOverrides copyOverride;
    float program;
    float copy;
#if CC_WFQ_ZENO
    // Use WAF based on current conditions
    float cw_ratio = WAF_f;
    float cost_nw = 0.8;
#else
    // Use a WAF calculated based on OP
    float cw_ratio = ctxt->blockData.Overprovisioning ? 100.0 / ctxt->blockData.Overprovisioning : 20.0;
    float cost_nw = ctxt->pSLC ? 1.0 / 8.0      // May still block writes
                               : 1.0 / 3.25;    // Write-only drops to 18% but blocks on 1/3
#endif

    // GC runs at a rate based on how many blocks will be freed in a single
    // cycle so it won't be overtaken by writes.
    program = ctxt->ioWeights.programWeight * cw_ratio;
    copy = ctxt->ioWeights.programWeight * cost_nw * (cw_ratio - 1) / cw_ratio;

#if CC_WFQ_ZENO
    // Weights adjust so write can consume half of remaining OP while GC runs.
    if (numBlocksFree > 2)    // hard coded min op
    {
        // todo: Pn+Cn should equal programWeight to stay balanced with read
        copy *= numBlocksFree / 2;    // retard copy for 1/2 of what's left
    }
#endif
    assert(program >= 0);
    assert(copy >= 0);
    ctxt->writeOverride.programWeight = program < 65535.5 ? program : 65535;
    ctxt->allocOverride.eraseWeight = ctxt->writeOverride.programWeight;
    copyOverride.programWeight = copy < 65535.5 ? copy : 65535;
    GCSetCopyIOWeights(ctxt->gctx, &copyOverride);

    // copy to counters
    ctxt->ioState.programWeight = ctxt->writeOverride.programWeight;
    ctxt->ioState.nlcProgramWeight = copyOverride.programWeight;
    // printf("W:%f w:%f p:%f c:%f r:%f\n", WAF_f, cw_ratio, program, copy, program/copy);
}

/**
 * @brief Completes a list of SefBlockWriteIocb requests as canceled
 *
 * The iocb's are removed from ioList before being completed
 *
 * @param ioList    List of SefBlockWriteIocb's to cancel
 */
static void sbmCancelIoList(TmaDList *ioList)
{
    struct SefBlockWriteIocb *node = NULL;
    while ((node = utl_DListPopHeadAs(ioList, struct SefBlockWriteIocb, link)))
    {
        node->iocb.common.status.error = -ECANCELED;
        node->iocb.common.complete_func(&node->iocb.common);
    }
}

/**
 * @brief Cancels all waiting i/o for a domain
 */
static void sbmCancelDomainIo(struct SSBDomainState *dstate)
{
    UTL_DECLARE_DLIST(ioList);
    pthread_mutex_lock(&dstate->fsLock);
    {
        utl_DListAppendList(&ioList, &dstate->fsWaiters);
    }
    pthread_mutex_unlock(&dstate->fsLock);
    sbmCancelIoList(&ioList);
}

/**
 *  @ingroup    SefSdkApi
 *  @brief Cancels all I/O in any queues.
 *
 *  @param ctxt      FTL context to operate on
 */
static void sbmCancelAllIo(FTLContext *ctxt)
{
    int i;
    for (i = 0; i < ctxt->ssbState->num_qos; i++)
    {
        sbmCancelDomainIo(&ctxt->ssbState->dstate[i]);
    }
}

static void sbmGCNotify(struct GCNotify *event)
{
    FTLContext *ctxt = event->data;

    switch (event->type)
    {
        case kGcNotifyDown:
            if (event->status.error)
            {
                LogError(ctxt->logHandle, "GC is down with status %d, disabling i/o",
                         event->status.error);
                if (ctxt->notifyFunc)
                {
                    struct SEFBlockNotify notify = {.type = kSefBlockNotifyGCDown};
                    ctxt->notifyFunc(notify, ctxt->notifyContext);
                }
                ctxt->disabled = true;
                sbmCancelAllIo(ctxt);
            }
            break;
        case kGcCopy:
            sbmAdjustIOWeights(ctxt, event->WAF, event->numBlocksFree);
            break;
        case kGcCycleStart:
            break;
        case kGcCycleEnd:
            // restore default weights for read and write
            ctxt->ioState.eraseWeight = ctxt->allocOverride.eraseWeight = ctxt->ioWeights.eraseWeight;
            ctxt->ioState.programWeight = ctxt->writeOverride.programWeight =
                ctxt->ioWeights.programWeight;
            ctxt->ioState.readWeight = ctxt->readOverride.readWeight = ctxt->readWeight;
            LogTrace(ctxt->logHandle, "GC cycle end adding %u to %" PRId64 " adus used", event->adusUsed,
                     atomic_load(&((struct SSBDomainState *)event->dstate)->adusUsed));
            break;
        default:
            LogTrace(ctxt->logHandle, "Unhandled GC event %d", event->type);
            break;
    }
}

static FTLContext *sbmAllocContext(uint16_t unitIndex, struct SEFQoSDomainID qosId)
{
    FTLContext *ctxt = SUzalloc(sizeof(*ctxt));

    ctxt->sefUnitIndex = unitIndex;
    ctxt->qosId = qosId;
    sefGatewayInit(&ctxt->ioGateway);
    return ctxt;
}

static void sbmQueueToWriteBlockMetaComp(struct PDLData *pdlData, int wasFlushed)
{
    SUfree(pdlData->Obj);
}

static struct SEFStatus sbmQueueToWriteBlockMeta(PDLHandle pdlHandle, struct SEFBlockMeta *blockData)
{
    struct PDLData pdlData;
    struct SEFBlockMeta *storedMetaData;

    // store data as little Endian
    storedMetaData = SUmalloc(sizeof(struct SEFBlockMeta));
    storedMetaData->MaxLBA = htole64(blockData->MaxLBA);
    storedMetaData->Wsn = htole64(blockData->Wsn);
    storedMetaData->Overprovisioning = htole32(blockData->Overprovisioning);
    storedMetaData->Rsvd = 0;
    storedMetaData->MaxAdus = htole64(blockData->MaxAdus);

    // populate persistance object
    strcpy(pdlData.Key.Name, SBM_BLOCK_META_PERSISTENCE_KEY);
    pdlData.Key.Index = 0;
    pdlData.EncodingType = kPDLNoEncoding;
    pdlData.Obj = storedMetaData;
    pdlData.ObjSize = sizeof(struct SEFBlockMeta);
    pdlData.FlushCompleteFunc = &sbmQueueToWriteBlockMetaComp;

    // queue data for persistence
    return PDLQueueData(pdlHandle, &pdlData);
}

static uint64_t sbmMetaReserve(const struct SEFInfo *sefInfo,
                               struct SEFQoSDomainInfo *qdInfo,
                               int64_t capacity)
{
    uint64_t sbStateSize = SBSStateSize(sefInfo, qdInfo, capacity);
    uint64_t lutSize = capacity * sizeof(uint64_t);
    uint64_t totalADUNeeded = 0;

    totalADUNeeded += dtoSizeOf(lutSize, qdInfo->ADUsize);
    totalADUNeeded += dtoSizeOf(sbStateSize, qdInfo->ADUsize);
    totalADUNeeded += 1;    // Block config data
    totalADUNeeded += 1;    // persistence overhead (directory & config)

    return totalADUNeeded;
}

// GC collects into 1, but when collecting two smaller blocks into a larger
// block, used ADUs will go up as a result of GC so an extra block is
// reserved to ensure the domain does not go over quota.  This is an issue
// when small blocks are mostly full and it takes many loops in GC to empty
// two blocks during one GC cycle.
#define NUM_GC_RESERVE                               \
    2 /*< Number of super blocks GC has allocated at \
          the same time.  Affects how many ADUs are  \
          reserved for GC */

static struct SEFStatus sbmCalculateDeviceCapacity(FTLContext *ctxt,
                                                   int Overprovisioning,
                                                   int numQosDomains,
                                                   struct SEFBlockMeta *blockData)
{
    struct SEFQoSDomainInfo *qosInfo = &ctxt->qosInfo;
    bool pSLC = (qosInfo->flashQuota < qosInfo->pSLCFlashQuota);
    uint64_t totalNumADUs = pSLC ? qosInfo->pSLCFlashQuota : qosInfo->flashQuota;
    uint64_t superblockCapacity = pSLC ? qosInfo->pSLCSuperBlockCapacity : qosInfo->superBlockCapacity;
    const struct SEFInfo *sefInfo = ctxt->sefInfo;
    LogHandle logHandle = ctxt->logHandle;
    int OP = Overprovisioning;
    uint64_t numStreams;     // number of write streams above high water
    uint64_t userAdus;       // Adus for user and meta data
    uint64_t metaReserve;    // Adus reserved for persistence
    uint64_t maxAdus;        // Adus for data (includes OP)
    uint64_t maxLba;         // Adus for data w/o OP
    uint64_t minLba;         // Minimum value of maxLba;
    uint64_t minAdus;        // Minimum # of Adus required in a QoS Domain
    uint64_t opADUs;         // OP in ADUs
    uint64_t minOpADUs;      // Min req op
    uint64_t minMeta;

    numStreams = qosInfo->numPlacementIDs + NUM_GC_RESERVE;
    // One superblock gap between maxAdus and maxLba to allow some writes to make
    // a superblock's worth of invalid entries before GC is triggered.  Plus one
    // since space is allocated at open of the super block with few ADUs marked
    // as invalid.
    minOpADUs = 2 * superblockCapacity;

#if CC_GC_SINGLE_PLID
    // When an LBA changes placement id, it uses space in the dest w/o freeing
    // space in the destination.  Need an extra SB per placement id so someone
    // will have a super block's worth of invalid ADUs.
    minOpADUs += qosInfo->numPlacementIDs * superblockCapacity;
#endif
    minLba = superblockCapacity * qosInfo->numPlacementIDs;
    minMeta = DIV_ROUND_UP(sbmMetaReserve(sefInfo, qosInfo, minLba), superblockCapacity);
    // adding 1 extra SB to account for overestimation of metaReserve size below
    minAdus = minLba + MAX(minOpADUs, minLba * 100 / (100 - OP)) +
              superblockCapacity * (minMeta + numStreams + 1);
    if (totalNumADUs < minAdus)
    {
        LogFatal(logHandle,
                 "Domain too small, only %" PRId64 " ADUs (s), minimum %" PRId64 " required\n",
                 totalNumADUs, minAdus);
        return SUMakeStatus(-ENOSPC, minAdus);
    }
    userAdus = totalNumADUs - superblockCapacity * numStreams;
    // Estimate numLba's with userAdus, which is larger.
    metaReserve = sbmMetaReserve(sefInfo, qosInfo, userAdus - userAdus * OP / 100);
    maxAdus = userAdus - DIV_ROUND_UP(metaReserve, superblockCapacity) * superblockCapacity;
    opADUs = maxAdus * OP / 100;
    if (opADUs < minOpADUs)
    {
        LogInfo(logHandle, "OP set to minimum value of %" PRIu64, minOpADUs);
        opADUs = minOpADUs;
    }
    maxLba = maxAdus - opADUs;
    LogInfo(logHandle, "Superblock size = %" PRIu64 " ADUs", superblockCapacity);
    LogInfo(logHandle, "Capacity reserved for OP %" PRIu64 " ADUs", opADUs);
    LogInfo(logHandle, "Capacity reserved for metadata %" PRId64 " ADUs", metaReserve);

    blockData->MaxAdus = maxAdus;
    LogInfo(logHandle, "High water mark %" PRId64, maxAdus);

    blockData->MaxLBA = maxLba;
    blockData->Overprovisioning = Overprovisioning;
    blockData->Wsn = 0;

    blockData->MaxLBA *= numQosDomains;

    return SUMakeStatusOk();
}

static void sbmInitLogging(FTLContext *ctxt, struct SEFBlockOption *options)
{
    ctxt->logHandle = options->logHandle;
    if (ctxt->logHandle == NULL)
    {
        int logPathLen = 1 + snprintf(NULL, 0, Block_Default_Log, ctxt->sefUnitIndex, ctxt->qosId.id);
        char filePath[logPathLen];

        snprintf(filePath, logPathLen, Block_Default_Log, ctxt->sefUnitIndex, ctxt->qosId.id);
        DFLInit(&ctxt->logHandle, filePath);
        ctxt->defaultLogHandle = true;
    }

    LogSetLevel(ctxt->logHandle, options->logLevel);
    // log the build version
    LogInfo(ctxt->logHandle, "SEF API version 0x%04x", SEFAPIVersion);
    LogInfo(ctxt->logHandle, "SEF SDK version %s", SEFSDKVersion);
}

struct SEFStatus sbmInitSefInfo(FTLContext *ctxt)
{
    struct SEFStatus status;
    uint64_t baseQuota;

    ctxt->handle = SEFGetHandle(ctxt->sefUnitIndex);
    if (ctxt->handle == NULL)
    {
        LogFatal(ctxt->logHandle, "Invalid SEF unit index of %d", ctxt->sefUnitIndex);
        return SUMakeStatus(-EINVAL, 1);
    }
    ctxt->sefInfo = SEFGetInformation(ctxt->handle);
    LogInfo(ctxt->logHandle, "SEF Device(%d): [%d, %d, %d, %d, %d, %d]", ctxt->sefUnitIndex,
            ctxt->sefInfo->numChannels, ctxt->sefInfo->numBanks, ctxt->sefInfo->numPlanes,
            ctxt->sefInfo->numBlocks, ctxt->sefInfo->numPages, ctxt->sefInfo->pageSize);

    status = SEFGetQoSDomainInformation(ctxt->handle, ctxt->qosId, &ctxt->qosInfo);
    if (status.error)
    {
        LogFatal(ctxt->logHandle, "Failed to open QoS Domain %d", ctxt->qosId.id);
        return status;
    }
    ctxt->pSLC = (ctxt->qosInfo.flashQuota <= ctxt->qosInfo.pSLCFlashQuota);
    if (ctxt->pSLC)
    {
        baseQuota = ctxt->qosInfo.pSLCFlashQuota;
        ctxt->superBlockCapacity = ctxt->qosInfo.pSLCSuperBlockCapacity;
    }
    else
    {
        baseQuota = ctxt->qosInfo.flashQuota;
        ctxt->superBlockCapacity = ctxt->qosInfo.superBlockCapacity;
    }
    LogInfo(ctxt->logHandle,
            "QoS Domain(%d): q: %" PRIu64
            " a:%u, api:%d, "
            "e:%d, nr:%d, np:%d bt:%d",
            ctxt->qosId.id, baseQuota, ctxt->qosInfo.ADUsize, ctxt->qosInfo.api,
            ctxt->qosInfo.encryption, ctxt->qosInfo.numReadQueues, ctxt->qosInfo.numPlacementIDs,
            ctxt->pSLC);
    LogInfo(ctxt->logHandle, "Superblock size = %u ADUs", ctxt->superBlockCapacity);

    ctxt->flashWriteSize =
        (ctxt->sefInfo->pageSize / ctxt->qosInfo.ADUsize.data) * ctxt->sefInfo->numPlanes;
    ctxt->maxIOSize = 65536 - (65536 % ctxt->flashWriteSize);
    ctxt->ioWeights = ctxt->qosInfo.weights;
    ctxt->readWeight = 0;
    ctxt->ioState.eraseWeight = ctxt->allocOverride.eraseWeight = ctxt->ioWeights.eraseWeight;
    ctxt->ioState.programWeight = ctxt->writeOverride.programWeight = ctxt->ioWeights.programWeight;
    ctxt->readOverride.readQueue = ctxt->qosInfo.defaultReadQueue;
    ctxt->ioState.readWeight = ctxt->readOverride.readWeight = 0;    // not overridden

    LogInfo(ctxt->logHandle, "Read Queue: %d", ctxt->qosInfo.defaultReadQueue);
    LogInfo(ctxt->logHandle, "Read Weight: %d", ctxt->readWeight);
    LogInfo(ctxt->logHandle, "Erase Weight: %d", ctxt->ioWeights.eraseWeight);
    LogInfo(ctxt->logHandle, "Program Weight: %d", ctxt->ioWeights.programWeight);

    return SUMakeStatusOk();
}

static struct SEFStatus sbmInitNumDomains(FTLContext *ctxt)
{
    uint16_t maxDomains = ctxt->sefInfo->numBanks * ctxt->sefInfo->numChannels;

    if (SEFIsNullFlashAddress(ctxt->qosInfo.rootPointers[FTL_PDL_ROOT_INDEX]))
    {
        return SUMakeStatusError(-ENOENT);
    }
    if (!SEFIsNullFlashAddress(ctxt->qosInfo.rootPointers[FTL_PDL_QOS_DOMAINS_INDEX]))
    {
        ctxt->numDomains = ctxt->qosInfo.rootPointers[FTL_PDL_QOS_DOMAINS_INDEX].bits;
    }
    if (ctxt->numDomains > maxDomains)
    {
        LogFatal(ctxt->logHandle,
                 "Persisted block data is corrupt, the number "
                 "of QoS Domains %d is higher than maximum number of QoS Domains %d",
                 ctxt->numDomains, maxDomains);
        ctxt->numDomains = 0;
        return SUMakeStatusError(-ENXIO);
    }
    return SUMakeStatusOk();
}

static void sbmCloseDomains(FTLContext *ctxt)
{
    int i;

    if (ctxt->qosHandles == NULL)
    {
        assert(ctxt->qosIds == NULL);
        return;
    }

    for (i = 0; i < ctxt->numDomains; i++)
    {
        SEFQoSHandle qosHandle = ctxt->qosHandles[i];
        struct SEFStatus status = SUMakeStatusOk();

        if (qosHandle)
        {
            status = SEFCloseQoSDomain(ctxt->qosHandles[i]);
        }
        if (status.error)
        {
            struct SEFQoSDomainID qosId;

            qosId = SEFGetQoSHandleProperty(qosHandle, kSefPropertyQoSDomainID).qosID;
            LogError(ctxt->logHandle, "Failed to close QoSDomain %d, error %d/%d", qosId.id,
                     status.error, status.info);
        }
    }
}

static struct SEFStatus sbmOpenDomains(FTLContext *ctxt)
{
    struct SEFStatus status = SUMakeStatusError(-ENXIO);
    struct SEFProperty logKey;
    int i;

    logKey.ptr = ctxt->logHandle;
    logKey.type = kSefPropertyTypePtr;
    ctxt->qosHandles = SUzalloc(sizeof(*ctxt->qosHandles) * ctxt->numDomains);
    ctxt->qosIds = SUzalloc(sizeof(*ctxt->qosIds) * ctxt->numDomains);
    for (i = 0; i < ctxt->numDomains; i++)
    {
        struct SEFQoSDomainID qosId = {ctxt->qosId.id + i};

        status = SEFOpenQoSDomain(ctxt->handle, qosId, HandleSEFNotification, ctxt, 0,
                                  &ctxt->qosHandles[i]);
        if (status.error)
        {
            LogFatal(ctxt->logHandle, "Could not open the QoS Domain with Id of %d", qosId.id);
            return status;
        }
        ctxt->qosIds[i] = qosId;
        SEFSetQoSHandleProperty(ctxt->qosHandles[i], kSefPropertyPrivateData, logKey);
    }
    return status;
}

static struct SEFStatus sbmInitPersistence(FTLContext *ctxt, struct SEFBlockConfig *config)
{
    struct PDLGuid blockGuid = {SEFBlockGUID};
    struct QoSState qosState = {};
    struct SEFStatus status;

    ctxt->numDomains = config ? MAX(config->numDomains, 1) : 1;    // we don't know yet
    status = sbmInitNumDomains(ctxt);
    if (!status.error)
    {
        LogInfo(ctxt->logHandle, "Number of domains: %d", ctxt->numDomains);
    }
    else if (ctxt->numDomains)
    {
        LogInfo(ctxt->logHandle, "Number of configured domains: %d", ctxt->numDomains);
    }
    else
    {
        LogFatal(ctxt->logHandle, "Bad persisted domain count: %d", ctxt->numDomains);
        return status;
    }

    status = sbmOpenDomains(ctxt);
    if (!status.error)
    {
        qosState.blockType = ctxt->pSLC ? kForPSLCWrite : kForWrite;
        qosState.numDomains = ctxt->numDomains;
        qosState.qosDomainHandles = ctxt->qosHandles;
        qosState.qosDomainIds = ctxt->qosIds;
        status = PDLInit(&ctxt->pdlHandle, ctxt->handle, &qosState, &ctxt->qosInfo, blockGuid);
    }
    else
    {
        ctxt->numDomains = 0;
    }
    if (status.error)
    {
        LogFatal(ctxt->logHandle, "Could not init the persistence layer");
        return status;
    }

    return SUMakeStatusOk();
}

struct SEFStatus sbmMarkPdlBlocks(void *arg, struct SEFFlashAddress flashAddress)
{
    FTLContext *ctxt = arg;
    struct SSBParsedFLA pfa;
    struct SEFStatus status;

    status = SSBParseFlashAddress(ctxt, flashAddress, &pfa);
    if (status.error == 0)
    {
        SSBMarkAsMeta(ctxt, &pfa);
    }
    return status;
}

static struct SEFStatus sbmSaveMetaData(FTLContext *ctxt)
{
    // flush persistence to the flash
    struct SEFStatus status;

    ctxt->blockData.Wsn = atomic_load(&ctxt->wsn);
    sbmQueueToWriteBlockMeta(ctxt->pdlHandle, &ctxt->blockData);
    SSBQueueToWriteState(ctxt, ctxt->pdlHandle);
    SFTQueueToWriteLut(ctxt);
    status = PDLFreeFlash(ctxt->pdlHandle);
    if (status.error)
    {
        LogError(ctxt->logHandle, "Was unable to free old persistance data on the flash");
        return status;
    }
    status = PDLFlushToFlash(ctxt->pdlHandle);
    if (status.error == 0 && ctxt->ssbState)
    {
        status = PDLForEachSuperBlock(ctxt->pdlHandle, sbmMarkPdlBlocks, ctxt);
    }
    if (status.error)
    {
        LogError(ctxt->logHandle, "Was unable to flush persistance data to the flash");
    }
    return status;
}

struct SEFStatus SEFBlockConfig(uint16_t SEFUnitIndex,
                                struct SEFQoSDomainID QoSDomainID,
                                struct SEFBlockConfig *blockConfig)
{
    struct SEFBlockConfig config = Block_Default_Config;
    struct SEFStatus status;
    FTLContext *ctxt;

    // validate the input configs
    if (blockConfig != NULL)
    {
        config = *blockConfig;
    }
    if (config.overprovisioning < 0 || config.overprovisioning > 99)
    {
        return SUMakeStatusError(-EINVAL);
    }

    ctxt = sbmAllocContext(SEFUnitIndex, QoSDomainID);
    sbmInitLogging(ctxt, &config.blockOption);
    status = sbmInitSefInfo(ctxt);
    if (status.error == 0)
    {
        status = sbmInitPersistence(ctxt, &config);
    }

    if (status.error == 0)
    {
        // check for preconfig device
        if (PDLIsFlushed(ctxt->pdlHandle))
        {
            LogFatal(
                ctxt->logHandle,
                "The device is already configured, can not reconfigure a pre-configured device");
            status = SUMakeStatusError(-EEXIST);
        }
    }

    if (status.error == 0)
    {
        // calculate metadata
        status = sbmCalculateDeviceCapacity(ctxt, config.overprovisioning, ctxt->numDomains,
                                            &ctxt->blockData);
    }

    if (status.error == 0)
    {
        ctxt->timeToDie = 1;    // Disable notification processing (no SSB/SFT state)
        sbmSaveMetaData(ctxt);
    }
    else
    {
        LogFatal(ctxt->logHandle, "Could not calculate block metadata");
    }

    SEFBlockCleanup(&ctxt);
    return status;
}

int blkSetState(void *arg, int index, int64_t val)
{
    FTLContext *ctxt = arg;

    if (index == 3)
    {
        ctxt->ioState.readWeight = ctxt->readOverride.readWeight = ctxt->readWeight = val;
    }
    else if (index == 4)
    {
        ctxt->ioState.programWeight = ctxt->ioWeights.programWeight = val;
    }
    else if (index == 5)
    {
        ctxt->ioState.eraseWeight = ctxt->ioWeights.eraseWeight = val;
    }
    else if (index == 8)
    {
        ctxt->ioState.readQueue = ctxt->readOverride.readQueue = val;
    }
    else
    {
        return 1;
    }
    if (index == 4 || index == 5)
    {
        struct SEFWeights weights;

        weights.programWeight = ctxt->ioWeights.programWeight;
        weights.eraseWeight = ctxt->ioWeights.eraseWeight;
        SEFSetWeights(ctxt->ssbState->dstate->qosHandle, weights);
    }
    return 0;
}

/**
 *  @ingroup    SefSdkApi
 *  @brief      Releases writes waiting for GC to complete.
 *
 *  If the number of used super blocks is less than the high water mark,
 *  any queued write I/O will be submitted by calling this function.
 *
 *  @param      ctxt        A pointer to an instance of FTLContext
 *  @param      dstate      Domain state structure with blocked writes
 */
static void sbmReleaseBlockedWrites(FTLContext *ctxt, struct SSBDomainState *dstate)
{
    if (atomic_load(&dstate->adusQueued))
    {
        UTL_DECLARE_DLIST(ioList);
        struct SefBlockWriteIocb *node = NULL;
        uint64_t adusActive;
        uint64_t adusUsed;

        pthread_mutex_lock(&dstate->fsLock);
        if (utl_DListIsEmpty(&dstate->fsWaiters))
        {
            assert(dstate->adusQueued == 0);
            pthread_mutex_unlock(&dstate->fsLock);
            return;
        }

        // While this is running adusUsed may be increasing.  Using a stale
        // value is ok because if i/o is increasing it, adusQueued will also
        // go up (eventually) keeping the active count the same.  If GC
        // increased it, it will be releasing a super block soon and it will
        // go back down.  GC can increase the # of adus when smaller blocks
        // are collected into a larger block.  To prevent the domain from
        // going beyond its quota, GC reserves an extra full capacity of
        // ADUs.  Given enough GC cycles, GC will eventually cause the # of
        // adus to go down.
        adusUsed = atomic_load(&dstate->adusUsed);
        adusActive = adusUsed - dstate->adusQueued;
        // Dequeue IOCBs as long as the active count is less than the maximum
        while (adusActive <= dstate->maxAdus)
        {
            node = utl_DListGetHeadAs(&dstate->fsWaiters, struct SefBlockWriteIocb, link);
            if (node == NULL || adusActive + node->iocb.numADU > dstate->maxAdus)
            {
                break;
            }
            utl_DListRemove(&node->link);
            utl_DListPushTail(&ioList, &node->link);
            dstate->adusQueued -= node->iocb.numADU;
            adusActive += node->iocb.numADU;
        }
        assert(!utl_DListIsEmpty(&dstate->fsWaiters) || !dstate->adusQueued);
        if (!utl_DListIsEmpty(&ioList))
        {
            LogTrace(ctxt->logHandle, "Releasing with active: %" PRId64 " queued %" PRId64,
                     adusActive, dstate->adusQueued);
        }
        pthread_mutex_unlock(&dstate->fsLock);

        // Issue the dequeued IOCBs
        int nAdus = 0;
        int nIos = 0;
        while ((node = utl_DListPopHeadAs(&ioList, struct SefBlockWriteIocb, link)))
        {
            assert(ctxt->ioState.blockedWrites);
            atomic_fetch_sub(&ctxt->ioState.blockedWrites, 1);
            nAdus += node->iocb.numADU;
            nIos++;
            clock_gettime(CLOCK_MONOTONIC, &node->start);
            SEFWriteWithoutPhysicalAddressAsync(dstate->qosHandle, &node->iocb);
        }
        if (nAdus)
        {
            LogTrace(ctxt->logHandle, "Released %d ADUS from %d I/Os of %lu active", nAdus, nIos,
                     ctxt->ioState.activeWrite);
        }
    }
}

static void sbmRegisterCounters(FTLContext *ctxt)
{
    struct INSCounter a[] = {
        InstructionCounterDef(maxQueue, "Unused, always 0", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(read, "Number of block read requests", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(readADU, "Number of read ADUs", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(readIocb, "Number of device read IOCBs", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(readLatency_us, "Total read latency in µs", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(readMinLatency_ns, "Minimum read latency in ns", SEFIOCounter,
                              &ctxt->counter),
        InstructionCounterDef(readMaxLatency_ns, "Maximum read latency in ns", SEFIOCounter,
                              &ctxt->counter),
        InstructionCounterDef(readZero, "Number of ADUs read without being written (returns zeros)",
                              SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(write, "Number of block write requests", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(writeADU, "Number of written ADUs ", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(writeIocb, "Number of device write IOCBs", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(writeLatency_us, "Total write latency in µs", SEFIOCounter, &ctxt->counter),
        InstructionCounterDef(writeMinLatency_ns, "Minimum write latency in ns", SEFIOCounter,
                              &ctxt->counter),
        InstructionCounterDef(writeMaxLatency_ns, "Maximum write latency in ns", SEFIOCounter,
                              &ctxt->counter),
        InstructionCounterDef(trim, "Number of trim requests", SEFIOCounter, &ctxt->counter),
    };

    INSRegisterIoCounters(ctxt->hInst, a, NELEM(a), NULL, NULL);

    struct INSCounter s[] = {
        InstructionCounterDef(activeRead, "SEFBlock-reads in-flight", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(activeWrite, "SEFBlock-writes in-flight", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(blockedWrites, "Number of writes blocked on GC", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(readWeight, "Read I/O weight override", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(programWeight, "Write default I/O weight", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(eraseWeight, "Erase default I/O weight", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(nlcProgramWeight, "Copy write weight", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(nlcEraseWeight, "Copy erase weight", SEFIOState, &ctxt->ioState),
        InstructionCounterDef(readQueue, "readQueue", SEFIOState, &ctxt->ioState),
    };

    INSRegisterStateCounters(ctxt->hInst, s, NELEM(s), ctxt, NULL, blkSetState);
}

static struct SEFStatus sbmLoadBlockMetaData(FTLContext *ctxt)
{
    struct PDLKey pdlKey = {SBM_BLOCK_META_PERSISTENCE_KEY, 0};
    struct PDLData pdlData = {};
    struct SEFStatus status;

    // read block metadata from flash
    status = PDLReadFlash(ctxt->pdlHandle, pdlKey, &pdlData);
    if (!status.error)
    {
        struct SEFBlockMeta *meta = pdlData.Obj;

        ctxt->blockData.MaxAdus = le64toh(meta->MaxAdus);
        ctxt->blockData.MaxLBA = le64toh(meta->MaxLBA);
        ctxt->blockData.Wsn = le64toh(meta->Wsn);
        atomic_store(&ctxt->wsn, ctxt->blockData.Wsn);
        ctxt->blockData.Overprovisioning = le32toh(meta->Overprovisioning);

        LogTrace(ctxt->logHandle, "Retrieved block metadata from pre-configured device");
        LogInfo(ctxt->logHandle, "Configured overprovisioning is %d%%",
                ctxt->blockData.Overprovisioning);
        LogInfo(ctxt->logHandle, "Max LBA is %" PRIu64, ctxt->blockData.MaxLBA);
        LogInfo(ctxt->logHandle, "Max ADUs is %" PRIu64, ctxt->blockData.MaxAdus);

        // free read data
        SUfree(pdlData.Obj);
    }
    return status;
}

static void sbmInitInstrumentation(FTLContext *ctxt, const char *path)
{
    int instPathSize;
    char *instPath;

    if (path == NULL)
    {
        path = Block_Default_Instrumentation;
    }
    // initialize the Instrumentation layer
    instPathSize = 1 + snprintf(NULL, 0, path, ctxt->sefUnitIndex, ctxt->qosId.id);
    instPath = SUmalloc(instPathSize);
    snprintf(instPath, instPathSize, path, ctxt->sefUnitIndex, ctxt->qosId.id);
    if (INSInit(instPath, &ctxt->hInst, ctxt->logHandle))
    {
        LogError(ctxt->logHandle, "Could not init the Instrumentation layer");
    }
    SUfree(instPath);
}

struct SEFStatus sbmInitGC(FTLContext *ctxt)
{
    struct GCConfig gcConfig = {
        .ctxt = ctxt,
        .hLog = ctxt->logHandle,
        .hInst = ctxt->hInst,
        .op = ctxt->blockData.Overprovisioning / 100.0,
        .maxLba = ctxt->blockData.MaxLBA,
        .notifyData = ctxt,
        .notify = sbmGCNotify,
        .dstate = ctxt->ssbState->dstate,
        .nQoS = ctxt->numDomains,
    };

    ctxt->gctx = GCInit(&gcConfig);
    if (ctxt->gctx == NULL)
    {
        return SUMakeStatusError(-ENOMEM);
    }
    SSBSetGC(ctxt->ssbState->dstate, ctxt->gctx);
    return SUMakeStatusOk();
}

static struct SEFStatus sbmLoadFtl(FTLContext *ctxt)
{
    struct SEFStatus status;

    if (ctxt->ssbState)
    {    // ftl already loaded
        assert(ctxt->sftState);
        return SUMakeStatusOk();
    }
    status = SSBInitialize(ctxt, ctxt->pdlHandle, sbmReleaseBlockedWrites);
    if (status.error == 0)
    {
        status = SFTInitialize(ctxt, ctxt->blockData.MaxLBA);
    }
    if (ctxt->pdlHandle)
    {
        if (status.error == 0)
        {
            status = PDLForEachSuperBlock(ctxt->pdlHandle, sbmMarkPdlBlocks, ctxt);
        }
        if (status.error == 0)
        {
            status = SSBFindLeakedBlocks(ctxt, ctxt->ssbState, false);
        }
    }
    return status;
}

static struct SEFStatus sbmMount(SEFBlockHandle ctxt)
{
    struct SEFStatus status;

    status = PDLIsDirty(ctxt->pdlHandle);
    if (status.error || status.info)
    {
        LogError(ctxt->logHandle,
                 "The metadata Table was marked as dirty; Consider running Check Disk");
        return SUMakeStatusError(-EBADF);
    }

    status = sbmLoadFtl(ctxt);
#if CC_DIE_STATS
    DieStatsInit(ctxt->qosHandles[0], ctxt->qosInfo.ADUsize, ctxt->hInst, &ctxt->dieStats,
                 ctxt->logHandle);
#endif
    if (status.error == 0)
    {
        status = sbmInitGC(ctxt);
    }
    if (status.error == 0)
    {
        sbmRegisterCounters(ctxt);
    }
    if (status.error == 0)
    {
        // mark root pointer as dirty
        status = PDLMarkDirty(ctxt->pdlHandle);
    }
    if (status.error == 0)
    {
        // Allow i/o and mark FTL as initialized/mounted
        sefGatewayOpen(&ctxt->ioGateway);
        ctxt->initialized = 1;
    }
    return status;
}

struct SEFStatus SEFBlockInit(uint16_t unitIndex,
                              struct SEFQoSDomainID qosId,
                              struct SEFBlockOption *options,
                              SEFBlockHandle *blockHandle)
{
    struct SEFBlockOption defaultOptions = Block_Default_Option;
    struct SEFStatus status;
    SEFBlockHandle ctxt;

    if (options == NULL)
    {
        options = &defaultOptions;
    }
    ctxt = sbmAllocContext(unitIndex, qosId);
    ctxt->notifyFunc = options->notifyFunc;
    ctxt->notifyContext = options->notifyContext;
    sbmInitLogging(ctxt, options);
    status = sbmInitSefInfo(ctxt);
    if (status.error == 0)
    {
        status = sbmInitPersistence(ctxt, NULL);
    }
    if (status.error == 0)
    {
        status = sbmLoadBlockMetaData(ctxt);
    }
    if (status.error == 0)
    {
        sbmInitInstrumentation(ctxt, options->instrumentationPath);
    }
    if (status.error == 0 && !options->delayMount)
    {
        status = sbmMount(ctxt);
    }
    if (status.error == 0)
    {
        *blockHandle = ctxt;
        return SUMakeStatusOk();
    }
    SEFBlockCleanup(&ctxt);
    return status;
}

static void sbmRebuildCleanup(FTLContext *ctxt)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SFTState *sftState = ctxt->sftState;

    ctxt->ssbState = ctxt->rebuiltSsbState;
    ctxt->sftState = ctxt->rebuiltSftState;
    ctxt->rebuiltSsbState = NULL;
    ctxt->rebuiltSftState = NULL;
    SSBCleanup(ctxt);
    SFTCleanup(ctxt);
    ctxt->ssbState = ssbState;
    ctxt->sftState = sftState;
}

struct SEFStatus SEFBlockMount(SEFBlockHandle blockHandle)
{
    if (blockHandle->initialized)
    {
        return SUMakeStatusOk();
    }
    sbmRebuildCleanup(blockHandle);    // free rebuilt lut if post check, no repair
    return sbmMount(blockHandle);
}

void SEFBlockGetInfo(SEFBlockHandle blockHandle, struct SEFBlockInfo *info)
{
    FTLContext *ctxt = blockHandle;

    info->aduSize = ctxt->qosInfo.ADUsize;
    info->numPlacementIDs = ctxt->qosInfo.numPlacementIDs;
    info->superBlockSize = ctxt->superBlockCapacity;
    info->flashWriteSize = ctxt->flashWriteSize;
    info->superPageSize = ctxt->superBlockCapacity / ctxt->sefInfo->numPages;
    info->capacity = ctxt->blockData.MaxLBA;
    info->overprovisioning = ctxt->blockData.Overprovisioning;
    info->numDomains = ctxt->numDomains;
}

/**
 * @brief Cancels a block i/o and returns list of child write iocbs
 *
 * The waiters list is scanned looking for SefBlockWriteIocb's created by
 * the block i/o request context.  The child iocbs are removed from the
 * waiters lists and added to the ioList.  If any are found, the context
 * i/o request is marked as canceled.
 *
 * @param context   Block write i/o attempting to cancel
 * @param waiters   List of write i/o cb's waiting to scan
 * @param ioList    context's child write i/o requests found
 */
static void sbmCancelWaiters(struct SEFMultiContext *context, TmaDList *waiters, TmaDList *ioList)
{
    struct SefBlockWriteIocb *node = NULL;
    struct SefBlockWriteIocb *prev = NULL;
    // struct SEFStatus status;
    FTLContext *ctxt = context->blockHandle;

    while ((node = utl_DListNextAs(waiters, node, struct SefBlockWriteIocb, link)))
    {
        struct SEFMultiContext *mctx = node->iocb.common.param1;

        if (mctx == context || (mctx->parent && mctx->parent == context))
        {
            LogTrace(ctxt->logHandle, "Found i/o %p/%p %" PRId64 "/%c/%c to cancel", context, mctx,
                     mctx->lba, mctx->lbc, mctx->cancel);
            mctx->cancel = 1;
            utl_DListPushTail(ioList, utl_DListRemove(&node->link));
            node = prev;
        }
        prev = node;
    }
}

struct SEFStatus SEFBlockCancel(struct SEFMultiContext *context)
{
    FTLContext *ctxt = context->blockHandle;
    struct SSBDomainState *dstate = ctxt->ssbState->dstate;
    UTL_DECLARE_DLIST(ioList);
    struct SEFStatus status;

    // Scan the lists of waiters and look for the i/o or it's children
    context->cancel = 1;
    dstate += (context->qosIndex);

    pthread_mutex_lock(&dstate->fsLock);
    {
        struct SefBlockWriteIocb *node = NULL;

        sbmCancelWaiters(context, &dstate->fsWaiters, &ioList);
        while ((node = utl_DListNextAs(&ioList, node, struct SefBlockWriteIocb, link)))
        {
            dstate->adusQueued -= node->iocb.numADU;
            ctxt->ioState.blockedWrites--;
        }
    }
    pthread_mutex_unlock(&dstate->fsLock);
    status.error = utl_DListIsEmpty(&ioList) ? -ENOENT : 0;
    sbmCancelIoList(&ioList);

    return status;
}


static struct SEFStatus sbmRebuildLut(FTLContext *ctxt)
{
    struct SEFStatus status;
    PDLHandle pdlHandle = ctxt->pdlHandle;

    if (ctxt->rebuiltSsbState)
    {    // already rebuilt
        assert(ctxt->rebuiltSftState);
        return SUMakeStatusOk();
    }
    assert(ctxt->ssbState == NULL);
    assert(ctxt->sftState == NULL);
    assert(ctxt->rebuiltSftState == NULL);
    ctxt->pdlHandle = NULL;    // Prevent loading of the LUT
    status = sbmLoadFtl(ctxt);
    if (status.error == 0)
    {
        // do call back for status
        status = SFTRebuildLut(ctxt, (struct SEFFlashAddress){0});
    }
    if (status.error == 0)
    {
        ctxt->rebuiltSftState = ctxt->sftState;
        ctxt->rebuiltSsbState = ctxt->ssbState;
        ctxt->sftState = NULL;
        ctxt->ssbState = NULL;
    }
    else
    {
        SFTCleanup(ctxt);
        SSBCleanup(ctxt);
    }
    ctxt->pdlHandle = pdlHandle;
    return status;
}

static struct SEFStatus sbmValidateLut(FTLContext *ctxt)
{
    int isInvalid = 0;
    uint32_t i;

    for (i = 0; i < ctxt->sftState->lutSize && !isInvalid; i++)
    {
        isInvalid += (ctxt->sftState->lut[i] != ctxt->rebuiltSftState->lut[i]);
    }

    if (isInvalid)
    {
        LogError(ctxt->logHandle, "The stored data does not match the calculated ones");
        return SUMakeStatusError(-EIO);
    }
    return SUMakeStatusOk();
}

struct SEFStatus SEFBlockCheck(SEFBlockHandle blockHandle, int shouldRepair)
{
    struct FTLContext_ *ctxt = blockHandle;
    struct SEFStatus status;
    bool isValid;

    if (ctxt->initialized)
    {
        return SUMakeStatusOk();
    }

    status = sbmRebuildLut(ctxt);
    if (status.error)
    {
        LogFatal(ctxt->logHandle, "Could not rebuild the Look-up table");
        return status;
    }
    status = sbmLoadFtl(ctxt);
    if (status.error)
    {
        LogFatal(ctxt->logHandle, "Could not load the Look-up table");
        return status;
    }

    status = sbmValidateLut(ctxt);
    isValid = (status.error == 0);
    if (shouldRepair)
    {
        if (!isValid)
        {
            PDLHandle pdlHandle = ctxt->pdlHandle;

            // discard persisted state
            ctxt->pdlHandle = NULL;
            SFTCleanup(ctxt);
            SSBCleanup(ctxt);
            ctxt->pdlHandle = pdlHandle;
            // swap in repaired state and persist
            ctxt->ssbState = ctxt->rebuiltSsbState;
            ctxt->sftState = ctxt->rebuiltSftState;
            ctxt->rebuiltSsbState = NULL;
            ctxt->rebuiltSftState = NULL;
            status = sbmSaveMetaData(ctxt);
        }
        if (status.error == 0)
        {
            status = SSBFindLeakedBlocks(ctxt, ctxt->ssbState, true);
        }
        // clean unmounted state
        SFTCleanup(ctxt);
        SSBCleanup(ctxt);
        return status;
    }

    if (isValid)    // don't need the rebuilt lut
    {
        PDLHandle pdlHandle = ctxt->pdlHandle;

        ctxt->pdlHandle = NULL;
        PDLMarkClean(pdlHandle);
        SFTCleanup(ctxt);
        SSBCleanup(ctxt);
        sbmRebuildCleanup(ctxt);
        ctxt->pdlHandle = pdlHandle;
    }

    return status;
}

static struct SEFStatus sbmUnmount(FTLContext *ctxt)
{
    struct SEFStatus status = SUMakeStatusOk();

    if (ctxt->initialized)    // mounted
    {
        SSBCloseOpenBlocks(ctxt, ctxt->ssbState);
        ctxt->timeToDie = 1;    // Disables notification processing
        if (ctxt->sftState->lutDirty)
        {
            status = sbmSaveMetaData(ctxt);
        }
        else
        {
            // mark root pointer as clean if no writes
            status = PDLMarkClean(ctxt->pdlHandle);
            if (status.error)
            {
                LogError(ctxt->logHandle, "Was unable to mark the root pointer as clean");
            }
        }
    }
#if CC_DIE_STATS
    DieStatsCleanup(ctxt->dieStats);
#endif
    SFTCleanup(ctxt);
    assert(ctxt->sftState == NULL);
    SSBCleanup(ctxt);
    assert(ctxt->ssbState == NULL);
    return status;
}

static void sbmStopGC(FTLContext *ctxt)
{
    if (ctxt->gctx)
    {
        GCCleanup(ctxt->gctx);
        ctxt->gctx = NULL;
    }
}

static void sbmCleanupSefInfo(FTLContext *ctxt)
{
    if (!ctxt->handle)
    {
        return;
    }

    sbmCloseDomains(ctxt);
    SUfree(ctxt->qosHandles);
    SUfree(ctxt->qosIds);
    ctxt->qosHandles = NULL;
    ctxt->qosIds = NULL;
    ctxt->handle = 0;
}

static void sbmDumpCounters(FTLContext *ctxt)
{
    char out[1024];

    if (ctxt->hInst)
    {
        INSInvokeAction(ctxt->hInst, "dump json", out, sizeof(out));
        LogDebug(ctxt->logHandle, "Session Statistics: %s", out);
        INSUnRegisterAction(ctxt->hInst, "dump");
        INSUnRegisterAction(ctxt->hInst, "state");
    }
#if CC_PER_SB_LATENCY
    for (i = 0; i < ctxt->ssbState->dstate->blockCount; i++)
    {
        uint64_t readADU;
        uint64_t readLatency_ns;
        uint64_t readMaxLatency_ns;
        uint64_t readMinLatency_ns;

        if (ctxt->ssbState->dstate->valid == NULL || ctxt->logHandle == NULL)
        {
            continue;
        }
        readADU = atomic_load(&ctxt->ssbState->dstate->valid[i].readADU);
        readLatency_ns = atomic_load(&ctxt->ssbState->dstate->valid[i].readLatency_ns);
        readMaxLatency_ns = atomic_load(&ctxt->ssbState->dstate->valid[i].readMaxLatency_ns);
        readMinLatency_ns = atomic_load(&ctxt->ssbState->dstate->valid[i].readMinLatency_ns);
        if (readADU == 0)
        {
            continue;
        }

        LogDebug(ctxt->logHandle, "SB: %u avg %lu min %lu max %lu", i, readLatency_ns / readADU,
                 readMinLatency_ns, readMaxLatency_ns);
    }
#endif
}

struct SEFStatus SEFBlockCleanup(SEFBlockHandle *blockHandle)
{
    struct SEFStatus status;
    FTLContext *ctxt;

    if (!blockHandle || !*blockHandle)
    {
        return SUMakeStatusError(-ENODATA);
    }

    ctxt = *blockHandle;
    sefGatewayClose(&ctxt->ioGateway);    // stop user i/o
    sbmStopGC(ctxt);
    sbmDumpCounters(ctxt);
    status = sbmUnmount(ctxt);
    if (status.error == 0)
    {
        INSCleanup(&ctxt->hInst);
        status = PDLCleanup(ctxt->pdlHandle);
    }
    if (status.error == 0)
    {
        sbmCleanupSefInfo(ctxt);
        if (ctxt->defaultLogHandle)
        {
            DFLCleanup(ctxt->logHandle);
        }
        ctxt->initialized = 0;
        SUfree(ctxt);
        *blockHandle = 0;
    }
    return status;
}

/**
 *  @ingroup    SefSdkAPI
 *  @brief      This function is used to update the LUT and mark the LBA as empty
 *              without changing values stored in flash.
 *
 *  *lba and *lbc are updated with how much was trimmed
 *
 *  Note: The data will not be removed from the flash memory, and if a recovery is performed, the data is restored.
 *
 *  @param          ctxt                A pointer to an instance of FTLContext
 *  @param[in,out]  lba                 Logical block address
 *  @param[in,out]  lbc                 Logical block count
 *
 *  @retval     0       SUCCESS
 *  @retval     -EINVAL lba+lbc is outside the LUT
 */
static struct SEFStatus sbmWholeTrim(FTLContext *ctxt, int64_t *lba, int32_t *lbc)
{
    int returnVal;
    struct SEFPlacementID placementId = {0};
    struct SEFFlashAddress flashAddress;

    flashAddress = SFTLookup(ctxt, *lba);

    if (!SEFIsNullFlashAddress(flashAddress))
    {
        returnVal = SFTSet(ctxt, *lba, SEFNullFlashAddress, placementId);
        if (returnVal)
        {
            LogError(ctxt->logHandle, "Was unable to trim the 0x%lx; the data is valid at LUT",
                     flashAddress.bits);
            return SUMakeStatus(-EIO, *lbc);
        }
    }

    // update the lba/lbc
    *lba += 1;
    *lbc -= 1;

    return SUMakeStatusOk();
}

struct SEFStatus SEFBlockTrim(SEFBlockHandle blockHandle, int64_t lba, int32_t lbc)
{
    SEFBlockHandle ctxt;

    if (!blockHandle)
    {
        return SUMakeStatusError(-ENOTBLK);
    }

    ctxt = blockHandle;

    if (!ctxt->initialized || sefGatewayEnter(&ctxt->ioGateway))
    {
        LogError(ctxt->logHandle, "FTL is either not initialized or shutting down, failing I/O");
        return SUMakeStatusError(-ENOTBLK);
    }

    if (lba + lbc > ctxt->sftState->lutSize)
    {
        sefGatewayLeave(&ctxt->ioGateway);
        LogError(ctxt->logHandle, "I/O exceeds the device capacity max/lba/lbc (0x%lx/0x%lx/%x)",
                 ctxt->sftState->lutSize, lba, lbc);
        return SUMakeStatusError(-EINVAL);
    }

    atomic_fetch_add(&ctxt->counter.trim, 1);

    while (lbc)
    {
        struct SEFStatus status = sbmWholeTrim(ctxt, &lba, &lbc);
        if (status.error)
        {
            sefGatewayLeave(&ctxt->ioGateway);
            return status;
        }
    }

    sefGatewayLeave(&ctxt->ioGateway);
    return SUMakeStatusOk();
}

/**
 * @brief Sets the error field of a block i/o request if not already set
 *
 * @param context   I/O request to put in an errored state
 * @param error     Error value to set
 */
static void sbmSetIOError(struct SEFMultiContext *context, int error)
{
    int tmp = 0;
    atomic_compare_exchange_strong(&context->error, &tmp, error);
}

/**
 *  @ingroup    SefSdkApi
 *  @brief      Dereferences a multi-context I/O
 *
 *  Decrement counters and call user's completion function when 0.
 *  I/O context pointer set to null to protect against post-completion
 *  dereference.
 *
 *  @param pContext Pointer to pointer to I/O to dereference
 */
static void sbmIoDereference(struct SEFMultiContext **pContext)
{
    struct SEFMultiContext *context = *pContext;
    FTLContext *ctxt = context->blockHandle;

    *pContext = NULL;
    int count = atomic_fetch_sub(&context->count, 1) - 1;
    if (count < 0)
    {
        // at this point, atomic counters are off, disable, assume issue is
        // context reuse or memory reuse, try to let caller know.
        // not decrementing counters so more likely to hang than corrupt memory
        LogError(ctxt->logHandle, "I/O context %p corrupt, i/o disabled, process may hang", context);
        ctxt->disabled = true;
        atomic_store(&context->error, -EBUSY);
        context->completion(context);
    }

    if (!count)
    {
        enum SEFBlockIOType ioType = context->ioType;
        context->completion(context);
        context = NULL;    // Likely has been freed

        switch (ioType)
        {
            case kSEFRead:
                atomic_fetch_sub(&ctxt->ioState.activeRead, 1);
                break;
            case kSEFWrite:
                atomic_fetch_sub(&ctxt->ioState.activeWrite, 1);
                break;
        }

        sefGatewayLeave(&ctxt->ioGateway);
    }
}

static void sbmWholeReadComplete(struct SEFCommonIOCB *common);
static void sbmWholeWriteComplete(struct SEFCommonIOCB *common);

/**
 *  @ingroup    SefSdkApi
 *  @brief      This function is used to perform read from the flash given the LBA
 *
 *  *lba, *lbc and *iovOffset are updated with how much was read.
 *
 *  @param          context             A pointer to an instance of SEFMultiContext
 *  @param          ctxt                A pointer to an instance of FTLContext
 *  @param[in,out]  lba                 Logical block address
 *  @param[in,out]  lbc                 Logical block count
 *  @param[in,out]  iovOffset           The offset for the scatter/gather list
 */
static void sbmWholeRead(
    struct SEFMultiContext *context, FTLContext *ctxt, int64_t *lba, int32_t *lbc, size_t *iovOffset)
{
    // read is more complex; attempt to coalesce adjacent ADUs
    uint32_t numWhole = 0;
    struct SefBlockReadIocb *readIocb;
    struct SEFFlashAddress addr;
    struct SSBParsedFLA pfa;

    if (*lba + *lbc > ctxt->sftState->lutSize)
    {
        LogError(ctxt->logHandle, "Read I/O exceeded the capacity of the device");
        sbmSetIOError(context, -EINVAL);
        return;
    }

    // Find a defined address
    addr = SFTLookupForRead(ctxt, *lba);
    if (SEFIsNullFlashAddress(addr))
    {
        memsetIov(context->iov, context->iovcnt, *iovOffset, 0, ctxt->qosInfo.ADUsize.data);
        *lba += 1;
        *lbc -= 1;
        *iovOffset += ctxt->qosInfo.ADUsize.data;
        atomic_fetch_add(&ctxt->counter.readZero, 1);
        return;
    }

    readIocb = SUzalloc(sizeof(*readIocb));
    if (!readIocb)
    {
        LogError(ctxt->logHandle, "Failed to allocate space for async read object");
        SFTReleaseForRead(ctxt, addr);
        sbmSetIOError(context, -ENOMEM);
        return;
    }

    readIocb->iocb.flashAddress = addr;
    readIocb->iocb.userAddress = SEFCreateUserAddress(*lba, SEF_USR_META_TAG);

    SSBParseFlashAddress(ctxt, addr, &pfa);
    for (numWhole = 1; numWhole < *lbc && numWhole < ctxt->maxIOSize; numWhole++)
    {
        uint32_t nextBlock;
        struct SEFQoSDomainID domain;
        uint32_t adu;

        if (SEFIsNullFlashAddress(addr = SFTLookup(ctxt, (*lba) + numWhole)))
        {
            break;
        }
        SEFParseFlashAddress(pfa.dstate->qosHandle, addr, &domain, &nextBlock, &adu);
        if (adu != pfa.adu + numWhole || pfa.block != nextBlock || pfa.domain.id != domain.id)
        {
            break;
        }
    }
    *lba += numWhole;
    *lbc -= numWhole;

    // issue async read
    readIocb->iocb.common.param1 = context;
    readIocb->iocb.numADU = numWhole;
    readIocb->iocb.iovcnt = context->iovcnt;
    readIocb->iocb.iovOffset = *iovOffset;
    readIocb->iocb.iov = context->iov;
    readIocb->iocb.common.complete_func = sbmWholeReadComplete;

    if (context->ioWeight || context->readQueue < ctxt->qosInfo.numReadQueues)
    {
        readIocb->iocb.common.flags |= kSefIoFlagOverride;
        readIocb->iocb.overrides.readWeight = context->ioWeight;
        readIocb->iocb.overrides.readQueue = context->readQueue;
    }
    if (ctxt->readOverride.readWeight || ctxt->qosInfo.defaultReadQueue != ctxt->readOverride.readQueue)
    {
        readIocb->iocb.common.flags |= kSefIoFlagOverride;
        readIocb->iocb.overrides = ctxt->readOverride;
    }
    clock_gettime(CLOCK_MONOTONIC, &readIocb->start);
    atomic_fetch_add(&context->count, 1);
    SEFReadWithPhysicalAddressAsync(pfa.dstate->qosHandle, &readIocb->iocb);
    *iovOffset += numWhole * ctxt->qosInfo.ADUsize.data;
}

static void sbmWholeReadComplete(struct SEFCommonIOCB *common)
{
    struct SEFReadWithPhysicalAddressIOCB *iocb = (void *)common;
    struct SefBlockReadIocb *readIocb = (void *)iocb;
    struct SEFMultiContext *context = (struct SEFMultiContext *)iocb->common.param1;
    FTLContext *ctxt = (FTLContext *)context->blockHandle;
    struct SEFFlashAddress flashAddress = iocb->flashAddress;
    struct timespec stop;

    clock_gettime(CLOCK_MONOTONIC, &stop);
    if (iocb->common.status.error)
    {
        bool interrupted = (iocb->common.status.error == -EINTR);

#ifdef UNIT_TEST
        if (ctxt->notifyFunc)
        {
            struct SEFBlockNotify notify = {.type = kSefBlockNotifyReadError};
            ctxt->notifyFunc(notify, ctxt->notifyContext);
        }
#endif

        // retry read !interrupted & lut changed
        if (!interrupted && SFTLutChanged(ctxt, readIocb->iocb.userAddress,
                                          readIocb->iocb.flashAddress, readIocb->iocb.numADU))
        {
            int64_t lba = context->lba;
            int32_t lbc = context->lbc;
            size_t iovOffset = context->iovOffset;

            while (lbc && !context->cancel)
            {
                sbmWholeRead(context, ctxt, &lba, &lbc, &iovOffset);
            }

            // clean up after retry
            goto exit;
        }

        int tmp = 0;

        atomic_compare_exchange_strong(&context->error, &tmp, iocb->common.status.error);
        LogError(ctxt->logHandle,
                 "Async read failed for LBA %" PRId64 ", FA 0x%" PRIx64 " with error %d",
                 context->lba, iocb->flashAddress.bits, iocb->common.status.error);
    }
    else
    {
        uint64_t latency = (stop.tv_sec - readIocb->start.tv_sec) * 1000000000UL +
                           (stop.tv_nsec - readIocb->start.tv_nsec);
#if CC_PER_SB_LATENCY
        {
            struct SSBParsedFLA pfa;
            uint64_t expected;
            uint64_t latency_per_adu = latency / iocb->numADU;

            SSBParseFlashAddress(ctxt, iocb->flashAddress, &pfa);
            atomic_fetch_add(&pfa.bstate->readADU, iocb->numADU);
            atomic_fetch_add(&pfa.bstate->readLatency_ns, latency);
            expected = atomic_load(&pfa.bstate->readMaxLatency_ns);
            do
            {
                if (expected >= latency_per_adu)
                {
                    break;
                }
            } while (!atomic_compare_exchange_weak(&pfa.bstate->readMaxLatency_ns, &expected,
                                                   latency_per_adu));
            expected = atomic_load(&pfa.bstate->readMinLatency_ns);
            do
            {
                if (expected && expected <= latency_per_adu)
                {
                    break;
                }
            } while (!atomic_compare_exchange_weak(&pfa.bstate->readMinLatency_ns, &expected,
                                                   latency_per_adu));
        }
#endif
        atomic_fetch_add(&context->transferred, iocb->numADU);
        atomic_fetch_add(&ctxt->counter.readADU, iocb->numADU);
        atomic_fetch_add(&ctxt->counter.readIocb, 1);
        atomic_fetch_add(&ctxt->counter.readLatency_us, (latency + 999) / 1000);
        if (ctxt->counter.readMaxLatency_ns < latency)
        {
            ctxt->counter.readMaxLatency_ns = latency;
        }
        if (!ctxt->counter.readMinLatency_ns || ctxt->counter.readMinLatency_ns >= latency)
        {
            ctxt->counter.readMinLatency_ns = latency;
        }
#if CC_DIE_STATS
        struct SSBParsedFLA pfa;
        SSBParseFlashAddress(ctxt, flashAddress, &pfa);
        DieStatsRead(ctxt->dieStats, pfa.dstate->dieList, flashAddress, iocb->numADU);
#endif
    }
exit:
    // clean up...
    SFTReleaseForRead(ctxt, flashAddress);
    SUfree(iocb);
    sbmIoDereference(&context);
}

/**
 * @brief queues i/o if it needs to wait for GC to run
 *
 * SSB will call block notify cb when space is available
 * which will issued this queued i/o.
 *
 * @retval true    I/O was queued
 * @retval false   There is space and i/o wasn't queued
 */
static bool sbmWaitForGC(FTLContext *ctxt, struct SefBlockWriteIocb *iocb)
{
    bool queued = false;
    bool disabled = false;
    struct SSBDomainState *dstate = iocb->dstate;
    uint32_t numADU = iocb->iocb.numADU;
    uint64_t adusUsed;

    adusUsed = atomic_fetch_add(&dstate->adusUsed, numADU) + numADU;
    if (adusUsed > dstate->maxAdus)
    {
        pthread_mutex_lock(&dstate->fsLock);
        disabled = ctxt->disabled;
        uint64_t adusActive = atomic_load(&dstate->adusUsed) - dstate->adusQueued;
        if (!disabled && atomic_load(&dstate->adusUsed) > dstate->maxAdus)
        {
            queued = true;
            utl_DListPushHead(&dstate->fsWaiters, &iocb->link);
            ctxt->ioState.blockedWrites++;
            dstate->adusQueued += numADU;
            if (ctxt->ioState.blockedWrites == 1)
            {
                LogTrace(ctxt->logHandle,
                         "Domain %d: writes blocked at used: %" PRIu64 " active: %" PRId64
                         " queued %" PRId64,
                         dstate->domainId.id, atomic_load(&dstate->blocksUsed), adusActive,
                         dstate->adusQueued);
            }
        }
        pthread_mutex_unlock(&dstate->fsLock);
    }
    // when queued, adusQueued goes up making it possible a write may be released
    if (queued)
    {
        sbmReleaseBlockedWrites(ctxt, dstate);
    }
    if (disabled)
    {
        iocb->canceled = true;
        iocb->iocb.common.status.error = -ECANCELED;
        LogTrace(ctxt->logHandle, "i/o is disabled, canceling i/o %p", iocb);
    }
    return queued;
}

/**
 *  @ingroup    SefSdkApi
 *  @brief      This function is used to perform write to the flash and update the LUT.
 *
 *  *lba, *lbc and *iovOffset are updated with how much was written.
 *
 *  @param          context             A pointer to an instance of SEFMultiContext
 *  @param          ctxt                A pointer to an instance of FTLContext
 *  @param[in,out]  lba                 Logical block address
 *  @param[in,out]  lbc                 Logical block count
 *  @param[in,out]  iovOffset           The offset for the scatter/gather list
 */
static void sbmWholeWrite(
    struct SEFMultiContext *context, FTLContext *ctxt, int64_t *lba, int32_t *lbc, size_t *iovOffset)
{
    int32_t numWhole = MIN(*lbc, ctxt->maxIOSize);
    struct SSBDomainState *dstate;
    struct SefBlockWriteIocb *iocb =
        SUzalloc(sizeof(*iocb) + *lbc * (ctxt->qosInfo.ADUsize.meta + sizeof(iocb->tentative[0])));
    struct SEFWriteWithoutPhysicalAddressIOCB *writeIocb = &iocb->iocb;
    int32_t i;

    if (!iocb || *lba + *lbc > ctxt->sftState->lutSize)
    {
        if (*lba + *lbc > ctxt->sftState->lutSize)
        {
            LogError(ctxt->logHandle, "Write i/o exceeds the capacity of the device");
            sbmSetIOError(context, -EINVAL);
        }
        else
        {
            LogError(ctxt->logHandle, "Failed to allocate space for async write object");
            sbmSetIOError(context, -ENOMEM);
        }
        SUfree(iocb);
        return;
    }

    if (!ctxt->notifyFunc && (context->flags & kSEFBlockIOFlagNotifyBufferRelease))
    {
        LogError(ctxt->logHandle, "Buffer release flag used with no notification function");
        sbmSetIOError(context, -EINVAL);
        SUfree(iocb);
        return;
    }

    // issue async write
    writeIocb->common.param1 = context;
    writeIocb->flashAddress = ctxt->pSLC ? SEFAutoAllocatePSLC : SEFAutoAllocate;
    writeIocb->numADU = numWhole;
    iocb->numLeft = *lbc - numWhole;
    if (*iovOffset)
    {
        struct iovec *iov;
        context->iovOffset = *iovOffset;
        copyIov(context->iov, context->iovcnt, *iovOffset, &iov, &writeIocb->iovcnt);
        writeIocb->iov = iov;
        iocb->iov = iov;
    }
    else
    {
        writeIocb->iovcnt = context->iovcnt;
        writeIocb->iov = context->iov;
    }
    atomic_fetch_add(&context->count, 1);
    writeIocb->userAddress = SEFCreateUserAddress(*lba, SEF_USR_META_TAG);
    writeIocb->tentativeAddresses = iocb->tentative;
    writeIocb->common.complete_func = sbmWholeWriteComplete;
    if (context->flags & kSEFBlockIOFlagNotifyBufferRelease)
    {
        writeIocb->common.flags |= kSefIoFlagNotifyBufferRelease;
    }
    if (ctxt->qosInfo.ADUsize.meta >= sizeof(uint64_t))
    {
        uint64_t wsn = atomic_fetch_add(&ctxt->wsn, 1) + 1;
        uint8_t *metadata = (uint8_t *)(&iocb->tentative[writeIocb->numADU]);
        writeIocb->metadata = metadata;
        for (i = 0; i < writeIocb->numADU; i++)
        {
            memcpy(metadata, &wsn, sizeof(wsn));
            metadata += ctxt->qosInfo.ADUsize.meta;
        }
    }
    //    writeIocb->overrides = iocb->dstate->inGC ? &iocb->override : NULL;
    // setting programWeight sets ioWeights so they can be restored after GC
    // sets it so they are always equal when GC isn't running
    if (context->ioWeight)
    {
        writeIocb->common.flags |= kSefIoFlagOverride;
        writeIocb->overrides.programWeight = context->ioWeight;
    }
    if (ctxt->writeOverride.programWeight != ctxt->ioWeights.programWeight)
    {
        writeIocb->common.flags |= kSefIoFlagOverride;
        writeIocb->overrides = ctxt->writeOverride;
    }
    iocb->dstate = dstate = &ctxt->ssbState->dstate[context->qosIndex];
    writeIocb->placementID = context->placementID;

    if (sbmWaitForGC(ctxt, iocb))
    {
        GCTriggerDomain(ctxt->gctx, dstate);    // Fail safe when domain is at highwater at init time ()
    }
    else if (iocb->canceled)
    {
        iocb->iocb.common.complete_func(&iocb->iocb.common);
    }
    else
    {
        clock_gettime(CLOCK_MONOTONIC, &iocb->start);
        SEFWriteWithoutPhysicalAddressAsync(iocb->dstate->qosHandle, writeIocb);
    }
    *lba += *lbc;
    *iovOffset += *lbc * ctxt->qosInfo.ADUsize.data;
    *lbc = 0;
}

static void sbmWholeWriteComplete(struct SEFCommonIOCB *common)
{
    struct SEFWriteWithoutPhysicalAddressIOCB *iocb = (void *)common;
    struct SefBlockWriteIocb *writeIocb = container_of(iocb, struct SefBlockWriteIocb, iocb);
    struct SEFMultiContext *context = (struct SEFMultiContext *)iocb->common.param1;
    FTLContext *ctxt = (FTLContext *)context->blockHandle;
    // struct SSBDomainState *dstate = writeIocb->dstate;
    struct timespec stop = {0};
    bool dataWritten = (iocb->common.status.error == 0 || iocb->common.status.error == -ENOSPC ||
                        iocb->common.status.error == -EDQUOT) &&
                       iocb->common.status.info;

    clock_gettime(CLOCK_MONOTONIC, &stop);
    assert(!writeIocb->completed);
    writeIocb->completed = true;
    if (!dataWritten)
    {
        int tmp = 0;
        atomic_compare_exchange_strong(&context->error, &tmp, iocb->common.status.error);
        LogError(ctxt->logHandle,
                 "Async write failed for LBA %" PRId64 ", FA 0x%" PRIx64 " with error %d",
                 context->lba, iocb->flashAddress.bits, iocb->common.status.error);
    }
    else
    {
        int numADU = iocb->common.status.info;
        uint64_t latency = (stop.tv_sec - writeIocb->start.tv_sec) * UINT64_C(1000000000) +
                           (stop.tv_nsec - writeIocb->start.tv_nsec);
        int i;

        if (numADU != iocb->numADU)
        {
            LogError(ctxt->logHandle, "Partial write, requested %u ADUs but only %u completed",
                     iocb->numADU, numADU);
        }
        for (i = 0; i < numADU; i++)
        {
            SFTSet(ctxt, SEFGetUserAddressLba(iocb->userAddress) + i, iocb->tentativeAddresses[i],
                   iocb->placementID);
        }

        atomic_fetch_add(&context->transferred, numADU);
        atomic_fetch_add(&ctxt->counter.writeADU, numADU);
        atomic_fetch_add(&writeIocb->dstate->pState[iocb->placementID.id].writeADU, numADU);
        atomic_fetch_add(&ctxt->counter.writeIocb, 1);
        atomic_fetch_add(&ctxt->counter.writeLatency_us, (latency + 999) / 1000);
        if (ctxt->counter.writeMaxLatency_ns < latency)
        {
            ctxt->counter.writeMaxLatency_ns = latency;
        }
        if (!ctxt->counter.writeMinLatency_ns || ctxt->counter.writeMinLatency_ns >= latency)
        {
            ctxt->counter.writeMinLatency_ns = latency;
        }

#if CC_DIE_STATS
        struct SSBParsedFLA pfa;
        SSBParseFlashAddress(ctxt, iocb->tentativeAddresses[0], &pfa);
        DieStatsWrite(ctxt->dieStats, pfa.dstate->dieList, iocb->tentativeAddresses, numADU);
#endif
    }

    if (iocb->common.status.error == 0 && writeIocb->numLeft && iocb->common.status.info == iocb->numADU)
    {
        int32_t numWhole = MIN(writeIocb->numLeft, ctxt->maxIOSize);
        struct iovec *iov = NULL;
        uint64_t userLBA = SEFGetUserAddressLba(iocb->userAddress);

        copyIov(iocb->iov, iocb->iovcnt, iocb->common.status.info * ctxt->qosInfo.ADUsize.data,
                &iov, &iocb->iovcnt);
        writeIocb->numLeft -= numWhole;
        writeIocb->completed = false;
        iocb->numADU = numWhole;
        iocb->tentativeAddresses += iocb->common.status.info;
        iocb->userAddress = SEFCreateUserAddress(userLBA + iocb->common.status.info,
                                                 SEFGetUserAddressMeta(iocb->userAddress));
        if (writeIocb->iov)
        {
            SUfree(writeIocb->iov);
        }
        writeIocb->iov = iov;
        iocb->iov = iov;
        if (iov != NULL)
        {
            SEFWriteWithoutPhysicalAddressAsync(writeIocb->dstate->qosHandle, iocb);
        }
        else
        {
            iocb->common.status = SUMakeStatusError(-ENOMEM);
            iocb->common.complete_func(&iocb->common);
        }
    }
    else
    {
        // todo: return sb's size?  Not know until closed
        context->numLbl = iocb->distanceToEndOfSuperBlock;
        // clean up...
        if (writeIocb->iov)
        {
            SUfree(writeIocb->iov);
        }
        SUfree(writeIocb);

        sbmIoDereference(&context);
    }
}

struct SEFStatus SEFBlockIO(struct SEFMultiContext *context)
{
    int64_t lba = context->lba;
    int32_t lbc = context->lbc;
    size_t iovOffset = context->iovOffset;
    ssize_t bufLen = lenIov(context->iov, context->iovcnt);
    FTLContext *ctxt = (FTLContext *)context->blockHandle;

    if (bufLen < iovOffset + lbc * ctxt->qosInfo.ADUsize.data)
    {
        LogError(ctxt->logHandle,
                 "Scatter gather list is too small. Buffer is %zd bytes but i/o needs %d bytes",
                 bufLen, iovOffset + lbc * ctxt->qosInfo.ADUsize.data);
        return SUMakeStatusError(-EINVAL);
    }

    if (atomic_load(&context->count))
    {
        LogError(ctxt->logHandle, "I/o context %p appears to be in use", context);
        return SUMakeStatusError(-EBUSY);
    }

    if (!ctxt->initialized || sefGatewayEnter(&ctxt->ioGateway))
    {
        LogError(ctxt->logHandle, "FTL is either not initialized or shutting down, failing I/O");
        return SUMakeStatusError(-ENOPROTOOPT);
    }

    if (lba + lbc > ctxt->sftState->lutSize)
    {
        sefGatewayLeave(&ctxt->ioGateway);
        LogError(ctxt->logHandle, "I/O exceeds the device capacity max/lba/lbc (0x%lx/0x%lx/%x)",
                 ctxt->sftState->lutSize, lba, lbc);
        return SUMakeStatusError(-EINVAL);
    }

    atomic_store(&context->count, 1);

    switch (context->ioType)
    {
        case kSEFRead:
            assert(context->iov);
            atomic_fetch_add(&ctxt->ioState.activeRead, 1);
            atomic_fetch_add(&ctxt->counter.read, 1);
            break;
        case kSEFWrite:
            assert(context->iov);
            atomic_fetch_add(&ctxt->ioState.activeWrite, 1);
            atomic_fetch_add(&ctxt->counter.write, 1);
            break;
    }

    // Handle aligned full IOs
    while (lbc && !context->cancel)
    {
        switch (context->ioType)
        {
            case kSEFRead:
                sbmWholeRead(context, ctxt, &lba, &lbc, &iovOffset);
                break;
            case kSEFWrite:
                sbmWholeWrite(context, ctxt, &lba, &lbc, &iovOffset);
                break;
        }
    }

    sbmIoDereference(&context);
    return SUMakeStatusOk();
}

