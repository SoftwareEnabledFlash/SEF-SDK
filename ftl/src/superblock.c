/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * superblock.c
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
#include "superblock.h"

#include <SEFAPI.h>
#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "flash-translation.h"
#include "log-manager.h"
#include "persistence.h"
#include "sef-utils.h"

#define SSB_DOMAIN_STATE_PERSISTENCE_KEY "DomainState"

struct StoredValidMap
{
    struct SEFFlashAddress sbAddress;
    uint32_t refCnt;                      // num valid
    uint32_t writtenADUs;                 // num total
    struct SEFPlacementID placementID;    // Placement ID (set by GC or copied from sb info)
    uint16_t unused[3];
    int64_t map[];
};

struct StoredDomainState
{
    int aduMapCount;
    int validMapCount;
    struct StoredValidMap validMap[];
};

static void ssbSetStateOpen(FTLContext *ctxt, struct SSBParsedFLA *pfa);

static struct SEFStatus ssbStateRestore(FTLContext *ctxt, PDLHandle hPdl, struct SSBDomainState *dstate)
{
    struct PDLData pdlData;
    struct SEFStatus status;
    struct PDLKey pdlKey = {SSB_DOMAIN_STATE_PERSISTENCE_KEY, dstate->domainId.id};
    bool restored = false;

    // check for persisted domain state
    status = PDLReadFlash(hPdl, pdlKey, &pdlData);
    if (!status.error)
    {
        int i, num64s, aduMapCount;
        struct StoredDomainState *storedDomainState;

        // load the persisted DomainState
        storedDomainState = pdlData.Obj;
        aduMapCount = le32toh(storedDomainState->aduMapCount);
        assert(aduMapCount >= ctxt->superBlockCapacity);
        num64s = DIV_ROUND_UP(aduMapCount, 64);
        for (i = 0; i < le32toh(storedDomainState->validMapCount); i++)
        {
            struct SSBParsedFLA pfa;
            struct StoredValidMap *storedValidMap = pdlData.Obj + sizeof(struct StoredDomainState) +
                                                    sizeof(struct StoredValidMap) * i +
                                                    sizeof(uint64_t) * num64s * i;

            SSBParseFlashAddress(ctxt, storedValidMap->sbAddress, &pfa);
            ssbSetStateOpen(ctxt, &pfa);
            pfa.bstate->vData->refCnt = le32toh(storedValidMap->refCnt) + 1;
            pfa.bstate->placementID.id = le16toh(storedValidMap->placementID.id);
            memcpy(pfa.bstate->vData->map, storedValidMap->map,
                   num64s * sizeof(uint64_t));    // assumes will be used in Little Endian
            SSBClosed(ctxt, &pfa, le32toh(storedValidMap->writtenADUs),
                      le32toh(storedValidMap->writtenADUs));
        }
        restored = true;
        SUfree(pdlData.Obj);
    }
    else if (status.error != -ENOENT)
    {
        LogFatal(ctxt->logHandle, "Error restoring persisted valid map");
        return status;
    }
    return SUMakeStatusInfo(restored);
}

static struct SEFStatus ssbStateInit(FTLContext *ctxt, struct SSBDomainState *dstate)
{
    pthread_mutex_init(&dstate->validLock, NULL);
    pthread_mutex_init(&dstate->fsLock, NULL);

    dstate->maxAdus = ctxt->blockData.MaxAdus;
    utl_DListInit(&dstate->fsWaiters);
    PWCInit(&dstate->blkRelease);
    dstate->valid = SUzalloc(dstate->blockCount * sizeof(struct SSBBlockData));
    if (!dstate->valid)
    {
        LogFatal(ctxt->logHandle, "Error allocating validMap array");
        return SUMakeStatusError(-ENOMEM);
    }

    dstate->pState = SUzalloc(ctxt->qosInfo.numPlacementIDs * sizeof(struct SSBPlacementState));
    if (!dstate->pState)
    {
        LogFatal(ctxt->logHandle, "Error allocating pState array");
        return SUMakeStatusError(-ENOMEM);
    }

    return SUMakeStatusOk();
}

static void ssbDStateCleanup(struct SSBState *ssbState)
{
    int i;

    for (i = 0; i < ssbState->num_qos; i++)
    {
        struct SSBDomainState *dstate = &ssbState->dstate[i];

        pthread_mutex_destroy(&dstate->validLock);
        pthread_mutex_destroy(&dstate->fsLock);
        if (dstate->pState)
        {
            SUfree(dstate->pState);
        }
        dstate->pState = NULL;

        if (dstate->valid)
        {
            uint32_t j;

            for (j = 0; j < dstate->blockCount; j++)
            {
                uint32_t state = atomic_load(&dstate->valid[j].state);
                assert((state & ~kSSBStateMask) == 0);    // Nothing inflight
                assert((state & kSSBStateMask) != kSSBStateBusy);

                if (dstate->valid[j].vData)
                {
                    SUfree(dstate->valid[j].vData);
                }
            }

            SUfree(dstate->valid);
            dstate->valid = NULL;
        }
    }
}

struct SEFStatus SSBRestore(FTLContext *ctxt, PDLHandle hPdl)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SEFStatus status;
    int i;

    if (hPdl == NULL || ssbState == NULL)
    {
        return SUMakeStatusOk();
    }
    for (i = 0; i < ssbState->num_qos; i++)
    {
        if ((status = ssbStateRestore(ctxt, hPdl, &ssbState->dstate[i])).error)
        {
            break;
        }
    }

    return status;
}

#if CC_DEMO_DP
void sbUsageAction(const char *verb, char **savePtr, void *arg, char *out, ssize_t size)
{
    FTLContext *ctxt = arg;
    struct SSBState *ssbState = ctxt->ssbState;
    struct SSBDomainState *dstate = ssbState->dstate;
    struct SSBBlockData *bstate;
    int i, n;

    if (strcmp(verb, "help") == 0)
    {
        snprintf(out, size, "* sb_usage: prints non-zero sb usage as list a [<sbid adus> ]*\n");
        return;
    }

    // check if mounted
    if (ssbState == NULL)
    {
        return;
    }

    bstate = dstate->valid;

    for (i = 0; i < dstate->blockCount && size > 0; ++i, ++bstate)
    {
        struct SSBParsedFLA pfa = {.bstate = bstate, .dstate = dstate, .domain = dstate->domainId};
        uint32_t state = atomic_load(&bstate->state) & kSSBStateMask;
        uint32_t refCnt;
        uint16_t plid;
        uint32_t used;

        if (state != kSSBStateOpen && state != kSSBStateClosed)
        {
            continue;
        }

        if (!SSBMarkBusy(ctxt, &pfa))
        {
            continue;
        }
        refCnt = atomic_load(&bstate->vData->refCnt);
        used = atomic_load(&bstate->vData->writableADUs);
        SSBMarkNotBusy(ctxt, &pfa);
        if (refCnt && state == kSSBStateOpen)    // open keeps it's own refCnt
        {
            refCnt--;
        }
        if (refCnt == 0)
        {
            continue;
        }
        plid = bstate->placementID.id;
        n = snprintf(NULL, 0, "%d,%d,%d,%d\n", i, plid, used, refCnt);
        if (size < n)
        {
            return;
        }
        n = snprintf(out, size, "%d,%d,%d,%d\n", i, plid, used, refCnt);
        out += n;
        size -= n;
    }

    n = snprintf(NULL, 0, "\n");
    if (size < n)
    {
        return;
    }

    n = snprintf(out, size, "\n");
}
#endif

static void ssbRegisterCounters(INSHandle hInst, struct SSBState *ssbState)
{
    struct INSCounter a[] = {
        InstructionCounterDef(releasedBlockCount, "Number of blocks released", struct SSBState, ssbState),
        InstructionCounterDef(openedBlockCount, "Number of blocks opened", struct SSBState, ssbState),
        InstructionCounterDef(maintBlocks, "Number of blocks released for maintenance",
                              struct SSBState, ssbState),
#if CC_DEBUG_COUNTERS
        InstructionCounterDef(openRetry, "Moving block to open retry cnt", struct SSBState, ssbState),
        InstructionCounterDef(emptyRetry, "Moving block to empty retry cnt", struct SSBState, ssbState),
        InstructionCounterDef(closeRetry, "Moving block to closed retry cnt", struct SSBState, ssbState),
        InstructionCounterDef(staleRead, "LUT entry used to read was stale", struct SSBState, ssbState),
        InstructionCounterDef(busyOverflow,
                              "Busy count at max (will never happen without coding mistake)",
                              struct SSBState, ssbState),
        InstructionCounterDef(busyRetry, "Moving block to busy retry cnt", struct SSBState, ssbState),
        InstructionCounterDef(emptyNotClosed, "Block became empty before closed", struct SSBState,
                              ssbState),
#endif
    };

    ssbState->ssbIoCntId = INSRegisterIoCounters(hInst, a, NELEM(a), NULL, NULL);

    struct INSCounter s[] = {
        InstructionCounterDef(openBlocks, "Number of open super blocks", struct SSBState, ssbState),
        InstructionCounterDef(blocksUsed, "Number of allocated super blocks", struct SSBState,
                              ssbState->dstate),
#if CC_DEMO_DP
        InstructionCounterDef(validAdus, "Number valid ADUs", struct SSBState, ssbState),
#endif
    };
    ssbState->ssbStateCntId = INSRegisterStateCounters(hInst, s, NELEM(s), NULL, NULL, NULL);
}

struct SEFStatus SSBInitialize(FTLContext *ctxt,
                               PDLHandle hPdl,
                               void (*blockReleased)(struct FTLContext_ *ctxt,
                                                     struct SSBDomainState *dstate))
{
    struct SSBState *ssbState;
    struct SEFStatus status;
    size_t size;
    int i;

    assert(ctxt->ssbState == NULL);
    size = sizeof(*ssbState);
    // Note: +1 so the list is null terminated - used by ssbForEachDomain
    size += sizeof(ssbState->dstate[0]) * (ctxt->numDomains + 1);
    ctxt->ssbState = SUzalloc(size);
    ssbState = ctxt->ssbState;
    ssbState->num_qos = ctxt->numDomains;
    for (i = 0; i < ssbState->num_qos; i++)
    {
        struct SEFQoSDomainID qosId;
        struct SEFVirtualDeviceID vdId;
        uint16_t superBlockDies;
        uint32_t blockCount;
        uint16_t numDies;

        qosId = SEFGetQoSHandleProperty(ctxt->qosHandles[i], kSefPropertyQoSDomainID).qosID;
        vdId = SEFGetQoSHandleProperty(ctxt->qosHandles[i], kSefPropertyVirtualDeviceID).vdID;
        ssbState->dstate[i].qosHandle = ctxt->qosHandles[i];
        ssbState->dstate[i].domainId = qosId;
        SUGetDieList(ctxt->handle, vdId, &ssbState->dstate[i].dieList);
        numDies = ssbState->dstate[i].dieList->numDies;
        status = SUGetSuperBlockDies(ctxt->qosHandles[i]);
        if (status.error)
        {
            LogError(ctxt->logHandle, "Failed to calculate block count for domain %d", qosId);
            return status;
        }
        superBlockDies = status.info;
        blockCount = ctxt->sefInfo->numBlocks * (numDies / superBlockDies);
        ssbState->dstate[i].blockCount = blockCount;
        ssbState->dstate[i].mapSize = DIV_ROUND_UP(ctxt->superBlockCapacity, 64);
    }

    LogInfo(ctxt->logHandle, "Virtual Device block count = %u blocks", ssbState->dstate->blockCount);

    ssbState->blockReleased = blockReleased;
    for (i = 0; i < ssbState->num_qos; i++)
    {
        if ((status = ssbStateInit(ctxt, &ssbState->dstate[i])).error)
        {
            break;
        }
    }

    if (status.error == 0)
    {
        ssbRegisterCounters(ctxt->hInst, ssbState);

#if CC_DEMO_DP
        INSRegisterAction(ctxt->hInst, "sb_usage", sbUsageAction, ctxt);
#endif

        status = SSBRestore(ctxt, hPdl);
    }

    return status;
}

#define ssbForEachDomain(D) for (; (D)->qosHandle; (D)++)

void SSBCleanup(FTLContext *ctxt)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SSBDomainState *dstate;

    if (ctxt->ssbState == NULL)
    {
        return;
    }

    ssbDStateCleanup(ctxt->ssbState);

    // destroying locks and mutexes
    // SSBQueueToWriteState(ctxt, ctxt->pdlHandle);

#if CC_DEMO_DP
    INSUnRegisterAction(ctxt->hInst, "sb_info");
#endif

    if (ssbState->ssbIoCntId >= 0)
    {
        INSUnRegisterIoCounters(ctxt->hInst, ssbState->ssbIoCntId);
    }
    if (ssbState->ssbStateCntId >= 0)
    {
        INSUnRegisterStateCounters(ctxt->hInst, ssbState->ssbStateCntId);
    }

    dstate = ssbState->dstate;
    ssbForEachDomain(dstate)
    {
        PWCFlush(&dstate->blkRelease);
        PWCCleanup(&dstate->blkRelease);
        SUfree(dstate->dieList);
    }
    SUfree(ssbState);
    ctxt->ssbState = NULL;
}

// Create a superblock address from dstate and bstate
static struct SEFFlashAddress ssbCreateFlashAddress(struct SSBDomainState *dstate,
                                                    struct SSBBlockData *bstate)
{
    return SEFCreateFlashAddress(dstate->qosHandle, dstate->domainId, bstate - dstate->valid, 0);
}

// save current sb.info.writableADUs in vData->writableADUs
// state should not be open and expected to be marked as busy
static uint32_t ssbGetWritableADUs(struct SSBBlockData *bstate)
{
    assert(bstate->vData->writableADUs);    // should always be up to date
    return bstate->vData->writableADUs;
}

uint32_t SSBGetWritableADUs(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBBlockData *bstate = pfa->bstate;
    uint32_t writableADUs = 0;

    if (SSBMarkBusy(ctxt, pfa))
    {
        writableADUs = bstate->vData->writableADUs;
        if (writableADUs == 0 && (atomic_load(&bstate->state) & kSSBStateMask) != kSSBStateOpen)
        {
            writableADUs = ssbGetWritableADUs(bstate);
        }
        SSBMarkNotBusy(ctxt, pfa);
    }
    return writableADUs;
}

void SSBGetInfo(FTLContext *ctxt, struct SSBDomainState *dstate, uint32_t block, struct SSBItrInfo *info)
{
    struct SSBBlockData *bstate = &dstate->valid[block];
    struct SEFFlashAddress flashAddress = ssbCreateFlashAddress(dstate, bstate);
    struct SSBParsedFLA pfa = {.adu = 0,
                               .block = block,
                               .bstate = bstate,
                               .domain = dstate->domainId,
                               .dstate = dstate,
                               .flashAddress = flashAddress};

    info->flashAddress = flashAddress;
    info->block = block;
    if (SSBMarkBusy(ctxt, &pfa))    // essentially a reader lock
    {                               // readable blocks
        info->nValid = atomic_load(&bstate->vData->refCnt);
        info->nInvalid = bstate->vData->writableADUs - info->nValid;
        info->placementID = bstate->placementID;
        info->GCRequired = bstate->GCRequired;
        info->id = bstate->id + 1;
        SSBMarkNotBusy(ctxt, &pfa);
    }
    else
    {    // meta/empty/deleted blocks
        info->nValid = 0;
        info->nInvalid = 0;
        info->placementID.id = 0;
        atomic_thread_fence(memory_order_acquire);
        info->GCRequired = bstate->GCRequired;
        info->id = bstate->id + 1;
    }
}

void SSBMarkForMaintenance(FTLContext *ctxt, struct SEFFlashAddress flashAddress)
{
    struct SSBParsedFLA pfa;

    SSBParseFlashAddress(ctxt, flashAddress, &pfa);
    if (SSBMarkBusy(ctxt, &pfa))    // essentially a read lock
    {
        pfa.bstate->GCRequired = 1;
        SSBMarkNotBusy(ctxt, &pfa);
        GCTriggerDomain(ctxt->gctx, pfa.dstate);
    }
}

bool SSBEnumBlocks(FTLContext *ctxt, struct SSBIterator *itr)
{
    struct SSBDomainState *dstate = itr->filter.dstate;
    uint32_t blockCount = dstate->blockCount;
    struct SSBBlockData *bstate = NULL;
    uint32_t state;
    bool found = false;
    uint32_t index;
    uint16_t id;

    for (index = itr->filter.index; !found && index < blockCount; index++)
    {
        bstate = &dstate->valid[index];
        id = bstate->id + 1;
        state = atomic_load(&bstate->state) & kSSBStateMask;
        found = (state == itr->filter.state) &&
                ((itr->filter.placement == bstate->placementID.id) || (itr->filter.placement == -1));
        if (found)
        {
            SSBGetInfo(ctxt, dstate, index, &itr->info);
            state = atomic_load(&bstate->state) & kSSBStateMask;
            found = (state == itr->filter.state) && (id == itr->info.id);
            // if found, placement id must be set at the point
            assert(!found || itr->info.placementID.id != SEFPlacementIdUnused);
        }
    }
    itr->filter.index = index;
    return found;
}

struct SEFStatus SSBCopyValidBits(struct SSBParsedFLA *pfa, void *buffer, ssize_t size)
{
    size_t bitmapSize = pfa->dstate->mapSize * sizeof(uint64_t);
    struct SEFStatus status = {0};

    // refCnt only valid if closed, fetch then check, if not, block is empty now
    status.info = atomic_load(&pfa->bstate->vData->refCnt);
    if ((atomic_load(&pfa->bstate->state) & kSSBStateMask) != kSSBStateClosed)
    {
        status.info = 0;
    }
    if (size < bitmapSize)
    {
        status.error = -ENOBUFS;
    }
    else
    {
        // namelessCopy requires the bitmap to be copied...
        //
        // note: # of bits set may be less than count returned in status.info
        //       but never more.  This ensure buffers allocated are large enough
        //       to hold the results.
        memcpy(buffer, pfa->bstate->vData->map, size);
    }
    return status;
}

static void ssbCloseOpenBlocks(FTLContext *ctxt, struct SSBDomainState *dstate)
{
    int i;

    // Close super blocks open by placement id
    dstate->blocksOpen = 0;
    for (i = 0; i < ctxt->qosInfo.numPlacementIDs; i++)
    {
        struct SSBPlacementState *pState = &dstate->pState[i];
        struct SSBBlockData *bstate = &dstate->valid[pState->openBlock];

        uint32_t state = atomic_load(&bstate->state) & kSSBStateMask;
        if (state == kSSBStateOpen && atomic_load(&pState->writeADU))
        {
            struct SEFFlashAddress flashAddress;
            struct SEFStatus status;

            flashAddress = ssbCreateFlashAddress(dstate, bstate);
            status = SEFCloseSuperBlock(dstate->qosHandle, flashAddress);
            if (status.error != 0)
            {
                LogDebug(ctxt->logHandle, "Failed to close superblock 0x%lx (%d/%d)",
                         flashAddress.bits, status.error, status.info);
                dstate->blocksOpen = 1;
                continue;
            }
        }
    }

    // Error check that other blocks aren't open - maybe this goes some place else
    uint64_t adusUsed = 0;
    for (i = 0; i < dstate->blockCount; i++)
    {
        struct SSBBlockData *bstate = &dstate->valid[i];
        uint32_t state = atomic_load(&bstate->state) & kSSBStateMask;

        if (state == kSSBStateOpen)
        {
            struct SEFFlashAddress flashAddress;
            struct SEFStatus status;

            flashAddress = ssbCreateFlashAddress(dstate, bstate);
            LogDebug(ctxt->logHandle, "Found unclosed superblock 0x%lx", flashAddress.bits);
            status = SEFCloseSuperBlock(dstate->qosHandle, flashAddress);
            if (status.error != 0)
            {
                LogDebug(ctxt->logHandle, "Failed to close superblock 0x%lx (%d/%d)",
                         flashAddress.bits, status.error, status.info);
                dstate->blocksOpen = 1;
            }
            // should be closed but could be empty at this point so reload.
            state = atomic_load(&bstate->state) & kSSBStateMask;
        }
        if (state == kSSBStateClosed)
        {
            adusUsed += ssbGetWritableADUs(bstate);
        }
    }
    if (adusUsed != atomic_load(&dstate->adusUsed))
    {
        // This is bad the count is off but we're shutting down so it doesn't
        // matter now.  Marked as error to stand out in the log that there's an
        // issue.
        LogError(ctxt->logHandle, "Used adu count mismatch - count %" PRId64 " had %" PRId64,
                 adusUsed, dstate->adusUsed);
    }
}

void SSBCloseOpenBlocks(FTLContext *ctxt, struct SSBState *ssbState)
{
    int i;
    for (i = 0; i < ssbState->num_qos; i++)
    {
        ssbCloseOpenBlocks(ctxt, &ssbState->dstate[i]);
    }
    SSBFlushEmptyBlocks(ssbState);
}

void SSBFlushQdEmptyBlocks(struct SSBDomainState *dstate)
{
    if (PWCFlush(&dstate->blkRelease))
    {
        assert(false);
    }
}

void SSBFlushEmptyBlocks(struct SSBState *ssbState)
{
    int i;
    for (i = 0; i < ssbState->num_qos; i++)
    {
        SSBFlushQdEmptyBlocks(&ssbState->dstate[i]);
    }
}

uint64_t SBSStateSize(const struct SEFInfo *sefInfo, struct SEFQoSDomainInfo *qdInfo, int64_t numADUs)
{
    bool pSLC = (qdInfo->flashQuota < qdInfo->pSLCFlashQuota);
    uint64_t superBlockCapacity = pSLC ? qdInfo->pSLCSuperBlockCapacity : qdInfo->superBlockCapacity;
    uint32_t mapBytes = DIV_ROUND_UP(superBlockCapacity, 64);
    uint32_t numSuperBlocks = numADUs / superBlockCapacity;
    uint64_t requiredBytes;

    if (qdInfo->defectStrategy != kPerfect)
    {    // assumes sb can shrink to a single plane
        numSuperBlocks *= sefInfo->numPlanes;
    }
    requiredBytes = sizeof(struct StoredValidMap) + mapBytes;
    return sizeof(struct StoredDomainState) + requiredBytes * numSuperBlocks;
}

static void ssbFreePdlData(struct PDLData *pdlData, int isFlushed)
{
    if (pdlData->ObjSize)
    {
        SUfree(pdlData->Obj);
    }
}

void ssbQueueToWriteState(PDLHandle pdlHandle, struct SSBDomainState *domain)
{
    int i, storedValidMapCount, storedValidMapIndex;
    size_t mapBytes = domain->mapSize * sizeof(uint64_t);
    struct PDLData pdlData;

    if (!domain->valid)
    {
        return;
    }

    strcpy(pdlData.Key.Name, SSB_DOMAIN_STATE_PERSISTENCE_KEY);
    pdlData.Key.Index = domain->domainId.id;
    pdlData.FlushCompleteFunc = ssbFreePdlData;
    pdlData.arg = NULL;
    pdlData.EncodingType = kPDLNoEncoding;

    // prepare the persisted Domain State
    storedValidMapCount = 0;
    for (i = 0; i < domain->blockCount; i++)
    {
        uint32_t state = atomic_load(&domain->valid[i].state);

        assert(domain->blocksOpen || state == kSSBStateClosed || state == kSSBStateDeleted ||
               state == kSSBStateMeta);

        // only store closed states
        if ((state & kSSBStateMask) == kSSBStateClosed)
        {
            storedValidMapCount++;
        }
    }

    // clean up domain state if no valid maps should be stored
    if (!storedValidMapCount)
    {
        // pdlData.ObjSize = 0;
        // pdlData.Obj = NULL;

        // SSBDStateCleanup(&pdlData, 0);
        return;
    }

    // prepare stored valid map (Little Endian)
    pdlData.ObjSize = sizeof(struct StoredDomainState);                        // wrapper size
    pdlData.ObjSize += sizeof(struct StoredValidMap) * storedValidMapCount;    // valid map
    pdlData.ObjSize += mapBytes * storedValidMapCount;                         // adu bit map
    pdlData.Obj = SUzalloc(pdlData.ObjSize);

    ((struct StoredDomainState *)pdlData.Obj)->aduMapCount = htole32(mapBytes * 8);
    ((struct StoredDomainState *)pdlData.Obj)->validMapCount = htole32(storedValidMapCount);

    for (i = 0, storedValidMapIndex = 0; i < domain->blockCount; i++)
    {
        uint32_t state = atomic_load(&domain->valid[i].state);

        if ((state & kSSBStateMask) == kSSBStateClosed)
        {
            struct StoredValidMap *storedValidMap =
                pdlData.Obj + sizeof(struct StoredDomainState) +
                sizeof(struct StoredValidMap) * storedValidMapIndex + mapBytes * storedValidMapIndex;

            storedValidMap->sbAddress =
                SEFCreateFlashAddress(domain->qosHandle, domain->domainId, i, 0);
            storedValidMap->placementID.id = htole16(domain->valid[i].placementID.id);
            storedValidMap->refCnt = htole32(atomic_load(&domain->valid[i].vData->refCnt));
            storedValidMap->writtenADUs = htole32(domain->valid[i].vData->writableADUs);
            memcpy(storedValidMap->map, domain->valid[i].vData->map,
                   mapBytes);    // assumes Little Endian use

            storedValidMapIndex++;
        }
    }

    PDLQueueData(pdlHandle, &pdlData);
}

void SSBQueueToWriteState(FTLContext *ctxt, PDLHandle pdlHandle)
{
    struct SSBState *ssbState = ctxt->ssbState;
    int i;

    if (pdlHandle == NULL || ssbState == NULL)
    {
        return;
    }

    for (i = 0; i < ssbState->num_qos; i++)
    {
        ssbQueueToWriteState(pdlHandle, &ssbState->dstate[i]);
    }
}

// Update used block stats, check if GC needs to be started
static void ssbIncInUseCount(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBState *ssbState = ctxt->ssbState;

    atomic_fetch_add(&ssbState->openedBlockCount, 1);
    uint32_t blocksUsed = atomic_fetch_add(&pfa->dstate->blocksUsed, 1) + 1;

    LogTrace(ctxt->logHandle, "New block 0x%lx - in use %d open %d", pfa->flashAddress.bits,
             blocksUsed, ssbState->openBlocks);

    if (GCShouldRun(ctxt->gctx, pfa->dstate))
    {
        LogTrace(ctxt->logHandle,
                 "Kicking off garbage for domain %d reaching low water - %d in use.",
                 pfa->dstate->domainId.id, blocksUsed);
        GCTriggerDomain(ctxt->gctx, pfa->dstate);
    }
}

void ssbSetStateAlloc(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBBlockData *bstate = pfa->bstate;

    assert(atomic_load(&pfa->bstate->state) == kSSBStateDeleted);

    assert(!bstate->vData);
    int num64s = pfa->dstate->mapSize;
    size_t bitmapSize = sizeof(struct SSBValidMap) + (sizeof(uint64_t) * num64s);
    struct SSBValidMap *temp = SUzalloc(bitmapSize);

    // init bitmap
    temp->writableADUs = 0;                // super.writableADUs;
    bstate->placementID.id = UINT8_MAX;    // super.placementID;

    atomic_init(&temp->refCnt, 1);    // Open's refCnt is decremented by close
    bstate->vData = temp;
    bstate->id++;
    atomic_store(&bstate->state, kSSBStateAlloc);
}

// Grabs the validLock so when more than one thread calls, when it returns,
// it will be in the open state.
//
// if deleted
//   allocates bitmap, sets alloc state
// if alloc
//   sets open
static void ssbSetStateOpen(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBState *ssbState = ctxt->ssbState;
    atomic_uint *state = &pfa->bstate->state;
    uint32_t desired;
    uint32_t expected;
    int retries = -1;
    bool opened = false;

    pthread_mutex_lock(&pfa->dstate->validLock);
    expected = atomic_load(state);
    if (expected == kSSBStateDeleted)
    {
        ssbSetStateAlloc(ctxt, pfa);
        expected = atomic_load(state);
    }

    do
    {
        retries++;
        opened = false;
        if (expected == kSSBStateAlloc)
        {
            desired = expected - kSSBStateAlloc + kSSBStateOpen;
            opened = true;
        }
        else
        {
            // Close notification from alloc will skip open.  This happens to
            // GC destination blocks, closed can precede the block being
            // marked as open.
            LogTrace(ctxt->logHandle, "Block 0x%x, unexpected state 0x%x", pfa->block, expected);
            break;
        }
    } while (!atomic_compare_exchange_weak(state, &expected, desired));

    if (opened)
    {
        atomic_fetch_add(&ssbState->openBlocks, 1);
        ssbIncInUseCount(ctxt, pfa);
    }
    pthread_mutex_unlock(&pfa->dstate->validLock);
#if CC_DEBUG_COUNTERS
    if (retries)
    {
        atomic_fetch_add(&ssbState->openRetry, retries);
    }
#endif
}

int SSBSetAduValid(FTLContext *ctxt, struct SEFFlashAddress flashAddress, struct SEFPlacementID placementId)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SSBParsedFLA parsedAddr;
    struct SEFStatus status;
    struct SSBValidMap **validPtr = NULL;

    status = SSBParseFlashAddress(ctxt, flashAddress, &parsedAddr);
    if (status.error)
    {
        return status.error;
    }
    atomic_thread_fence(memory_order_acquire);
    validPtr = &parsedAddr.bstate->vData;

    if (!(*validPtr))
    {    // first address in this block
        ssbSetStateOpen(ctxt, &parsedAddr);
        if (!(*validPtr))
        {
            return -ENOMEM;
        }
        // note: no lock so more than one thread may do this but all should agree
        //       on the placement id.
        if (placementId.id < ctxt->qosInfo.numPlacementIDs)
        {
            parsedAddr.bstate->placementID = placementId;
            parsedAddr.dstate->pState[placementId.id].openBlock = parsedAddr.block;
            LogTrace(ctxt->logHandle, "Block 0x%lx placement id set to %d",
                     parsedAddr.flashAddress.bits, placementId.id);
        }
    }

    // Note: Assumes this was called from write completion.  Block
    // can't close until completion returns so won't be reaped by GC.

#if CC_DEMO_DP
    atomic_fetch_add(&ssbState->validAdus, 1);
    uint32_t writableADUs = atomic_load(&(*validPtr)->writableADUs);
    while (parsedAddr.adu >= writableADUs)
    {
        if (atomic_compare_exchange_weak(&(*validPtr)->writableADUs, &writableADUs, parsedAddr.adu + 1))
        {
            break;
        }
    }
#endif

    // mark the superblock valid map as valid and inc ref cnt
    uint64_t bit = htole64((uint64_t)1 << (parsedAddr.adu % 64));
    uint64_t v = atomic_fetch_or(&((*validPtr)->map[parsedAddr.adu / 64]), bit);
    atomic_fetch_add(&(*validPtr)->refCnt, 1);

    assert((v & bit) == 0);

    return 0;
}

void SSBAddAdusUsed(struct SSBDomainState *dstate, uint32_t adusUsed)
{
    atomic_fetch_add(&dstate->adusUsed, adusUsed);
}

void ssbBlockReleased(struct SEFCommonIOCB *common)
{
    struct SEFReleaseSuperBlockIOCB *iocb = (void *)common;
    uint32_t *writableADUs = (uint32_t *)(iocb + 1);
    FTLContext *ctxt = common->param1;
    struct SSBParsedFLA pfa;
    struct SSBState *ssbState;
    bool metaBlock = !(*writableADUs);

    ssbState = ctxt->ssbState;
    SSBParseFlashAddress(ctxt, iocb->flashAddress, &pfa);
    if (common->status.error)
    {
        LogError(ctxt->logHandle, "Failed to release SB 0x%lx (%d/%d)", iocb->flashAddress.bits,
                 common->status.error, common->status.info);
    }
    else
    {
        if (!metaBlock)
        {
            atomic_fetch_sub(&pfa.dstate->adusUsed, *writableADUs);
            atomic_fetch_add(&ssbState->releasedBlockCount, 1);
            atomic_fetch_sub(&pfa.dstate->blocksUsed, 1);
            LogTrace(ctxt->logHandle,
                     "Handle %p: Superblock 0x%lx reclaimed %u ADUs - %" PRIu64 " blocks %" PRId64
                     " ADUs in use",
                     ctxt->handle, pfa.flashAddress.bits, *writableADUs, pfa.dstate->blocksUsed,
                     pfa.dstate->adusUsed);
            // Have FTL check if writes can go again.
            ssbState->blockReleased(ctxt, pfa.dstate);
        }
    }
    SUfree(iocb);
    if (!metaBlock && PWCComplete(&pfa.dstate->blkRelease))
    {
        assert(false);    // PWCStart() not called??
    }
}

// current state must be empty or meta
// Free resources, dec usedBlock
void SSBDeleted(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SEFReleaseSuperBlockIOCB *iocb;
    uint32_t *writableADUs;
    int currentState;

    currentState = atomic_load(&pfa->bstate->state);
    assert(currentState == kSSBStateEmpty || currentState == kSSBStateMeta);

    iocb = SUzalloc(sizeof(*iocb) + sizeof(*writableADUs));
    writableADUs = (uint32_t *)(iocb + 1);
    if (currentState == kSSBStateEmpty)
    {
        *writableADUs = pfa->bstate->vData->writableADUs;
        assert(*writableADUs);    // 0 used to indicate a meta block
        SUfree(pfa->bstate->vData);
        pfa->bstate->vData = NULL;
    }
    atomic_thread_fence(memory_order_release);
    if (pfa->bstate->GCRequired)
    {
        pfa->bstate->GCRequired = 0;
        atomic_fetch_add(&ssbState->maintBlocks, 1);
    }
    iocb->common.complete_func = ssbBlockReleased;
    iocb->common.param1 = ctxt;
    iocb->flashAddress = pfa->flashAddress;

    // Block can be reused before release completion called, assume it will
    // release.  If not, it'll be leaked.
    atomic_store(&pfa->bstate->state, kSSBStateDeleted);
    // iocb released by ssbBlockReleased
    SEFReleaseSuperBlockAsync(pfa->dstate->qosHandle, iocb);
}

// reader count is 0
static void ssbSetStateNotBusy(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    atomic_uint *state = &pfa->bstate->state;

    // assert reader count won't go up (no valid adus, only stuff inflight)
    assert(atomic_load(&pfa->bstate->vData->refCnt) == 0);
    // todo: remove, for debug of ci/cd.  Can only get here from callers that
    //       have checked state is one of these 3, yet the assert triggers.
    if (!(*state == kSSBStateBusy || *state == kSSBStateClosed || *state == kSSBStateAlloc))
    {
        LogFatal(ctxt->logHandle, "Unexpected state %x\n", *state);
    }
    // assert reader count is 0 coming from busy, closed or an abandonded alloc
    assert(*state == kSSBStateBusy || *state == kSSBStateClosed || *state == kSSBStateAlloc);

    atomic_store(state, kSSBStateEmpty);
    SSBDeleted(ctxt, pfa);
}

// undo a call to SSBMarkBusy
void SSBMarkNotBusy(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBBlockData *bstate = pfa->bstate;

    uint32_t state = atomic_fetch_sub(&bstate->state, kSSBBusyInc) - kSSBBusyInc;
    assert((state & kSSBStateMask) == kSSBStateOpen || (state & kSSBStateMask) == kSSBStateClosed ||
           (state & kSSBStateMask) == kSSBStateBusy);
    if (state == kSSBStateBusy)    // note: busy count is 0 if this is true
    {
        ssbSetStateNotBusy(ctxt, pfa);
    }
}

// balance with call to SSBMarkNotBusy
int SSBMarkBusy(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SSBBlockData *bstate = pfa->bstate;

#if CC_DEBUG_COUNTERS
    int retries = -1;
#endif
    uint32_t expected = atomic_load(&bstate->state);
    uint32_t desired;

    do
    {
        uint32_t state = expected & kSSBStateMask;

#if CC_DEBUG_COUNTERS
        retries++;
#endif
        if (state != kSSBStateOpen && state != kSSBStateClosed)
        {
            LogTrace(ctxt->logHandle, "block 0x%" PRIx64 " now 0x%x %d", pfa->flashAddress.bits,
                     expected, retries);
#if CC_DEBUG_COUNTERS
            atomic_fetch_add(&ssbState->staleRead, 1);
#endif
            return 0;
        }
        if ((expected & kSSBBusyMask) == kSSBBusyMask)
        {
#if CC_DEBUG_COUNTERS
            atomic_fetch_add(&ssbState->busyOverflow, 1);
#endif
            return 0;    // Too many readers, must fail the lock
        }
        desired = expected + kSSBBusyInc;
    } while (!atomic_compare_exchange_weak(&bstate->state, &expected, desired));
#if CC_DEBUG_COUNTERS
    if (retries)
    {
        atomic_fetch_add(&ssbState->busyRetry, retries);
    }
#endif
    return bstate->id + 1;
}

//
// called when valid becomes 0
//
static void ssbSetStateEmpty(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBState *ssbState = ctxt->ssbState;
    atomic_uint *state = &pfa->bstate->state;
    uint32_t desired;
    uint32_t expected;
    int retries = -1;

    LogTrace(ctxt->logHandle, "Domain %d Superblock 0x%" PRIx64 " is empty", pfa->domain.id,
             pfa->flashAddress.bits);

    if (PWCStart(&pfa->dstate->blkRelease))
    {
        assert(false);    // Should never happen
    }

    expected = atomic_load(state);
    do
    {
        retries++;
        // in order of likelyhood
        if (expected == kSSBStateClosed || expected == kSSBStateAlloc)
        {    // no readers
            ssbSetStateNotBusy(ctxt, pfa);
            break;
        }
        if ((expected & kSSBStateMask) == kSSBStateClosed)
        {
            desired = expected - kSSBStateClosed + kSSBStateBusy;
        }
        else
        {
            LogFatal(ctxt->logHandle, "Unexpected block state 0x%x", expected);
            abort();
        }
    } while (!atomic_compare_exchange_weak(state, &expected, desired));

#if CC_DEBUG_COUNTERS
    if (retries)
    {
        atomic_fetch_add(&ssbState->emptyRetry, retries);
    }
#endif
}

// Remove a reference to a block, if it hits zero, it will move to empty
uint32_t SSBRemoveRef(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    uint32_t refCnt = atomic_fetch_sub(&pfa->bstate->vData->refCnt, 1);

    assert(refCnt != 0);
    if (refCnt == 1)
    {
        ssbSetStateEmpty(ctxt, pfa);
    }

    return refCnt;
}

void ssbUpdateUsedAdus(FTLContext *ctxt, struct SSBParsedFLA *pfa, uint32_t writtenADUs, uint32_t numADUs)
{
    uint32_t padding;
    uint32_t writableADUs;

    if (numADUs < writtenADUs)
    {
        LogError(ctxt->logHandle, "0x%" PRIx64 " numADUs of 0x%x < 0x%x writtenAdus",
                 pfa->flashAddress.bits, numADUs, writtenADUs);
    }

    writableADUs = atomic_load(&pfa->bstate->vData->writableADUs);

#if 1    // compensate for NLC close event not knowing the allocated size
    // GC sets writable for dest block with SSBAllocated(), but copy close
    // notification came up with a different value.  This can happen when
    // errors shorten a copy. We want to regard the difference as padding.
    if (writableADUs && writableADUs > numADUs)
    {
        LogTrace(ctxt->logHandle,
                 "0x%" PRIx64 " numADUs of 0x%x < 0x%x writableADUs - using writableADUs",
                 pfa->flashAddress.bits, numADUs, writableADUs);
        numADUs = writableADUs;
    }
#endif

    // Detect if written ADUs differs from writableADUs. With CC_DEMO_DP set,
    // writableADUs will be set to what's been written so far for SB's opened by
    // PID, which do need to be adjusted up.
    if (writableADUs == 0)
    {    // write by pid case with CC_DEMO_DP not set
        atomic_store(&pfa->bstate->vData->writableADUs, numADUs);
    }
    else if (writableADUs < numADUs)
    {    // write by pid case with CC_DEMO_DP set
        LogTrace(ctxt->logHandle, "0x%" PRIx64 " writableADUs set to %u, setting to %u",
                 pfa->flashAddress.bits, writableADUs, numADUs);
        atomic_store(&pfa->bstate->vData->writableADUs, numADUs);
    }

    assert(numADUs >= writtenADUs);
    padding = numADUs - writtenADUs;
    // include adus the device wrote (e.g. flush) in adusUsed
    if (padding)
    {
        uint64_t adusUsed = padding;

        adusUsed += atomic_fetch_add(&pfa->dstate->adusUsed, padding);
        LogTrace(ctxt->logHandle, "0x%" PRIx64 " padded with %u ADUs - %" PRId64 " ADUs in use",
                 pfa->flashAddress.bits, padding, adusUsed);
    }
}

void ssbClose(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    struct SSBState *ssbState = ctxt->ssbState;
    atomic_uint *state = &pfa->bstate->state;
    uint32_t expected = atomic_load(state);
    bool wasOpen = true;
    int retries = -1;
    uint32_t desired;

    do
    {
        retries++;
        wasOpen = true;
        if ((expected & kSSBStateMask) == kSSBStateOpen)
        {
            desired = expected - kSSBStateOpen + kSSBStateClosed;
        }
        else if (expected == kSSBStateAlloc)
        {
            // NLC runs with the block in the allocated state so it's closed
            // before it's opened.
            desired = expected - kSSBStateAlloc + kSSBStateClosed;
            wasOpen = false;
        }
        else
        {
            LogFatal(ctxt->logHandle, "Unexpected block state 0x%x", expected);
            abort();
        }
    } while (!atomic_compare_exchange_weak(state, &expected, desired));

    if (wasOpen)
    {
        atomic_fetch_sub(&ssbState->openBlocks, 1);
    }
    else
    {
        ssbIncInUseCount(ctxt, pfa);    // skipped open state
    }

    LogTrace(ctxt->logHandle, "Block 0x%lx closed", pfa->flashAddress.bits);

    // check if block is empty
    if (SSBRemoveRef(ctxt, pfa) == 1)
    {
#if CC_DEBUG_COUNTERS
        atomic_fetch_add(&ssbState->emptyNotClosed, 1);
#endif
    }

#if CC_DEBUG_COUNTERS
    if (retries)
    {
        atomic_fetch_add(&ssbState->closeRetry, retries);
    }
#endif
}

//
// called when notification from libsef indicates the block is closed
//
void SSBClosed(FTLContext *ctxt, struct SSBParsedFLA *pfa, uint32_t writtenADUs, uint32_t numADUs)
{
    // this can only happen if an allocated block is closed with no i/o sent
    // to it.  Happens when closing unreferenced/leaked blocks (while no i/o is
    // going on) so no race with the state changing while processing it.
    if (atomic_load(&pfa->bstate->state) == kSSBStateDeleted)
    {
        return;
    }

    ssbUpdateUsedAdus(ctxt, pfa, writtenADUs, numADUs);
    ssbClose(ctxt, pfa);
}

void SSBMarkAsMeta(FTLContext *ctxt, struct SSBParsedFLA *pfa)
{
    uint32_t state = atomic_load(&pfa->bstate->state) & kSSBStateMask;

    // Persistence doesn't unmark its blocks so if reallocated, pfa will point
    // to a block already marked as meta.  Ignore it.
    if (state != kSSBStateDeleted && state != kSSBStateMeta)
    {
        LogError(ctxt->logHandle, "Pdl flash address 0x%lx points to data (0x%x)",
                 pfa->flashAddress.bits, pfa->bstate->state);
        return;
    }
    pfa->bstate->state = kSSBStateMeta;
    return;
}

void SSBAllocated(FTLContext *ctxt,
                  struct SSBParsedFLA *pfa,
                  struct SEFPlacementID placementID,
                  uint32_t writableADUs)
{
    ssbSetStateAlloc(ctxt, pfa);
    pfa->bstate->placementID = placementID;
    // block can't be active
    assert(atomic_load(&pfa->bstate->vData->refCnt) == 1);
    pfa->bstate->vData->writableADUs = writableADUs;
    atomic_store(&pfa->bstate->vData->refCnt, 2);
}

int SSBClearAduValid(FTLContext *ctxt, struct SEFFlashAddress flashAddress)
{
    struct SSBState *ssbState = ctxt->ssbState;
    struct SSBValidMap *validPtr;
    struct SSBParsedFLA pfa;
    struct SEFStatus status;

    // get the superblock valid map
    status = SSBParseFlashAddress(ctxt, flashAddress, &pfa);
    if (status.error)
    {
        return status.error;
    }

    atomic_thread_fence(memory_order_acquire);
    validPtr = pfa.bstate->vData;
    assert(validPtr);

    // clear ADU's valid map bit & decrease refCnt, if 0 set as empty
    uint64_t bit = htole64((uint64_t)1 << (pfa.adu % 64));
    int64_t v = atomic_fetch_and(&validPtr->map[(pfa.adu) / 64], ~bit);

    assert(v & bit);

    // note: validPtr must be considered invalid after decrementing refCnt
    if (atomic_fetch_sub(&validPtr->refCnt, 1) == 1)
    {
        ssbSetStateEmpty(ctxt, &pfa);
    }
#if CC_DEMO_DP
    atomic_fetch_sub(&ssbState->validAdus, 1);
#endif

    return 0;
}

struct SEFSuperBlockList *SSBGetSuperBlockList(FTLContext *ctxt, struct SSBDomainState *dstate)
{
    struct SEFSuperBlockList *qosInfo = NULL;
    struct SEFStatus status = {-1, 0};
    ssize_t listSize = 0;

    do
    {
        if (listSize < status.info)
        {
            listSize = status.info;
            SUfree(qosInfo);
            qosInfo = SUzalloc(listSize);
            if (qosInfo == NULL)
            {
                LogError(ctxt->logHandle,
                         "Out of memory allocating %d super block records for domain %d",
                         (listSize - sizeof(struct SEFSuperBlockList)) /
                             (sizeof(struct SEFSuperBlockRecord)));
                break;
            }
        }
        status = SEFGetSuperBlockList(dstate->qosHandle, qosInfo, listSize);
        if (status.error)
        {
            LogError(ctxt->logHandle, "Failed to get qos info for domain %d (%u.%u)",
                     dstate->domainId.id, status.error, status.info);
            SUfree(qosInfo);
            qosInfo = NULL;
            break;
        }
    } while (status.info);

    return qosInfo;
}

static struct SEFStatus ssbFindLeakedBlocks(FTLContext *ctxt, struct SSBDomainState *dstate, bool release)
{
    struct SEFSuperBlockList *qosInfo = SSBGetSuperBlockList(ctxt, dstate);
    struct SSBState *ssbState = ctxt->ssbState;
    struct SEFStatus status = SUMakeStatusOk();
    uint64_t adusUsed = 0;
    int i;
    int cnt = 0;

    if (qosInfo == NULL)
    {
        return SUMakeStatus(-ENOMEM, 0);
    }
    for (i = 0; i < qosInfo->numSuperBlocks; i++)
    {
        struct SSBParsedFLA pfa;
        struct SEFFlashAddress flashAddress = qosInfo->superBlockRecords[i].flashAddress;
        struct SEFStatus status = SSBParseFlashAddress(ctxt, flashAddress, &pfa);
        if (status.error)
        {
            LogError(ctxt->logHandle, "Bad address 0x%lx from domain info (%d,%d)",
                     flashAddress.bits, status.error, status.info);
            break;
        }

        // Blocks open for copy by GC/Erase don't auto close, do that here
        if (release && qosInfo->superBlockRecords[i].state == kSuperBlockOpenedByErase)
        {
            SEFCloseSuperBlock(dstate->qosHandle, flashAddress);
            qosInfo->superBlockRecords[i].state = kSuperBlockClosed;
        }

        if (pfa.bstate->state == kSSBStateDeleted)
        {
            LogError(ctxt->logHandle, "Leaked block 0x%" PRIx64, flashAddress.bits);
            if (release)
            {
                if (qosInfo->superBlockRecords[i].state != kSuperBlockClosed)
                {
                    SEFCloseSuperBlock(dstate->qosHandle, flashAddress);
                }
                SEFReleaseSuperBlock(dstate->qosHandle, flashAddress);
            }
            cnt++;
        }
        if (pfa.bstate->state == kSSBStateClosed)
        {
            adusUsed += ssbGetWritableADUs(pfa.bstate);
#if CC_DEMO_DP
            if (pfa.bstate->vData)
            {
                atomic_fetch_add(&ssbState->validAdus, pfa.bstate->vData->refCnt);
            }
#endif
            LogTrace(ctxt->logHandle, "Closed block adusUsed now %" PRId64, adusUsed);
        }
    }

    SUfree(qosInfo);
    qosInfo = NULL;
    atomic_init(&dstate->adusUsed, adusUsed);
    status.info = cnt;
    return status;
}

struct SEFStatus SSBFindLeakedBlocks(FTLContext *ctxt, struct SSBState *ssbState, bool release)
{
    struct SEFStatus status = SUMakeStatusOk();
    int i;

    for (i = 0; i < ssbState->num_qos && !status.error; i++)
    {
        status = ssbFindLeakedBlocks(ctxt, ssbState->dstate + i, release);
    }
    return status;
}

SEFQoSHandle SSBGetQoSHandle(struct SSBDomainState *dstate)
{
    return dstate->qosHandle;
}

void SSBSetPatrol(struct SSBDomainState *dstate, bool patrol)
{
    dstate->needPatrol = patrol;
    if (patrol && dstate->gctx)
    {
        GCTriggerDomain(dstate->gctx, dstate);
    }
}

bool SSBGetPatrol(struct SSBDomainState *dstate)
{
    return dstate->needPatrol;
}

void SSBSetGC(struct SSBDomainState *dstate, struct gcContext *gctx)
{
    ssbForEachDomain(dstate) dstate->gctx = gctx;
}

struct SEFQoSDomainID SSBGetQoSID(struct SSBDomainState *dstate)
{
    return dstate->domainId;
}

uint32_t SSBGetNumAvailable(FTLContext *ctxt, struct SSBDomainState *dstate)
{
    uint64_t adusUsed = atomic_load(&dstate->adusUsed);

    if (ctxt->blockData.MaxAdus < adusUsed)
    {
        return 0;
    }
    return (ctxt->blockData.MaxAdus - adusUsed) / ctxt->superBlockCapacity;
}
