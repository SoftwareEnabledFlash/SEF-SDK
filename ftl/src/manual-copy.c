/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * manual-copy.c
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
#include "manual-copy.h"

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "flash-translation.h"
#include "log-manager.h"
#include "sef-utils.h"
#include "utils/dlist.h"
#include "utils/instrumentation.h"

#if CC_MANUAL_NLC

struct MCPHandle_
{
    pthread_t thread;

    int timeToDie;

    struct sefEvent queueEvent;
    atomic_int queuedNum; /**< Number of queued copy commands */
    TmaDList queuedCopy;
    pthread_mutex_t queueLock;

    int ioCounterId;
    atomic_uint_least64_t readMCPADU;  /**< Number of read ADUs by the MCP */
    atomic_uint_least64_t writeMCPADU; /**< Number of written ADUs by the MCP */
    INSHandle hInst;

    LogHandle logHandle;
};

struct MCPQueueCopy
{
    TmaDListEntry link;
    SEFQoSHandle qosHandle;
    struct SEFNamelessCopyIOCB *iocb;
};

static void *MCPTThread(void *arg)
{
    UTL_DECLARE_DLIST(work);
    struct SEFStatus status;
    MCPHandle mcpHandle = arg;
    struct MCPQueueCopy *entry;

    LogDebug(mcpHandle->logHandle, "MCP has started");
    do
    {
        // wait for a new copy to get queued
        SefEventWait(&mcpHandle->queueEvent);

        pthread_mutex_lock(&mcpHandle->queueLock);
        utl_DListAppendList(&work, &mcpHandle->queuedCopy);
        pthread_mutex_unlock(&mcpHandle->queueLock);

        while ((entry = utl_DListPopHeadAs(&work, struct MCPQueueCopy, link)))
        {
            struct SEFNamelessCopyIOCB *iocb = entry->iocb;

            if (mcpHandle->timeToDie)
            {
                SUfree(entry);
                atomic_fetch_sub(&mcpHandle->queuedNum, 1);
                break;
            }

            // do sync nameless copy
            status = MCPCopy(mcpHandle, entry->qosHandle, iocb->copySource, iocb->dstQosHandle,
                             iocb->copyDestination, iocb->filter, &iocb->overrides,
                             iocb->numAddressChangeRecords, iocb->addressChangeInfo);

            // do the Async callback
            iocb->common.status.info = status.info;
            iocb->common.status.error = status.error;
            if (iocb->common.complete_func != NULL)
            {
                iocb->common.complete_func(&iocb->common);
            }

            // remove from the queue
            SUfree(entry);
            atomic_fetch_sub(&mcpHandle->queuedNum, 1);
        }

    } while (!mcpHandle->timeToDie);

    while ((entry = utl_DListPopHeadAs(&work, struct MCPQueueCopy, link)))
    {
        SUfree(entry);
        atomic_fetch_sub(&mcpHandle->queuedNum, 1);
    }

    return NULL;
}

static void mcpRegisterCounters(MCPHandle mcpHandle)
{
    struct INSCounter a[] = {
        InstructionCounterDef(readMCPADU, "Number of read ADUs by the MCP", MCPHandle_, mcpHandle),
        InstructionCounterDef(writeMCPADU, "Number of written ADUs by the MCP", MCPHandle_, mcpHandle)};

    mcpHandle->ioCounterId = INSRegisterIoCounters(mcpHandle->hInst, a, NELEM(a), NULL, NULL);
}
void MCPInit(MCPHandle *mcpHandle, INSHandle hInst, LogHandle logHandle)
{
    // allocate memory the handle
    *mcpHandle = SUzalloc(sizeof(struct MCPHandle_));

    // init the handle
    (*mcpHandle)->timeToDie = 0;
    atomic_store(&(*mcpHandle)->queuedNum, 0);
    (*mcpHandle)->hInst = hInst;
    (*mcpHandle)->logHandle = logHandle;
    pthread_mutex_init(&(*mcpHandle)->queueLock, NULL);
    SefEventInit(&(*mcpHandle)->queueEvent, kSefEventFlagAutoReset);
    utl_DListInit(&(*mcpHandle)->queuedCopy);
    mcpRegisterCounters(*mcpHandle);

    if (pthread_create(&(*mcpHandle)->thread, NULL, MCPTThread, *mcpHandle))
    {
        return;
    }
}

void MCPCleanup(MCPHandle mcpHandle)
{
    int returnValue;

    // set the time to die and wait
    mcpHandle->timeToDie = 1;
    SefEventSet(&mcpHandle->queueEvent);

    // join the MCP thread
    returnValue = pthread_join(mcpHandle->thread, NULL);
    if (returnValue)
    {
        LogError(mcpHandle->logHandle, "Unable to join MCP thread while shutting down; (%d) - %s",
                 returnValue, strerror(returnValue));
    }

    // clean up the queue
    if (atomic_load(&mcpHandle->queuedNum))
    {
        struct MCPQueueCopy *entry;

        while ((entry = utl_DListPopHeadAs(&mcpHandle->queuedCopy, struct MCPQueueCopy, link)))
        {
            SUfree(entry);
            atomic_fetch_sub(&mcpHandle->queuedNum, 1);
        }
    }

    LogDebug(mcpHandle->logHandle, "MCP has ended");
    if (mcpHandle->ioCounterId >= 0)
    {
        INSUnRegisterIoCounters(mcpHandle->hInst, mcpHandle->ioCounterId);
    }
    SUfree(mcpHandle);
}

static void mcpWriteDone(struct SEFCommonIOCB *common)
{
    struct SEFWriteWithoutPhysicalAddressIOCB *iocb = (void *)common;

    SefEventSet(iocb->common.param1);
}

struct SEFStatus MCPCopy(MCPHandle mcpHandle,
                         SEFQoSHandle srcQosHandle,
                         struct SEFCopySource copySource,
                         SEFQoSHandle dstQosHandle,
                         struct SEFFlashAddress copyDestination,
                         const struct SEFUserAddressFilter *filter,
                         const struct SEFCopyOverrides *overrides,
                         uint32_t numAddressChangeRecords,
                         struct SEFAddressChangeRequest *addressChangeInfo)
{
    int i, sourceUserAddressesSize, continuesCopyNum, startIndex, startAdu, statusInfo;
    struct SEFSuperBlockInfo sourceSuperBlockInfo, destinationSuperBlockInfo, dstSuperblockInfo;
    struct SEFUserAddressList *sourceUserAddresses;
    struct SEFQoSDomainInfo qdInfo;
    struct SEFStatus status;
    SEFHandle sefHandle;
    uint32_t adu, adusLeft;
    struct SEFWriteOverrides writeOverrides = {0};
    struct SEFQoSDomainID qdId = SEFGetQoSHandleProperty(srcQosHandle, kSefPropertyQoSDomainID).qosID;

    if (copySource.format == kList)
    {
        return SUMakeStatusError(-EPROTONOSUPPORT);
    }

    // populate variables
    if (overrides != NULL)
    {
        writeOverrides.programWeight = overrides->programWeight;
    }

    // get sef handle
    sefHandle = SUGetHandle(srcQosHandle);

    // get Qos domain information
    status = SEFGetQoSDomainInformation(sefHandle, qdId, &qdInfo);
    if (status.error)
    {
        LogError(mcpHandle->logHandle, "Was unable to get QoS Domain %d information", qdId.id);
        return status;
    }

    // get source superblock info
    LogDebug(mcpHandle->logHandle, "Started to Copy for 0x%lx", copySource.srcFlashAddress.bits);
    status = SEFGetSuperBlockInfo(srcQosHandle, copySource.srcFlashAddress, 0, &sourceSuperBlockInfo);
    if (status.error)
    {
        LogError(mcpHandle->logHandle, "Was unable to get source superblockInfo 0x%lx",
                 copySource.srcFlashAddress.bits);
        return status;
    }

    // get destination superblock info
    status = SEFGetSuperBlockInfo(dstQosHandle, copyDestination, 0, &destinationSuperBlockInfo);
    if (status.error)
    {
        LogError(mcpHandle->logHandle, "Was unable to get source superblockInfo 0x%lx",
                 copyDestination.bits);
        return status;
    }

    // read user addresses of the source
    sourceUserAddressesSize = sizeof(struct SEFUserAddressList) +
                              sizeof(struct SEFUserAddress) * qdInfo.superBlockCapacity;
    sourceUserAddresses = SUzalloc(sourceUserAddressesSize);
    status = SEFGetUserAddressList(srcQosHandle, copySource.srcFlashAddress, sourceUserAddresses,
                                   sourceUserAddressesSize);
    if (status.error)
    {
        LogError(mcpHandle->logHandle, "Was unable to get source address list 0x%lx",
                 copySource.srcFlashAddress.bits);
        SUfree(sourceUserAddresses);
        return status;
    }

    startAdu = 0;
    startIndex = 0;
    adusLeft = destinationSuperBlockInfo.writableADUs;
    continuesCopyNum = 0;
    addressChangeInfo->numReadErrorADUs = 0;
    addressChangeInfo->numProcessedADUs = 0;
    addressChangeInfo->nextADUOffset = 0;
    uint32_t bitOffset = (uint32_t)le64toh(copySource.srcFlashAddress.bits) % 64;
    SEFParseFlashAddress(srcQosHandle, copySource.srcFlashAddress, NULL, NULL, &adu);

    for (i = bitOffset; i < copySource.arraySize * 64 &&
                        addressChangeInfo->numProcessedADUs < numAddressChangeRecords && adusLeft;
         i++, adu++)
    {
        struct iovec dataIov;
        struct SEFFlashAddress sourceFlashAddress;
        struct SEFUserAddress sourceUserAddress;
        void *metadata = NULL;
        int copySize, j;
        struct SEFPlacementID placementId;
        addressChangeInfo->nextADUOffset = i;
        uint64_t isToCopyMask = (uint64_t)1 << (i % 64);
        uint64_t isToCopy = htole64(le64toh(copySource.validBitmap[i / 64]) & isToCopyMask);

        if (isToCopy)
        {
            continuesCopyNum++;
            if (continuesCopyNum == 1)
            {
                startAdu = adu;
                startIndex = i - bitOffset;
            }

            if (i + 1 != (copySource.arraySize * 64) &&
                addressChangeInfo->numProcessedADUs + continuesCopyNum != numAddressChangeRecords &&
                continuesCopyNum != adusLeft)
            {
                struct SEFUserAddress nextUserAddress = SEFCreateUserAddress(
                    SEFGetUserAddressLba(sourceUserAddresses->userAddressesRecovery[adu]) + 1,
                    SEFGetUserAddressMeta(sourceUserAddresses->userAddressesRecovery[adu]));

                if (nextUserAddress.unformatted ==
                    sourceUserAddresses->userAddressesRecovery[adu + 1].unformatted)
                {
                    continue;
                }
            }
        }

        if (continuesCopyNum == 0)
        {
            continue;
        }

        // read data to be copied from source
        copySize = qdInfo.ADUsize.data * continuesCopyNum;
        dataIov.iov_base = SUzalloc(copySize);
        dataIov.iov_len = copySize;
        sourceFlashAddress.bits = htole64(le64toh(copySource.srcFlashAddress.bits) + startIndex);
        assert(startIndex < sourceUserAddresses->numADUs);
        sourceUserAddress = sourceUserAddresses->userAddressesRecovery[startAdu];
        metadata = qdInfo.ADUsize.meta == 0 ? NULL : SUmalloc(qdInfo.ADUsize.meta * continuesCopyNum);

        status = SEFReadWithPhysicalAddress(srcQosHandle, sourceFlashAddress, continuesCopyNum,
                                            &dataIov, 1, 0, sourceUserAddress, metadata, NULL);
        if (status.error)
        {
            LogError(mcpHandle->logHandle, "Was unable to read data for copy from 0x%lx",
                     sourceFlashAddress.bits);

            // update structure
            addressChangeInfo->numProcessedADUs += continuesCopyNum;
            addressChangeInfo->numReadErrorADUs += continuesCopyNum;
            continuesCopyNum = 0;

            // free data
            SUfree(metadata);
            SUfree(dataIov.iov_base);

            continue;
        }

        // update read counters
        atomic_fetch_add(&mcpHandle->readMCPADU, continuesCopyNum);

        // write data to the destination
        placementId.id = 0;
        struct SEFFlashAddress *permanentAddresses =
            SUzalloc(sizeof(struct SEFFlashAddress) * continuesCopyNum);
        struct SEFWriteWithoutPhysicalAddressIOCB iocb = {0};
        struct sefEvent done = SEFEVENT_NON_AUTORESET_INITIALIZER;
        iocb.common.param1 = &done;
        iocb.common.complete_func = mcpWriteDone;
        iocb.tentativeAddresses = permanentAddresses;
        iocb.overrides = overrides ? writeOverrides : (struct SEFWriteOverrides){0};
        iocb.flashAddress = copyDestination;
        iocb.userAddress = sourceUserAddress;
        iocb.metadata = metadata;
        iocb.iov = &dataIov;
        iocb.iovcnt = 1;
        iocb.placementID = placementId;
        iocb.numADU = continuesCopyNum;
        SEFWriteWithoutPhysicalAddressAsync(dstQosHandle, &iocb);
        SefEventWait(&done);
        status = iocb.common.status;

        if (status.error)
        {
            LogError(mcpHandle->logHandle, "Was unable to write data for copy from 0x%lx to 0x%lx",
                     sourceFlashAddress.bits, copyDestination.bits);

            SUfree(metadata);
            SUfree(dataIov.iov_base);
            SUfree(permanentAddresses);
            SUfree(sourceUserAddresses);
            return status;
        }

        // update write counters
        atomic_fetch_add(&mcpHandle->writeMCPADU, continuesCopyNum);

        // update list of new addressed
        for (j = 0; j < continuesCopyNum; j++)
        {
            int index = j + addressChangeInfo->numProcessedADUs;

            addressChangeInfo->addressUpdate[index].newFlashAddress = permanentAddresses[j];
            addressChangeInfo->addressUpdate[index].oldFlashAddress.bits =
                htole64(le64toh(sourceFlashAddress.bits) + j);
            addressChangeInfo->addressUpdate[index].userAddress =
                sourceUserAddresses->userAddressesRecovery[startAdu + j];
        }

        // reset counters
        addressChangeInfo->numProcessedADUs += continuesCopyNum;
        continuesCopyNum = 0;
        adusLeft = iocb.distanceToEndOfSuperBlock;

        // free resources
        SUfree(metadata);
        SUfree(dataIov.iov_base);
        SUfree(permanentAddresses);
    }

    // free resources
    SUfree(sourceUserAddresses);

    // get superblock info
    status = SEFGetSuperBlockInfo(dstQosHandle, copyDestination, 0, &dstSuperblockInfo);
    if (status.error)
    {
        return status;
    }

    // check to make sure the bitMap is all used
    statusInfo = kCopyConsumedSource;
    for (; i < copySource.arraySize * 64; i++)
    {
        uint64_t isToCopyMask = (uint64_t)1 << (i % 64);
        uint64_t isToCopy = htole64(le64toh(copySource.validBitmap[i / 64]) & isToCopyMask);

        if (isToCopy)
        {
            statusInfo = 0;
            break;
        }
    }

    // close superblock if it is full
    if (dstSuperblockInfo.writableADUs - dstSuperblockInfo.writtenADUs == 0)
    {
        LogDebug(mcpHandle->logHandle, "Closing the superblock because it's full 0x%lx",
                 copyDestination.bits);
        status = SEFCloseSuperBlock(dstQosHandle, copyDestination);
        if (status.error)
        {
            return status;
        }

        statusInfo |= kCopyClosedDestination;
    }

    addressChangeInfo->numADUsLeft = dstSuperblockInfo.writableADUs - dstSuperblockInfo.writtenADUs;
    addressChangeInfo->copyStatus = statusInfo;
    return SUMakeStatusInfo(statusInfo);
}

void MCPCopyAsync(MCPHandle mcpHandle, SEFQoSHandle qosHandle, struct SEFNamelessCopyIOCB *iocb)
{
    struct MCPQueueCopy *mcpQueueCopy;

    // prepare and push the item to queue
    mcpQueueCopy = SUzalloc(sizeof(struct MCPQueueCopy));
    utl_DListInitEntry(&mcpQueueCopy->link);
    mcpQueueCopy->qosHandle = qosHandle;
    mcpQueueCopy->iocb = iocb;

    pthread_mutex_lock(&mcpHandle->queueLock);
    utl_DListPushTail(&mcpHandle->queuedCopy, &mcpQueueCopy->link);
    pthread_mutex_unlock(&mcpHandle->queueLock);

    atomic_fetch_add(&mcpHandle->queuedNum, 1);

    // setting the event
    SefEventSet(&mcpHandle->queueEvent);
}

#endif
