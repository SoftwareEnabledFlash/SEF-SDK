/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * data-tree-object.c
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
#include "data-tree-object.h"

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SEFAPI.h"
#include "flash-translation.h"
#include "log-manager.h"
#include "sef-utils.h"
#include "utils/hashset.h"

#define CC_MAX_READ_IO_ADUS (1024 / 4) /* Limit to 1M */

/**
 * Persisted structure used to map from a single flash address to an arbitrarily
 * sized persisted object.  The size of an address tree is a single ADU. If all
 * the flash addresses can't fit IsLeaf will be 0 and the flash addresses point
 * to the next level of addressTree structures.  When IsLeaf is 1, the flash
 * addresses are of the persisted object.
 */
struct addressTree
{
    int8_t IsLeaf;
    int8_t reserved[7];
    uint64_t numFlashAddresses;
    struct SEFFlashAddress flashAddresses[];
};

static LogHandle dtoGetLogHandle(QoSState qosState)
{
    struct SEFProperty logKey;

    logKey = SEFGetQoSHandleProperty(qosState->qosDomainHandles[0], kSefPropertyPrivateData);
    return (logKey.type == kSefPropertyTypeNull) ? NULL : logKey.ptr;
}

static struct SEFStatus SEFAllocateManualSuperBlock(QoSState qosState,
                                                    struct SEFFlashAddress *flashAddress,
                                                    const struct SEFAllocateOverrides *overrides)
{
    int qosDomainIndex;
    LogHandle logHandle;

    logHandle = dtoGetLogHandle(qosState);

    qosState->lastusedQosDomainId.id++;
    if (qosState->lastusedQosDomainId.id > qosState->qosDomainIds[qosState->numDomains - 1].id)
    {
        qosState->lastusedQosDomainId = qosState->qosDomainIds[0];
    }

    qosDomainIndex = qosState->lastusedQosDomainId.id - qosState->qosDomainIds[0].id;

    if (qosDomainIndex >= qosState->numDomains || qosDomainIndex < 0)
    {
        LogInfo(logHandle,
                "The qos domain index is outside the bounds of the device; defaulting to base qos "
                "domain");
        qosDomainIndex = 0;
    }

    assert(qosState->blockType == kForWrite || qosState->blockType == kForPSLCWrite);
    return SEFAllocateSuperBlock(qosState->qosDomainHandles[qosDomainIndex], flashAddress,
                                 qosState->blockType, NULL, overrides);
}

/**
 * @brief Returns the next super block to use for dtoContinuousWrite
 *
 * Implements the round robin scheme of allocating super blocks across the set
 * of domains in qosState.  The caller passes in the current flash address,
 * and the next flash address to use is returned.  If the current flash address
 * still has space in it, the flash address is not updated.  If it's full, then
 * a new super block is allocated and return from the next domain.
 *
 * @param          sefHandle       Handle to the SEF Unit
 * @param          qosState        Handle to the QoS Domain
 * @param [in,out] flashAddress    Current super block passed in, SEFNullFlashAddress
 *                                 if this is the first call. The next super
 *                                 block to use is returned.
 * @param          overrides       Allocation weight to use when a new super
 *                                 block is allocated.
 * @param [out]    distanceToEndOfSuperBlock    Remaining writable ADUs in the
 *                                 returned super block.
 *
 * @return     Status and info summarizing result.
 *
 * @retval     0                   Success, flashAddress and distanceToEndOfSuperBlock
 *                                 are valid.
 * @retval     -ENOSPC             A QoS Domain is out of space
 */
static struct SEFStatus dtoGetSuperBlock(SEFHandle sefHandle,
                                         QoSState qosState,
                                         struct SEFFlashAddress *flashAddress,
                                         const struct SEFAllocateOverrides *overrides,
                                         uint32_t *distanceToEndOfSuperBlock)
{
    int qosDomainIndex;
    LogHandle logHandle;
    struct SEFSuperBlockInfo info;
    struct SEFStatus status;

    logHandle = dtoGetLogHandle(qosState);

    do
    {
        // allocate new superblock
        if (SEFIsNullFlashAddress(*flashAddress))
        {
            status = SEFAllocateManualSuperBlock(qosState, flashAddress, overrides);
            if (status.error)
            {
                LogError(logHandle, "Was unable to allocate a superblock");
                return status;
            }

            LogTrace(logHandle, "Allocating new superblock for the persistance 0x%lx",
                     flashAddress->bits);
        }

        // get the allocated superblock's info
        status = GetQosDomainIndex(sefHandle, qosState, *flashAddress);
        if (status.error)
        {
            LogError(logHandle, "qosState is corrupt");
            return status;
        }

        qosDomainIndex = status.info;
        status =
            SEFGetSuperBlockInfo(qosState->qosDomainHandles[qosDomainIndex], *flashAddress, 0, &info);
        if (status.error)
        {
            LogError(logHandle, "Was unable to get the allocated superblock's info");
            return status;
        }

        // ensure superblock has enough space
        if (info.writableADUs == info.writtenADUs)
        {
            LogTrace(logHandle, "Persistance superblock 0x%lx closed because it is full",
                     flashAddress->bits);
            *flashAddress = SEFNullFlashAddress;
        }

    } while (SEFIsNullFlashAddress(*flashAddress));

    *distanceToEndOfSuperBlock = info.writableADUs - info.writtenADUs;
    return SUMakeStatusOk();
}

/**
 * @brief Given the root address of an addressTree returns the leaf flash addresses
 *
 * Starting at rootAddress, reads in each level of an addressTree until the leaf
 * nodes are read, which are returned in outReadAddress.
 *
 * @param          sefHandle       Handle to the SEF Unit
 * @param          qosState        Handle to the QoS Domain
 * @param          rootAddress     Flash address of the root node of the addressTree
 *                                 to read.
 * @param          aduSize         ADU size for the domains used by qosState.
 * @param [out]    outAdusNeeded   Length of *outReadAddress
 * @param [out]    outReadAddress  Leaf flash addresses of the addressTree
 * @param [out]    usedSuperBlocks Hash set updated for each super block used by
 *                                 the addressTree itself
 * @return     Status and info summarizing result.
 *
 */
static struct SEFStatus dtoReadTree(SEFHandle sefHandle,
                                    QoSState qosState,
                                    struct SEFFlashAddress rootAddress,
                                    struct SEFADUsize aduSize,
                                    uint64_t *outAdusNeeded,
                                    struct SEFFlashAddress **outReadAddress,
                                    HashSet usedSuperBlocks)
{
    int i;
    int8_t IsLeaf;
    struct SEFStatus status;
    struct SEFFlashAddress superBlockAddress;
    struct SEFFlashAddress *readAddress;
    uint64_t ADUsNeeded;
    LogHandle logHandle;

    // prepare root
    IsLeaf = 0;
    ADUsNeeded = 1;
    readAddress = SUmalloc(sizeof(struct SEFFlashAddress));
    readAddress[0] = rootAddress;

    // add root to superblock list
    if (usedSuperBlocks != NULL)
    {
        superBlockAddress = SUFlashAddressSuperBlock(qosState->qosDomainHandles[0], readAddress[0]);
        HashSetAdd(usedSuperBlocks, superBlockAddress.bits);
    }

    logHandle = dtoGetLogHandle(qosState);

    while (!IsLeaf)
    {
        struct addressTree *readTree;
        uint64_t maxAduValue;

        // read the tree
        assert(ADUsNeeded);
        readTree = SUmalloc(ADUsNeeded * aduSize.data);
        status = dtoContinuousRead(sefHandle, qosState, readAddress, ADUsNeeded, readTree,
                                   SEFCreateUserAddress(0, SEF_LUT_META_TAG), NULL, aduSize);
        if (status.error)
        {
            LogError(logHandle, "couldn't read address tree");
            SUfree(readTree);
            SUfree(readAddress);
            return status;
        }

        // free the previous layers memory
        SUfree(readAddress);

        // validate the read tree (fail if 0 or math will overflow or ADUsNeeded don't match)
        maxAduValue = 1UL << SEFUserAddressLbaBits;    // because each UA is unique
        if (le64toh(readTree->numFlashAddresses) == 0 ||
            le64toh(readTree->numFlashAddresses) > maxAduValue ||
            ADUsNeeded !=
                DIV_ROUND_UP(le64toh(readTree->numFlashAddresses) * sizeof(struct SEFFlashAddress) +
                                 sizeof(struct addressTree),
                             aduSize.data))
        {
            LogError(logHandle, "stored tree has garbage values");
            SUfree(readTree);

            return SUMakeStatusError(-1);
        }

        // read the value from the stored tree
        IsLeaf = readTree->IsLeaf;
        ADUsNeeded = le64toh(readTree->numFlashAddresses);
        readAddress = SUmalloc(ADUsNeeded * sizeof(struct SEFFlashAddress));
        memcpy(readAddress, readTree->flashAddresses, ADUsNeeded * sizeof(struct SEFFlashAddress));

        // add to used superblock list
        if (usedSuperBlocks != NULL)
        {
            for (i = 0; i < ADUsNeeded; i++)
            {
                superBlockAddress =
                    SUFlashAddressSuperBlock(qosState->qosDomainHandles[0], readAddress[i]);
                HashSetAdd(usedSuperBlocks, superBlockAddress.bits);
            }
        }

        // free read's memory
        SUfree(readTree);
    }

    *outAdusNeeded = ADUsNeeded;
    *outReadAddress = readAddress;

    return SUMakeStatusOk();
}

struct SEFStatus dtoWrite(SEFHandle sefHandle,
                          QoSState qosState,
                          void *obj,
                          uint64_t objSize,
                          struct SEFADUsize aduSize,
                          struct SEFFlashAddress *activeSuperBlock,
                          struct SEFFlashAddress *treeRoot)
{
    struct SEFStatus status;
    struct SEFUserAddress userAddress = SEFCreateUserAddress(0, SEF_LUT_META_TAG);
    struct SEFFlashAddress *tentativeAddresses;    //, rootAddress;
    // struct SEFSuperBlockRecord superBlockInfo;
    uint64_t ADUsNeeded;
    LogHandle logHandle;
    int8_t isLeaf;

    logHandle = dtoGetLogHandle(qosState);

    // calculate the ADUs needed and round up the size to the nearest complete ADU count
    ADUsNeeded = DIV_ROUND_UP(objSize, aduSize.data);
    tentativeAddresses =
        (struct SEFFlashAddress *)SUmalloc(ADUsNeeded * sizeof(struct SEFFlashAddress));

    // write data to the device
    isLeaf = 1;
    status = dtoContinuousWrite(sefHandle, qosState, userAddress, ADUsNeeded, objSize, obj,
                                tentativeAddresses, NULL, aduSize, activeSuperBlock);
    if (status.error)
    {
        LogError(logHandle, "Was unable to store data tree");
        SUfree(tentativeAddresses);

        return status;
    }

    LogDebug(logHandle, "Stored starting at 0x%lx", tentativeAddresses[0]);
    // write the tree to the device
    while (ADUsNeeded > 1 || isLeaf)
    {
        uint32_t treeSize =
            DIV_ROUND_UP(ADUsNeeded * sizeof(struct SEFFlashAddress) + sizeof(struct addressTree),
                         aduSize.data) *
            aduSize.data;
        struct addressTree *tree = SUmalloc(treeSize);

        tree->IsLeaf = isLeaf;
        if (isLeaf)
        {
            isLeaf = 0;
        }
        tree->numFlashAddresses = htole64(ADUsNeeded);
        memcpy(tree->flashAddresses, tentativeAddresses, ADUsNeeded * sizeof(struct SEFFlashAddress));

        ADUsNeeded = DIV_ROUND_UP(
            ADUsNeeded * sizeof(struct SEFFlashAddress) + sizeof(struct addressTree), aduSize.data);
        struct SEFFlashAddress *tempAddress =
            (struct SEFFlashAddress *)SUmalloc(ADUsNeeded * sizeof(struct SEFFlashAddress));

        status = dtoContinuousWrite(sefHandle, qosState, userAddress, ADUsNeeded, treeSize, tree,
                                    tempAddress, NULL, aduSize, activeSuperBlock);
        if (status.error)
        {
            LogError(logHandle, "Was unable to store data's tree");
            SUfree(tree);
            SUfree(tempAddress);
            SUfree(tentativeAddresses);

            return status;
        }
        LogDebug(logHandle, "Stored %s of %u ADUs at 0x%lx", tree->IsLeaf ? "leaves" : "tree",
                 ADUsNeeded, tempAddress[0]);

        SUfree(tree);
        SUfree(tentativeAddresses);
        tentativeAddresses = tempAddress;
    }

    assert(ADUsNeeded == 1);
    *treeRoot = tentativeAddresses[0];
    SUfree(tentativeAddresses);

    return SUMakeStatusOk();
}

struct SEFStatus dtoRead(SEFHandle sefHandle,
                         QoSState qosState,
                         struct SEFFlashAddress rootAddress,
                         struct SEFADUsize aduSize,
                         void **object,
                         uint64_t *objSize)
{
    struct SEFStatus status;
    struct SEFUserAddress userAddress = SEFCreateUserAddress(0x0, SEF_LUT_META_TAG);
    struct SEFFlashAddress *readAddress;
    uint64_t ADUsNeeded;
    LogHandle logHandle;
    void *buffer;
    size_t size;

    logHandle = dtoGetLogHandle(qosState);

    // read the address tree until getting to the leaf
    status = dtoReadTree(sefHandle, qosState, rootAddress, aduSize, &ADUsNeeded, &readAddress, NULL);
    if (status.error)
    {
        return status;
    }

    // read the object
    size = ADUsNeeded * aduSize.data;
    buffer = SUmalloc(size);
    status = dtoContinuousRead(sefHandle, qosState, readAddress, ADUsNeeded, buffer, userAddress,
                               NULL, aduSize);
    if (status.error)
    {
        LogError(logHandle, "couldn't read tree leaf");
        SUfree(buffer);
        SUfree(readAddress);

        return status;
    }
    *objSize = size;
    *object = buffer;

    // free memory for the leaf layer
    SUfree(readAddress);

    return SUMakeStatusOk();
}

uint64_t dtoSizeOf(uint64_t objSize, struct SEFADUsize aduSize)
{
    uint64_t ADUsNeeded, totalADUNeeded = 0;

    assert((sizeof(struct SEFFlashAddress) + sizeof(struct addressTree)) < aduSize.data);

    // calculate the space needed for the object
    ADUsNeeded = DIV_ROUND_UP(objSize, aduSize.data);
    totalADUNeeded += ADUsNeeded;

    // calculate the space needed for the tree
    do
    {
        ADUsNeeded = DIV_ROUND_UP(
            ADUsNeeded * sizeof(struct SEFFlashAddress) + sizeof(struct addressTree), aduSize.data);
        totalADUNeeded += ADUsNeeded;
    } while (ADUsNeeded > 1);

    return totalADUNeeded;
}

struct SEFStatus dtoUsedSuperblock(SEFHandle sefHandle,
                                   QoSState qosState,
                                   struct SEFFlashAddress rootAddress,
                                   struct SEFADUsize aduSize,
                                   HashSet usedSuperBlocks)
{
    int i;
    struct SEFStatus status;
    struct SEFFlashAddress *readAddress;
    uint64_t ADUsNeeded;

    // read the address tree until getting to the leaf
    status = dtoReadTree(sefHandle, qosState, rootAddress, aduSize, &ADUsNeeded, &readAddress,
                         usedSuperBlocks);
    if (status.error)
    {
        return status;
    }

    // mark leaf superblocks for release
    for (i = 0; i < ADUsNeeded; i++)
    {
        struct SEFFlashAddress superBlockAddress;

        superBlockAddress = SUFlashAddressSuperBlock(qosState->qosDomainHandles[0], readAddress[i]);
        HashSetAdd(usedSuperBlocks, superBlockAddress.bits);
    }

    // free memory for the leaf layer
    SUfree(readAddress);

    return SUMakeStatusOk();
}

struct SEFStatus dtoContinuousWrite(SEFHandle sefHandle,
                                    QoSState qosState,
                                    struct SEFUserAddress userAddress,
                                    uint64_t numADU,
                                    uint64_t dataSize,
                                    void *data,
                                    struct SEFFlashAddress *tentativeAddresses,
                                    const struct SEFWriteOverrides *overrides,
                                    struct SEFADUsize aduSize,
                                    struct SEFFlashAddress *activeSuperBlock)
{
    struct SEFStatus status;
    LogHandle logHandle;
    uint64_t numWrittenADU;
    uint32_t distanceToEndOfSuperBlock;

    logHandle = dtoGetLogHandle(qosState);

    numWrittenADU = 0;
    while (numWrittenADU != numADU)
    {
        int qosDomainIndex;
        struct SEFFlashAddress *partAddress;
        struct iovec partIov[2];
        uint64_t partWriteSize, partWriteADU;
        int partIovCount;

        // get the active superblock
        status =
            dtoGetSuperBlock(sefHandle, qosState, activeSuperBlock, NULL, &distanceToEndOfSuperBlock);
        if (status.error)
        {
            LogError(logHandle, "Was unable to allocated a superblock");
            return status;
        }

        // calculate and prepare copy per superblock
        partWriteADU = distanceToEndOfSuperBlock;
        if (numWrittenADU + partWriteADU > numADU)
        {
            partWriteADU = numADU - numWrittenADU;
        }

        partWriteSize = partWriteADU * aduSize.data;
        partAddress = tentativeAddresses + numWrittenADU;
        partIovCount = 1;
        partIov[0].iov_base = data + (numWrittenADU * aduSize.data);
        partIov[0].iov_len = partWriteSize;

        // create an zeroed out iov if last write is smaller than one ADU
        if (numWrittenADU + partWriteADU == numADU && numADU * aduSize.data != dataSize)
        {
            partIovCount = 2;
            partIov[0].iov_len = dataSize - (numWrittenADU * aduSize.data);
            partIov[1].iov_len = (numADU * aduSize.data) - dataSize;
            partIov[1].iov_base = SUzalloc(partIov[1].iov_len);
        }

        status = GetQosDomainIndex(sefHandle, qosState, *activeSuperBlock);
        if (status.error)
        {
            LogError(logHandle, "Was unable to get domain index for superblock 0x%lx",
                     activeSuperBlock->bits);
            if (partIovCount == 2)
            {
                SUfree(partIov[1].iov_base);
            }
            return status;
        }

        qosDomainIndex = status.info;
        status = SEFWriteWithoutPhysicalAddress(
            qosState->qosDomainHandles[qosDomainIndex], *activeSuperBlock,
            (struct SEFPlacementID){0}, userAddress, partWriteADU, partIov, partIovCount, NULL,
            partAddress, &distanceToEndOfSuperBlock, overrides);
        if (status.error)
        {
            LogError(logHandle, "Was unable to write data to the superblock");
            if (partIovCount == 2)
            {
                SUfree(partIov[1].iov_base);
            }
            return status;
        }

        if (status.info != partWriteADU)
        {
            LogTrace(logHandle, "Expected to write %u but wrote %u", partWriteADU, status.info);
        }

        // update data
        numWrittenADU += status.info;
        userAddress = SEFCreateUserAddress(SEFGetUserAddressLba(userAddress) + status.info,
                                           SEFGetUserAddressMeta(userAddress));

        // check for closed super block
        if (distanceToEndOfSuperBlock == 0)
        {
            LogTrace(logHandle, "Persistance superblock 0x%lx closed because it is full",
                     activeSuperBlock->bits);
            (*activeSuperBlock) = SEFNullFlashAddress;
        }

        if (partIovCount == 2)
        {
            SUfree(partIov[1].iov_base);
        }
    }

    return SUMakeStatusOk();
}

struct SEFStatus dtoContinuousRead(SEFHandle sefHandle,
                                   QoSState qosState,
                                   struct SEFFlashAddress *flashAddress,
                                   uint64_t numADU,
                                   void *data,
                                   struct SEFUserAddress userAddress,
                                   const struct SEFReadOverrides *overrides,
                                   struct SEFADUsize aduSize)
{
    struct SEFStatus status;
    LogHandle logHandle;
    uint64_t numReadADU;

    logHandle = dtoGetLogHandle(qosState);

    numReadADU = 0;
    while (numReadADU < numADU)
    {
        struct iovec readIov;
        struct SEFQoSDomainID qosDomainId, expectedQosDomainId = {0};
        uint32_t blockNumber, expectedBlockNumber = 0;
        uint32_t aduOffset;
        uint64_t expectedADUOffset, continuousADU = 0;

        // calculate number of continuous ADUs
        SEFParseFlashAddress(qosState->qosDomainHandles[0], flashAddress[numReadADU + continuousADU],
                             &qosDomainId, &blockNumber, &aduOffset);
        do
        {
            expectedADUOffset = aduOffset + 1;
            continuousADU++;

            if (numReadADU + continuousADU < numADU)
            {
                SEFParseFlashAddress(qosState->qosDomainHandles[0],
                                     flashAddress[numReadADU + continuousADU], &expectedQosDomainId,
                                     &expectedBlockNumber, &aduOffset);
            }

        } while (aduOffset == expectedADUOffset && blockNumber == expectedBlockNumber &&
                 qosDomainId.id == expectedQosDomainId.id && continuousADU < CC_MAX_READ_IO_ADUS);

        uint32_t continuesSize = continuousADU * aduSize.data;
        readIov.iov_base = data + (numReadADU * aduSize.data);
        readIov.iov_len = continuesSize;

        int qosDomainIndex = qosDomainId.id - qosState->qosDomainIds[0].id;

        if (qosDomainIndex >= qosState->numDomains || qosDomainIndex < 0)
        {
            LogError(logHandle, "The data stored outside the bounds of the device");
            return SUMakeStatusError(-ENOENT);
        }

        status = SEFReadWithPhysicalAddress(qosState->qosDomainHandles[qosDomainIndex],
                                            flashAddress[numReadADU], continuousADU, &readIov, 1, 0,
                                            userAddress, NULL, overrides);
        if (status.error)
        {
            LogError(logHandle, "Was unable to read data from the superblock");
            return status;
        }

        numReadADU += continuousADU;
        userAddress = SEFCreateUserAddress(SEFGetUserAddressLba(userAddress) + continuousADU,
                                           SEFGetUserAddressMeta(userAddress));
    }

    return SUMakeStatusOk();
}

struct SEFStatus GetQosDomainIndex(SEFHandle sefHandle, QoSState qosState, struct SEFFlashAddress flashAddress)
{
    int qosDomainIndex;
    struct SEFQoSDomainID qosDomainId;
    LogHandle logHandle;

    logHandle = dtoGetLogHandle(qosState);

    // validate flash address
    if (SEFIsNullFlashAddress(flashAddress))
    {
        LogError(logHandle, "The flash address is zero");
        return SUMakeStatusError(-EINVAL);
    }

    // parse flash address to get the index
    SEFParseFlashAddress(qosState->qosDomainHandles[0], flashAddress, &qosDomainId, NULL, NULL);
    qosDomainIndex = qosDomainId.id - qosState->qosDomainIds[0].id;
    if (qosDomainIndex >= qosState->numDomains || qosDomainIndex < 0)
    {
        LogError(logHandle, "The flash address is outside the bounds of the device");
        return SUMakeStatusError(-ECHRNG);
    }

    return SUMakeStatusInfo(qosDomainIndex);
}
