/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * persistence.c
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
#include "persistence.h"

#include <errno.h>
#include <string.h>

#include "config.h"
#include "data-tree-object.h"
#include "flash-translation.h"
#include "log-manager.h"
#include "sef-utils.h"
#include "utils/crc32.h"
#include "utils/dlist.h"

/* Persisted info about a persisted object */
struct PDLStoredData
{
    struct PDLKey key; /* It's name */
    int EncodingType;  /* How object is encode (raw/none encoding is only option) */
    uint64_t objSize;  /* Object size in bytes */
    uint64_t hashSize; /* How many bytes of the object is CRCed */
    uint32_t objCrc;   /* CRC of the object up to hashSize */
    uint32_t reserved;
    struct SEFFlashAddress address; /* Root of the object tree */
};

struct PDLQueuedData
{
    struct PDLKey key;  /* Queue entry for items to persist */
    TmaDListEntry link; /* Link for being in the meta data queue */
    void *obj;          /* Object to persisted */
    uint64_t ObjSize;   /* Size of object in bytes */
    int EncodingType;   /* How object is to be encoded when saved (raw/none */
                        /* is only option) */
    void *arg;          /* Context for FlushCompleteFunc */
    void (*FlushCompleteFunc)(struct PDLData *data, int wasFlushed); /* Called */
    /* once object is persisted/flushed when wasFlushed is */
    /* 1, otherwise is 0 and object was not written. */
};

/* Holds state used to read/write persisted objects */
struct PDLHandle_
{
    int canFlush;                        /* Set to 1 when pdl handle is initialized */
    struct PDLStoredData *flashMetaData; /* Persistence meta data, an instance */
                                         /* of PDLStoreDataWrapper */
    int flashNum;                        /* Number of user objects in persistence meta data */
    TmaDList queueMetaData;              /* Linked list of user PDLStoreData to persist */
    int queueNum;                        /* Number of items in queueMetaData */
    SEFHandle sefHandle;                 /* Sef handle of device used for persistence */
    struct SEFFlashAddress rootPointer;  /* Location of flashMetaData on the SEF device */
    struct SEFADUsize ADUsize;           /* ADU size of sefHandle */
    QoSState qosState;                   /* State of domains used for persistence */
    LogHandle logHandle;                 /* LogHandle for logging errors */
    struct PDLGuid guid;                 /*GUID of the type of objects saved (e.g. SEFBlockGUID) */
};

/**
 * Persistence meta data - a flat directory of the object that have been
 * persisted.
 */
struct PDLStoreDataWrapper
{
    struct PDLGuid guid;                  /* GUID of the tpe of objects saved (e.g. SEFBlockGUID) */
    uint32_t objCrc;                      /* CRC of this structure */
    uint32_t flashNum;                    /* Number of items in flashMetaData */
    struct PDLStoredData flashMetaData[]; /* Meta data of stored objects */
};

/* Call back context for PDLFreeFlash() */
struct PDLFreeArg
{
    QoSState qosState;
    SEFHandle sefHandle;
};

/**
 * updates pdlHandle->rootPointer to be clean if not already clean.  See
 * makeDirtyRootPointer()
 */
static void makeCleanRootPointer(PDLHandle pdlHandle, struct SEFFlashAddress *cleanRootPointer)
{
    struct SEFQoSDomainID cleanQosDomain;
    struct SEFQoSDomainID qosDomainId;
    uint32_t blockNumber;
    uint32_t aduOffset;

    // check if it's already clean
    if (!PDLIsDirty(pdlHandle).info)
    {
        *cleanRootPointer = pdlHandle->rootPointer;
        return;
    }

    if (SEFIsEqualFlashAddress(pdlHandle->rootPointer, SEFAutoAllocate))
    {
        *cleanRootPointer = SEFNullFlashAddress;
        return;
    }

    // generate clean root pointer
    SEFParseFlashAddress(pdlHandle->qosState->qosDomainHandles[0], pdlHandle->rootPointer,
                         &qosDomainId, &blockNumber, &aduOffset);

    cleanQosDomain.id =
        qosDomainId.id - (pdlHandle->qosState->qosDomainIds[0].id + pdlHandle->qosState->numDomains);
    *cleanRootPointer = SEFCreateFlashAddress(pdlHandle->qosState->qosDomainHandles[0],
                                              cleanQosDomain, blockNumber, aduOffset);
}

/**
 * Updates the root pointer in pdlHandle to be dirty by adding the number of
 * domains in qosState to the qosId in the root pointer flash address.  This
 * process is reversed by makeCleanRootPointer.
 */
static void makeDirtyRootPointer(PDLHandle pdlHandle, struct SEFFlashAddress *dirtyRootPointer)
{
    struct SEFQoSDomainID dirtyQosDomain;
    struct SEFQoSDomainID qosDomainId;
    uint32_t blockNumber;
    uint32_t aduOffset;

    // check if it's already dirty
    if (PDLIsDirty(pdlHandle).info)
    {
        *dirtyRootPointer = pdlHandle->rootPointer;
        return;
    }

    if (SEFIsNullFlashAddress(pdlHandle->rootPointer))
    {
        *dirtyRootPointer = SEFAutoAllocate;
        return;
    }

    // generate dirty root pointer
    SEFParseFlashAddress(pdlHandle->qosState->qosDomainHandles[0], pdlHandle->rootPointer,
                         &qosDomainId, &blockNumber, &aduOffset);

    dirtyQosDomain.id =
        qosDomainId.id + pdlHandle->qosState->qosDomainIds[0].id + pdlHandle->qosState->numDomains;
    *dirtyRootPointer = SEFCreateFlashAddress(pdlHandle->qosState->qosDomainHandles[0],
                                              dirtyQosDomain, blockNumber, aduOffset);
}

/**
 * reads the persistence meta data (PDLStoredDataWrapper) using the
 * root pointer in pdlHandle storing it in the flashMetaData member
 */
struct SEFStatus readStoredMeta(PDLHandle pdlHandle)
{
    struct SEFStatus status;
    struct PDLStoreDataWrapper *storedNandDataWrapper;
    uint64_t readObjSize, flashMetaDataSize;
    struct SEFFlashAddress cleanRootPointer;
    uint32_t i, objCrc;

    // generate clean root pointer
    makeCleanRootPointer(pdlHandle, &cleanRootPointer);

    if (SEFIsNullFlashAddress(cleanRootPointer))
    {
        return SUMakeStatusOk();
    }

    // get SEF Block Module config
    status = dtoRead(pdlHandle->sefHandle, pdlHandle->qosState, cleanRootPointer,
                     pdlHandle->ADUsize, (void **)&storedNandDataWrapper, &readObjSize);
    if (status.error)
    {
        LogError(pdlHandle->logHandle, "Persistence data layer map is not retrieved correctly");
        return SUMakeStatusError(-ENOEXEC);
    }

    // change Endianess from Little to Host
    storedNandDataWrapper->objCrc = le32toh(storedNandDataWrapper->objCrc);
    storedNandDataWrapper->flashNum = le32toh(storedNandDataWrapper->flashNum);

    // check data validity
    flashMetaDataSize = storedNandDataWrapper->flashNum * sizeof(struct PDLStoredData);
    objCrc = utl_crc32c((unsigned char *)storedNandDataWrapper->flashMetaData, flashMetaDataSize);
    if (storedNandDataWrapper->objCrc != objCrc)
    {
        LogError(pdlHandle->logHandle,
                 "The hash of stored persistence data table doesn't match the expected hash");
        SUfree(storedNandDataWrapper);
        return SUMakeStatusError(-ENOEXEC);
    }

    // change meta Endianess from Little to Host
    for (i = 0; i < storedNandDataWrapper->flashNum; i++)
    {
        storedNandDataWrapper->flashMetaData[i].key.Index =
            le32toh(storedNandDataWrapper->flashMetaData[i].key.Index);
        storedNandDataWrapper->flashMetaData[i].EncodingType =
            le32toh(storedNandDataWrapper->flashMetaData[i].EncodingType);
        storedNandDataWrapper->flashMetaData[i].objSize =
            le64toh(storedNandDataWrapper->flashMetaData[i].objSize);
        storedNandDataWrapper->flashMetaData[i].hashSize =
            le64toh(storedNandDataWrapper->flashMetaData[i].hashSize);
        storedNandDataWrapper->flashMetaData[i].objCrc =
            le32toh(storedNandDataWrapper->flashMetaData[i].objCrc);
    }

    // check if the Unique Identifier is the same as a block module
    if (memcmp(storedNandDataWrapper->guid.data, pdlHandle->guid.data, sizeof(struct PDLGuid)))
    {
        LogError(pdlHandle->logHandle,
                 "Persistence signature doesn't match, not a sef block module");
        SUfree(storedNandDataWrapper);
        return SUMakeStatusError(-ENOEXEC);
    }

    // copy stored data
    pdlHandle->flashNum = storedNandDataWrapper->flashNum;
    pdlHandle->flashMetaData = SUmalloc(flashMetaDataSize);
    memcpy(pdlHandle->flashMetaData, storedNandDataWrapper->flashMetaData, flashMetaDataSize);

    SUfree(storedNandDataWrapper);

    return SUMakeStatusOk();
}

/**
 * Utility fnc used by PDLFlushTOFlash to write out the object described by
 * entry to flash filling in newMetaData and updating activeSuperBlock with the
 * next flash address to write to.
 */
struct SEFStatus flushQueueData(PDLHandle pdlHandle,
                                struct PDLQueuedData *entry,
                                struct PDLStoredData *newMetaData,
                                struct SEFFlashAddress *activeSuperBlock)
{
    struct SEFStatus status;

    LogDebug(pdlHandle->logHandle, "Persisting the %s (%d) - %ld ADUs", entry->key.Name,
             entry->key.Index, entry->ObjSize / pdlHandle->ADUsize.data);
    status = dtoWrite(pdlHandle->sefHandle, pdlHandle->qosState, entry->obj, entry->ObjSize,
                      pdlHandle->ADUsize, activeSuperBlock, &newMetaData->address);
    if (status.error)
    {
        LogError(pdlHandle->logHandle, "Was unable to store %s:%d during persistence",
                 entry->key.Name, entry->key.Index);
        return status;
    }

    // prep data for flushed array
    newMetaData->key = entry->key;
    newMetaData->EncodingType = entry->EncodingType;
    newMetaData->objSize =
        DIV_ROUND_UP(entry->ObjSize, pdlHandle->ADUsize.data) * pdlHandle->ADUsize.data;
    newMetaData->hashSize = entry->ObjSize;
    newMetaData->objCrc = utl_crc32c((unsigned char *)entry->obj, entry->ObjSize);

    // call the complete function
    if (entry->FlushCompleteFunc != NULL)
    {
        struct PDLData pdlData;

        pdlData.Key = entry->key;
        pdlData.Obj = entry->obj;
        pdlData.ObjSize = entry->ObjSize;
        pdlData.EncodingType = entry->EncodingType;
        pdlData.arg = entry->arg;
        pdlData.FlushCompleteFunc = entry->FlushCompleteFunc;

        entry->FlushCompleteFunc(&pdlData, 1);
    }

    return SUMakeStatusOk();
}

/**
 * Utility function used by PDLFlushToFlash to write out the persistence
 * meta data (newMetaData) returning the ADU it was stored in as the tree root.
 */
struct SEFStatus flushStoredMeta(PDLHandle pdlHandle,
                                 struct SEFFlashAddress *activeSuperBlock,
                                 struct PDLStoredData *newMetaData,
                                 uint32_t flashNum,
                                 struct SEFFlashAddress *treeRoot)
{
    struct SEFStatus status;
    struct PDLStoreDataWrapper *storedDataWrapper;
    uint64_t newMetaDataSize, storedDataWrapperSize;
    uint32_t i;

    newMetaDataSize = sizeof(struct PDLStoredData) * flashNum;
    storedDataWrapperSize = newMetaDataSize + sizeof(struct PDLStoreDataWrapper);
    storedDataWrapper = SUzalloc(storedDataWrapperSize);

    // store data in Little Endian
    for (i = 0; i < flashNum; i++)
    {
        storedDataWrapper->flashMetaData[i].key = newMetaData[i].key;
        storedDataWrapper->flashMetaData[i].EncodingType = htole32(newMetaData[i].EncodingType);
        storedDataWrapper->flashMetaData[i].objSize = htole64(newMetaData[i].objSize);
        storedDataWrapper->flashMetaData[i].hashSize = htole64(newMetaData[i].hashSize);
        storedDataWrapper->flashMetaData[i].objCrc = htole32(newMetaData[i].objCrc);
        storedDataWrapper->flashMetaData[i].address = newMetaData[i].address;
    }

    // populate the wrapper
    storedDataWrapper->flashNum = htole32(flashNum);
    storedDataWrapper->guid = pdlHandle->guid;
    storedDataWrapper->objCrc =
        htole32(utl_crc32c((unsigned char *)storedDataWrapper->flashMetaData, newMetaDataSize));

    // store metadata wrapper
    status = dtoWrite(pdlHandle->sefHandle, pdlHandle->qosState, (void **)storedDataWrapper,
                      storedDataWrapperSize, pdlHandle->ADUsize, activeSuperBlock, treeRoot);
    if (status.error)
    {
        SUfree(storedDataWrapper);
        return status;
    }

    LogDebug(pdlHandle->logHandle, "Persisted The metadata table at 0x%lx", (*treeRoot).bits);

    SUfree(storedDataWrapper);
    return SUMakeStatusOk();
}

struct SEFStatus PDLInit(PDLHandle *pdlHandlePtr,
                         SEFHandle sefHandle,
                         QoSState qosState,
                         struct SEFQoSDomainInfo *qosInfo,
                         struct PDLGuid persistedGUID)
{
    struct SEFProperty logKey;
    PDLHandle pdlHandle;

    // validate input
    if (sefHandle == 0)
    {
        return SUMakeStatusError(-EINVAL);
    }

    // allocate memory the handle
    *pdlHandlePtr = SUzalloc(sizeof(struct PDLHandle_));
    pdlHandle = *pdlHandlePtr;

    // populate the handle
    utl_DListInit(&pdlHandle->queueMetaData);
    pdlHandle->canFlush = 0;
    pdlHandle->flashMetaData = NULL;
    pdlHandle->flashNum = 0;
    pdlHandle->queueNum = 0;
    pdlHandle->sefHandle = sefHandle;
    pdlHandle->ADUsize = qosInfo->ADUsize;
    pdlHandle->rootPointer = SEFNullFlashAddress;
    pdlHandle->guid = persistedGUID;

    logKey = SEFGetQoSHandleProperty(qosState->qosDomainHandles[0], kSefPropertyPrivateData);
    pdlHandle->logHandle = logKey.type == kSefPropertyTypeNull ? NULL : logKey.ptr;
    pdlHandle->qosState = SUmalloc(sizeof(*pdlHandle->qosState));
    pdlHandle->qosState->numDomains = qosState->numDomains;
    pdlHandle->qosState->qosDomainIds = SUzalloc(sizeof(struct SEFQoSDomainID) * qosState->numDomains);
    pdlHandle->qosState->qosDomainHandles = SUzalloc(sizeof(SEFQoSHandle) * qosState->numDomains);
    pdlHandle->qosState->lastusedQosDomainId.id =
        qosState->qosDomainIds[0].id + qosState->numDomains - 1;
    pdlHandle->qosState->blockType = qosState->blockType;

    memcpy(pdlHandle->qosState->qosDomainIds, qosState->qosDomainIds,
           sizeof(struct SEFQoSDomainID) * qosState->numDomains);
    memcpy(pdlHandle->qosState->qosDomainHandles, qosState->qosDomainHandles,
           sizeof(SEFQoSHandle) * qosState->numDomains);
    pdlHandle->rootPointer = qosInfo->rootPointers[FTL_PDL_ROOT_INDEX];

    // enable flushing
    pdlHandle->canFlush = 1;

    if (SEFIsNullFlashAddress(pdlHandle->rootPointer))
    {
        LogInfo(pdlHandle->logHandle,
                "Empty root pointer can't load the persistence data layer map");
        return SUMakeStatusInfo(-ENODATA);
    }

    struct SEFFlashAddress cleanRootpointer;

    // generate clean root pointer
    makeCleanRootPointer(pdlHandle, &cleanRootpointer);

    // get last used qos domain Id
    SEFParseFlashAddress(pdlHandle->qosState->qosDomainHandles[0], cleanRootpointer,
                         &(pdlHandle->qosState->lastusedQosDomainId), NULL, NULL);

    // load stored metadata
    return readStoredMeta(pdlHandle);
}

bool PDLQueueIsEmpty(PDLHandle pdlHandle)
{
    return pdlHandle->queueNum == 0;
}

struct SEFStatus PDLQueueData(PDLHandle pdlHandle, struct PDLData *data)
{
    struct SEFStatus status;

    // validate data
    if (data->Obj == NULL || data->ObjSize == 0)
    {
        return SUMakeStatusError(-EINVAL);
    }

    // check if data is enqueued
    status = PDLReadQueue(pdlHandle, data->Key, data);
    if (!status.error)
    {
        return SUMakeStatusError(-EFAULT);
    }

    // enqueue data
    struct PDLQueuedData *newData = SUzalloc(sizeof(struct PDLQueuedData));
    utl_DListInitEntry(&newData->link);
    strcpy(newData->key.Name, data->Key.Name);
    newData->key.Index = data->Key.Index;
    newData->obj = data->Obj;
    newData->ObjSize = data->ObjSize;
    newData->EncodingType = data->EncodingType;
    newData->FlushCompleteFunc = data->FlushCompleteFunc;
    newData->arg = data->arg;

    utl_DListPushHead(&pdlHandle->queueMetaData, &newData->link);

    pdlHandle->queueNum++;
    return SUMakeStatusOk();
}

struct SEFStatus PDLFlushToFlash(PDLHandle pdlHandle)
{
    int qosDomainIndex;
    uint32_t flashNum;
    uint64_t newMetaDataSize;
    struct SEFStatus status;
    struct PDLQueuedData *entry = NULL;
    struct PDLStoredData *newMetaData;
    struct SEFFlashAddress activeSuperBlock, rootPointer;

    if (!pdlHandle->canFlush)
    {
        LogFatal(pdlHandle->logHandle, "Was unable to flush the data to the Persistence");
        return SUMakeStatusError(-EROFS);
    }

    // prep meta data wrapper
    newMetaDataSize = sizeof(struct PDLStoredData) * pdlHandle->queueNum;
    newMetaData = SUzalloc(newMetaDataSize);

    // flush queued data to the NAND
    flashNum = 0;
    activeSuperBlock = SEFNullFlashAddress;
    while ((entry = utl_DListPopHeadAs(&pdlHandle->queueMetaData, struct PDLQueuedData, link)))
    {
        status = flushQueueData(pdlHandle, entry, &newMetaData[flashNum], &activeSuperBlock);
        SUfree(entry);

        if (status.error)
        {
            SUfree(newMetaData);
            return status;
        }

        flashNum++;
        pdlHandle->queueNum--;
    }

    status = flushStoredMeta(pdlHandle, &activeSuperBlock, newMetaData, flashNum, &rootPointer);
    if (status.error)
    {
        SUfree(newMetaData);
        return status;
    }

    // close superblock
    if (!SEFIsNullFlashAddress(activeSuperBlock))
    {
        LogTrace(pdlHandle->logHandle, "Closing persistence superblock 0x%lx", activeSuperBlock.bits);

        status = GetQosDomainIndex(pdlHandle->sefHandle, pdlHandle->qosState, activeSuperBlock);
        if (status.error)
        {
            SUfree(newMetaData);
            return status;
        }

        qosDomainIndex = status.info;
        status = SEFCloseSuperBlock(pdlHandle->qosState->qosDomainHandles[qosDomainIndex],
                                    activeSuperBlock);
        if (status.error)
        {
            LogInfo(pdlHandle->logHandle, "Was unable to close the superblock %d/%d", status.error,
                    status.info);
        }
    }

    // store root pointer
    pdlHandle->rootPointer = rootPointer;
    status = SEFSetRootPointer(pdlHandle->qosState->qosDomainHandles[0], FTL_PDL_QOS_DOMAINS_INDEX,
                               (struct SEFFlashAddress){pdlHandle->qosState->numDomains});
    if (status.error)
    {
        LogError(pdlHandle->logHandle,
                 "Was unable to set the Number of QoS Domains in Root Pointer");
        SUfree(newMetaData);
        return status;
    }

    status =
        SEFSetRootPointer(pdlHandle->qosState->qosDomainHandles[0], FTL_PDL_ROOT_INDEX, rootPointer);
    if (status.error)
    {
        LogError(pdlHandle->logHandle, "Was unable to set the RootPointer");
        SUfree(newMetaData);
        return status;
    }

    LogInfo(pdlHandle->logHandle, "Stored Persistence Meta Table pointer at 0x%lx", rootPointer.bits);

    // update the global metadata struct
    if (pdlHandle->flashMetaData != NULL)
    {
        SUfree(pdlHandle->flashMetaData);
    }

    pdlHandle->flashMetaData = newMetaData;
    pdlHandle->flashNum = flashNum;

    return SUMakeStatusOk();
}

struct SEFStatus PDLReadQueue(PDLHandle pdlHandle, struct PDLKey key, struct PDLData *data)
{
    struct PDLQueuedData *entry = NULL, *foundEntry = NULL;

    if (PDLQueueIsEmpty(pdlHandle))
    {
        return SUMakeStatusError(-EINVAL);
    }

    // find the entry based on key and index
    while ((entry = utl_DListNextAs(&pdlHandle->queueMetaData, entry, struct PDLQueuedData, link)))
    {
        if (strcasecmp(key.Name, entry->key.Name) == 0 && key.Index == entry->key.Index)
        {
            foundEntry = entry;
        }
    }

    if (foundEntry == NULL)
    {
        return SUMakeStatusError(-ENOENT);
    }

    // populate the data object
    data->Key = key;
    data->Obj = foundEntry->obj;
    data->ObjSize = foundEntry->ObjSize;
    data->EncodingType = foundEntry->EncodingType;
    data->arg = foundEntry->arg;
    data->FlushCompleteFunc = foundEntry->FlushCompleteFunc;

    return SUMakeStatusOk();
}

struct SEFStatus PDLReadFlash(PDLHandle pdlHandle, struct PDLKey key, struct PDLData *data)
{
    struct SEFStatus status;
    int i;

    // check if key and index stored
    for (i = 0; i < pdlHandle->flashNum; i++)
    {
        if (!strcmp(key.Name, pdlHandle->flashMetaData[i].key.Name) &&
            key.Index == pdlHandle->flashMetaData[i].key.Index)
        {
            uint32_t objCrc;
            void *readObj;
            uint64_t readObjSize;

            // read flash data
            status = dtoRead(pdlHandle->sefHandle, pdlHandle->qosState,
                             pdlHandle->flashMetaData[i].address, pdlHandle->ADUsize, &readObj,
                             &readObjSize);
            if (status.error)
            {
                LogError(pdlHandle->logHandle, "Error loading data from the persistence layer");
                return status;
            }

            // check data validity
            objCrc = utl_crc32c((unsigned char *)readObj, pdlHandle->flashMetaData[i].hashSize);
            if (pdlHandle->flashMetaData[i].objCrc != objCrc)
            {
                LogError(pdlHandle->logHandle,
                         "The hash of stored object doesn't match the expected hash");
                return SUMakeStatus(-EBADF, -EILSEQ);
            }

            if (readObjSize != pdlHandle->flashMetaData[i].objSize)
            {
                LogError(pdlHandle->logHandle,
                         "The size of stored object doesn't match the expected size");
                return SUMakeStatus(-EBADF, -EMSGSIZE);
            }

            data->Key = pdlHandle->flashMetaData[i].key;
            data->EncodingType = pdlHandle->flashMetaData[i].EncodingType;
            data->Obj = readObj;
            data->ObjSize = readObjSize;

            return SUMakeStatusOk();
        }
    }
    return SUMakeStatusError(-ENOENT);
}

static struct SEFStatus pdlGetFlashBlocks(PDLHandle pdlHandle, HashSet blockVec)
{
    struct SEFStatus status;
    struct SEFFlashAddress cleanRootPointer;
    int i;

    if (!pdlHandle->flashNum)
    {
        return SUMakeStatusOk();
    }

    // get stored data's super blocks
    for (i = 0; i < pdlHandle->flashNum; i++)
    {
        status = dtoUsedSuperblock(pdlHandle->sefHandle, pdlHandle->qosState,
                                   pdlHandle->flashMetaData[i].address, pdlHandle->ADUsize, blockVec);
        if (status.error)
        {
            LogError(pdlHandle->logHandle,
                     "Error retrieving super blocks used for the persistence layer");
            return status;
        }
    }

    // ensure root pointer is clean
    makeCleanRootPointer(pdlHandle, &cleanRootPointer);

    // get metadata's super blocks
    status = dtoUsedSuperblock(pdlHandle->sefHandle, pdlHandle->qosState, cleanRootPointer,
                               pdlHandle->ADUsize, blockVec);
    if (status.error)
    {
        LogError(pdlHandle->logHandle,
                 "Error retrieving super blocks used for the persistence layer");
        return status;
    }

    return SUMakeStatusOk();
}

/**
 * Action used by PDLFreeFlash with PDLForEachSuperBlock to free all the super
 * blocks used by persistence.
 */
static struct SEFStatus pdlFreeBlock(void *arg, struct SEFFlashAddress flashAddress)
{
    struct SEFProperty logKey;
    LogHandle logHandle;
    int qosDomainIndex;
    struct SEFStatus status;
    struct PDLFreeArg *freeBlockArg = arg;

    // get log handle
    logKey =
        SEFGetQoSHandleProperty(freeBlockArg->qosState->qosDomainHandles[0], kSefPropertyPrivateData);
    logHandle = logKey.type == kSefPropertyTypeNull ? NULL : logKey.ptr;

    // release superblock
    status = GetQosDomainIndex(freeBlockArg->sefHandle, freeBlockArg->qosState, flashAddress);
    if (status.error)
    {
        LogError(logHandle, "Was unable to get QoS Domain Index");
        return status;
    }

    qosDomainIndex = status.info;
    status =
        SEFReleaseSuperBlock(freeBlockArg->qosState->qosDomainHandles[qosDomainIndex], flashAddress);
    if (status.error)
    {
        LogError(logHandle, "Error releasing the super block");
        return status;
    }

    LogTrace(logHandle, "Releasing superblock 0x%lx, used for the persistence layer", flashAddress.bits);
    return SUMakeStatusOk();
}

struct SEFStatus PDLForEachSuperBlock(PDLHandle pdlHandle,
                                      struct SEFStatus (*callback)(void *, struct SEFFlashAddress),
                                      void *arg)
{
    int i, numSuperBlocks;
    HashSet usedSuperblock;
    uint64_t *usedSuperBlocksArr;
    struct SEFStatus status;

    HashSetInit(&usedSuperblock);

    // get used super blocks
    status = pdlGetFlashBlocks(pdlHandle, usedSuperblock);
    if (status.error)
    {
        HashSetCleanup(usedSuperblock);
        return status;
    }

    // get the superblock array
    numSuperBlocks = HashSetCount(usedSuperblock);
    usedSuperBlocksArr = SUmalloc(sizeof(uint64_t) * numSuperBlocks);
    HashSetGetElems(usedSuperblock, usedSuperBlocksArr, sizeof(uint64_t) * numSuperBlocks);

    // call callback function for each superblock
    for (i = 0; i < numSuperBlocks; i++)
    {
        struct SEFFlashAddress usedSuperblockAddress = {usedSuperBlocksArr[i]};

        status = callback(arg, usedSuperblockAddress);
        if (status.error)
        {
            break;
        }
    }

    SUfree(usedSuperBlocksArr);
    HashSetCleanup(usedSuperblock);

    return SUMakeStatusOk();
}

struct SEFStatus PDLFreeFlash(PDLHandle pdlHandle)
{
    struct SEFStatus status;

    // free all used super blocks
    struct PDLFreeArg freeblockArg;
    freeblockArg.sefHandle = pdlHandle->sefHandle;
    freeblockArg.qosState = pdlHandle->qosState;

    status = PDLForEachSuperBlock(pdlHandle, pdlFreeBlock, &freeblockArg);
    if (status.error)
    {
        return status;
    }

    // store empty root pointer
    status = SEFSetRootPointer(pdlHandle->qosState->qosDomainHandles[0], FTL_PDL_ROOT_INDEX,
                               SEFNullFlashAddress);
    if (status.error)
    {
        LogError(pdlHandle->logHandle, "Was unable to empty the root pointer");
        return status;
    }

    // reset global data
    pdlHandle->rootPointer = SEFNullFlashAddress;
    if (pdlHandle->flashNum)
    {
        SUfree(pdlHandle->flashMetaData);
        pdlHandle->flashMetaData = NULL;
        pdlHandle->flashNum = 0;
    }

    return SUMakeStatusOk();
}

struct SEFStatus PDLCleanup(PDLHandle pdlHandle)
{
    struct PDLQueuedData *entry = NULL;

    if (pdlHandle == NULL)
    {
        return SUMakeStatusOk();
    }

    // remove queue list
    while ((entry = utl_DListPopHeadAs(&pdlHandle->queueMetaData, struct PDLQueuedData, link)))
    {
        // call the complete function
        if (entry->FlushCompleteFunc != NULL)
        {
            struct PDLData pdlData;

            pdlData.Key = entry->key;
            pdlData.Obj = entry->obj;
            pdlData.ObjSize = entry->ObjSize;
            pdlData.EncodingType = entry->EncodingType;
            pdlData.arg = entry->arg;
            pdlData.FlushCompleteFunc = entry->FlushCompleteFunc;

            entry->FlushCompleteFunc(&pdlData, 0);
        }

        SUfree(entry);
    }

    // clean up the memory
    if (pdlHandle->flashMetaData != NULL)
    {
        SUfree(pdlHandle->flashMetaData);
    }

    // clean up copy of qosState
    if (pdlHandle->qosState->numDomains)
    {
        SUfree(pdlHandle->qosState->qosDomainIds);
        SUfree(pdlHandle->qosState->qosDomainHandles);
    }

    SUfree(pdlHandle->qosState);

    SUfree(pdlHandle);

    return SUMakeStatusOk();
}

struct SEFStatus PDLIsDirty(PDLHandle pdlHandle)
{
    struct SEFQoSDomainID expectedQosDomain;

    if (SEFIsNullFlashAddress(pdlHandle->rootPointer))
    {
        return SUMakeStatusOk();
    }

    if (!pdlHandle->canFlush)
    {
        return SUMakeStatusError(-EROFS);
    }

    // check for dirty root pointer
    if (SEFIsEqualFlashAddress(SEFAutoAllocate, pdlHandle->rootPointer))
    {
        return SUMakeStatus(0, 1);
    }

    SEFParseFlashAddress(pdlHandle->qosState->qosDomainHandles[0], pdlHandle->rootPointer,
                         &expectedQosDomain, NULL, NULL);
    return SUMakeStatus(0, expectedQosDomain.id > pdlHandle->qosState->numDomains +
                                                      pdlHandle->qosState->qosDomainIds[0].id);
}

struct SEFStatus PDLMarkDirty(PDLHandle pdlHandle)
{
    struct SEFStatus status;
    struct SEFFlashAddress dirtyRootpointer;

    if (!pdlHandle->canFlush)
    {
        return SUMakeStatusError(-ESRCH);
    }

    // generate dirty root pointer
    makeDirtyRootPointer(pdlHandle, &dirtyRootpointer);

    // store dirty root pointer
    status = SEFSetRootPointer(pdlHandle->qosState->qosDomainHandles[0], FTL_PDL_ROOT_INDEX,
                               dirtyRootpointer);
    if (status.error)
    {
        LogError(pdlHandle->logHandle, "Was unable to set the RootPointer marked as dirty");
        return status;
    }

    pdlHandle->rootPointer = dirtyRootpointer;
    LogInfo(pdlHandle->logHandle, "Marked Root pointer as dirty as 0x%lx", pdlHandle->rootPointer.bits);
    return SUMakeStatusOk();
}

struct SEFStatus PDLMarkClean(PDLHandle pdlHandle)
{
    struct SEFStatus status;
    struct SEFFlashAddress cleanRootpointer;

    if (!pdlHandle->canFlush && !PDLIsDirty(pdlHandle).info)
    {
        return SUMakeStatusError(-EROFS);
    }

    // generate clean root pointer
    makeCleanRootPointer(pdlHandle, &cleanRootpointer);

    // store clean root pointer
    status = SEFSetRootPointer(pdlHandle->qosState->qosDomainHandles[0], FTL_PDL_ROOT_INDEX,
                               cleanRootpointer);
    if (status.error)
    {
        LogError(pdlHandle->logHandle, "Was unable to set the RootPointer marked as clean");
        return status;
    }

    pdlHandle->rootPointer = cleanRootpointer;
    LogInfo(pdlHandle->logHandle, "Marked Root pointer as clean as 0x%lx", pdlHandle->rootPointer.bits);

    return SUMakeStatusOk();
}

bool PDLIsFlushed(PDLHandle pdlHandle)
{
    return !SEFIsNullFlashAddress(pdlHandle->rootPointer);
}
