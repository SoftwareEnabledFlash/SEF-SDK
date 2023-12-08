/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * persistence.h
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
/** @defgroup  SefSdkPdl Persistence API */

#ifndef PERSISTENCE_H
#define PERSISTENCE_H

#include <SEFAPI.h>
#include <stdbool.h>

#include "data-tree-object.h"
#include "log-manager.h"

enum PDLEncodingType { kPDLNoEncoding };

/**
 *  @ingroup    SefSdkPdl
 *  @brief      A composite key used to identify data
 */
struct PDLKey
{
    char Name[24]; /**< A name associated with the data */
    int Index;     /**< A index associated with the data */
};

/**
 *  @ingroup    SefSdkPdl
 *
 *  @brief      State for an object saved/restored by persistence
 */
struct PDLData
{
    struct PDLKey Key; /**< A unique used to identify the data */
    void *Obj;         /**< Pointer to the data */
    uint64_t ObjSize;  /**< Size of data */
    int EncodingType;  /**< Encoding used to stored the data to the flash memory */
    void *arg; /**< A void* pointer passed to the callback function (used to pass user context information) */
    void (*FlushCompleteFunc)(struct PDLData *,
                              int isFlushed); /**< Callback function that is called after flush or cleanup */
};

typedef struct PDLHandle_ *PDLHandle;

/**
 *  @ingroup    SefSdkPdl
 *  @brief      A unique identifier used to identify the persisted data
 */
struct PDLGuid
{
    char data[16]; /**< bytes of the guid */
};

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function is used to initialize the persistence layer.
 *
 *  @param[out] pdlHandlePtr        A pointer to the persistence handle to be used for access to persistence instance
 *  @param      sefHandle           SEF handle
 *  @param      qosState            QoS Domain handle
 *  @param      qosInfo             A pointer to the QoS Domain information
 *  @param      persistedGUID       The GUID used to identify the persisted data
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     0      Persistence successfully initialized; if info is -ENODATA, the root pointers were empty
 */
struct SEFStatus PDLInit(PDLHandle *pdlHandlePtr,
                         SEFHandle sefHandle,
                         QoSState qosState,
                         struct SEFQoSDomainInfo *qosInfo,
                         struct PDLGuid persistedGUID);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function adds the data to a queue to store to the flash memory.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *  @param      data                The data to be stored to the flash memory; the Obj memory should
 *                                  be accessible until the FlushCompleteFunc is called
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     -EINVAL         The input data value is invalid
 *  @retval     -EFAULT         The data was queued previously
 */
struct SEFStatus PDLQueueData(PDLHandle pdlHandle, struct PDLData *data);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function returns if the data queue to be flushed to the flash memory is empty.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @retval     true                There is no data queued to be flushed
 */
bool PDLQueueIsEmpty(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function reads the data queued to be flushed to the flash memory by calling PDLFlushToFlash.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *  @param      key                 Unique key associated with the queued data
 *  @param[out] data                Data that is retrieved from the queue
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     -EINVAL         The input handle is not valid
 *  @retval     -ENOENT         Was unable to find the data based on the key
 */
struct SEFStatus PDLReadQueue(PDLHandle pdlHandle, struct PDLKey key, struct PDLData *data);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function persists the queued data to the flash memory.
 *
 *  After each piece of data is stored, the completion function is called and the caller is in charge of cleaning the
 * memory.
 *
 *  @note It is the caller's responsibility to clean the previously persisted data by calling PDLFreeFlash.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     -EROFS         Can't flush the data to the flash
 */
struct SEFStatus PDLFlushToFlash(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function reads the data stored to the flash memory.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *  @param      key                 Unique key associated with the stored data
 *  @param[out] data                Data that is retrieved from the flash memory
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     -EBADF         The stored data couldn't be recovered correctly; if info is -EILSEQ,
 * the hash didn't match; if info is -EMSGSIZE, the size didn't match
 */
struct SEFStatus PDLReadFlash(PDLHandle pdlHandle, struct PDLKey key, struct PDLData *data);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function is used as a helper to iterate over super blocks that are used to store on the flash memory.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *  @param      callback            The function called for each data stored
 *  @param      arg                 A void pointer to be passed to the called function
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus PDLForEachSuperBlock(PDLHandle pdlHandle,
                                      struct SEFStatus (*callback)(void *, struct SEFFlashAddress),
                                      void *arg);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function releases the super blocks used by the persistence layer.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus PDLFreeFlash(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function cleans the persistence layer and frees all used resources.
 *
 *  If the queued data is not flushed to flash memory, the complete functions are called to clean up the queued data.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus PDLCleanup(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function returns if the stored data was marked as dirty, signaling an unclean shutdown.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @return     Status and info summarizing result. Will return 1 in the .info if dirty.
 */
struct SEFStatus PDLIsDirty(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function marks the root pointer as dirty to help detect unclean shutdown of the FTL.
 *
 *  The function sets the root pointer to dirty by changing QoS Domain in the flash address to zero.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     -EROFS         Can't flush the data to the flash
 *  @retval     0              Cleanup was successful, info denotes if the root pointer was set as clean
 */
struct SEFStatus PDLMarkDirty(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function marks the root pointer as clean.
 *
 *  The function sets the root pointer to clean by pointing to the location of the stored metadata table.
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval     -EROFS         Can't flush the data to the flash
 */
struct SEFStatus PDLMarkClean(PDLHandle pdlHandle);

/**
 *  @ingroup    SefSdkPdl
 *  @brief      This function would return if any data has been persisted
 *
 *  @param      pdlHandle           Persistence handle to be used for access to persistence instance
 *
 *  @retval     true                Data has been flushed to the device
 */
bool PDLIsFlushed(PDLHandle pdlHandle);

#endif /* PERSISTENCE_H */
