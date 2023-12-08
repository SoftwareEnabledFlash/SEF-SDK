/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * data-tree-object.h
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

/** @defgroup  SefSdkDTO Data Tree Object */
#ifndef DATA_TREE_OBJECT_H
#define DATA_TREE_OBJECT_H

#include <stdint.h>

#include "SEFAPI.h"
#include "log-manager.h"
#include "utils/hashset.h"

/**
 * @brief QoS Domain state used by DTO i/o functions
 *
 * The DTO i/o functions support round-robin writes across multiple domains
 * to either native flash or pSLC.  The state in this structure control this
 * functionality.
 */
typedef struct QoSState
{
    int numDomains; /**< Length of qosDomainIds and qosDomainHandles arrays */
    struct SEFQoSDomainID lastusedQosDomainId; /**< Last domain id used when
                                                   allocating a super block */
    uint8_t blockType;                         /**< Type of super block to allocate (native/pSLC)*/

    struct SEFQoSDomainID *qosDomainIds; /**< Array of domain id's to write to */
    SEFQoSHandle *qosDomainHandles;      /**< Array of domain handles to write to */
} *QoSState;

/**
 *  @brief      This function writes data to a tree structure
 *
 *  The tree structure is used to write data
 *
 *  @param          sefHandle           Handle to the SEF Unit
 *  @param          qosState            Handle to the QoS Domain
 *  @param          obj                 A pointer to a memory location for data to be written
 *  @param          objSize             Size of data to be written in bytes
 *  @param          aduSize             Size of ADU in bytes
 *  @param[in,out]  activeSuperBlock    The address of the super block that is being written to.
 *                                      The super block is manually allocated by the function if
 *                                      it is set to zero or if it gets full.
 *  @param[out]     rootAddress         The flash address for the root of the tree
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus dtoWrite(SEFHandle sefHandle,
                          QoSState qosState,
                          void *obj,
                          uint64_t objSize,
                          struct SEFADUsize aduSize,
                          struct SEFFlashAddress *activeSuperBlock,
                          struct SEFFlashAddress *rootAddress);

/**
 *  @brief      This function loads the data stored in a tree structure
 *
 *  @param          sefHandle       Handle to the SEF Unit
 *  @param          qosState        Handle to the QoS Domain
 *  @param          rootAddress     The flash address for the root of the tree
 *  @param          aduSize         Size of ADU in bytes
 *  @param[out]     object          A pointer to the location of data read from the tree
 *  @param[out]     objSize         A pointer to the size of the read tree
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus dtoRead(SEFHandle sefHandle,
                         QoSState qosState,
                         struct SEFFlashAddress rootAddress,
                         struct SEFADUsize aduSize,
                         void **object,
                         uint64_t *objSize);

/**
 *  @brief      This function calculates number of ADUs needed to write data and the needed tree structure
 *
 *  @param          objSize         Size of data to be written in bytes
 *  @param          aduSize         Size of ADU in bytes
 *
 *  @return         returns the number of ADUs needed to write data and the needed tree structure
 */
uint64_t dtoSizeOf(uint64_t objSize, struct SEFADUsize aduSize);

/**
 *  @brief      This function returns a vector of all the super blocks used for the tree
 *
 *  @param          sefHandle       Handle to the SEF Unit
 *  @param          qosState        Handle to the QoS Domain
 *  @param          rootAddress     The flash address for the root of the tree
 *  @param          aduSize         Size of ADU in bytes
 *  @param[in, out] usedSuperBlocks A vector of used super blocks for the tree
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus dtoUsedSuperblock(SEFHandle sefHandle,
                                   QoSState qosState,
                                   struct SEFFlashAddress rootAddress,
                                   struct SEFADUsize aduSize,
                                   HashSet usedSuperBlocks);

/**
 *  @brief      This function writes data separate from any other write on a manually allocated
 * super block.
 *
 *  Unlike the built-in write function the data is separated and will not be mixed with other data.
 * This is achieved by using manually allocated Super blocks. The functionality offered is similar
 * to use of a Placement ID when using auto-allocation.
 *
 *  @param      sefHandle                  Handle to the SEF Unit
 *  @param      qosState                   Handle to the QoS Domain
 *  @param      userAddress                Stored data by the FTL
 *  @param      numADU                     Total amount of write data size calculated in ADU
 *  @param      dataSize                   Total amount of write data size in bytes. The dataSize
 *                                         can be lower than full number of ADUs, the extra space
 *                                         is padded with zeros.
 *  @param      data                       A pointer to a memory location for data to be written
 *  @param[out] tentativeAddresses         Must allocate space for returned tentative addresses
 *                                         equal to 8*length (e.g. 8*number of ADUs)
 *  @param      overrides                  Overrides to scheduler parameters; pointer can be null
 *                                         for none required.
 *  @param      aduSize                    Size of ADU in bytes
 *  @param[in,out] activeSuperBlock        The address of the super block that is being written to.
 *                                         The super block is manually allocated by the function
 *                                         if it is set to zero or if it gets full.
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus dtoContinuousWrite(SEFHandle sefHandle,
                                    QoSState qosState,
                                    struct SEFUserAddress userAddress,
                                    uint64_t numADU,
                                    uint64_t dataSize,
                                    void *data,
                                    struct SEFFlashAddress *tentativeAddresses,
                                    const struct SEFWriteOverrides *overrides,
                                    struct SEFADUsize aduSize,
                                    struct SEFFlashAddress *activeSuperBlock);

/**
 *  @brief      This function reads data given an array of flash addresses with minimal number of read commands.
 *
 *  The data is read using the least number of commands by issuing finding the most number of ADUs with no gap. The
 * caller doesn't have to worry if the flash addresses have a gap or if located across multiple super blocks.
 *
 *  @param      sefHandle        Handle to the SEF Unit
 *  @param      qosState         Handle to the QoS Domain
 *  @param      flashAddress     Array of Physical addresses for the read command. The data can be
 *                               located across multiple super blocks
 *  @param      numADU           Length of data to read (in ADUs)
 *  @param[out] data             A pointer to a memory location to store the read data
 *  @param      userAddress      Stored data by the FTL
 *  @param      overrides        Overrides to scheduler parameters; pointer can be null for none required
 *  @param      aduSize          Size of ADU in bytes
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus dtoContinuousRead(SEFHandle sefHandle,
                                   QoSState qosState,
                                   struct SEFFlashAddress *flashAddress,
                                   uint64_t numADU,
                                   void *data,
                                   struct SEFUserAddress userAddress,
                                   const struct SEFReadOverrides *overrides,
                                   struct SEFADUsize aduSize);

/**
 *  @brief      This function returns the index of the QoS Domain in QoSState
 *              for the given flash address.
 *
 *  @param          sefHandle       Handle to the SEF Unit
 *  @param          qosState        Handle to the QoS Domain
 *  @param          flashAddress    The flash address to find the index for
 *
 *  @return     Status and info summarizing result.
 *
 *  @retval    0              The info field is the Index of the QoS Domain
 *  @retval    -ECHRNG        The flash address is outside the bounds of the device
 *  @retval    -EINVAL        The flash address is zero
 */
struct SEFStatus GetQosDomainIndex(SEFHandle sefHandle,
                                   QoSState qosState,
                                   struct SEFFlashAddress flashAddress);

#endif /* DATA_TREE_OBJECT_H */
