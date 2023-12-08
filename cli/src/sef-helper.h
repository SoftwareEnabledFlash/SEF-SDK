/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-helper.h
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
#ifndef SEF_HELPER_H
#define SEF_HELPER_H

#include <stdbool.h>
#include "SEFAPI.h"
#include "sef-utils.h"

#define makeStatusOk()         makeStatus(0, 0)
#define makeStatusInfo(info)   makeStatus(0, info)
#define makeStatusError(error) makeStatus(error, 0)

/**
 *  @brief
 *
 *  @param          error            Status errno information
 *  @param          info             Additional context-based descriptive information
 *
 *  @return     Status and info based on the input
 */
static inline struct SEFStatus makeStatus(int32_t error, int32_t info)
{
    return (struct SEFStatus){error, info};
}

/**
 *  @brief  Sets an error in a status struct if there currently isn't an error
 *
 *  @param          status           an instance of the status
 *  @param          error            Status errno information
 *  @param          info             Additional context-based descriptive information
 *
 *  @return     Status and info based on the input
 */
static inline struct SEFStatus setStatus(struct SEFStatus status, int32_t error, int32_t info)
{
    return status.error ? status : makeStatus(error, info);
}

/**
 * @brief       Checks that there's already a QoS Domain with the given ID
 *
 * @param       sefHandle       Handle to the SEF Unit where QD will or does exist
 * @param       QoSDomainId     The QoS Domain ID in question
 *
 * @retval      true            The QoS Domain Id is in use and is valid
 **/
bool CSHIsQosDomainIdValid(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainId);

/**
 * @brief       Checks that there's already a Virtual Device with the given ID
 *
 * @param       sefHandle               Handle to the SEF Unit where VD will or does exist
 * @param       VirtualDeviceId         The Virtual Device Id in question
 *
 * @retval      true                    The Virtual Device Id is in use and is valid
 **/
bool CSHIsVirtualDeviceIdValid(SEFHandle sefHandle, struct SEFVirtualDeviceID VirtualDeviceId);

/**
 * @brief       Checks that there's already a Virtual Device with the given ID
 *
 * @param       sefHandle               Handle to the SEF Unit where VD will or does exist
 * @param       VirtualDeviceId         The Virtual Device Id in question
 *
 * @return     Status and info summarizing result.
 *
 * @retval     0            The QoS Domain Id associated with the given label is stored in the .info
 * @retval     ENXIO        Was unable to find any associated QoS Domains with given label
 * @retval     EBADF        More than one QoS Domain is associated with the given label
 **/
struct SEFStatus CSHGetQosDomainId(SEFHandle sefHandle, struct SIFLabel label);

/**
 * @brief       Creates a new I/O Control Block, populates it using supplied arguments, and writes it
 *
 * @param       qosHandle               Handle for the QoS Domain
 * @param       flashAddress            Physical address of the super block. 0xFFFFFFFFFFFFFFFF if auto allocate.
 * @param       placementID             Only valid if the flashAddress is auto allocated. A value from 0 to
 *                                      numPlacementIds–1 indicating what logical data group to place this data in.
 * @param       userAddress             FTL can store meta-data related to this operation by this field. For
 *                                      example, storing LBA address to bind to this write operation such as data tags.
 * @param       numADU                  Total amount of write data size calculated in ADU.  Maximum allowed is 64k ADUs.
 * @param       iov                     A pointer to the scatter gather list
 * @param       iovCnt                  The number of elements in the scatter gather list
 *  @param      metadata                Pointer to metadata to write with the data; The number of bytes per ADU
 *                                      required is SEFQoSDomainInfo::ADUsize.meta. May be NULL.
 * @param       permanentAddresses      Must allocate space for returned permanent addresses equal to 8*length
 * @param       distanceToEndOfSuperBlock   Indicates remaining size in ADU after this write operation. May be NULL.
 * @param       overrides               Overrides to scheduler parameters; pointer can be null for none required.
 *
 * @retval      Information about the SEF Unit
 **/
struct SEFStatus CSHWriteSync(SEFQoSHandle qosHandle,
                              struct SEFFlashAddress flashAddress,
                              struct SEFPlacementID placementID,
                              struct SEFUserAddress userAddress,
                              uint32_t numADU,
                              const struct iovec *iov,
                              uint16_t iovCnt,
                              const void *metadata,
                              struct SEFFlashAddress *permanentAddresses,
                              uint32_t *distanceToEndOfSuperBlock,
                              const struct SEFWriteOverrides *overrides);

#endif /* SEF_HELPER_H */
