/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * manual-copy.h
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

/**
 * The manual copy API implements host side nameless copy.  It's
 * used by demos to show the difference in the amount of i/o doing it
 * manually vs the device's copy offload.
 */

/** @defgroup  SefSdkMcp Manual Copy API */
#ifndef MANUAL_COPY_H
#define MANUAL_COPY_H

#include <SEFAPI.h>

#include "log-manager.h"
#include "utils/instrumentation.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MCPHandle_ *MCPHandle;

/**
 *  @ingroup    SefSdkMcp
 *  @brief      This function is used to initialize the manual copy processor
 *
 *  @param      mcpHandle           A pointer to the manual copy handle
 *  @param      hInst               The handle for instrumentation
 *  @param      logHandle           The handle for the logger
 */
void MCPInit(MCPHandle *mcpHandle, INSHandle hInst, LogHandle logHandle);

/**
 *  @ingroup    SefSdkMcp
 *  @brief      This function cleans the manual copy and frees all used resources.
 *
 *  @param      mcpHandle           Manual Copy handle
 */
void MCPCleanup(MCPHandle mcpHandle);

/**
 *  @ingroup    SefSdkMcp
 *  @brief      Performs manual Copy with map or list; optional user address filtering.
 *
 *  @param      mcpHandle                    Manual Copy handle
 *  @param      srcQosHandle                 Handle to the source QoS Domain
 *  @param      copySource                   Physical addresses to copy
 *  @param      dstQosHandle                 Handle to the destination QoS Domain
 *  @param      copyDestination              Flash address of destination super block
 *  @param      filter                       Pointer to user address filter parameters,
 *                                           null indicates no filtering
 *  @param      overrides                    Pointer to overrides to scheduler parameters;
 *                                           pointer can be null for none required.
 *  @param      numAddressChangeRecords      Maximum number of ADUs to copy (size of
 *                                           SEFAddressChangeRequest userAddress array)
 *  @param      addressChangeInfo            Filled with changed addresses
 *
 *  @return     Status and info summarizing result
 */
struct SEFStatus MCPCopy(MCPHandle mcpHandle,
                         SEFQoSHandle srcQosHandle,
                         struct SEFCopySource copySource,
                         SEFQoSHandle dstQosHandle,
                         struct SEFFlashAddress copyDestination,
                         const struct SEFUserAddressFilter *filter,
                         const struct SEFCopyOverrides *overrides,
                         uint32_t numAddressChangeRecords,
                         struct SEFAddressChangeRequest *addressChangeInfo);

/**
 *  @ingroup    SefSdkMcp
 *  @brief      This function is the asynchronous version of MCPCopy().
 *
 *  @see        MCPCopy()
 *
 *  @param          mcpHandle    Manual Copy handle
 *  @param          qosHandle    Handle to the source QoS Domain
 *  @param[in,out]  iocb         For asynchronous response from SEF Library
 *                               Unused fields should be set to 0.
 */
void MCPCopyAsync(MCPHandle mcpHandle, SEFQoSHandle qosHandle, struct SEFNamelessCopyIOCB *iocb);

#ifdef __cplusplus
}
#endif

#endif /* MANUAL_COPY_H */
