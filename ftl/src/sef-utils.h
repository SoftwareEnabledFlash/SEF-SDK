/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-utils.h
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

/** @defgroup  SefSdkUtils SEF Utilities */
#pragma once

#ifndef SEF_UTILS_H
#define SEF_UTILS_H

#include <SEFAPI.h>
#include <stdlib.h>    // for the SEFxalloc fncs
#include <unistd.h>    // _exit

/* General Common Functions */

#ifndef NELEM
#define NELEM(I) (sizeof(I) / sizeof(I[0]))
#endif /* NELEM */

#ifndef MIN
#define MIN(a, b)               \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a < _b ? _a : _b;      \
    })
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)               \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b;      \
    })
#endif /* MAX */

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(a, b)      \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        (_a + _b - 1) / _b;     \
    })
#endif /* DIV_ROUND_UP */

#ifndef member_size
#define member_size(a, m) sizeof(((a *)0)->m)
#endif /* member_size */

#ifndef struct_size
/**
 * @brief size of struct p->m[n]
 *
 */
#define struct_size(p, m, n) (sizeof(*(p)) + sizeof(*(p)->m) * (n))
#endif /* struct_size */

#define SUzalloc(size) \
    (calloc(1, size) ?: (SEF_EXIT_ON_NO_Meme ? (_exit(EXIT_FAILURE), NULL) : NULL))
#define SUmalloc(size) (malloc(size) ?: (SEF_EXIT_ON_NO_Meme ? (_exit(EXIT_FAILURE), NULL) : NULL))
#define SUrealloc(ptr, size) \
    (realloc(ptr, size) ?: (SEF_EXIT_ON_NO_Meme ? (_exit(EXIT_FAILURE), NULL) : NULL))
#define SUfree(ptr) free(ptr)

#define SUMakeStatusOk()         SUMakeStatus(0, 0)
#define SUMakeStatusInfo(info)   SUMakeStatus(0, info)
#define SUMakeStatusError(error) SUMakeStatus(error, 0)

/**
 *  @ingroup    SefSdkUtils
 *  @brief      Function to create a status struct
 *
 *  @param      error            Status errno information
 *  @param      info             Additional context-based descriptive information
 *
 *  @return     Status and info based on the input
 */
static inline struct SEFStatus SUMakeStatus(int32_t error, int32_t info)
{
    return (struct SEFStatus){error, info};
}

/**
 *  @ingroup    SefSdkUtils
 *  @brief  Sets an error in a status struct if there currently isn't an error
 *
 *  @param          status           an instance of the status
 *  @param          error            Status errno information
 *  @param          info             Additional context-based descriptive information
 *
 *  @return     Status and info based on the input
 */
static inline struct SEFStatus SUSetStatus(struct SEFStatus status, int32_t error, int32_t info)
{
    return status.error ? status : SUMakeStatus(error, info);
}

/**
 * @ingroup    SefSdkUtils
 * @brief Returns a sef handle given a QoS domain handle
 *
 * @param qosHandle     Handle to a QoS Domain
 * @returns             Pointer to SEFHandle for the unit the passed in
 *                      QoS domain handle is open on.
 */
SEFHandle SUGetHandle(SEFQoSHandle qosHandle);

/**
 * @ingroup    SefSdkUtils
 * @brief Returns the SEF information given a QoS domain handle
 *
 * @param qosHandle     Handle to a QoS Domain
 * @returns             Pointer to SEFInfo struct for the unit the passed in
 *                      QoS domain handle is open on.
 */
const struct SEFInfo *SUGetInformation(SEFQoSHandle qosHandle);

/**
 * @ingroup    SefSdkUtils
 * @brief Returns the number of Dies in a super block given a QoS domain handle
 *
 * @param qosHandle     Handle to a QoS Domain
 * @retval  0           info member is the number of dies in a super block
 *          !0          Error code from fetching virtual device information
 */
struct SEFStatus SUGetSuperBlockDies(SEFQoSHandle qosHandle);

/*
 * Functions to get variably size info structures
 */

/**
 * @ingroup    SefSdkUtils
 * @brief Retuns the die list for a virtual device
 *
 * Caller is required to free the returned die list with SUfree
 *
 * @param hSef          Sef handle of a device
 * @param vid           Virtual device id to fetch a die list for
 * @param dieList       Pointer to dieList point to update with the die list
 *
 * @retval      0       *dieList now points to the dieList for vid
 * @retval      !0      *dieList set to 0
 */
struct SEFStatus SUGetDieList(SEFHandle hSef, struct SEFVirtualDeviceID vid, struct SEFDieList **dieList);

struct SIFLabel
{
    uint64_t data[2];
};

/**
 * @ingroup    SefSdkUtils
 * @brief Returns the label assigned to a QoS Domain
 *
 * The assigned label is stored in the last two root pointers of the QoS Domain.
 *
 * @param      sefHandle          Sef handle of a device
 * @param      qosId              QoS Domain Id of the device
 * @param[out] label              The label assigned to the QoS Domain
 *
 * @return     Status and info summarizing result.
 *
 * @retval    -ENODEV            Was unable to get SEF device information; The SEF Handle can be not valid
 * @retval    -EPROTONOSUPPORT   There are not enough root pointers to use; Two root pointers are required
 */
struct SEFStatus SUGetLabel(SEFHandle sefHandle, struct SEFQoSDomainID qosId, struct SIFLabel *label);

/**
 * @ingroup    SefSdkUtils
 * @brief Returns a list of the QoS Domains that are labeled with the given label
 *
 *  When list is NULL or insufficiently sized or bufferSize is 0, status.info
 *  returns the minimum buffer size for the complete list. The data that fits
 *  in an insufficiently sized buffer is valid but incomplete. The buffer must
 *  be at least the size of the list structure.
 *
 * @param      sefHandle          Sef handle of a device
 * @param      label              The label assigned to the QoS Domain
 * @param[out] list               Buffer for storing list of QoS Domains
 * @param      bufferSize         Buffer size
 *
 * @return     Status and info summarizing result.
 *
 * @retval    0                  info field returns the minimum buffer size if the buffer is
 * insufficient or NULL; otherwise, 0
 * @retval    -EINVAL            Input parameters are invalid
 */
struct SEFStatus SUGetQosDomainId(SEFHandle sefHandle,
                                  struct SIFLabel label,
                                  struct SEFQoSDomainList *list,
                                  int bufferSize);

/**
 * @ingroup    SefSdkUtils
 * @brief Sets the label for a given QoS Domain
 *
 * The assigned label is stored in the last two root pointers of the QoS Domain.
 * The label is only assigned if the last two root pointers are empty unless override is enabled.
 *
 * @param qosHandle     Handle to a QoS Domain
 * @param label         The label to be assigned to the QoS Domain
 * @param override      Should override the label if the QoS Domain is already labeled
 *
 * @return     Status and info summarizing result.
 *
 * @retval    -ENODEV            Was unable to get SEF device information; The SEF Handle can be not valid
 * @retval    -EPROTONOSUPPORT   There are not enough root pointers to use; Two root pointers are required
 * @retval    -EEXIST            The device is labeled and override flag was not enabled
 */
struct SEFStatus SUSetLabel(SEFQoSHandle qosHandle, struct SIFLabel label, int override);

/**
 *  @ingroup    SefSdkUtils
 *  @brief      This function creates the super block address given a flash address
 *
 *  @param      sefHandle        Handle to the QoS domain of the flash address
 *  @param      flashAddress     The flash address
 *
 *  @return     returns the super block's flash Address
 */
struct SEFFlashAddress SUFlashAddressSuperBlock(SEFQoSHandle qosHandle,
                                                struct SEFFlashAddress flashAddress);

#endif /* SEF_UTILS_H */
