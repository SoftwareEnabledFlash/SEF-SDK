/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-block-module.h
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
/** @defgroup  SefSdkApi SEF FTL Public API */

#ifndef SEF_BLOCK_MODULE_H
#define SEF_BLOCK_MODULE_H

#include <SEFAPI.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/uio.h>

#ifdef __cplusplus
#include <atomic>
using std::atomic_int;
#else /* not __cplusplus */
#ifdef __STDC_NO_ATOMICS__
#error C11 atomics are required
#endif /* __STDC_NO_ATOMICS__ */

#include <stdatomic.h>
#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct LogHandle_ *LogHandle;
typedef struct FTLContext_ *SEFBlockHandle;

/**
 * @ingroup    SefSdkApi
 * @brief      Type of i/o in a SEFMultiContext
 */
enum SEFBlockIOType {
    kSEFRead, /**< Performs a read operation */
    kSEFWrite /**< Performs a write operation */
} PACKED;

/**
 * @ingroup    SefSdkApi
 * @brief      I/O flags in a SEFMultiContext
 */
enum SEFBlockIOFlags {
    kSEFBlockIOFlagNotifyBufferRelease = 0x01 /**< Buffer lifetime is controlled by caller */
};

/**
 * @ingroup    SefSdkApi
 * @brief      I/O request for SEFBlockIO()
 */
struct SEFMultiContext
{
    SEFBlockHandle blockHandle; /**< SEF Block handle to be used for access to the block instance */
    struct SEFMultiContext *parent; /**< A pointer to an instance of SEFMultiContext used for compound operations */
    void (*completion)(struct SEFMultiContext *); /**< The function that is called when the transaction is completed */
    void *arg;    /**< A pointer that can be used by caller for any reason */
    uint64_t lba; /**< Logical block address */
    uint32_t lbc; /**< Logical block count */
    struct
    {
        uint8_t ioType : 1; /**< enum SEFBlockIOType that needs to be performed */
        uint8_t flags : 7;  /**< I/O flags enum SEFBlockIOFlags */
    };
    uint8_t readQueue;                 /**< kSEFRead queue override when valid */
    uint16_t ioWeight;                 /**< I/O weight to override when non-zero */
    struct iovec *iov;                 /**< A pointer to the scatter/gather list */
    size_t iovOffset;                  /**< Starting byte offset into iov array */
    int iovcnt;                        /**< The number of elements in the scatter/gather list */
    uint16_t qosIndex;                 /**< 0 based, used for multi-domain FTL */
    struct SEFPlacementID placementID; /**< Placement ID for writes */
    uint32_t numLbl;                   /**< Num logical blocks left in the super block */
    atomic_int transferred; /**< Counter denoting number of bytes transferred for the transaction */
    atomic_int count;       /**< Reference count, I/O is completed -> 0 */
    atomic_int error;       /**< First error for the transaction */
    int cancel;             /**< Set to indicate cancel in progress */
};

/**
 * @ingroup    SefSdkApi
 * @brief      Event types for SEFBlockNotify
 */
enum SEFBlockNotifyType {
    kSefBlockNotifyGCDown,        /**< GC can no longer make progress, write I/O will fail */
    kSefBlockNotifyBufferRelease, /**< A portion of an I/O buffer can be released */
#ifdef UNIT_TEST
    kSefBlockNotifyReadError,
#endif
};

/**
 * @ingroup    SefSdkApi
 * @brief      Event data sent to a client's notification function.
 *
 * The notification function is set when SEFBlockInit() is called.
 */
struct SEFBlockNotify
{
    enum SEFBlockNotifyType type; /**< Type of notification */
    union
    {
        struct
        {
            const struct iovec *iov; /**< Vector of buffers to release */
            int16_t iovcnt;          /**< Count of buffers in iov */
        }; /**< kSefBlockNotifyBufferRelease */
    };
};

/**
 * @ingroup    SefSdkApi
 * @brief      Init time options.
 *
 * Initialization time options supplied to SEFBlockInit()
 */
struct SEFBlockOption
{
    int logLevel;                    /**< Initial log level (Trace = 0 , Debug = 1,
                                          Info = 2, Error = 3, Fatal = 4) */
    LogHandle logHandle;             /**< Use an external logger */
    const char *instrumentationPath; /**< The location for the Unix Domain Socket.
                                          Will replace the first two format specifier with UnitIndex and QoSDomainId.
                                          Defaults to /tmp/SEFFTLDomain.[UnitIndex].[QoSDomainId] */
    void *notifyContext;             /**< User context passed to notifyFunc() */
    void (*notifyFunc)(struct SEFBlockNotify event,
                       void *notifyContext); /**< Set to receive notifications */
    bool delayMount; /**< When true, SEFBlockInit() delays mounting until SEFBlockMount() is called */
};

/**
 * @ingroup    SefSdkApi
 * @brief      Configuration when calling SEFBlockConfig()
 */
struct SEFBlockConfig
{
    int overprovisioning; /**< Percentage of over-provisioning (e.g., 20 for 20 percent) */
    int unused0;          /**< Unused member */
    int numDomains;       /**< Number of domains to use for a block-device.
                               0 indicates all defined domains */
    struct SEFBlockOption blockOption; /**< Run time options while configuring */
};

/**
 * @ingroup    SefSdkApi
 * @brief      This function is used to configure the SEF Block Module.
 *
 *  It configures freshly created QoS domains so they can used as a SEF Block
 *  Module domain.  It sets the number of QoS domains and the amount of over
 *  provisioning to use.  Once these values are set, they cannot be changed
 *  without erasing or recreating the QoS domains.
 *
 *  A QoS domain must be configured as a SEF Block Module domain before calling
 *  SEFBlockModuleInit().
 *
 * @see        SEFBlockInit()
 *
 * @param      SEFUnitIndex         The index of the SEF Unit; the index is zero-based
 * @param      QoSDomainID          QoS Domain ID
 * @param      config               A pointer to an instance of SEFBlockConfig, for config settings
 *
 * @return     Status and info summarizing result.
 *
 * @retval    -EINVAL          Invalid over provisioning percentage
 * @retval    -EEXIST          The device is already configured, can not reconfigure a pre-configured device
 * @retval    -EBADF           The block module was not shutdown cleanly; Consider running Check Disk
 */
struct SEFStatus SEFBlockConfig(uint16_t SEFUnitIndex,
                                struct SEFQoSDomainID QoSDomainID,
                                struct SEFBlockConfig *config);

/**
 * @ingroup    SefSdkApi
 * @brief      This function is used to get a SEFBlockModuleHandle for issuing I/O.
 *
 * The QoS domain must be configured as a SEF Block Module domain using
 * SEFBlockConfig() before calling SEFBlockInit().  Configuration only needs to
 * be done once.
 *
 * @see        SEFBlockConfig()
 *
 * @param      SEFUnitIndex         The index of the SEF Unit; the index is zero-based
 * @param      QoSDomainID          QoS Domain ID
 * @param      options              A pointer to an instance of SEFBlockOption, for runtime options
 * @param      blockHandle          A pointer to the SEF Block handle to be used for access to the block instance
 *
 * @return     Status and info summarizing result.
 *
 * @retval    -EINVAL          Invalid log level
 * @retval    -EBADF           The block module was not shutdown cleanly; Consider running Check Disk
 * @retval    -ENOENT          Block layer has not been configured with SEFBlockConfig()
 */
struct SEFStatus SEFBlockInit(uint16_t SEFUnitIndex,
                              struct SEFQoSDomainID QoSDomainID,
                              struct SEFBlockOption *options,
                              SEFBlockHandle *blockHandle);

/**
 * @ingroup    SefSdkApi
 * @brief      This function mounts the FTL configured domain if not already mounted.
 *
 * It's required to be called before calling SEFBlockIO() when SEFBlockInit() was
 * called with the option delayMount set to true.  Calling it when it's not
 * required has no effect.
 *
 * @see         SEFBlockInit() SEFBlockIO()
 *
 * @param       blockHandle     SEF Block handle to be used for access to the block instance
 *
 * @return      Status and info summarizing result.
 */
struct SEFStatus SEFBlockMount(SEFBlockHandle blockHandle);

/**
 * @ingroup    SefSdkApi
 * @brief      Information about a block-module device
 */
struct SEFBlockInfo
{
    struct SEFADUsize aduSize; /**< Size of ADU and Meta */
    // int16_t aduShift;           /**< log2 of aduSize */
    int16_t numPlacementIDs; /**< Number of Placement IDs */
    int superBlockSize;      /**< Maximum number of ADUs in a super block */
    int superPageSize;       /**< Number of ADUs in a super page */
    int flashWriteSize;      /**< Number of ADUs to write to avoid padding (e.g., sync write) */
    uint64_t capacity;       /**< Capacity in ADUs */
    int overprovisioning;    /**< Percentage of over-provisioning (e.g., 20 for 20 percent) */
    int numDomains;          /**< Number of domains the FTL covers */
};

/**
 * @ingroup    SefSdkApi
 * @brief      This function returns size information about a block device.
 *
 * The function requires the block layer to be initialized before it is called.
 *
 * @param       blockHandle     SEF Block handle to be used for access to the block instance
 * @param[out]  info            SEFBlockInfo struct to fill with data
 */
void SEFBlockGetInfo(SEFBlockHandle blockHandle, struct SEFBlockInfo *info);

/**
 * @ingroup    SefSdkApi
 * @brief      This function is used to perform I/O commands.
 *
 * @param      context              A pointer to an instance of SEFMultiContext
 *
 * @return     Status and info summarizing result.
 *
 * @retval     -EINVAL         The input parameters are invalid; either Scatter gather list is too
 * small or I/O exceeds the device capacity
 * @retval     -ENOPROTOOPT    FTL is either not initialized or shutting down, failing I/O
 * @retval     -EBUSY          I/o context %p appears to be in use
 * @retval     -ECANCELED      GC shutdown, inflight writes canceled
 */
struct SEFStatus SEFBlockIO(struct SEFMultiContext *context);

/**
 * @ingroup    SefSdkApi
 * @brief      This function is used to discard or TRIM LBAs that are no longer
 *             needed by the application.
 *
 * @param      blockHandle          SEF Block handle to be used for access to the block instance
 * @param      lba                  Logical block address
 * @param      lbc                  Logical block count
 *
 * @return     Status and info summarizing result.
 *
 * @retval     -ENOTBLK       The Block context is not valid
 * @retval     -EINVAL        The LBA or LBC is not valid
 * @retval     -EIO           Was unable to trim; the Info would indicate LBC that wasn't completed
 */
struct SEFStatus SEFBlockTrim(SEFBlockHandle blockHandle, int64_t lba, int32_t lbc);

/**
 * @ingroup    SefSdkApi
 * @brief      Requests the passed in I/O be canceled.
 *
 * It's the caller's responsibility to ensure that the context is valid for the
 * duration of the call.  Because of internal race conditions, the
 * call may fail and yet still cancel the I/O.  Also possible is the
 * function returns success, yet the I/O completes without error
 * instead of a canceled I/O error.
 *
 * @param      context              a pointer to an instance of SEFMultiContext
 *
 * @return     Status and info summarizing result.
 *
 * @retval    -ENOENT   The I/O was not found in the any queues
 */
struct SEFStatus SEFBlockCancel(struct SEFMultiContext *context);

/**
 * @ingroup    SefSdkApi
 * @brief      This function is used to check if the stored data and metadata match.
 *
 * Moreover, it is capable of repairing the device.  To create a blockHandle for
 * a device that won't mount, call SEFBlockInit() with the delayMount option
 * set to true.
 *
 * @param      blockHandle      A pointer to the SEF Block handle to be used for
 *                              access to the block instance.
 * @param      shouldRepair     Should the block module be repaired if it is faulty?
 *                              1 denotes true
 *
 * @return     Status and info summarizing result. An error is returned if the device is faulty.
 *
 */
struct SEFStatus SEFBlockCheck(SEFBlockHandle blockHandle, int shouldRepair);

/**
 * @ingroup    SefSdkApi
 * @brief      This function is used to clean up the SEF Block Module by freeing up memory and stopping all
 * related functionality.
 *
 * @param      blockHandle      A pointer to the SEF Block handle to be used for access to the block instance
 *
 * @return     Status and info summarizing result.
 *
 * @retval     -ENODATA         The input block handle is not valid
 */
struct SEFStatus SEFBlockCleanup(SEFBlockHandle *blockHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SEF_BLOCK_MODULE_H */
