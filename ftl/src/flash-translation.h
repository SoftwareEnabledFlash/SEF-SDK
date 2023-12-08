/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * flash-translation.h
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
/** @defgroup  SefSdkSft Flash Translation Layer API */

#ifndef FLASH_TRANSLATION_H
#define FLASH_TRANSLATION_H

#include <SEFAPI.h>
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

#include "config.h"
#if CC_DIE_STATS
#include "die-stats.h"    // struct dieStats
#endif
#include "garbage-collection.h"
#include "log-manager.h"
#include "manual-copy.h"
#include "persistence.h"
#include "sef-block-module.h"
#include "utils/dlist.h"
#include "utils/gateway.h"
#include "utils/instrumentation.h"
#include "utils/pw-counter.h"
#include "utils/sef-event.h"
#include "utils/sef-slist.h"

/**
 *  @ingroup    SefSdkSft
 *  @brief      Dynamic state of the FTL.
 */
struct SEFIOState
{
    // sbm
    atomic_uint_least64_t activeRead;       /**< SEFBlock-reads in-flight */
    atomic_uint_least64_t activeWrite;      /**< SEFBlock-writes in-flight */
    atomic_uint_least64_t blockedWrites;    /**< Number of writes blocked on GC */
    atomic_uint_least64_t readWeight;       /**< Read I/O weight override */
    atomic_uint_least64_t programWeight;    /**< Write I/O weight override */
    atomic_uint_least64_t eraseWeight;      /**< Erase I/O weight override */
    atomic_uint_least64_t nlcProgramWeight; /**< Copy write weight */
    atomic_uint_least64_t nlcEraseWeight;   /**< Copy erase weight */
    atomic_uint_least64_t readQueue;        /**< Read Queue override */
};

/**
 *  @ingroup    SefSdkSft
 *  @brief      Flash translation layer I/O counters
 */
typedef struct SEFIOCounter
{
    atomic_uint_least64_t maxQueue;          /**< Unused, always 0 */
    atomic_uint_least64_t read;              /**< Number of block read requests */
    atomic_uint_least64_t readADU;           /**< Number of read ADUs */
    atomic_uint_least64_t readIocb;          /**< Number of device read IOCBs */
    atomic_uint_least64_t readLatency_us;    /**< Total read latency in µs */
    atomic_uint_least64_t readMinLatency_ns; /**< Minimum read latency in ns */
    atomic_uint_least64_t readMaxLatency_ns; /**< Maximum read latency in ns */
    atomic_uint_least64_t readZero; /**< Number of ADUs read that were not written (returns zeros)*/
    atomic_uint_least64_t write;    /**< Number of block write requests */
    atomic_uint_least64_t writeADU; /**< Number of written ADUs */
    atomic_uint_least64_t writeIocb;          /**< Number of device write IOCBs */
    atomic_uint_least64_t writeLatency_us;    /**< Total write latency in µs */
    atomic_uint_least64_t writeMinLatency_ns; /**< Minimum write latency in ns */
    atomic_uint_least64_t writeMaxLatency_ns; /**< Maximum write latency in ns */
    atomic_uint_least64_t trim;               /**< Number of trim requests */
} SEFIOCounter;

/**
 *  @ingroup    SefSdkSft
 *  @brief Configuration for sef-block-module
 */
struct SEFBlockMeta
{
    uint64_t Wsn;         /**< Last used write serial number */
    uint64_t MaxLBA;      /**< The maximum number of LBAs that can be stored in the LUT */
    int Overprovisioning; /**< Percent of over-provisioning supplied when
                               configured. (e.g., 20 for 20 percent); Actual
                               OP may be larger. */
    int Rsvd;             /**< Unused member */
    uint64_t MaxAdus;     /**< Point at which I/O will block until GC.  Quota
                               - MaxAdus is what is reserved for Meta and GC */
};

static_assert(sizeof(struct SEFBlockMeta) == 32, "persisted structure has padding");

/**
 *  @ingroup    SefSdkSft
 *  @brief      Flash translation layer state
 */
struct SFTState
{
#if CC_DEBUG_COUNTERS
    atomic_uint_least64_t lateUpdate; /**< ADU was rewritten before GC update */
    atomic_uint_least64_t lutRetry;   /**< LUT read was retried */
#endif
    int sftIoCntId;              /**< SFT io counter registration id */
    bool lutDirty;               /**< True if LUT has been updated */
    ssize_t lutSize;             /**< Lookup table size (max LBA) */
    atomic_uint_least64_t lut[]; /**< Lookup table for the FTL */
};

/**
 *  @ingroup    SefSdkSft
 *  @brief      State for the block layer/FTL
 */
typedef struct FTLContext_
{
    uint16_t sefUnitIndex;         /**< The index of the SEF Unit */
    SEFHandle handle;              /**< Handle to the SEF Unit */
    const struct SEFInfo *sefInfo; /**< Pointer to the SEF Info struct */

    struct SEFQoSDomainID qosId;     /**< Base qos domain id */
    struct SEFQoSDomainInfo qosInfo; /**< Base qos info */

    LogHandle logHandle;   /**< Handle for logging */
    bool defaultLogHandle; /**< True if the default logger is being used */

    int numDomains;                /**< Number of qos domains used by the FTL */
    SEFQoSHandle *qosHandles;      /**< Domain handles for [qosId, qosId+numDomains) */
    struct SEFQoSDomainID *qosIds; /**< Domain Ids of Domain handles */

    void *notifyContext; /**< The context to be passed into the notify function */
    void (*notifyFunc)(struct SEFBlockNotify event,
                       void *notifyContext); /**< Notify function supplied during block init */

    struct SSBState *ssbState;        /**< Active state for the super block layer */
    struct SSBState *rebuiltSsbState; /**< Super block state resulting from a rebuild */

    struct SEFIOCounter counter; /**< Global counters/statistics */
    struct SEFIOState ioState;   /**< I/O state/counters */
#if CC_DIE_STATS
    DieStatsHandle dieStats; /**< Die/heat map statistics */
#endif
    atomic_uint_fast64_t wsn; /**< Write serial number; will get inc. on every write */

    struct SFTState *sftState;        /**< Active flash translation layer state */
    struct SFTState *rebuiltSftState; /**< Flash translation layer state resulting from a rebuild */

    // vd info
    // struct SEFADUsize aduSize;   /**< ADU and Meta Size */
    // int aduShift;                /**< Log base 2 of aduSize */
    int64_t superBlockCapacity; /**< Total super block capacity in ADUs */
    int flashWriteSize;         /**< Number of ADUs to write to avoid padding (e.g., sync write) */
    uint32_t maxIOSize;         /**< Max number of ADUs for I/O and be die page-aligned */
    INSHandle hInst;            /**< Handle returned by INSInit() */

    bool timeToDie;              /**< Detect use while cleaning up */
    bool initialized;            /**< Detect reuse after cleanup */
    bool disabled;               /**< Don't process I/O (error)*/
    struct sefGateway ioGateway; /**< Control accepting new I/O */

    struct SEFWeights ioWeights; /**< Configured weights for I/O */
    uint16_t readWeight;         /**< Configured read weight for I/O */
    // uint8_t defaultReadQueue;               /**< Configured default read queue */
    bool pSLC;                                 /**< use pSLC when writing */
    struct SEFWriteOverrides writeOverride;    /**< Weight to use for writes */
    struct SEFAllocateOverrides allocOverride; /**< Weight to use for erase */
    struct SEFReadOverrides readOverride;      /**< Weights and FQM to use for reads */

    // gc data
    struct gcContext *gctx; /**< The context used by the GC */
#if CC_MANUAL_NLC
    bool gcManual;       /**< GC will manually copy ADUs */
    MCPHandle mcpHandle; /**< Manual copy thread state */
#endif

    PDLHandle pdlHandle; /**< Handle for persistence layer */

    struct SEFBlockMeta blockData; /**< Block layer meta configuration */
} FTLContext;

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to queue to ctxt->pdlHandle a write of
 *              the LUT when the FTL hasn't been initialized.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 */
void SFTQueueToWriteLut(FTLContext *ctxt);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to init the flash translation layer.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *  @param      numADUs              Number of ADUs supported by the FTL
 */
struct SEFStatus SFTInitialize(FTLContext *ctxt, int64_t numADUs);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to free the memory used by the flash
 *              translation layer and to persist the metadata.
 *
 *  As part of the cleanup process, this function prepares the metadata for persistence; however, if
 * no changes were made to the metadata, it is not persisted.
 *
 *  @param      ctxt                 A pointer to an instance of FTLContext
 *
 *  @return     if successful, it will return 0; otherwise, a negative number is returned
 */
int SFTCleanup(FTLContext *ctxt);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to look up the flash address for an lba
 *              and to lock the super block for read.
 *
 *  The returned flash will continue to valid until a call to SFTReleaseForRead()
 *  is called.  However, the LBA may subsequently be assigned to a new flash
 *  address by a garbage collect or write i/o.  The returned flash address can
 *  still be read, but if the lba was rewritten it will return stale data.
 *
 *  @param      ctxt    A pointer to an instance of FTLContext
 *  @param      lba     Logical block address to look up
 *
 *  @return     Flash address for the last data written to lba.  SEFNullFlashAddress
 *              is returned when the LBA is out of bounds, trimmed or had never
 *              been written.
 */
struct SEFFlashAddress SFTLookupForRead(FTLContext *ctxt, int64_t lba);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to undo the locking of the super block
 *              as part of SEFLookupForRead()
 *
 *  @param      ctxt            A pointer to an instance of FTLContext
 *  @param      flashAddress    Flash Address returned by SFTLookupForRead()
 */
void SFTReleaseForRead(FTLContext *ctxt, struct SEFFlashAddress flashAddress);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to determine if a range of user addresses
 *              has been rewritten to different flash addresses.
 *
 *  When a delayed write encounters a media error, it is rewritten to new flash
 *  addresses.  Read iocbs created before the move may fail when processed by the
 *  device after the data is moved.  When this function returns true, the read
 *  should be retried with the updated flash addresses.
 *
 *  @param      ctxt            A pointer to an instance of FTLContext
 *  @param      userAddress     Starting user address being read
 *  @param      flashAddress    Expected flash Address for userAddress
 *  @param      numADU          Number of ADUs to verify
 */
bool SFTLutChanged(FTLContext *ctxt,
                   struct SEFUserAddress userAddress,
                   struct SEFFlashAddress flashAddress,
                   uint32_t numADU);

/**
 *  @ingroup    SefSdkSft
 *  @brief Assigns a physical flash address to a logical block address.
 *
 *  This is the authoritative way to update the LUT; last writer wins.
 *  It implies the data has been rewritten by the client.
 *
 *  @param      ctxt             A pointer to an instance of FTLContext
 *  @param      userLBA          Logical block address of the ADU set by the host
 *  @param      flashAddress     The physical flash address of the ADU
 *  @param      placementId      Placement ID the flash address was written to
 *
 *  @return     if successful, it will return 0; otherwise, a negative number is returned
 */
int SFTSet(FTLContext *ctxt,
           int64_t userLBA,
           struct SEFFlashAddress flashAddress,
           struct SEFPlacementID placementId);

/**
 *  @ingroup    SefSdkSft
 *  @brief Updates the physical flash address for the given user logical block address.
 *
 *  This is the non-authoritative way to update the LUT; first writer wins.
 *  It implies the data has been moved and not rewritten.
 *
 *  @param      ctxt             A pointer to an instance of FTLContext
 *  @param      userLBA          Logical block address of the ADU set by the host
 *  @param      oldFlashAddress  The old physical flash address of the ADU
 *  @param      newFlashAddress  The new physical flash address of the ADU
 *
 *  @return     If successful, it will return 0; otherwise, a negative number is returned
 */
int SFTUpdate(FTLContext *ctxt,
              uint64_t userLBA,
              struct SEFFlashAddress oldFlashAddress,
              struct SEFFlashAddress newFlashAddress);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to get the physical flash address of the given user logical block address.
 *
 *  @param      ctxt             A pointer to an instance of FTLContext
 *  @param      userLBA          Logical block address of the ADU set by the host
 *
 *  @return     Returns the flash address of the user Logical Block Address; returns 0 if invalid
 */
struct SEFFlashAddress SFTLookup(FTLContext *ctxt, int64_t userLBA);

/**
 *  @ingroup    SefSdkSft
 *  @brief      This function is used to rebuild the lookup table based on the data stored on the device.
 *
 *  If a NULL flash address is passed in for the base block, the LUT is entirely
 *  rebuilt.  If any other flash address is passed in, only blocks written after
 *  that are applied to the current LUT updating it.  Typically, the root pointer
 *  used to load the LUT is passed in to apply any changes that happened after
 *  the LUT was saved.
 *
 *  @param      ctxt            A pointer to an instance of FTLContext
 *  @param      baseBlock       Sets the rebuild point
 *
 *  @return     Status and info summarizing result.
 */
struct SEFStatus SFTRebuildLut(FTLContext *ctxt, struct SEFFlashAddress baseBlock);


#endif /* FLASH_TRANSLATION_H */
