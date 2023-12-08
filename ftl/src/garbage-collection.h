/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * garbage-collection.h
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
/** @defgroup  SefSdkGc Garbage Collection API */

#ifndef GARBAGE_COLLECTION_H
#define GARBAGE_COLLECTION_H

#include <SEFAPI.h>
#include <stdint.h>

#include "config.h"
#include "utils/file-logger.h"
#include "utils/instrumentation.h"
#include "utils/sef-slist.h"

#define GC_ADU_RESERVE_SB 2 /**< Number of super blocks reserved for GC */

typedef struct FTLContext_ FTLContext;
struct SSBDomainState;
struct gcContext;

/**
 * @ingroup    SefSdkGc
 * @brief      GC Notify Types
 */
enum GCNotifyType {
    kGcNotifyDown, /**< GC thread is shutting down */
    kGcCycleStart, /**< A copy cycle is starting */
    kGcCopy,       /**< A source block is about to be copied */
    kGcCycleEnd    /**< A copy cycle has ended */
};

/**
 * @ingroup    SefSdkGc
 * @brief      GC Notify Data
 */
struct GCNotify
{
    void *data;             /**< notifyData supplied by GCConfig */
    void *dstate;           /**< domain the event applies to (not used
                                 by kGcNotifyDown */
    enum GCNotifyType type; /**< type of notification */
    union
    {
        struct SEFStatus status; /**< kGcNotifyDown, error status if can't
                                      start or make progress */
        uint32_t adusUsed;       /**< kGcCycleEnd, number of ADUs used by GC */
        struct
        {
            float WAF;              /**< WAF of source block */
            uint32_t numBlocksFree; /**< How many blocks free in the domain */
        }; /**< kGcCopy */
    };
};

/**
 * @ingroup    SefSdkGc
 * @brief      GC active states.
 */
enum GCActiveStates {
    kGCStateIdle,       /**< Waiting to be triggered */
    kGCStateReleasing,  /**< Releasing empty blocks */
    kGCStateWaiting,    /**< Waiting for busy blocks to drain readers */
    kGCStateSelecting,  /**< Selecting source blocks */
    kGCStateAllocating, /**< Allocating a destination block */
    kGCStateCopying,    /**< Copy source blocks to the destination block */
    kGCStatePatrol,     /**< Patrolling blocks returned by SEFGetCheckList() */
    kGCStateDown,       /**< GC is not running */
};

/**
 * @ingroup    SefSdkGc
 * @brief      GC Config Data
 */
struct GCConfig
{
    FTLContext *ctxt; /**< context to pass to SBS fnc's that require it */
    LogHandle hLog;   /**< Logger to use, may be NULL */
    INSHandle hInst;  /**< Instrumentation Handle to use, may be NULL */

    int nQoS;                          /**< Number of domains dstate points to */
    double op;                         /**< Percent of over-provisioning use
                                              before GC starts */
    uint64_t maxLba;                   /**< max # of valid adus.  Use beyond this
                                            is considered over-provisioned flash */
    struct SSBDomainState *dstate;     /**< Domain(s) to collect */
    void *notifyData;                  /**< context to pass to notify() */
    void (*notify)(struct GCNotify *); /**< Notify Fnc, may be NULL */
};

/**
 * @ingroup    SefSdkGc
 * @brief  Request GC to look for items to collect in a specific domain.
 *
 * @param  ctxt     A pointer to an instance of gcContext
 * @param  domain   Domain to garbage collect
 *
 * @retval true  GC was triggered
 * @retval false GC was already triggered
 */
bool GCTriggerDomain(struct gcContext *ctxt, struct SSBDomainState *domain);

/**
 * @ingroup    SefSdkGc
 * @brief   Returns if GC should be started
 *
 * @param   gctx      A pointer to an instance of gcContext
 * @param   dstate    Domain to garbage collect
 *
 * @retval  true      GC needs to run
 * @retval  false     GC does not need to run
 */
bool GCShouldRun(struct gcContext *gctx, struct SSBDomainState *dstate);

/**
 * @ingroup    SefSdkGc
 * @brief   Request GC to look for items to collect.
 *
 * @param   gctx    The pointer returned by GCInit()
 *
 * @retval  true    GC was triggered
 * @retval  false   GC was already triggered
 */
bool GCTrigger(struct gcContext *gctx);

/**
 * @ingroup    SefSdkGc
 * @brief  Suspends the GC thread
 *
 * Each call to GcSuspend() must be balanced by a call to GcResume()
 * before the GC thread will start again.  If GC is in the middle of
 * collection, it won't suspend until it's back below the low water mark.
 *
 * @param  gctx  The pointer returned by GCInit()
 *
 */
void GcSuspend(struct gcContext *gctx);

/**
 * @ingroup    SefSdkGc
 * @brief  Resumes the GC thead
 *
 * See GCSuspend()
 *
 * @param  gctx  The pointer returned by GCInit()
 */
void GcResume(struct gcContext *gctx);

/**
 * @ingroup    SefSdkGc
 * @brief Sets a fatal error for GC, causing a controlled stop.
 *
 * The error set will be sent to the gcNotify thread once GC has
 * shut down.
 *
 * @param  gctx   The pointer returned by GCInit()
 * @param  status The status value to send to the gcNotify thread
 */
void GCSetFatalError(struct gcContext *gctx, struct SEFStatus status);

/**
 * @ingroup    SefSdkGc
 * @brief Returns the GC thread active state.
 *
 * @param  gctx  The pointer returned by GCInit()
 *
 * @returns The current state of the GC thread
 */
enum GCActiveStates GCActiveState(struct gcContext *gctx);

/**
 * @ingroup    SefSdkGc
 * @brief   Set I/O weights to use when copying data
 *
 * Set weights to all 0xff to use domain values for NLC weights.
 *
 * @param  gctx         The pointer returned by GCInit()
 * @param  overrides    I/O weights to use for nameless copy
 */
void GCSetCopyIOWeights(struct gcContext *gctx, struct SEFCopyOverrides *overrides);

/**
 * @ingroup    SefSdkGc
 * @brief Starts up the garbage collector.
 *
 * @param      gcConfig     Configuration for GC
 *
 * @returns     GC thread context required when calling GC public functions
 */
struct gcContext *GCInit(struct GCConfig *gcConfig);

/**
 * @ingroup    SefSdkGc
 * @brief Shuts down the garbage collector.
 *
 * @param     gctx    The pointer returned by GCInit()
 */
void GCCleanup(struct gcContext *gctx);

#endif /* GARBAGE_COLLECTION_H */
