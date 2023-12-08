/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * fpw-counter.h
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
#ifndef PW_COUNTER_H
#define PW_COUNTER_H

#ifdef __cplusplus
#include <atomic>
using std::atomic_uint_least64_t;
#else /* not __cplusplus */
#ifdef __STDC_NO_ATOMICS__
#error C11 atomics are required
#endif /* __STDC_NO_ATOMICS__ */

#include <stdatomic.h>
#endif /* __cplusplus */
#include <stdint.h>

#include "sef-event.h"

struct FencedPendingWorkCounter
{
    union
    {
        atomic_uint_least64_t val;
        struct
        {
            uint16_t cnt;
            uint16_t fcnt;
            uint32_t sn;
        };
    };
    uint32_t fsn;
    void (*Complete)(void *args);    // complete function to be called on flush
    void *args;                      // arguments for the complete function
};

/**
 *  @brief      Initializes the FencedPendingWorkCounter
 *
 *  @param      pwc   A pointer to an instance of the FencedPendingWorkCounter struct
 *
 *  @return     Returns 0 for successful initialization of the
 *              FencedPendingWorkCounter, or an errno for failure
 */
int FPWCInit(struct FencedPendingWorkCounter *pwc);

/**
 *  @brief      Cleans up resources used by an initialized FencedPendingWorkCounter
 *
 *  @param      pwc   A pointer to an instance of the FencedPendingWorkCounter struct
 *
 *  @return     Returns 0 for successful cleanup of the FencedPendingWorkCounter,
 *              or an errno for failure
 */
int FPWCCleanup(struct FencedPendingWorkCounter *pwc);

/**
 *  @brief      Makes a copy of the FPWC's current count and serial number as the
 *              fence, and once FPWCComplete() has been called this number of
 *              times by completers with an sn lower than the fence serial number,
 *              this function returns
 *
 *  @note       Do not call from a thread that is required for entries to complete (i.e.
 *              don't call from a thread pool thread if the thread pool is required for
 *              FPWCComplete() to be called)
 *
 *  @param      pwc   A pointer to an instance of the FencedPendingWorkCounter struct
 *
 *  @return     Returns 0 when the same number of entries that were in the counter at call time
 *              have completed and are no longer part of the counter, or returns an errno
 *              for flush failure
 *
 */
int FPWCFlush(struct FencedPendingWorkCounter *pwc);

/**
 *  @brief      Makes a copy of the FPWC's current count and serial number as the
 *              fence, and once FPWCComplete() has been called this number of
 *              times by completers with an sn lower than the fence serial number,
 *              this function calls the provided function (Complete) using the
 *              provided arguments (args)
 *
 *  @param      pwc         A pointer to an instance of the FencedPendingWorkCounter struct
 *  @param      Complete    Complete function to be called upon flush
 *  @param      args        A pointer to arguments to be provided to the complete function
 *
 *  @return     Returns 0 when the complete/callback function for the flush has been successfully
 *              set, or returns an errno for failure to set this function
 *
 */
int FPWCFlushAsync(struct FencedPendingWorkCounter *pwc, void (*Complete)(void *args), void *args);

/**
 *  @brief      Signals that a resource monitored by a FencedPendingWorkCounter
 *              is in use by the caller
 *
 * FPWCStart increments the referenced FencedPendingWorkCounter's count by 1
 *
 *  @param [in]  pwc    A pointer to an instance of the FencedPendingWorkCounter struct
 *  @param [out] sn     Returns the counter serial number to supply when calling
 *                      FPWCComplete
 *
 *  @return     Returns 0 when the start has been accounted for, or returns an errno when the start
 *              cannot be accounted for
 */
int FPWCStart(struct FencedPendingWorkCounter *pwc, uint32_t *sn);

/**
 *  @brief      Signals that the caller has completed use of a resource monitored by a
 *              FencedPendingWorkCounter.
 *
 *  FPWCComplete() decrements the referenced FencedPendingWorkCounter's count by 1.  When
 *  the FPWC is in a flushing state, the captured flush count is also decremented,
 *  but only if when the passed in counter serial number is less than the fence
 *  serial number that was captured when the flush was initiated.
 *
 *  If a call to FPWCComplete() decrements the captured flush counter to the point
 *  there is no more outstanding work to complete, the completion routine supplied
 *  to FPWCFlush() is called using the callers thread.
 *
 *  @param      pwc   A pointer to an instance of the FencedPendingWorkCounter struct
 *  @param      sn    The count serial number returned by FPWCStart()
 *
 *  @return     Returns 0 when the caller's completion has been accounted for, or returns an errno
 *              when the completion cannot be accounted for
 */
int FPWCComplete(struct FencedPendingWorkCounter *pwc, uint32_t sn);

#endif /* PW_COUNTER_H */
