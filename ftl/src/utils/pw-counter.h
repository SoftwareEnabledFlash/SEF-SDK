/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * pw-counter.h
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

#include <stdatomic.h>
#include <stdint.h>
#include "dlist.h"

#include "sef-event.h"

struct PendingWorkCounter
{
    union
    {
        atomic_uint_least64_t val;
        struct
        {
            uint32_t cnt;
            uint32_t fcnt;
        };
    };
    void (*Complete)(void *args);    // complete function to be called on flush
    void *args;                      // arguments for the complete function
};

struct MultiPendingWorkCounter
{
    struct PendingWorkCounter pwc;
    TmaDList queuedFlushRequest;
    pthread_mutex_t queueLock;
};

/**
 *  @brief      Initializes the PendingWorkCounter
 *
 *  @param      pwc   A pointer to an instance of the PendingWorkCounter struct
 *
 *  @return     Returns 0 for successful initialization of the PendingWorkCounter, or an errno for failure
 */
int PWCInit(struct PendingWorkCounter *pwc);

/**
 *  @brief      Cleans up resources used by an initialized PendingWorkCounter
 *
 *  @param      pwc   A pointer to an instance of the PendingWorkCounter struct
 *
 *  @return     Returns 0 for successful cleanup of the PendingWorkCounter, or an errno for failure
 */
int PWCCleanup(struct PendingWorkCounter *pwc);

/**
 *  @brief      Makes a copy of the PWC's current count, and once PWCComplete() has
 *              been called this number of times, this function returns
 *
 *  @note       Do not call from a thread that is required for entries to complete (i.e.
 *              don't call from a thread pool thread if the thread pool is required for
 *              PWCComplete() to be called)
 *
 *  @param      pwc   A pointer to an instance of the PendingWorkCounter struct
 *
 *  @return     Returns 0 when the same number of entries that were in the counter at call time
 *              have completed and are no longer part of the counter, or returns an errno
 *              for flush failure
 *
 */
int PWCFlush(struct PendingWorkCounter *pwc);

/**
 *  @brief      Makes a copy of the PWC's current count, and once PWCComplete() has
 *              been called this number of times, this function calls the provided
 *              function (Complete) using the provided arguments (args)
 *
 *  @param      pwc         A pointer to an instance of the PendingWorkCounter struct
 *  @param      Complete    Complete function to be called upon flush
 *  @param      args        A pointer to arguments to be provided to the complete function
 *
 *  @return     Returns 0 when the complete/callback function for the flush has been successfully
 *              set, or returns an errno for failure to set this function
 *
 */
int PWCFlushAsync(struct PendingWorkCounter *pwc, void (*Complete)(void *args), void *args);

/**
 *  @brief      Signals that a resource monitored by a PendingWorkCounter is in use by the caller
 *              by incrementing the referenced PendingWorkCounter's count by 1
 *
 *  @param      pwc   A pointer to an instance of the PendingWorkCounter struct
 *
 *  @return     Returns 0 when the start has been accounted for, or returns an errno when the start
 *              cannot be accounted for
 */
int PWCStart(struct PendingWorkCounter *pwc);

/**
 *  @brief      Signals that the caller has completed use of a resource monitored by a
 *              PendingWorkCounter by decrementing the referenced PendingWorkCounter's count
 *              by 1
 *
 *  @param      pwc   A pointer to an instance of the PendingWorkCounter struct
 *
 *  @return     Returns 0 when the caller's completion has been accounted for, or returns an errno
 *              when the completion cannot be accounted for
 */
int PWCComplete(struct PendingWorkCounter *pwc);

int PWCMultiInit(struct MultiPendingWorkCounter *mpwc);

int PWCMultiFlushAsync(struct MultiPendingWorkCounter *mpwc, void (*Complete)(void *args), void *args);

int PWCMultiCleanup(struct MultiPendingWorkCounter *mpwc);

bool PWCMultiIsEmpty(struct MultiPendingWorkCounter *mpwc);

#endif /* PW_COUNTER_H */
