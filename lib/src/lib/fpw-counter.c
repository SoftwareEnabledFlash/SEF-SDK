/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * fpw-counter.c
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
#include "fpw-counter.h"

#include <assert.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdint.h>

#define INITIALIZED_EMPTY 1U

static void syncFlushComplete(void *arg)
{
    struct sefEvent *flushed = arg;
    SefEventSet(flushed);
}

int FPWCInit(struct FencedPendingWorkCounter *pwc)
{
    struct FencedPendingWorkCounter tempPWC = {.cnt = INITIALIZED_EMPTY, .fcnt = INITIALIZED_EMPTY};
    atomic_store(&pwc->val, tempPWC.val);
    pwc->Complete = NULL;
    pwc->args = NULL;
    return 0;
}

int FPWCCleanup(struct FencedPendingWorkCounter *pwc)
{
    struct FencedPendingWorkCounter tempPWC;
    tempPWC.val = atomic_load(&pwc->val);
    // don't allow cleanup of PWCs being actively used
    if (tempPWC.cnt != INITIALIZED_EMPTY)
    {
        return -EACCES;
    }
    atomic_store(&pwc->val, 0U);
    pwc->Complete = NULL;
    pwc->args = NULL;
    return 0;
}

int FPWCFlush(struct FencedPendingWorkCounter *pwc)
{
    struct sefEvent flushed;
    int ret;

    if ((ret = SefEventInit(&flushed, 0)))
    {
        return ret;
    }

    if ((ret = FPWCFlushAsync(pwc, syncFlushComplete, &flushed)))
    {
        return ret;
    }

    if ((ret = SefEventWait(&flushed)))
    {
        return ret;
    }

    SefEventFree(&flushed);
    return 0;
}

int FPWCFlushAsync(struct FencedPendingWorkCounter *pwc, void (*Complete)(void *args), void *args)
{
    struct FencedPendingWorkCounter tempPWC;
    uint64_t oldVal = atomic_load(&pwc->val);

    for (;;)
    {
        tempPWC.val = oldVal;

        // flushes of unitialized PWCs not permitted
        if (!tempPWC.cnt)
        {
            return -EACCES;
        }

        // only one flush allowed in a single FPWC at a time
        if (tempPWC.fcnt > INITIALIZED_EMPTY)
        {
            return -EAGAIN;
        }

        // if FPWC is empty, call callback immediately and return
        if (tempPWC.cnt == INITIALIZED_EMPTY)
        {
            if (Complete != NULL)
            {
                Complete(args);
            }
            return 0;
        }

        tempPWC.fcnt = tempPWC.cnt;
        pwc->fsn = tempPWC.sn;
        pwc->Complete = Complete;
        pwc->args = args;

        // attempt to store new values for fcnt
        if (!atomic_compare_exchange_strong(&pwc->val, &oldVal, tempPWC.val))
        {
            continue;
        }

        return 0;
    }
}

static int fpwcStartComplete(struct FencedPendingWorkCounter *pwc, bool start, uint32_t *sn)
{
    struct FencedPendingWorkCounter tempPWC;
    uint64_t oldVal = atomic_load(&pwc->val);

    for (;;)
    {
        bool setFlushed = false;

        tempPWC.val = oldVal;

        // block all start/complete calls made on a cleaned-up FPWC
        if (!tempPWC.cnt)
        {
            return -EACCES;
        }

        if (start)
        {
            // fail start calls that would cause overflow
            if (tempPWC.cnt == UINT16_MAX)
            {
                return -EOVERFLOW;
            }

            tempPWC.cnt++;
            *sn = ++tempPWC.sn;
        }
        else
        {
            // stop complete calls made on empty PWCs
            if (tempPWC.cnt == INITIALIZED_EMPTY)
            {
                return -EACCES;
            }

            tempPWC.cnt--;

            // if flushing, decrement fcnt
            if (tempPWC.fcnt > INITIALIZED_EMPTY)
            {
                uint32_t delta = pwc->fsn - *sn;
                bool belowFence = (delta < UINT32_MAX / 2);

                if (belowFence)
                {
                    tempPWC.fcnt--;
                    setFlushed = (tempPWC.fcnt == INITIALIZED_EMPTY);
                }
                else
                {    // state is corrupt
                    assert(tempPWC.cnt >= tempPWC.fcnt);
                }
            }
        }

        // try storing changed counts
        if (!atomic_compare_exchange_strong(&pwc->val, &oldVal, tempPWC.val))
        {
            continue;
        }
        // if stored and last leaving, call the complete function if one is set
        else if (setFlushed && pwc->Complete)
        {
            void (*Complete)(void *args) = pwc->Complete;
            void *args = pwc->args;

            pwc->Complete = pwc->args = (void *)(intptr_t)-1;
            Complete(args);
        }

        return 0;
    }
}

int FPWCStart(struct FencedPendingWorkCounter *pwc, uint32_t *sn)
{
    return fpwcStartComplete(pwc, true, sn);
}

int FPWCComplete(struct FencedPendingWorkCounter *pwc, uint32_t sn)
{
    return fpwcStartComplete(pwc, false, &sn);
}
