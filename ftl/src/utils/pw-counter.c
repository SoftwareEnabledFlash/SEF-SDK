/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * pw-counter.c
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
#include "pw-counter.h"

#include <errno.h>
#include <stdatomic.h>
#include <stdint.h>

#include "../config.h"
#include "../sef-utils.h"

#define INITIALIZED_EMPTY 1U

struct PWCCompleteRequest
{
    void (*Complete)(void *args);
    void *args;
    TmaDListEntry link;
};

void SyncFlushComplete(void *arg)
{
    struct sefEvent *flushed = arg;
    SefEventSet(flushed);
}

void MultiSyncFlushComplete(void *arg)
{
    bool isDone;
    struct MultiPendingWorkCounter *mpwc = arg;
    struct PWCCompleteRequest *completeRequest = NULL;

    // callback complete function
    pthread_mutex_lock(&mpwc->queueLock);
    completeRequest = utl_DListPopHeadAs(&mpwc->queuedFlushRequest, struct PWCCompleteRequest, link);
    isDone = utl_DListIsEmpty(&mpwc->queuedFlushRequest);
    pthread_mutex_unlock(&mpwc->queueLock);

    completeRequest->Complete(completeRequest->args);

    SUfree(completeRequest);

    if (!isDone)
    {
        PWCFlushAsync(&mpwc->pwc, MultiSyncFlushComplete, mpwc);
    }
}

int PWCInit(struct PendingWorkCounter *pwc)
{
    struct PendingWorkCounter tempPWC = {.cnt = INITIALIZED_EMPTY, .fcnt = INITIALIZED_EMPTY};
    atomic_store(&pwc->val, tempPWC.val);
    pwc->Complete = NULL;
    pwc->args = NULL;
    return 0;
}

int PWCMultiInit(struct MultiPendingWorkCounter *mpwc)
{
    int retVal;

    retVal = PWCInit(&mpwc->pwc);
    if (retVal)
    {
        return retVal;
    }

    pthread_mutex_init(&mpwc->queueLock, NULL);
    utl_DListInit(&mpwc->queuedFlushRequest);

    return retVal;
}

int PWCCleanup(struct PendingWorkCounter *pwc)
{
    struct PendingWorkCounter tempPWC;
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

int PWCMultiCleanup(struct MultiPendingWorkCounter *mpwc)
{
    int retVal;
    struct PWCCompleteRequest *entry;

    retVal = PWCCleanup(&mpwc->pwc);
    if (retVal)
    {
        return retVal;
    }

    pthread_mutex_lock(&mpwc->queueLock);

    // check for queued items
    if (!utl_DListIsEmpty(&mpwc->queuedFlushRequest))
    {
        pthread_mutex_unlock(&mpwc->queueLock);
        return -EACCES;
    }

    // remove queue list
    while ((entry = utl_DListPopHeadAs(&mpwc->queuedFlushRequest, struct PWCCompleteRequest, link)))
    {
        SUfree(entry);
    }

    pthread_mutex_unlock(&mpwc->queueLock);

    // free resources
    pthread_mutex_destroy(&mpwc->queueLock);

    return retVal;
}

int PWCFlush(struct PendingWorkCounter *pwc)
{
    struct sefEvent flushed;
    int ret;

    if ((ret = SefEventInit(&flushed, 0)))
    {
        return ret;
    }

    if ((ret = PWCFlushAsync(pwc, SyncFlushComplete, &flushed)))
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

int PWCFlushAsync(struct PendingWorkCounter *pwc, void (*Complete)(void *args), void *args)
{
    struct PendingWorkCounter tempPWC;
    uint64_t oldVal = atomic_load(&pwc->val);

    for (;;)
    {
        tempPWC.val = oldVal;

        // flushes of uninitialized PWCs not permitted
        if (!tempPWC.cnt)
        {
            return -EACCES;
        }

        // only one flush allowed in a single PWC at a time
        if (tempPWC.fcnt > INITIALIZED_EMPTY)
        {
            return -EAGAIN;
        }

        // if PWC is empty, call callback immediately and return
        if (tempPWC.cnt == INITIALIZED_EMPTY)
        {
            if (Complete != NULL)
            {
                Complete(args);
            }
            return 0;
        }

        tempPWC.fcnt = tempPWC.cnt;
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

int PWCMultiFlushAsync(struct MultiPendingWorkCounter *mpwc, void (*Complete)(void *args), void *args)
{
    bool isInitCall;
    struct PWCCompleteRequest *newData;

    // create flush request
    newData = SUzalloc(sizeof(struct PWCCompleteRequest));
    utl_DListInitEntry(&newData->link);
    newData->Complete = Complete;
    newData->args = args;

    // add to queue and check for flush
    pthread_mutex_lock(&mpwc->queueLock);
    isInitCall = utl_DListIsEmpty(&mpwc->queuedFlushRequest);
    utl_DListPushTail(&mpwc->queuedFlushRequest, &newData->link);
    pthread_mutex_unlock(&mpwc->queueLock);

    // wait for the flush
    if (isInitCall)
    {
        return PWCFlushAsync(&mpwc->pwc, MultiSyncFlushComplete, mpwc);
    }

    return 0;
}

bool PWCMultiIsEmpty(struct MultiPendingWorkCounter *mpwc)
{
    bool isEmpty;

    pthread_mutex_lock(&mpwc->queueLock);
    isEmpty = utl_DListIsEmpty(&mpwc->queuedFlushRequest);
    pthread_mutex_unlock(&mpwc->queueLock);

    return isEmpty;
}

int PWCStartComplete(struct PendingWorkCounter *pwc, bool start)
{
    struct PendingWorkCounter tempPWC;
    uint64_t oldVal = atomic_load(&pwc->val);

    for (;;)
    {
        bool setFlushed = false;

        tempPWC.val = oldVal;

        // block all start/complete calls made on a cleaned-up PWC
        if (!tempPWC.cnt)
        {
            return -EACCES;
        }

        if (start)
        {
            // fail start calls that would cause overflow
            if (tempPWC.cnt == UINT32_MAX)
            {
                return -EOVERFLOW;
            }

            tempPWC.cnt++;
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
                tempPWC.fcnt--;
                setFlushed = (tempPWC.fcnt == INITIALIZED_EMPTY) ? true : false;
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
            pwc->Complete(pwc->args);
        }

        return 0;
    }
}

int PWCStart(struct PendingWorkCounter *pwc)
{
    return PWCStartComplete(pwc, true);
}

int PWCComplete(struct PendingWorkCounter *pwc)
{
    return PWCStartComplete(pwc, false);
}
