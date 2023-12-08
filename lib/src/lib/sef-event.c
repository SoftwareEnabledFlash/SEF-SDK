/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-event.c
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
#include "sef-event.h"

#include <pthread.h>
#include <stdbool.h>

int SefEventInit(struct sefEvent* event, int flags)
{
    int result = pthread_cond_init(&event->cond, NULL);
    if (result)
    {
        return result;
    }
    result = pthread_mutex_init(&event->lock, NULL);
    if (result)
    {
        pthread_cond_destroy(&event->cond);
        return result;
    }
    event->set = ((flags & kSefEventFlagSet) == kSefEventFlagSet);
    event->auto_reset = ((flags & kSefEventFlagAutoReset) == kSefEventFlagAutoReset);
    event->fair = ((flags & kSefEventFlagFair) == kSefEventFlagFair);
    return 0;
}

void SefEventFree(struct sefEvent* event)
{
    pthread_cond_destroy(&event->cond);
    pthread_mutex_destroy(&event->lock);
}

int SefEventWait(struct sefEvent* event)
{
    int result = pthread_mutex_lock(&event->lock);
    {
        if (result)
        {
            return result;
        }
        while (!result && !event->set)
        {
            result = pthread_cond_wait(&event->cond, &event->lock);
        }
        if (!result && event->auto_reset)
        {
            event->set = false;
        }
        pthread_mutex_unlock(&event->lock);
    }
    return result;
}

bool SefEventIsSet(struct sefEvent* event)
{
    return event->set;
}

int SefEventSet(struct sefEvent* event)
{
    int result = pthread_mutex_lock(&event->lock);
    {
        if (result)
        {
            return result;
        }
        if (!event->set)
        {
            event->set = true;
            if (event->fair)
            {
                result = pthread_cond_signal(&event->cond);
            }
            else
            {
                result = pthread_cond_broadcast(&event->cond);
            }
        }
        pthread_mutex_unlock(&event->lock);
    }
    return result;
}

int SefEventReset(struct sefEvent* event)
{
    // locking to get acquire/release
    int result = pthread_mutex_lock(&event->lock);
    {
        if (result)
        {
            return result;
        }
        event->set = false;
        result = pthread_mutex_unlock(&event->lock);
    }
    return result;
}
