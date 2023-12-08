/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-event.h
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
#pragma once

#include <pthread.h>
#include <stdbool.h>

enum { kSefEventFlagSet = 1, kSefEventFlagAutoReset = 2, kSefEventFlagFair = 4 };

/**
 *  @brief    SefEvent, constructed with SefEventInit or SEFEVENT_XXX_INITIALIZER
 */
struct sefEvent
{
    pthread_cond_t cond;
    pthread_mutex_t lock;
    bool set;        /**< when false, waiter blocks on cond */
    bool auto_reset; /**< if true, waiter clears the trigger */
    bool fair;       /**< if false, broadcast cond */
};

#define SEFEVENT_AUTORESET_INITIALIZER                                          \
    {                                                                           \
        PTHREAD_COND_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, false, true, false \
    }

#define SEFEVENT_NON_AUTORESET_INITIALIZER                                       \
    {                                                                            \
        PTHREAD_COND_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, false, false, false \
    }

/**
 *  @brief      Construct a sefEvent
 *
 *  @param      event           a pointer to the event structure to initialize
 *  @param	   flags		   Flags to control operation of the event
 *  							   kSefEventFlagSet          Event initialized to set
 *  							   kSefEventFlagAutoReset    Event auto resets after being
 * waited on kSefEventFlagFair         Waiting on event is FIFO vs first come
 *
 *  @return	   0 on success; ? On failure to allocate resources
 */
int SefEventInit(struct sefEvent* event, int flags);

/**
 *  @brief      Frees the resources owned by an event
 *
 *  @param      event           a pointer to the event structure
 */
void SefEventFree(struct sefEvent* event);

/**
 *  @brief      Waits on the event until set
 *
 *  @param      event           a pointer to the event structure
 *
 *  @return     true on success; false on failure to set (e.g., event was destroyed)
 */
int SefEventWait(struct sefEvent* event);

/**
 * @brief Returns if an event was set or not
 *
 * @param event 	a pointer to the event structure to test
 * @return true 	if event was set
 * @return false 	if event was not set
 */
bool SefEventIsSet(struct sefEvent* event);

/**
 *  @brief      Sets an event, releasing a waiting thread if present
 *
 *  @param      event           a pointer to the event structure
 *
 *  @return     true on success; false on failure to set (e.g., event was destroyed)
 */
int SefEventSet(struct sefEvent* event);

/**
 *  @brief      Resets an event, i.e. untrigger it
 *
 *  @param      event           a pointer to the event structure
 *
 *  @return     true on success; false on failure to reset (e.g., event was destroyed)
 */
int SefEventReset(struct sefEvent* event);
