/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * gateway.h
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
#ifndef GATEWAY_H
#define GATEWAY_H

#include <endian.h>
#include <stdatomic.h>
#include <stdint.h>
#include "sef-event.h"

struct sefGateway
{
    union
    {
        atomic_uint_least64_t val; /**< used for cmp exchange */
        struct                     /**< used for interpreting results */
        {
            uint64_t data : 62; /**< Number of entries currently in the gateway / The address for the event struct */
            uint64_t state : 2; /**< The state of the gateway */
        } gate;
    };
};

#define SEFGATEWAY_INITIALIZER \
    {                          \
    }

/**
 *  @brief      This function initializes a gateway, setting it to the closed
 *              state
 *
 *  @param      gateway           A pointer to an instance of the gateway struct
 */
void sefGatewayInit(struct sefGateway* gateway);

/**
 *  @brief      This function opens the gateway only if it is in the closed state
 *
 *  @param      gateway           A pointer to an instance of the gateway struct
 *
 *  @return     returns an errno stating whether the function was successful in opening the gateway
 */
int sefGatewayOpen(struct sefGateway* gateway);

/**
 *  @brief      This function closes the gateway if there are no entries or by waiting for all the
 * entries to leave
 *
 *  @note Do not call from a thread that is required for entries to leave (e.g.
 *        don't call from a thread pool thread if the thread pool is required for
 *        sefGatewayLeave() to be called)
 *
 *  @param      gateway           A pointer to an instance of the gateway struct
 *
 *  @return     returns an errno stating whether the function was successful in closing the gateway.
 * Moreover, it waits for all the entries to leave before returning
 */
int sefGatewayClose(struct sefGateway* gateway);

/**
 *  @brief      This function flushes the gateway if there are no entries or by waiting for all the
 * entries to leave
 *
 *  @note Do not call from a thread that is required for entries to leave (e.g.
 *        don't call from a thread pool thread if the thread pool is required for
 *        sefGatewayLeave() to be called)
 *
 *  @param      gateway           A pointer to an instance of the gateway struct
 *
 *  @return     returns an errno stating whether the function was successful in flushing the
 * gateway. Moreover, it waits for all the entries to leave before returning
 */
int sefGatewayFlush(struct sefGateway* gateway);

/**
 *  @brief      This function adds an entry to the gateway if it is in the open state
 *
 *  If the gateway is flushing, it blocks until the gateway is flushed before entering.
 *
 *  @param      gateway           A pointer to an instance of the gateway struct
 *
 *  @return     returns an errno stating whether the function was successful in adding an entry
 */
int sefGatewayEnter(struct sefGateway* gateway);

/**
 *  @brief      This function removes an entry from the gateway and closed the gateway if it is in the closing state
 *
 *  @param      gateway           A pointer to an instance of the gateway struct
 *
 *  @return     returns an errno stating whether the function was successful in removing an entry
 */
int sefGatewayLeave(struct sefGateway* gateway);

#endif /* GATEWAY_H */
