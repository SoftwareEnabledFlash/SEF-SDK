/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * instrumentation.h
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
/** @defgroup  SefSdkIns Instrumentation API */

#ifndef INSTRUMENTATION_H
#define INSTRUMENTATION_H

#ifdef __cplusplus
#include <atomic>
using std::atomic_int;
using std::atomic_uint_least64_t;
#else /* not __cplusplus */
#ifdef __STDC_NO_ATOMICS__
#error C11 atomics are required
#endif /* __STDC_NO_ATOMICS__ */

#include <stdatomic.h>
#endif /* __cplusplus */

#include <stdint.h>
#include <stdlib.h>

#include "../log-manager.h"

typedef struct INSHandle_ *INSHandle;

#define InstructionCounterDef(Name, Description, Struct, StructPtr)              \
    {                                                                            \
        .name = #Name, .description = Description, .value = &((StructPtr)->Name) \
    }

#define INSUnRegisterAction(ctxt, verb) INSRegisterAction(ctxt, verb, NULL, NULL)

/**
 *  @ingroup    SefSdkIns
 *  @brief      This function initializes the FTL's instrumentation layer, which
 *              opens a DGRAM Unix domain socket.
 *
 *  @param      socketPath   The location for the Unix Domain Socket
 *  @param[out] ictxt        The handle for the instrumentation layer
 *  @param      logHandle    A handle for the logging system
 *
 *  @return     Returns an errno stating whether the function was successful
 */
int INSInit(const char *socketPath, INSHandle *ictxt, LogHandle logHandle);

/**
 *  @ingroup    SefSdkIns
 *  @brief      This function closes and cleans up the DGRAM UNIX domain socket.
 *
 *  @param      ictxt           Pointer to handle returned by INSInit()
 */
void INSCleanup(INSHandle *ictxt);

/**
 *  @ingroup         SefSdkIns
 *  @brief           Action callback supplied to INSRegisterAction()
 *
 *  @param verb      Verb is the invoked the action.  This allows a single function
 *                   to support more than one verb and will be "help" when
 *                   instrumentation is building the help string.
 *  @param savePtr   savePtr was passed to strtok_r() to parse off the verb and
 *                   can be used to parse the rest of the input buffer
 *  @param arg       Argument supplied to INSRegisterAction()
 *  @param out       Buffer to write the result of the action to or error
 *                   text.  Required to be zero-terminated.
 *  @param size      Size of the out buffer.
 **/
typedef void (*INSAction)(const char *verb, char **savePtr, void *arg, char *out, ssize_t size);

/**
 *  @ingroup         SefSdkIns
 *  @brief           Registers the verb to call the action by instrumentation.
 *
 *  @param ictxt     Handle returned by INSInit()
 *  @param verb      Verb to register
 *  @param action    Function to invoke for supplied verb and "help" verb. A
 *                   NULL action unregisters the verb.
 *  @param arg       Argument supplied to the action when invoked
 **/
void INSRegisterAction(INSHandle ictxt, const char *verb, INSAction action, void *arg);

/**
 *  @ingroup         SefSdkIns
 *  @brief           Invoke the action based on the verb in incomingBuf.
 *
 *  @param ictxt        Handle returned by INSInit()
 *  @param verbStr      Verb with optional args to invoke as a null-terminated string.
 *  @param outgoingBuf  Buffer to write the result of the action to or error
 *                      text. Required to be zero-terminated.
 *  @param outgoingBufSize  Size of the out buffer.
 **/
void INSInvokeAction(INSHandle ictxt, const char *verbStr, char *outgoingBuf, ssize_t outgoingBufSize);

/**
 *  @ingroup    SefSdkIns
 *  @brief      Contains information about the counters that should be registered
 */
struct INSCounter
{
    char *name;                   /**< The public / human readable name for the counter */
    char *description;            /**< The description of the given counter */
    atomic_uint_least64_t *value; /**< Pointer to the value of the counter */
};

typedef void (*INSCounterUpdate)(void *arg);
typedef int (*INSCounterSet)(void *arg, int index, int64_t val);    // return 0 if set

/**
 *  @ingroup            SefSdkIns
 *  @brief              Registers a list of counters that are returned when the dump action is called
 *
 *  @param ictxt        Handle returned by INSInit()
 *  @param counters     Array of io counters to be registered
 *  @param counterNum   Number of io counters passed in the array
 *  @param arg          A void* pointer passed to the update function
 *  @param updateFunc   A function called to request updating of the counters' values
 *
 *  @returns            Counter instance id to unregister or error if less than 0
 *  @retval -EEXISTS    Counter already registered
 *  @retval -EINVAL     ictxt is NULL
 **/
int INSRegisterIoCounters(INSHandle ictxt,
                          struct INSCounter *counters,
                          int counterNum,
                          void *arg,
                          INSCounterUpdate updateFunc);

/**
 *  @ingroup            SefSdkIns
 *  @brief              Registers a list of counters that are returned when the state action is called
 *
 *  @param ictxt        Handle returned by INSInit()
 *  @param counters     Array of state counters to be registered
 *  @param counterNum   Number of state counters passed in the array
 *  @param arg          A void* pointer passed to the update function
 *  @param updateFunc   A function called to request updating of the counters' values
 *  @param setFunc      A function called to set a counters value
 *
 *  @returns            Counter instance id to unregister or error if less than 0
 *  @retval -EEXISTS    Counter already registered
 *  @retval -EINVAL     ictxt is NULL
 **/
int INSRegisterStateCounters(INSHandle ictxt,
                             struct INSCounter *counters,
                             int counterNum,
                             void *arg,
                             INSCounterUpdate updateFunc,
                             INSCounterSet setFunc);

/**
 *  @ingroup            SefSdkIns
 *  @brief              UnRegisters a list of counters based on instance id
 *
 *  @param ictxt        Handle returned by INSInit()
 *  @param instanceId   id returned by INSRegisterIoCounters
 *
 *  @retval 0           Counters unregistered
 *  @retval -ENOENT     Counters not found with that instance id
 *  @retval -EINVAL     ictxt is NULL
 **/
int INSUnRegisterIoCounters(INSHandle ictxt, int instanceId);

/**
 *  @ingroup            SefSdkIns
 *  @brief              UnRegisters a list of counters based on instance id
 *
 *  @param ictxt        Handle returned by INSInit()
 *  @param instanceId   id returned by INSRegisterStateCounters
 *
 *  @retval 0           Counters unregistered
 *  @retval -ENOENT     Counters not found with that instance id
 *  @retval -EINVAL     ictxt is NULL
 **/
int INSUnRegisterStateCounters(INSHandle ictxt, int instanceId);

#endif /* INSTRUMENTATION_H */
