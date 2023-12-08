/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * cli-engine.h
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
#ifndef SEF_ENGINE_H
#define SEF_ENGINE_H

#include "utils/dlist.h"

#define sefEngineInit __attribute__((constructor))
#define sefEngineExit __attribute__((destructor))

#define NELEM(I) (sizeof(I) / sizeof(I[0]))

#define newEngine(N, D, OS, O, A)                                                  \
    {                                                                              \
        .name = N, .description = D, .parsedOptionsStructSize = sizeof(struct OS), \
        .optionsNum = NELEM(O), .options = O, .actionsNum = NELEM(A), .actions = A \
    }

#define newEngineWithSetup(N, D, OS, O, A, SU, CU)                                  \
    {                                                                               \
        .name = N, .description = D, .parsedOptionsStructSize = sizeof(struct OS),  \
        .optionsNum = NELEM(O), .options = O, .actionsNum = NELEM(A), .actions = A, \
        .setupFunction = SU, .cleanupFunction = CU                                  \
    }

#define newEngineAction(N, D, F)                 \
    {                                            \
        .name = N, .description = D, .action = F \
    }

/**
 * @brief      details of a target-specific action
 */
struct CEHEngineAction
{
    const char *name;        /**< Action name */
    const char *description; /**< Action description */
    int (*action)(void **engineContext,
                  void *engineOptions); /**< Function called when the action is invoked */
};

/**
 * @brief      detailed information relating to a target
 */
struct CEHEngineConfig
{
    const char *name;        /**< Engine name */
    const char *description; /**< Engine description */
    int (*setupFunction)(void **engineContext,
                         void *engineOptions); /**< Function called before calling action */
    int (*cleanupFunction)(void **engineContext,
                           void *engineOptions); /**< Function called after calling action */
    int parsedOptionsStructSize;                 /**< Size of the engine options struct */
    int optionsNum;                              /**< Number of options in options array */
    struct CEOEngineOption *options;             /**< An array of engine's options */
    int actionsNum;                              /**< Number of actions in the actions array */
    struct CEHEngineAction *actions;             /**< An array of engine's actions */
    TmaDListEntry link;
};

/**
 * @brief       Attempts to decipher which target-specific action user meant to indicate, even if user input has no
 * exact match
 *
 * @param       action              The user's input action
 * @param       engine              The target of the action
 * @param       actionFunction
 *
 * @retval      0                   A single match for the user's input for action was identified
 * @retval      EINVAL              A single match could not be identified for user action input; user input was invalid
 * @retval      ENOTUNIQ            Multiple matches were found for user's input action, but one match could not be determined
 * @retval      EHOSTUNREACH        Help will be printed for the target instead of performing another action
 */
int CEHRouteAction(char const *action,
                   struct CEHEngineConfig *engine,
                   int (**actionFunction)(void **, void *));

/**
 * @brief       Attempts to identify which target user meant to indicate, even if user input does
 * not contain an exact match
 *
 * @param       target              The user's input target
 * @param       targetEngine        Passed in for later use outside of this function; if matching
 * target is found, can be used externally later
 *
 * @retval      0                   A single match for the user's input target was identified
 * @retval      EINVAL              No unique target could be identified based on user input
 */
int CEHRouteEngine(char const *target, struct CEHEngineConfig **targetEngine);

/**
 * @brief       Adds a target to the list of targets
 *
 * If the engine name is not unique the new registration would fail
 *
 * @param       engineConfig        New target to be added
 *
 * @returns     Would return if the engine was added or not
 */
bool CEHRegisterEngine(struct CEHEngineConfig *engineConfig);

/**
 * @brief       Removes the target to the list of targets
 *
 * @param       engineConfig        New target to be added
 *
 * @returns     Would return if the engine was removed or not
 */
bool CEHUnregisterEngine(struct CEHEngineConfig *engineConfig);

/**
 * @brief       Cycles through each of the SEF-CLI targets and prints their help messages, including actions and options
 */
void CEHPrintAllEngineHelp();

/**
 * @brief       Prints help for one particular target of SEF-CLI (includes actions and options for the target)
 *
 * @param       engine      The SEF-CLI target for which help should be printed
 */
void CEHPrintEngineHelp(struct CEHEngineConfig *engine);

/**
 * @brief       A helper to iterate over the registered engines
 *
 *  @param      func            The function called for each registered engine
 *  @param      data            A void pointer to be passed to the called function
 */
void CEHForEachEngine(void func(struct CEHEngineConfig *engine, void *data), void *data);

#endif /* SEF_ENGINE_H */
