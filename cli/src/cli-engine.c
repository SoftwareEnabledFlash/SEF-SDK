/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * cli-engine.c
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
#include "cli-engine.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "engine-option.h"
#include "io-helper.h"
#include "utils/dlist.h"
#include "utils/str-builder.h"

#define arrayVal(R, C, A, CS) *(A + ((C) * CS) + (R))

pthread_once_t cliEnginesInitialized = PTHREAD_ONCE_INIT;
TmaDList cliEngines;

// Creates a list (once only) to contain all of the SEF-CLI targets
void initCliEnginesList()
{
    utl_DListInit(&cliEngines);
}

bool CEHRegisterEngine(struct CEHEngineConfig *engineConfig)
{
    struct CEHEngineConfig *engine = NULL;

    // init the cli engines list
    pthread_once(&cliEnginesInitialized, initCliEnginesList);

    // checking for duplicate engines
    while ((engine = utl_DListNextAs(&cliEngines, engine, struct CEHEngineConfig, link)))
    {
        if (strcmp(engine->name, engineConfig->name) == 0)
        {
            CIHMessageInfo(
                "Warning: The engine %s is already registered. Will ignore the duplicate", engine->name);
            return false;
        }
    }

    // add the entry to the list
    utl_DListInitEntry(&engineConfig->link);
    utl_DListPushHead(&cliEngines, &engineConfig->link);

    return true;
}

bool CEHUnregisterEngine(struct CEHEngineConfig *engineConfig)
{
    struct CEHEngineConfig *engine = NULL;

    while ((engine = utl_DListNextAs(&cliEngines, engine, struct CEHEngineConfig, link)))
    {
        if (strcmp(engine->name, engineConfig->name) == 0)
        {
            utl_DListRemove(&engine->link);
            return true;
        }
    }

    return false;
}

int strDist(const char *str1, const char *str2)
{
    int *matrix;
    int strDist, str1Size, str2Size, i, j;

    if (str1 == NULL || str2 == NULL || strlen(str1) == 0 || strlen(str2) == 0)
    {
        return -1;
    }

    // compare least common size
    str1Size = strlen(str1) + 1;
    str2Size = strlen(str2) + 1;

    // create a matrix to calculate distance
    matrix = calloc(1, sizeof(int) * str1Size * str2Size);

    // init the matrix
    for (i = 0; i < str1Size; i++)
    {
        arrayVal(i, 0, matrix, str1Size) = i;
    }

    for (i = 0; i < str2Size; i++)
    {
        arrayVal(0, i, matrix, str1Size) = i;
    }

    for (i = 1; i < str1Size; i++)
    {
        char c1 = str1[i - 1];

        for (j = 1; j < str2Size; j++)
        {
            int minCost, cost = 0;
            char c2 = str2[j - 1];

            if (c1 != c2)
            {
                cost = 1;
            }

            minCost = arrayVal(i - 1, j - 1, matrix, str1Size) + cost;

            if (minCost > (arrayVal(i - 1, j, matrix, str1Size) + 1))
            {
                minCost = arrayVal(i - 1, j, matrix, str1Size) + 1;
            }

            if (minCost > (arrayVal(i, j - 1, matrix, str1Size) + 1))
            {
                minCost = arrayVal(i, j - 1, matrix, str1Size) + 1;
            }

            arrayVal(i, j, matrix, str1Size) = minCost;
        }
    }

    strDist = matrix[(str1Size * str2Size) - 1];

    free(matrix);

    return strDist;
}

int CEHRouteAction(char const *action,
                   struct CEHEngineConfig *engine,
                   int (**actionFunction)(void **, void *))
{
    int i, actionLen, bestMatchDist = 100, matches = 0;
    const char *bestMatch;

    actionLen = strlen(action);
    for (i = 0; i < engine->actionsNum; i++)
    {
        if (strncasecmp(action, engine->actions[i].name, actionLen) == 0)
        {
            *actionFunction = engine->actions[i].action;
            matches++;
        }
        else if (matches == 0)
        {
            int matchDist = strDist(action, engine->actions[i].name);
            if (matchDist < bestMatchDist)
            {
                bestMatchDist = matchDist;
                bestMatch = engine->actions[i].name;
            }
        }
    }

    // match found
    if (matches == 1)
    {
        return 0;
    }

    // multiple matches found
    if (matches > 1)
    {
        CIHMessageError(
            "Error: The action value '%s' is ambiguous and matching multiple supported actions", action);
        return EINVAL;
    }

    // no target-specific help; show the default
    if (strncasecmp(action, "help", actionLen) == 0)
    {
        // force verbose
        CIHSetLevel(kInfo);

        // print sef engine's help
        CEHPrintEngineHelp(engine);
        return EHOSTUNREACH;
    }

    // no action; show closest match for action
    CIHMessageError("Error: The action value '%s' did not match any supported actions", action);

    if (bestMatchDist < 7)
    {
        CIHMessageData("\nThe most similar action is `%s`", bestMatch);
    }

    return EINVAL;
}

int CEHRouteEngine(char const *target, struct CEHEngineConfig **targetEngine)
{
    struct CEHEngineConfig *engine = NULL;
    int targetLen, bestMatchDist = 100, matches = 0;
    const char *bestMatch;

    targetLen = strlen(target);

    while ((engine = utl_DListNextAs(&cliEngines, engine, struct CEHEngineConfig, link)))
    {
        if (strncasecmp(target, engine->name, targetLen) == 0)
        {
            *targetEngine = engine;
            matches++;
        }
        else if (matches == 0)
        {
            int matchDist = strDist(target, engine->name);
            if (matchDist < bestMatchDist)
            {
                bestMatchDist = matchDist;
                bestMatch = engine->name;
            }
        }
    }

    if (matches == 1)
    {
        return 0;
    }

    if (matches > 1)
    {
        CIHMessageError(
            "Error: The target value '%s' is ambiguous and matching multiple supported targets", target);
    }
    else
    {
        CIHMessageError("Error: The target value '%s' did not match any supported targets", target);

        if (bestMatchDist < 7)
        {
            CIHMessageData("\nThe most similar target is `%s`", bestMatch);
        }
    }

    return EINVAL;
}

void CEHPrintAllEngineHelp()
{
    struct CEHEngineConfig *engine = NULL;

    // print the targets
    CIHMessageDataNoWrap(CIHCreateHeader1("Target"));
    CIHMessageData("The following targets are supported by your sef-cli software:");

    // print target titles only
    engine = NULL;
    while ((engine = utl_DListNextAs(&cliEngines, engine, struct CEHEngineConfig, link)))
    {
        CIHMessagePrint(kData, 4, 20, "%-20s %s", engine->name, engine->description);
    }

    // print targets help
    engine = NULL;
    while ((engine = utl_DListNextAs(&cliEngines, engine, struct CEHEngineConfig, link)))
    {
        CEHPrintEngineHelp(engine);
    }
}

void CEHPrintEngineHelp(struct CEHEngineConfig *engine)
{
    int i;
    char *sefOptionTypeName[] = CEOOptionTypeName;
    char *sefOptionBitSizeName[] = CEOBitSizeName;

    CIHMessageDataNoWrap(CIHCreateHeader2(engine->name));

    if (engine->description)
    {
        CIHMessageData("%s", engine->description);
    }

    // print the actions
    if (engine->actionsNum)
    {
        bool hasHelp = false;

        CIHMessageDataNoWrap("\nActions:");
        for (i = 0; i < engine->actionsNum; i++)
        {
            if (strcmp(engine->actions[i].name, "help") == 0)
            {
                hasHelp = true;
            }

            if (engine->actions[i].description)
            {
                CIHMessagePrint(kData, 4, 20, "%-20s %s", engine->actions[i].name,
                                engine->actions[i].description);
            }
            else
            {
                CIHMessagePrint(kData, 4, 20, "%-20s", engine->actions[i].name);
            }
        }

        if (!hasHelp)
        {
            CIHMessagePrint(kData, 4, 20, "%-20s %s", "help", "Print verbose help for the target");
        }
    }

    // print target parameters
    if (engine->optionsNum)
    {
        CIHMessageInfo("\nParameters:");
        for (i = 0; i < engine->optionsNum; i++)
        {
            char *optionInfoOut;
            struct StrBuilder optionInfo;

            StrBldInit(&optionInfo, 32);

            CIHMessagePrint(kInfo, 2, 26, "-%c --%-21.20s %s", engine->options[i].shortName,
                            engine->options[i].longName, engine->options[i].description);

            if (engine->options[i].type == kSefOptionInt ||
                engine->options[i].type == kSefOptionUnsignedInt)
            {
                StrBldAppendFormat(&optionInfo, "%s(%s)%5s", sefOptionTypeName[engine->options[i].type],
                                   sefOptionBitSizeName[engine->options[i].bitSize], "");
            }
            else
            {
                StrBldAppendFormat(&optionInfo, "%-26s", sefOptionTypeName[engine->options[i].type]);
            }

            if (engine->options[i].IsArray)
            {
                StrBldAppend(&optionInfo, "[Array] ");
            }

            if (engine->options[i].IsTuple)
            {
                StrBldAppend(&optionInfo, "[Tuple] ");
            }

            if (engine->options[i].defaultVal != NULL)
            {
                StrBldAppendFormat(&optionInfo, "Default: %-15s", engine->options[i].defaultVal);
            }

            if (engine->options[i].type == kSefOptionEnum)
            {
                StrBldAppendFormat(&optionInfo, "Possible Values: %s", engine->options[i].enumVal);
            }

            // To avoid the -Wformat-security warning and possible attack vectors
            optionInfoOut = StrBldToString(&optionInfo);
            CIHMessagePrint(kInfo, 29, 0, "%s", optionInfoOut);

            StrBldCleanup(&optionInfo);
            free(optionInfoOut);
        }
    }
}

void CEHForEachEngine(void func(struct CEHEngineConfig *engine, void *data), void *data)
{
    struct CEHEngineConfig *engine = NULL;

    while ((engine = utl_DListNextAs(&cliEngines, engine, struct CEHEngineConfig, link)))
    {
        func(engine, data);
    }
}
