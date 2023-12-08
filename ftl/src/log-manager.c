/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * log-manager.c
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
#include "log-manager.h"

#include <assert.h>
#include <pthread.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "sef-utils.h"
#include "utils/dlist.h"
#include "utils/hashset.h"

// verbosity level names for logs
const char *LogLevelNames[] = {"Trace", "Debug", "Info", "Error", "Fatal", "NoLog"};

void LogSetLevel(LogHandle logHandle, int level)
{
    assert(logHandle);
    logHandle->logger.setLogLevel(logHandle, level);
    logHandle->logLevel = level;
}

void LogSetLevelByName(LogHandle logHandle, char *level)
{
    assert(logHandle);
    for (int i = 0; i < NELEM(LogLevelNames); i++)
    {
        if (strcasecmp(level, LogLevelNames[i]) == 0)
        {
            logHandle->logger.setLogLevel(logHandle, i);
            logHandle->logLevel = i;
            return;
        }
    }
}

int LogGetLevel(LogHandle logHandle)
{
    assert(logHandle);
    return logHandle->logLevel;
}

void LogGetLevelName(LogHandle logHandle, char *levelName, int bufferSize)
{
    assert(logHandle);
    int charNum = bufferSize / sizeof(char) - 1;
    strncpy(levelName, LogLevelNames[logHandle->logLevel], charNum);
    levelName[charNum] = '\0';
}

const char *LogLevelToName(int level)
{
    return LogLevelNames[level];
}
