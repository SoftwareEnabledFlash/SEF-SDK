/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * file-logger.c
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
#include "file-logger.h"

#include <assert.h>
#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <memory.h>
#include <pthread.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <wchar.h>

#include "../config.h"
#include "../sef-utils.h"
#include "dlist.h"
#include "hashset.h"

struct LogContext
{
    struct LogHandle_ logHandle;
    pthread_mutex_t fileMutex;    // lock for modifying handle members
    FILE *pFile;                  // output path for log file
    int logLevel;                 // level of verbosity for logging
};

// private functions
static const char *skipCommon(const char *fname, const char *q)
{
    const char *p = fname;
    mbstate_t ps = {0};
    mbstate_t qs = {0};
    ssize_t pn = 0;
    ssize_t qn = 0;

    do
    {
        p += pn;
        q += qn;
        pn = mbrlen(p, INT8_MAX, &ps);
        qn = mbrlen(q, INT8_MAX, &qs);
    } while (pn > 0 && pn == qn && memcmp(p, q, pn) == 0);
    return pn ? p : fname;
}

static void GetDateTime(char *dateTimeString)
{
    time_t rawTime;
    struct tm *timeInfo;

    time(&rawTime);
    timeInfo = localtime(&rawTime);

    sprintf(dateTimeString, "%02u/%02u/%4u %02u:%02u:%02u", timeInfo->tm_mday, timeInfo->tm_mon + 1,
            timeInfo->tm_year + 1900, timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
}

void DFLSetLevel(LogHandle logHandle, int level)
{
    assert(logHandle);
    struct LogContext *logContext = (struct LogContext *)logHandle;
    logContext->logLevel = level;
}

/**
 *  @ingroup    SefSdkLog
 *  @brief       Adds entry to existing log file.
 *
 *  @param       logHandle       Logging handle to be used for access to logging instance
 *  @param       level           Level of logging (debug, fatal, etc) under which new entry falls
 *  @param       file            File path for file which called to add log entry
 *  @param       line            Line within file which called to add log entry
 *  @param       function        Function within code which called to add log entry
 *  @param       format          Arguments to be placed in new log entry
 */
void DFLLogFile(LogHandle logHandle,
                int level,
                const char *file,
                int line,
                const char *function,
                const char *format,
                ...)
{
    struct LogContext *logContext;

    if (logHandle == NULL)
    {
        return;
    }

    logContext = (struct LogContext *)logHandle;
    assert(logHandle->logLevel >= kTrace && logHandle->logLevel <= kNoLog);
    if (level >= logHandle->logLevel && level > -1 && level <= kFatal)
    {
        char dateTimeString[20];

        // Lock
        pthread_mutex_lock(&(logContext->fileMutex));

        GetDateTime(dateTimeString);

        file = skipCommon(file, __FILE__);

        // Log to file
        if (logContext->pFile != NULL)
        {
            va_list args;

            fprintf(logContext->pFile, "%s %-5s %s[%s:%d]:", dateTimeString, LogLevelToName(level),
                    function, file, line);

            va_start(args, format);
            vfprintf(logContext->pFile, format, args);
            va_end(args);

            fprintf(logContext->pFile, "\n");
            fflush(logContext->pFile);
        }

        // unlock
        pthread_mutex_unlock(&(logContext->fileMutex));
    }
}

// public functions
void DFLInit(LogHandle *logHandle, const char *filePath)
{
    if (filePath == NULL)
    {
        return;
    }

    // allocate memory to the log handle
    struct LogContext *logContext = SUmalloc(sizeof(struct LogContext));
    (*logHandle) = (LogHandle)logContext;

    // populate log handle
    (*logHandle)->logger.log = DFLLogFile;
    (*logHandle)->logger.setLogLevel = DFLSetLevel;
    (*logHandle)->logLevel = kTrace;

    // populate log context
    logContext->logLevel = kTrace;
    pthread_mutex_init(&logContext->fileMutex, NULL);

    // open the log file - assume it already exists (having O_CREATE can
    // cause permissions errors in sticky-bit directories when this user doesn't
    // match the owner)
    int fd = open(filePath, O_WRONLY | O_APPEND);
    if (fd != -1)
    {
        logContext->pFile = fdopen(fd, "a");
    }
    else
    {
        // assume there was no file - retry and create it so everyone can rw
        mode_t old = umask(0);
        logContext->pFile = fopen(filePath, "a");
        umask(old);
    }
    if (logContext->pFile == NULL)
    {
        // cleanup memory
        SUfree(*logHandle);
        (*logHandle) = NULL;

        printf("Error: Was unable to access log file\n");
        return;
    }
}

void DFLCleanup(LogHandle logHandle)
{
    if (logHandle == NULL)
    {
        return;
    }

    // close log file
    struct LogContext *logContext = (struct LogContext *)logHandle;
    if (logContext->pFile)
    {
        fclose(logContext->pFile);
    }

    // free handle
    pthread_mutex_destroy(&(logContext->fileMutex));
    free(logContext);
}
