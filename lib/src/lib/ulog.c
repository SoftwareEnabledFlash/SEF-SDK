/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * ulog.c
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

#define MODULE_NAME_SELECT "ULOG"

#include "ulog.h"

#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "common.h"

/**
 *  @brief	ULOG configuration
 *  @see    enum UlogLevel, enum UlogCategory
 */
#ifdef SEFLIB_DEBUG_LOG
STATIC uint32_t logConfig =
    0xFFFFFFFF;    // Enables all logs (default configuration is applied to output format)
#else
STATIC uint32_t logConfig = (uint32_t)(ULOG_CATEGORY_MASK | ULOG_LEVEL_ERROR |
                                       ULOG_LEVEL_NOTICE);    // Enables ERROR and NOTICE
#endif

/**
 *  @brief	Retrieves elapsed time since system boot
 *  @return	Elapsed time [us]
 */
STATIC uint64_t GetTimeMsec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec) * 1000000 + (ts.tv_nsec) / 1000;
}

/**
 *  @brief	Returns whether specified log category and log level is valid for log output
 *  @param	[in] category: module category
 *  @param	[in] level: log level
 *  @return	true: log output is valid; false: log output is invalid
 */
STATIC bool IsLogValidated(enum UlogCategory category, enum UlogLevel level)
{
    uint32_t config = ((uint32_t)category) | ((uint32_t)level);
    if ((logConfig & config) != config)
    {
        return false;
    }
    return true;
}

/**
 *  @brief	Output string and parameters in specified output format to standard error
 *  @param	[in] module: module name
 *  @param	[in] syslogLv: syslog level
 *  @param	[in] ulogLv: ulog lovel
 *  @param	[in] category: log category
 *  @param	[in] prefix: log prefix
 *  @param	[in] extend: whether to output extended information (module name, function name, line
 * number)
 *  @param	[in] func: function name
 *  @param	[in] line: line number
 *  @param	[in] format: output string
 *  @return	1 or larger: log output succeeded (number of characters in output); 0: log output
 * suppressed; negative number: log output failed
 *  @note	Output formats are:
 *  @note	Standard: seflib:[ThreadID][Elapsed time since first log output] Log prefix: Log text
 *  @note	Extended: seflib:[ThreadID][Elapsed time since first log output] Log prefix: Module
 * name:Function name:Line number Log text
 *  @note	Format of Elapsed time since first log output is: [HHHH:UUUUUUUUUU] H: Elapsed time[h],
 * U:Elapsed time[usec]
 */
int UtilLogPrint(const char *module,
                 int syslogLevel,
                 enum UlogLevel ulogLevel,
                 enum UlogCategory category,
                 const char *prefix,
                 bool extend,
                 const char *func,
                 int line,
                 const char format[],
                 ...)
{
    static uint64_t firstTime =
        0;    // Elapsed time at the first call is retained (to calculate difference at subsequent calls)
    uint64_t nowTime;
    unsigned long threadId;
    size_t writableSize;
    char *pBuf;
    char *pStr;
    int num;
    va_list ap;
    enum UlogFormatSelect formatSelect;
    int ret;

    // Processing at the first call
    if (firstTime == 0)
    {
        const char *pEnvUlogConfigStr = (const char *)getenv("SEFLIB_ULOG_CONFIG");

        firstTime = GetTimeMsec();

        // Check environment variable and update log settings
        if (pEnvUlogConfigStr)
        {
            logConfig = (uint32_t)strtoul(pEnvUlogConfigStr, NULL, 0);
        }
    }

    // Do nothing if output setting is disabled
    if (!IsLogValidated(category, ulogLevel))
    {
        return 0;
    }

    nowTime = GetTimeMsec();
    threadId = (unsigned long)pthread_self();

    // Check output setting of extended information
    formatSelect = logConfig & ((uint32_t)ULOG_FORMAT_SELECT_MASK);
    if (formatSelect == ULOG_FORMAT_SELECT_NORMAL)
    {
        extend = false;
    }
    else if (formatSelect == ULOG_FORMAT_SELECT_EXTEND)
    {
        extend = true;
    }
    else
    {
        // Individual configuration is applied
    }

    // Perpare output string in buffer
    writableSize = ULOG_MAX_SYSLOG_MSG;
    pBuf = (char *)malloc(writableSize);
    if (pBuf == NULL)
    {
        return -1;
    }
    pStr = pBuf;
    if (nowTime > 0LL)
    {
        uint64_t diff = nowTime - firstTime;
        uint64_t hour = diff / 1000000 / 3600;
        uint64_t micro = diff - hour * 1000000 * 3600;
        num = snprintf(pStr, writableSize, "seflib:[%05lu][%04lu:%010lu] %s: ", threadId, hour,
                       micro, prefix);
    }
    else
    {
        num = snprintf(pStr, writableSize, "seflib:[%05lu] %s: ", threadId, prefix);
    }
    if (extend)
    {
        pStr += num;
        writableSize -= num;
        num = snprintf(pStr, writableSize, "%s:%s:L%05d ", module, func, line);
    }
    pStr += num;
    writableSize -= num;

    va_start(ap, format);
    ret = vsnprintf(pStr, writableSize, format, ap);
    va_end(ap);

    // Output log prepared in buffer
    syslog(syslogLevel, "%s", pBuf);

    free(pBuf);

    return ret;
}

/**
 *  @brief	Retrieves log configuration
 *  @return	Current log configuration
 */
uint32_t UtilLogGetConfig(void)
{
    return logConfig;
}

/**
 *  @brief	Set log configuration
 *  @param	[in] config: log configuration
 *  @return	None
 */
void UtilLogSetConfig(uint32_t config)
{
    logConfig = config;
}
