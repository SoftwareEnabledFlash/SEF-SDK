/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * log-manager.h
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
/** @defgroup  SefSdkLog Logging API */

#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H

typedef struct LogHandle_ *LogHandle;

/**
 *  @ingroup    SefSdkLog
 *  @brief      Interface structure for a logger
 */
struct iLogger
{
    void (*log)(LogHandle logHandle,
                int level,
                const char *file,
                int line,
                const char *function,
                const char *format,
                ...); /**< Function called to write a line to a logger */
    void (*setLogLevel)(LogHandle logHandle, int level); /**< Sets the logging level for a logger */
};

/**
 *  @ingroup    SefSdkLog
 *  @brief      Implementation for LogHandle
 */
struct LogHandle_
{
    struct iLogger logger; /**< Interface to logging implementation */
    int logLevel;          /**< Current log level */
};

/**
 *  @ingroup    SefSdkLog
 *  @brief      Log levels for configuring and logging
 */
enum LogLevels { kTrace, kDebug, kInfo, kError, kFatal, kNoLog };

#define LogTrace(LogHandle, ...)                                                                 \
    do                                                                                           \
    {                                                                                            \
        if (LogHandle != NULL && kTrace >= LogHandle->logLevel)                                  \
            LogHandle->logger.log(LogHandle, kTrace, __FILE__, __LINE__, __func__, __VA_ARGS__); \
    } while (0)

#define LogDebug(LogHandle, ...)                                                                 \
    do                                                                                           \
    {                                                                                            \
        if (LogHandle != NULL && kDebug >= LogHandle->logLevel)                                  \
            LogHandle->logger.log(LogHandle, kDebug, __FILE__, __LINE__, __func__, __VA_ARGS__); \
    } while (0)

#define LogInfo(LogHandle, ...)                                                                 \
    do                                                                                          \
    {                                                                                           \
        if (LogHandle != NULL && kInfo >= LogHandle->logLevel)                                  \
            LogHandle->logger.log(LogHandle, kInfo, __FILE__, __LINE__, __func__, __VA_ARGS__); \
    } while (0)

#define LogError(LogHandle, ...)                                                                 \
    do                                                                                           \
    {                                                                                            \
        if (LogHandle != NULL && kError >= LogHandle->logLevel)                                  \
            LogHandle->logger.log(LogHandle, kError, __FILE__, __LINE__, __func__, __VA_ARGS__); \
    } while (0)

#define LogFatal(LogHandle, ...)                                                                 \
    do                                                                                           \
    {                                                                                            \
        if (LogHandle != NULL && kFatal >= LogHandle->logLevel)                                  \
            LogHandle->logger.log(LogHandle, kFatal, __FILE__, __LINE__, __func__, __VA_ARGS__); \
    } while (0)

/**
 *  @ingroup    SefSdkLog
 *  @brief      Changes level of verbosity for logging calls using numbered levels.
 *
 *  @param      logHandle        Logging handle to be used for access to logging instance
 *  @param      level            New level of verbosity for associated log file
 */
void LogSetLevel(LogHandle logHandle, int level);

/**
 *  @ingroup    SefSdkLog
 *  @brief      Changes level of verbosity for logging calls using level names.
 *
 *  @param      logHandle        Logging handle to be used for access to logging instance
 *  @param      level            Name of level of verbosity to be set for associated log file
 */
void LogSetLevelByName(LogHandle logHandle, char *level);

/**
 *  @ingroup    SefSdkLog
 *  @brief      Returns current level of verbosity for associated log file.
 *
 *  @param      logHandle        Logging handle to be used for access to logging instance
 *
 *  @return     Current level of log verbosity
 */
int LogGetLevel(LogHandle logHandle);

/**
 *  @ingroup    SefSdkLog
 *  @brief      Given a logging handle, returns name of log's current level of verbosity.
 *
 *  @param      logHandle        Logging handle to be used for access to logging instance
 *  @param[out] levelName        char* to hold the name of the current log verbosity level
 *  @param      bufferSize       Size of the out buffer.
 */
void LogGetLevelName(LogHandle logHandle, char *levelName, int bufferSize);

/**
 *  @ingroup    SefSdkLog
 *  @brief      Given an int logging level, returns the name of that verbosity level.
 *
 *  @param      level           enum for a logging verbosity level
 *
 *  @return     Name of the log level
 */
const char *LogLevelToName(int level);

#endif /* LOG_MANAGER_H */
