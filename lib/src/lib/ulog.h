/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * ulog.h
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

#ifndef __ULOG_H__
#define __ULOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#ifndef MODULE_NAME_SELECT
#define MODULE_NAME_SELECT \
    "ANONYM"    //!< Module name when no module name is specified (assumes TP)
#endif

#define ULOG_MAX_SYSLOG_MSG (1024)    //!< Limited by: RFC 3164 The BSD syslog Protocol

/**
 *  @brief	ULOG log format selection (bit03-00)
 */
enum UlogFormatSelect {
    ULOG_FORMAT_SELECT_ANY = 0x00000000,    //!< Applies individually selected format (normal or extended)
    ULOG_FORMAT_SELECT_NORMAL = 0x00000001,    //!< Applies normal format to all logs
    ULOG_FORMAT_SELECT_EXTEND = 0x00000002,    //!< Applies extended format to all logs

    ULOG_FORMAT_SELECT_MASK = 0x0000000F,      //!< Mask
    ULOG_FORMAT_SELECT_FORCE32 = 0xFFFFFFFF    //!< Dummy
};

/**
 *  @brief	ULOG log level configuration (bit11-04)
 *  @note    Used as a flag
 */
enum UlogLevel {
    ULOG_LEVEL_ERROR = 0x00000010,          //!< Error (on by default)
    ULOG_LEVEL_NOTICE = 0x00000020,         //!< Notice (on by default)
    ULOG_LEVEL_IMPORTANT = 0x00000040,      //!< Important
    ULOG_LEVEL_INFORMATION = 0x00000080,    //!< Information
    ULOG_LEVEL_DEBUG = 0x00000100,          //!< Debug

    ULOG_LEVEL_MASK = 0x00000FF0,      //!< Mask
    ULOG_LEVEL_FORCE32 = 0xFFFFFFFF    //!< Dummy
};

/**
 *  @brief	ULOG log category configuration (bit31-12)
 *  @note    Used as a flag
 */
enum UlogCategory {
    ULOG_CATEGORY_ERROR = 0x00001000,          //!< Error
    ULOG_CATEGORY_NOTICE = 0x00002000,         //!< Notice
    ULOG_CATEGORY_IMPORTANT = 0x00004000,      //!< Important
    ULOG_CATEGORY_INFORMATION = 0x00008000,    //!< Information
    ULOG_CATEGORY_DEBUG = 0x00010000,          //!< Debug

    ULOG_CATEGORY_SYSTEM_INFO = 0x00040000,      //!< System information
    ULOG_CATEGORY_SQ_INFO = 0x00080000,          //!< SQ information
    ULOG_CATEGORY_CQ_INFO = 0x00100000,          //!< CQ information
    ULOG_CATEGORY_WRITE_COMMAND = 0x00200000,    //!< Write command information
    ULOG_CATEGORY_READ_COMMAND = 0x00400000,     //!< Read command information
    ULOG_CATEGORY_COPY_COMMAND = 0x00800000,     //!< Copy command information
    ULOG_CATEGORY_SBMNG_COMMAND = 0x01000000,    //!< SB management command information
    ULOG_CATEGORY_GETINFO_DATA = 0x10000000,     //!< Device information retrieved by GetInfo
    ULOG_CATEGORY_ENTER_SEFAPI = 0x40000000,     //!< Entering information of SEFAPI
    ULOG_CATEGORY_LEAVE_SEFAPI = 0x80000000,     //!< Leaving information of SEFAPI

    ULOG_CATEGORY_MASK = 0xFFFFF000,      //!< Mask
    ULOG_CATEGORY_FORCE32 = 0xFFFFFFFF    //!< Dummy
};

/*
 * Macros for log output
 */
#define ULOG_PRINT(mod, syslogLv, ulogLv, cate, pre, ext, ...) \
    UtilLogPrint(mod, syslogLv, ulogLv, cate, pre, ext, __func__, __LINE__, __VA_ARGS__)

#define ULOG_ERROR(...)                                                                           \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_ERR, ULOG_LEVEL_ERROR, ULOG_CATEGORY_ERROR, "ERROR", true, \
               __VA_ARGS__)
#define ULOG_NOTICE(...)                                                                       \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_NOTICE, ULOG_CATEGORY_NOTICE, "NOTIC", \
               false, __VA_ARGS__)
#define ULOG_IMPORTANT(...)                                                                 \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_IMPORTANT, \
               "IMPOR", false, __VA_ARGS__)
#define ULOG_INFORMATION(...)                                                                   \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_INFORMATION, \
               "INFOR", false, __VA_ARGS__)
#define ULOG_DEBUG(...)                                                                            \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_DEBUG, ULOG_CATEGORY_DEBUG, "DEBUG", true, \
               __VA_ARGS__)
#define ULOG_SYSTEM(...)                                                                   \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_NOTICE, ULOG_CATEGORY_SYSTEM_INFO, \
               "SYSTM", false, __VA_ARGS__)
#define ULOG_SQINFO(...)                                                                           \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_SQ_INFO, "SQINF", \
               false, __VA_ARGS__)
#define ULOG_CQINFO(...)                                                                           \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_CQ_INFO, "CQINF", \
               false, __VA_ARGS__)
#define ULOG_WTCMD_IMP(...)                                                                     \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_WRITE_COMMAND, \
               "WTCMD", false, __VA_ARGS__)
#define ULOG_WTCMD_INF(...)                                                                       \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_WRITE_COMMAND, \
               "WTCMD", false, __VA_ARGS__)
#define ULOG_WTCMD_DBG(...)                                                                 \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_DEBUG, ULOG_CATEGORY_WRITE_COMMAND, \
               "WTCMD", true, __VA_ARGS__)
#define ULOG_RDCMD_IMP(...)                                                                    \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_READ_COMMAND, \
               "RDCMD", false, __VA_ARGS__)
#define ULOG_RDCMD_INF(...)                                                                      \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_READ_COMMAND, \
               "RDCMD", false, __VA_ARGS__)
#define ULOG_RDCMD_DBG(...)                                                                \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_DEBUG, ULOG_CATEGORY_READ_COMMAND, \
               "RDCMD", true, __VA_ARGS__)
#define ULOG_CPCMD_IMP(...)                                                                    \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_COPY_COMMAND, \
               "CPCMD", false, __VA_ARGS__)
#define ULOG_CPCMD_INF(...)                                                                      \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_COPY_COMMAND, \
               "CPCMD", false, __VA_ARGS__)
#define ULOG_CPCMD_DBG(...)                                                                \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_DEBUG, ULOG_CATEGORY_COPY_COMMAND, \
               "CPCMD", true, __VA_ARGS__)
#define ULOG_SBCMD_IMP(...)                                                                     \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_IMPORTANT, ULOG_CATEGORY_SBMNG_COMMAND, \
               "SBCMD", false, __VA_ARGS__)
#define ULOG_SBCMD_INF(...)                                                                       \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_SBMNG_COMMAND, \
               "SBCMD", false, __VA_ARGS__)
#define ULOG_SBCMD_DBG(...)                                                                 \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_DEBUG, ULOG_CATEGORY_SBMNG_COMMAND, \
               "SBCMD", true, __VA_ARGS__)
#define ULOG_GETINFO_DATA(...)                                                             \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_DEBUG, ULOG_CATEGORY_GETINFO_DATA, \
               "GINFO", true, __VA_ARGS__)
#define ULOG_ENTER_SEFAPI()                                                                      \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_ENTER_SEFAPI, \
               "ENTER", false, __func__)
#define ULOG_LEAVE_SEFAPI()                                                                      \
    ULOG_PRINT(MODULE_NAME_SELECT, LOG_INFO, ULOG_LEVEL_INFORMATION, ULOG_CATEGORY_LEAVE_SEFAPI, \
               "LEAVE", false, __func__)

/*
 * Function prototypes
 */
#if !defined(_MSC_VER)
int UtilLogPrint(const char *module,
                 int syslogLevel,
                 enum UlogLevel ulogLevel,
                 enum UlogCategory category,
                 const char *prefix,
                 bool extend,
                 const char *func,
                 int line,
                 const char format[],
                 ...) __attribute__((format(printf, 9, 10)));
#else
int UtilLogPrint(const char *module,
                 int syslogLevel,
                 enum UlogLevel ulogLevel,
                 enum UlogCategory category,
                 const char *prefix,
                 bool extend,
                 const char *func,
                 int line,
                 const char format[],
                 ...);
}
#endif
uint32_t UtilLogGetConfig(void);
void UtilLogSetConfig(uint32_t config);

#ifdef __cplusplus
}
#endif

#endif /* __ULOG_H__ */
