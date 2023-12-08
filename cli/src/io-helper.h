/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * io-helper.h
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
#ifndef IO_HELPER_H
#define IO_HELPER_H

#include <stdbool.h>
#include <stdint.h>

// Possible output levels for different printed messages:
enum CIHMessageLevels { kInfo, kData, kError };

// Print a message at the output level kInfo (no text wrapping):
#define CIHMessageInfoNoWrap(...) CIHMessagePrint(kInfo, -1, -1, __VA_ARGS__)

// Print a message at the output level kData (no text wrapping):
#define CIHMessageDataNoWrap(...) CIHMessagePrint(kData, -1, -1, __VA_ARGS__)

// Print a message at the output level kError (no text wrapping):
#define CIHMessageErrorNoWrap(...) CIHMessagePrint(kError, -1, -1, __VA_ARGS__)

// Perform the same way as their above counterparts with "NoWrap" on the end, but these do implement text wrapping
#define CIHMessageInfo(...)  CIHMessagePrint(kInfo, 0, 0, __VA_ARGS__)
#define CIHMessageData(...)  CIHMessagePrint(kData, 0, 0, __VA_ARGS__)
#define CIHMessageError(...) CIHMessagePrint(kError, 0, 0, __VA_ARGS__)

// Two different header styles, with 1 being for more important titles
#define CIHCreateHeader1(TEXT)                                                                      \
    "=========================== %*s%*s ===========================", (int)(10 + strlen(TEXT) / 2), \
        TEXT, (int)(10 - strlen(TEXT) / 2), ""
#define CIHCreateHeader2(TEXT)                                        \
    "--------------------------- %*s%*s ---------------------------", \
        (int)(10 + strlen(TEXT) / 2), TEXT, (int)(10 - strlen(TEXT) / 2), ""

/**
 * @brief       Changes global message level
 *
 * @param       level       New level for messages output to terminal
 */
void CIHSetLevel(int level);

/**
 * @brief       Sets terminal output to given number of characters (default is 80)
 *
 * @param       new_width       New width (in characters) before text wrapping takes effect
 */
void CIHSetOutputWidth(int new_width);

/**
 * @brief       Prints a message to standard output, and provides optional text wrapping
 *
 * @param       Level               The level for the message being created (e.g. kInfo, kError, etc)
 * @param       frontIndent         Indent to be placed before any characters on each line, even after wrapping
 * @param       secondLineOffset    Additional indent to be applied after the first line (in case second needs further
 *                                      indentation)
 * @param       Format              The char* to be printed to output
 */
void CIHMessagePrint(int Level, int frontIndent, int secondLineOffset, const char *Format, ...)
    __attribute__((format(printf, 4, 5)));

/**
 * @brief       Queries user for a yes/no (true/false) answer to a prompt
 *
 * @param       MessagePrompt       The output (question) presented to the user
 *
 * @retval      true                User's answer was yes
 */
bool CIHGetBoolInput(const char *MessagePrompt, ...);

/**
 * @brief       Checks that user-supplied values are within property constraints
 *
 * @param       PropertyName        Name of property for which range is being checked
 * @param       PropertyShortName   Abbreviation (short form) for property for which range is being checked
 * @param       Value               The user-supplied value, to be checked
 * @param       MinValue            The minimum acceptable (within-range) value
 * @param       MaxValue            The maximum acceptable (within-range) value
 *
 * @retval      true                Value is outside of acceptable range
 */
bool CIHIsOutOfRange(const char *PropertyName,
                     char PropertyShortName,
                     uint64_t Value,
                     uint64_t MinValue,
                     uint64_t MaxValue);

/**
 * @brief       Checks that a necessary option/argument was provided by user, and displays message to user if not
 *
 * @param       PropertyName        The necessary property whose presence will be checked
 * @param       PropertyShortName   Abbreviation (short form) for property for which range is being checked
 * @param       IsGiven             Value indicating whether or not needed argument was provided by user
 *
 * @retval      true                Argument was not provided by the user
 */
bool CIHIsNotGiven(const char *PropertyName, char PropertyShortName, bool IsGiven);

#endif /* IO_HELPER_H */
