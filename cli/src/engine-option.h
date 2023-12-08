/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * engine-option.h
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
#ifndef ENGINE_OPTION_H
#define ENGINE_OPTION_H

#include <stdbool.h>
#include <stdint.h>

#define CEOFlag(SH, LN, D, S, O)                                                       \
    {                                                                                  \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL,         \
        .type = kSefOptionFlag, .structOffset = offsetof(struct S, O), .bitSize = k64, \
        .IsArray = false, .IsTuple = false                                             \
    }

#define CEOInt(SH, LN, D, BS, S, O, GO)                                                                \
    {                                                                                                  \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL,                         \
        .type = kSefOptionInt, .structOffset = offsetof(struct S, O),                                  \
        .structGivenOffset = offsetof(struct S, GO), .bitSize = BS, .IsArray = false, .IsTuple = false \
    }

#define CEOIntDefault(SH, LN, D, BS, V, S, O)                                                      \
    {                                                                                              \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = V, .type = kSefOptionInt, \
        .structOffset = offsetof(struct S, O), .bitSize = BS, .IsArray = false, .IsTuple = false   \
    }

#define CEOUInt(SH, LN, D, BS, S, O, GO)                                                               \
    {                                                                                                  \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL,                         \
        .type = kSefOptionUnsignedInt, .structOffset = offsetof(struct S, O),                          \
        .structGivenOffset = offsetof(struct S, GO), .bitSize = BS, .IsArray = false, .IsTuple = false \
    }

#define CEOUIntDefault(SH, LN, D, BS, V, S, O)                                               \
    {                                                                                        \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = V,                  \
        .type = kSefOptionUnsignedInt, .structOffset = offsetof(struct S, O), .bitSize = BS, \
        .IsArray = false, .IsTuple = false                                                   \
    }

#define CEOChar(SH, LN, D, S, O, GO)                                                                    \
    {                                                                                                   \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL,                          \
        .type = kSefOptionChar, .structOffset = offsetof(struct S, O),                                  \
        .structGivenOffset = offsetof(struct S, GO), .bitSize = k64, .IsArray = false, .IsTuple = false \
    }

#define CEOString(SH, LN, D, S, O, GO)                                                                  \
    {                                                                                                   \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL,                          \
        .type = kSefOptionString, .structOffset = offsetof(struct S, O),                                \
        .structGivenOffset = offsetof(struct S, GO), .bitSize = k64, .IsArray = false, .IsTuple = false \
    }

#define CEOStringDefault(SN, LN, D, V, S, O)                                                          \
    {                                                                                                 \
        .shortName = SN, .longName = LN, .description = D, .defaultVal = V, .type = kSefOptionString, \
        .structOffset = offsetof(struct S, O), .IsArray = false, .IsTuple = false                     \
    }

#define CEOTupleDefault(SN, LN, D, V, T, S, O)                                                   \
    {                                                                                            \
        .shortName = SN, .longName = LN, .description = D, .defaultVal = V, .type = T,           \
        .structOffset = offsetof(struct S, O), .IsTuple = true, .IsArray = false, .bitSize = k64 \
    }

#define CEOTupleArray(SH, LN, D, T, S, O, GO, LO)                                                      \
    {                                                                                                  \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL, .type = T,              \
        .structOffset = offsetof(struct S, O), .structGivenOffset = offsetof(struct S, GO),            \
        .bitSize = k64, .structLengthOffset = offsetof(struct S, LO), .IsArray = true, .IsTuple = true \
    }

#define CEOEnumDefault(SH, LN, D, V, E, S, O)                                             \
    {                                                                                     \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = V, .enumVal = E, \
        .type = kSefOptionEnum, .structOffset = offsetof(struct S, O), .bitSize = k64,    \
        .IsArray = false, .IsTuple = false                                                \
    }

#define CEOUIntArray(SH, LN, D, BS, S, O, GO, LO)                                                  \
    {                                                                                              \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = NULL,                     \
        .type = kSefOptionUnsignedInt, .structOffset = offsetof(struct S, O),                      \
        .structGivenOffset = offsetof(struct S, GO), .structLengthOffset = offsetof(struct S, LO), \
        .bitSize = BS, .IsArray = true, .IsTuple = false                                           \
    }

#define CEOUIntArrayDefault(SH, LN, D, BS, V, S, O, LO)                                      \
    {                                                                                        \
        .shortName = SH, .longName = LN, .description = D, .defaultVal = V,                  \
        .type = kSefOptionUnsignedInt, .structOffset = offsetof(struct S, O), .bitSize = BS, \
        .structLengthOffset = offsetof(struct S, LO), .IsArray = true, .IsTuple = false      \
    }

#define CEOOptionTypeName                                       \
    {                                                           \
        "Flag", "Int", "Unsigned Int", "Char", "Enum", "String" \
    }
#define CEOOptionTypeDescription                                                                     \
    {                                                                                                \
        "a specific argument which enables a corresponding option",                                  \
            "a whole number that's positive, negative, or zero", "a non-negative integer value",     \
            "a single character argument",                                                           \
            "an enumerator; should be one of a set of possible values", "a collection of characters" \
    }

#define CEOTupleDescription \
    "an argument consisting of a colon-separated group of same type, as in 'a:c' or '0:4'"
#define CEOArrayDescription                                                                        \
    "a space-separated list of integers, unsigned integers, characters, enumerators, strings, or " \
    "tuples, as in '1 1 3' or 'a:b d:e'"

/**
 * @brief      Types of options that can be passed into SEF-CLI
 */
enum CEOOptionType {
    kSefOptionFlag,
    kSefOptionInt,
    kSefOptionUnsignedInt,
    kSefOptionChar,
    kSefOptionEnum,
    kSefOptionString
};

#define CEOBitSizeName                             \
    {                                              \
        "8 -bits", "16-bits", "32-bits", "64-bits" \
    }

/**
 * @brief      Bit sizes supported by kSefOptionInt and kSefOptionUnsignedInt
 */
enum CEOBitSize { k8, k16, k32, k64 };

/**
 * @brief      detailed information about option
 */
struct CEOEngineOption
{
    const char shortName;      /**< Short name for the option */
    const char *longName;      /**< Long name for the option */
    const char *description;   /**< Description for the option */
    const char *defaultVal;    /**< The default value for the option if not passed in */
    const char *enumVal;       /**< Possible enum values as input */
    enum CEOOptionType type;   /**< Input option type */
    enum CEOBitSize bitSize;   /**< The BitSize of the int or unsigned int type */
    bool IsArray;              /**< Is option an array */
    bool IsTuple;              /**< Is option a tuple */
    unsigned int structOffset; /**< The offset of the option in the parsed options struct */
    unsigned int structGivenOffset; /**< The offset of the flag to be set if option is passed in the parsed options struct */
    unsigned int structLengthOffset; /**< The offset of the length of the array in the parsed options struct */
};

union CEOTuple
{
    struct
    {
        bool a;
        bool b;
    } tBool;
    struct
    {
        int64_t a;
        int64_t b;
    } tInt64;
    struct
    {
        uint64_t a;
        uint64_t b;
    } tUint64;
    struct
    {
        char a;
        char b;
    } tChar;
    struct
    {
        int a;
        int b;
    } tEnum;
};

/**
 * @brief   Attempts to parse out and interpret user-supplied options passed from main
 *
 * @param   argc                argc, passed from main
 * @param   argv                argv, passed from main
 * @param   ptrEngineOption     Will hold long versions of options
 * @param   parsedOptionsStructSize   Holds size of structure to hold options for particular target
 * @param   options             Holds options for particular target
 * @param   optionsNum          Number of options; to be set during parsing
 *
 * @retval  0                   Parsing complete
 **/
int CEOParseEngineOptions(int argc,
                          char **argv,
                          void **ptrEngineOption,
                          int parsedOptionsStructSize,
                          struct CEOEngineOption *options,
                          int optionsNum);

/**
 * @brief   Attempts to free the engine options memory
 *
 * @param   engineOption        Will hold long versions of options
 * @param   options             Holds options for particular target
 * @param   optionsNum          Number of options; to be set during parsing
 **/
void CEOFreeEngineOptions(void *engineOption, struct CEOEngineOption *options, int optionsNum);

#endif /* ENGINE_OPTION_H */
