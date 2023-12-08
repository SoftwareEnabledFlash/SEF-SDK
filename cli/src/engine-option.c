/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * engine-option.c
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
#include "engine-option.h"

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <regex.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli-helper.h"
#include "io-helper.h"

/**
 * @brief   Attempts to parse out and interpret unsigned int
 *
 * The function will ensure the parsed data is within the bit range of the option
 *
 * @param[out]  data        Buffer to hold the parsed data
 * @param       option      Holds option for particular target that is being parsed
 * @param       input       Input supplied by the user to be parsed
 *
 *  @return     Returns true if it's able to parse the value successfully
 **/
static bool parseUnsignedInt(void *data, struct CEOEngineOption *option, const char *input)
{
    bool isValid;
    uint64_t inti = strtoull(input, NULL, 0);

    switch (option->bitSize)
    {
        case k8:
            *((uint8_t *)data) = (uint8_t)inti;
            isValid = inti < UINT8_MAX && inti >= 0;
            break;

        case k16:
            *((uint16_t *)data) = (uint16_t)inti;
            isValid = inti < UINT16_MAX && inti >= 0;
            break;

        case k32:
            *((uint32_t *)data) = (uint32_t)inti;
            isValid = inti < UINT32_MAX && inti >= 0;
            break;

        case k64:
            *((uint64_t *)data) = (uint64_t)inti;
            isValid = inti < UINT64_MAX && inti >= 0;
            break;

        default:
            isValid = false;
            break;
    }

    if (!isValid)
    {
        CIHMessageError("Error: invalid argument, \"%s\", for option '--%s', is out of range",
                        input, option->longName);
    }

    return isValid;
}

/**
 * @brief   Attempts to parse out and interpret int
 *
 * The function will ensure the parsed data is within the bit range of the option
 *
 * @param[out]  data        Buffer to hold the parsed data
 * @param       option      Holds option for particular target that is being parsed
 * @param       input       Input supplied by the user to be parsed
 *
 *  @return     Returns true if it's able to parse the value successfully
 **/
static bool parseInt(void *data, struct CEOEngineOption *option, const char *input)
{
    bool isValid;
    int64_t inti = strtoll(input, NULL, 0);

    switch (option->bitSize)
    {
        case k8:
            *((int8_t *)data) = (int8_t)inti;
            isValid = inti > INT8_MIN && inti < INT8_MAX;
            break;

        case k16:
            *((int16_t *)data) = (int16_t)inti;
            isValid = inti > INT16_MIN && inti < INT16_MAX;
            break;

        case k32:
            *((int32_t *)data) = (int32_t)inti;
            isValid = inti > INT32_MIN && inti < INT32_MAX;
            break;

        case k64:
            *((int64_t *)data) = (int64_t)inti;
            isValid = inti > INT64_MIN && inti < INT64_MAX;
            break;

        default:
            isValid = false;
            break;
    }

    if (!isValid)
    {
        CIHMessageError("Error: invalid argument, \"%s\", for option '--%s', is out of range",
                        input, option->longName);
    }

    return isValid;
}

/**
 * @brief   Attempts to parse out and interpret enum
 *
 * The function will ensure the parsed enum matches possible enum options
 *
 * @param[out]  data        Buffer to hold the parsed data
 * @param       option      Holds option for particular target that is being parsed
 * @param       input       Input supplied by the user to be parsed
 *
 *  @return     Returns true if it's able to parse the value successfully
 **/
static bool parseEnum(void *data, struct CEOEngineOption *option, const char *input)
{
    int i, inputSize, enumValue;
    char *token;
    char *tmpValidEnums = strdup(option->enumVal);
    char *rest = tmpValidEnums;

    // find the expected value
    i = 0;
    enumValue = -1;
    inputSize = strlen(input);
    while ((token = strtok_r(rest, ",", &rest)) && enumValue == -1)
    {
        if (strncasecmp(input, token, inputSize) == 0)
        {
            enumValue = i;
        }

        i++;
    }

    // free memory
    free(tmpValidEnums);

    // set the enum value
    memcpy(data, &enumValue, sizeof(int));

    if (enumValue == -1)
    {
        CIHMessageError("Error: invalid argument, \"%s\", for option '--%s'; Possible Values %s",
                        input, option->longName, option->enumVal);
    }

    return enumValue != -1;
}

/**
 * @brief   Attempts to parse out and interpret the basic types that are defined by CEOOptionType
 *
 * @param[out]  data        Buffer to hold the parsed data
 * @param       option      Holds option for particular target that is being parsed
 * @param       input       Input supplied by the user to be parsed
 *
 *  @return     Returns true if it's able to parse the value successfully
 **/
static bool parseBase(void *data, struct CEOEngineOption *option, const char *input)
{
    bool setFlag = true;
    char *inputCopy;

    switch (option->type)
    {
        case kSefOptionInt:
            return parseInt(data, option, input);
            break;

        case kSefOptionUnsignedInt:
            return parseUnsignedInt(data, option, input);
            break;

        case kSefOptionFlag:
            memcpy(data, &setFlag, sizeof(bool));
            break;

        case kSefOptionChar:
            memcpy(data, input, sizeof(char));
            break;

        case kSefOptionEnum:
            return parseEnum(data, option, input);
            break;

        case kSefOptionString:
            inputCopy = strdup(input);
            memcpy(data, &inputCopy, sizeof(char *));
            break;

        default:
            return false;
            break;
    }

    return true;
}

/**
 * @brief   Attempts to parse out and interpret tuples
 *
 * @param[out]  data        Buffer to hold the parsed data
 * @param       option      Holds option for particular target that is being parsed
 * @param       input       Input supplied by the user to be parsed
 *
 *  @return     Returns true if it's able to parse the value successfully
 **/
static bool parseTuple(void *data, struct CEOEngineOption *option, const char *input)
{
    int i;
    bool isParsed = true;
    char *token, *rest, *tmpInput, *tuple1, *tuple2;
    union CEOTuple *tuple = data;

    tmpInput = strdup(input);
    i = 0;
    rest = tmpInput;
    tuple1 = NULL;
    tuple2 = NULL;
    while ((token = strtok_r(rest, ":", &rest)))
    {
        switch (i)
        {
            case 0:
                tuple1 = token;
                break;

            case 1:
                tuple2 = token;
                break;

            default:
                break;
        }

        i++;
    }

    // make sure tuple value is valid
    if (i != 2)
    {
        // free memory
        free(tmpInput);

        CIHMessageError(
            "Error: invalid argument, \"%s\", for option '--%s'; Tuples should follow X:X format",
            input, option->longName);
        return false;
    }

    switch (option->type)
    {
        case kSefOptionFlag:
            isParsed &= parseBase(&tuple->tBool.a, option, tuple1);
            isParsed &= parseBase(&tuple->tBool.b, option, tuple2);
            break;

        case kSefOptionInt:
            isParsed &= parseBase(&tuple->tInt64.a, option, tuple1);
            isParsed &= parseBase(&tuple->tInt64.b, option, tuple2);
            break;

        case kSefOptionUnsignedInt:
            isParsed &= parseBase(&tuple->tUint64.a, option, tuple1);
            isParsed &= parseBase(&tuple->tUint64.b, option, tuple2);
            break;

        case kSefOptionChar:
            isParsed &= parseBase(&tuple->tChar.a, option, tuple1);
            isParsed &= parseBase(&tuple->tChar.b, option, tuple2);
            break;

        default:
            break;
    }

    free(tmpInput);

    return isParsed;
}

/**
 * @brief   Attempts to parse out and interpret space seperated arrays
 *
 * An array can be made out of tuples or basic types
 *
 * @param[out]  data        Buffer to hold the parsed data
 * @param       option      Holds option for particular target that is being parsed
 * @param       input       Input supplied by the user to be parsed
 *
 *  @return     Returns true if it's able to parse the value successfully
 **/
static bool parseArray(void *data, struct CEOEngineOption *option, const char *input, void *engineOption)
{
    int i, arraySize, arrayOptionSize;
    bool returnValue;
    void *array;
    char *token, *rest, *tmpInput;
    int optionSize[] = {0, 0, sizeof(uint32_t), sizeof(char), sizeof(int)};
    int intOptionSize[] = {sizeof(int8_t), sizeof(int16_t), sizeof(int32_t), sizeof(int64_t)};
    int unsignedIntOptionSize[] = {sizeof(uint8_t), sizeof(uint16_t), sizeof(uint32_t),
                                   sizeof(uint64_t)};

    // get array size
    arraySize = 0;
    tmpInput = strdup(input);
    rest = tmpInput;
    while ((token = strtok_r(rest, " ", &rest)))
    {
        arraySize++;
    }

    free(tmpInput);

    // create array
    arrayOptionSize = optionSize[option->type];
    if (option->IsTuple)
    {
        arrayOptionSize = sizeof(union CEOTuple);
    }
    else if (option->type == kSefOptionInt)
    {
        arrayOptionSize = intOptionSize[option->bitSize];
    }
    else if (option->type == kSefOptionUnsignedInt)
    {
        arrayOptionSize = unsignedIntOptionSize[option->bitSize];
    }

    array = calloc(arraySize, arrayOptionSize);

    // populate the created array
    i = 0;
    returnValue = true;
    tmpInput = strdup(input);
    rest = tmpInput;
    while ((token = strtok_r(rest, " ", &rest)) && returnValue)
    {
        unsigned int offset = i * arrayOptionSize;

        if (option->IsTuple)
        {
            returnValue = parseTuple(array + offset, option, token);
        }
        else
        {
            returnValue = parseBase(array + offset, option, token);
        }

        i++;
    }

    free(tmpInput);

    memcpy(data, &array, sizeof(void *));
    memcpy(engineOption + option->structLengthOffset, &i, sizeof(int));

    return returnValue;
}

/**
 * @brief   Generates the the short and long options needed for getopt
 *
 **/
static void genGetOpts(struct CEOEngineOption *options,
                       int optionsNum,
                       struct option **longOptions,
                       char **shortOptions)
{
    int i, j, shortOptionsNum = 0;

    // allocated space for the long options
    *longOptions = malloc(sizeof(struct option) * (optionsNum + 1));

    // parse arguments to create long options array
    for (i = 0; i < optionsNum; i++)
    {
        (*longOptions)[i].name = options[i].longName;
        (*longOptions)[i].flag = NULL;
        (*longOptions)[i].val = 0;
        if (options[i].shortName != '-')
        {
            shortOptionsNum++;
            (*longOptions)[i].val = options[i].shortName;
        }

        if (options[i].type == kSefOptionFlag)
        {
            (*longOptions)[i].has_arg = no_argument;
        }
        else
        {
            shortOptionsNum++;
            (*longOptions)[i].has_arg = required_argument;
        }
    }

    // add the mandatory empty entry
    (*longOptions)[optionsNum].name = 0;
    (*longOptions)[optionsNum].has_arg = 0;
    (*longOptions)[optionsNum].flag = NULL;
    (*longOptions)[optionsNum].val = 0;

    // allocate space for the short options array
    *shortOptions = malloc(sizeof(char) * (shortOptionsNum + 1));

    // parse arguments to create short options array
    for (i = 0, j = 0; i < optionsNum; i++)
    {
        if (options[i].shortName != '-')
        {
            (*shortOptions)[j++] = (char)options[i].shortName;

            if (options[i].type != kSefOptionFlag)
            {
                (*shortOptions)[j++] = ':';
            }
        }
    }

    // add null char
    (*shortOptions)[shortOptionsNum] = '\0';
}

/**
 * @brief   Cleans the memory that is allocated by genGetOpts
 **/
static void cleanGetOpts(struct option *longOptions, char *shortOptions)
{
    // free short options array
    free(longOptions);
    free(shortOptions);

    // reset getopt
    optind = 1;
}

static bool parseAssign(struct CEOEngineOption option, void *engineOption, const char *optionValue)
{
    void *dataLocation = engineOption + option.structOffset;

    if (option.IsArray)
    {
        return parseArray(dataLocation, &option, optionValue, engineOption);
    }

    if (option.IsTuple)
    {
        return parseTuple(dataLocation, &option, optionValue);
    }

    return parseBase(dataLocation, &option, optionValue);
}

static void freeDynamicOption(void *engineOption, struct CEOEngineOption *option)
{
    void *data;

    if (option->type != kSefOptionString && !option->IsArray)
    {
        return;
    }

    memcpy(&data, engineOption + option->structOffset, sizeof(void *));

    free(data);
}

static void initEngineOptions(void **ptrEngineOption,
                              int parsedOptionsStructSize,
                              struct CEOEngineOption *options,
                              int optionsNum)
{
    int i;
    void *engineOption;

    // allocate space for the long options
    *ptrEngineOption = calloc(1, parsedOptionsStructSize);
    engineOption = *ptrEngineOption;

    // set options to default
    for (i = 0; i < optionsNum; i++)
    {
        if (options[i].defaultVal != NULL)
        {
            parseAssign(options[i], engineOption, options[i].defaultVal);
        }
    }
}

int CEOParseEngineOptions(int argc,
                          char **argv,
                          void **ptrEngineOption,
                          int parsedOptionsStructSize,
                          struct CEOEngineOption *options,
                          int optionsNum)
{
    int i;
    bool isArgValid;
    struct option *longOptions;
    char *shortOptions;
    void *engineOption;

    // initialize the engine options
    initEngineOptions(ptrEngineOption, parsedOptionsStructSize, options, optionsNum);
    engineOption = *ptrEngineOption;

    // generate getopt options struct
    genGetOpts(options, optionsNum, &longOptions, &shortOptions);

    // init parsing the arguments
    isArgValid = true;
    optarg = 0;
    optind = 0;
    opterr = 0;

    // parse engine options from cli
    while (1)
    {
        int c, optionIndex;
        c = getopt_long(argc, argv, shortOptions, longOptions, &optionIndex);

        if (c == -1)
        {
            break;
        }

        // handle unknown arguments
        if (c == '?')
        {
            // skip the arguments that are part of the cli
            if (CHFIsCliArg(argv[optind - 1]))
            {
                continue;
            }

            CIHMessageError("unrecognized option '%s'", argv[optind - 1]);
            isArgValid = false;
        }

        for (i = 0; i < optionsNum; i++)
        {
            if (options[i].shortName == (char)c ||
                (c == 0 && strcasecmp(options[i].longName, longOptions[optionIndex].name) == 0))
            {
                // free memory for defaults
                freeDynamicOption(engineOption, &options[i]);

                // set the value
                isArgValid &= parseAssign(options[i], engineOption, optarg);

                // set the isGiven
                if (options[i].structGivenOffset)
                {
                    bool value = true;
                    memcpy(engineOption + options[i].structGivenOffset, &value, sizeof(bool));
                }

                break;
            }
        }
    }

    cleanGetOpts(longOptions, shortOptions);

    if (isArgValid)
    {
        return 0;
    }

    CEOFreeEngineOptions(engineOption, options, optionsNum);
    return EIO;
}

void CEOFreeEngineOptions(void *engineOption, struct CEOEngineOption *options, int optionsNum)
{
    int i;

    // free dynamic options
    for (i = 0; i < optionsNum; i++)
    {
        freeDynamicOption(engineOption, &options[i]);
    }

    // free options
    free(engineOption);
}
