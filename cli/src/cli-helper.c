/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * cli-helper.c
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
#include "cli-helper.h"

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "devtool/dev-tool.h"
#include "engine-option.h"
#include "io-helper.h"

struct option long_options[] = {{"verbose", no_argument, NULL, 'V'},
                                {"help", no_argument, NULL, 'h'},
                                {"version", no_argument, NULL, 0},
                                {"column-width", required_argument, NULL, 'w'},
#ifdef DEVTOOL
                                DTFOptions
#endif
#ifdef SIMULATOR
                                {"simulator-config", optional_argument, NULL, 'c'},
#endif
                                {0, 0, 0, 0}};

/**
 * @brief   Validates the input arguments and options to make sure there are no missing
 *          quotation marks
 **/
bool CHFValidateCliArgs(int argc, char **argv)
{
    int i;
    bool isValue;

    isValue = false;
    for (i = 3; i < argc; i++)
    {
        if (argv[i][0] != '-' && isValue)
        {
            return false;
        }

        isValue = argv[i][0] != '-';
    }

    return true;
}

int CHFParseCliArgs(int argc, char **argv, struct sefCliOption *cliOption)
{
    int i;
    char **tmpArgv;

    // init cliOptions
    cliOption->cliEvocation = strdup(argv[0]);
    cliOption->target = NULL;
    cliOption->action = NULL;
#ifdef DEVTOOL
    cliOption->devtoolHandle.autoComplete = 0;
    cliOption->devtoolHandle.manPage = 0;
#endif
#ifdef SIMULATOR
    cliOption->simulatorConfig = NULL;
#endif
    cliOption->help = 0;
    cliOption->version = 0;
    cliOption->verbose = 0;
    cliOption->columnWidth = 80;

    // copy argv
    tmpArgv = malloc(argc * sizeof(char *));
    for (i = 0; i < argc; i++)
    {
        int argSize = strlen(argv[i]) + 1;
        tmpArgv[i] = malloc(argSize * sizeof(char));
        memcpy(tmpArgv[i], argv[i], argSize);
    }

    // init parsing the arguments
    optarg = 0;
    optind = 0;
    opterr = 0;

    while (1)
    {
        int optionIndex;
        int c;

        // parse the input
        c = getopt_long(argc, tmpArgv, "Vhc:w:", long_options, &optionIndex);

        if (c == -1)
        {
            break;
        }

        switch (c)
        {
            case 'V':
                cliOption->verbose = 1;
                break;

            case 'h':
                cliOption->help = 1;
                break;
            case 'w':
                cliOption->columnWidth = atoi(optarg);
                break;
#ifdef SIMULATOR
            case 'c':
                cliOption->simulatorConfig = strdup(optarg);
                break;
#endif
            case 0:
                if (strcasecmp("version", long_options[optionIndex].name) == 0)
                {
                    cliOption->version = 1;
                }
#ifdef DEVTOOL
                DTFParseDevTool(&cliOption->devtoolHandle, long_options[optionIndex].name);
#endif
                break;
        }
    }

    // free tmp argv
    for (i = 0; i < argc; i++)
    {
        free(tmpArgv[i]);
    }
    free(tmpArgv);

    // ignore the rest if help or version
#ifdef DEVTOOL
    if (cliOption->help || cliOption->version || DTFIsDevTool(&cliOption->devtoolHandle))
#else
    if (cliOption->help || cliOption->version)
#endif
        return 0;

    // set target and action
    if (argc > 2 && cliOption->target == NULL && cliOption->action == NULL)
    {
        cliOption->action = strdup(argv[1]);
        cliOption->target = strdup(argv[2]);
    }

    // validate arguments
    if (!CHFValidateCliArgs(argc, argv))
    {
        CIHMessageError(
            "Error: The arguments are not placed correctly; might be missing \" with arrays");
        return EINVAL;
    }

    // check for required inputs
    if (cliOption->target == NULL || cliOption->action == NULL)
    {
        CIHMessageError("Target and Action are required");
        CIHMessageError(
            "Basic Use: sef-cli <target-specific-action> <target> [target-specific-parameters] \n");

        CIHMessageError("For help, call with the -h or --help flag: sef-cli -h");
        CIHMessageError(
            "For further info beyond the above, add the -V or --verbose flag: sef-cli -h -V");

        return EINVAL;
    }

    return 0;
}

void CHFCleanupCliArgs(struct sefCliOption *cliOption)
{
    if (cliOption->cliEvocation != NULL)
    {
        free(cliOption->cliEvocation);
    }
    if (cliOption->target != NULL)
    {
        free(cliOption->target);
    }
    if (cliOption->action != NULL)
    {
        free(cliOption->action);
    }
#ifdef SIMULATOR
    if (cliOption->simulatorConfig != NULL)
    {
        free(cliOption->simulatorConfig);
    }
#endif
}

bool CHFIsCliArg(char *arg)
{
    int i, isLongArg, optionsNum;

    if (arg == NULL || strlen(arg) == 0)
    {
        return false;
    }

    // test if arg is long/short
    isLongArg = 1;
    if (strlen(arg) == 2 && arg[1] != '-')
    {
        isLongArg = 0;
    }

    // compare against cli options
    optionsNum = sizeof(long_options) / sizeof(struct option) - 1;
    for (i = 0; i < optionsNum; i++)
    {
        if (isLongArg && strcasecmp(arg + 2, long_options[i].name) == 0)
        {
            return true;
        }

        if (!isLongArg && arg[1] == long_options[i].val)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief       Prints detailed list of cli's supported types and the ranges for each
 **/
void CHFPrintTypes()
{
    char *sefOptionTypeName[] = CEOOptionTypeName;
    char *sefOptionTypeDescription[] = CEOOptionTypeDescription;

    CIHMessageInfoNoWrap(CIHCreateHeader1("Supported Types"));
    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", sefOptionTypeName[kSefOptionFlag],
                    sefOptionTypeDescription[kSefOptionFlag]);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", sefOptionTypeName[kSefOptionInt],
                    sefOptionTypeDescription[kSefOptionInt]);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: %" PRId8 " - %" PRId8, "8-bit", INT8_MIN, INT8_MAX);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: %" PRId16 " - %" PRId16, "16-bit", INT16_MIN, INT16_MAX);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: %" PRId32 " - %" PRId32, "32-bit", INT32_MIN, INT32_MAX);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: %" PRId64 " - %" PRId64, "64-bit", INT64_MIN, INT64_MAX);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", sefOptionTypeName[kSefOptionUnsignedInt],
                    sefOptionTypeDescription[kSefOptionUnsignedInt]);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: 0 - %" PRIu8, "8-bit", UINT8_MAX);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: 0 - %" PRIu16, "16-bit", UINT16_MAX);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: 0 - %" PRIu32, "32-bit", UINT32_MAX);
    CIHMessagePrint(kInfo, 21, 0, "%-10s Range: 0 - %" PRIu64, "64-bit", UINT64_MAX);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", sefOptionTypeName[kSefOptionChar],
                    sefOptionTypeDescription[kSefOptionChar]);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", sefOptionTypeName[kSefOptionEnum],
                    sefOptionTypeDescription[kSefOptionEnum]);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", sefOptionTypeName[kSefOptionString],
                    sefOptionTypeDescription[kSefOptionString]);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", "Tuple", CEOTupleDescription);

    CIHMessagePrint(kInfo, 2, 18, "%-15s %s", "Array", CEOArrayDescription);
}

void CHFPrintCliHelp()
{
    CIHMessageDataNoWrap(CIHCreateHeader1("General  Usage"));
    CIHMessageData(
        "./sef-cli <target-specific-action> <target> -[hV] [target-specific-parameters]\n");

    CIHMessageData("%s\n", CHFCliDescription);

    CHFPrintTypes();

    CIHMessageInfoNoWrap(CIHCreateHeader1("General  Arguments"));
    CIHMessageInfo("The following arguments are general and apply to all SEF CLI targets:");

    CIHMessagePrint(kInfo, 2, 26, "-h, --%-20s %s", "help", "Prints help and exits");
    CIHMessagePrint(kInfo, 29, 0, "%-20s %-7s", "optional", "Flag");
    CIHMessagePrint(kInfo, 2, 26, "- , --%-20s %s", "version",
                    "Prints version number and git commit hash and exits");
    CIHMessagePrint(kInfo, 29, 0, "%-20s %-7s", "optional", "Flag");
    CIHMessagePrint(kInfo, 2, 26, "-V, --%-20s %s", "verbose",
                    "Explains what is being done or shows additional information regarding the "
                    "action being taken");
    CIHMessagePrint(kInfo, 29, 0, "%-20s %-7s", "optional", "Flag");
    CIHMessagePrint(kInfo, 2, 26, "-w, --%-20s %s", "column-width",
                    "Sets output width (number of characters printed per line)");
    CIHMessagePrint(kInfo, 29, 0, "%-20s %-7s Default: %-15d", "optional", "Int", 80);
    CIHMessagePrint(kInfo, 2, 26, "- , --%-20s %s", "man-page",
                    "Prints man page for sef-cli and exits");
    CIHMessagePrint(kInfo, 29, 0, "%-20s %-7s", "optional", "Flag");

#ifdef SIMULATOR
    CIHMessagePrint(kInfo, 2, 26, "-c, --%-20s %s", "simulator-config",
                    "The path to the SEF Simulator's YAML config file");
    CIHMessagePrint(kInfo, 29, 0, "%-20s %-7s", "optional", "String");
#endif

    CIHMessageDataNoWrap("%s", "");
}
