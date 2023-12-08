/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * man-page.c
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
#include "man-page.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli-engine.h"
#include "cli-helper.h"
#include "engine-option.h"
#include "io-helper.h"
#include "utils/str-builder.h"

static void printTargetsManPage(struct CEHEngineConfig *engine, void *data)
{
    int j;
    char *sefOptionTypeName[] = CEOOptionTypeName;
    char *sefOptionBitSizeName[] = CEOBitSizeName;
    bool *shouldPrintOptions = (bool *)data;

    CIHMessageDataNoWrap("%4s\\fB%-20s\\fR %s", "", engine->name, engine->description);

    // print target parameters
    if (engine->optionsNum && *shouldPrintOptions)
    {
        for (j = 0; j < engine->optionsNum; j++)
        {
            char *optionInfoOut;
            struct StrBuilder optionInfo;

            StrBldInit(&optionInfo, 32);

            CIHMessageInfoNoWrap("%8s\\fB-%c --%-21.20s\\fR %s", "", engine->options[j].shortName,
                                 engine->options[j].longName, engine->options[j].description);

            if (engine->options[j].type == kSefOptionInt ||
                engine->options[j].type == kSefOptionUnsignedInt)
            {
                StrBldAppendFormat(&optionInfo, "%s(%s)%5s", sefOptionTypeName[engine->options[j].type],
                                   sefOptionBitSizeName[engine->options[j].bitSize], "");
            }
            else
            {
                StrBldAppendFormat(&optionInfo, ":%-26s", sefOptionTypeName[engine->options[j].type]);
            }

            if (engine->options[j].IsArray)
            {
                StrBldAppend(&optionInfo, "[Array] ");
            }

            if (engine->options[j].IsTuple)
            {
                StrBldAppend(&optionInfo, "[Tuple] ");
            }

            if (engine->options[j].defaultVal != NULL)
            {
                StrBldAppendFormat(&optionInfo, "Default: %-15s", engine->options[j].defaultVal);
            }

            if (engine->options[j].type == kSefOptionEnum)
            {
                StrBldAppendFormat(&optionInfo, "Possible Values: %s", engine->options[j].enumVal);
            }

            // To avoid the -Wformat-security warning and possible attack vectors
            optionInfoOut = StrBldToString(&optionInfo);
            CIHMessageInfoNoWrap("%35s%s", "", optionInfoOut);

            StrBldCleanup(&optionInfo);
            free(optionInfoOut);
        }
    }
}

static void printActionsManPage(struct CEHEngineConfig *engine, void *data)
{
    int j;
    bool hasHelp = false;

    CIHMessageDataNoWrap("%4s\\fI%s\\fR:", "", engine->name);
    for (j = 0; j < engine->actionsNum; j++)
    {
        if (strcmp(engine->actions[j].name, "help") == 0)
        {
            hasHelp = true;
        }

        if (engine->actions[j].description)
        {
            CIHMessageDataNoWrap("%8s\\fB%-20s\\fR %s", "", engine->actions[j].name,
                                 engine->actions[j].description);
        }
        else
        {
            CIHMessageDataNoWrap("%8s\\fB%-20s\\fR", "", engine->actions[j].name);
        }
    }

    if (!hasHelp)
    {
        CIHMessageDataNoWrap("%8s\\fB%-20s\\fR %s", "", "help", "Print verbose help for the target");
    }
}

static void printCliTypesManPage()
{
    char *sefOptionTypeName[] = CEOOptionTypeName;
    char *sefOptionTypeDescription[] = CEOOptionTypeDescription;

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", sefOptionTypeName[kSefOptionFlag],
                         sefOptionTypeDescription[kSefOptionFlag]);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", sefOptionTypeName[kSefOptionInt],
                         sefOptionTypeDescription[kSefOptionInt]);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR %" PRId8 " - %" PRId8, "", "8-bit",
                         INT8_MIN, INT8_MAX);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR %" PRId16 " - %" PRId16, "", "16-bit",
                         INT16_MIN, INT16_MAX);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR %" PRId32 " - %" PRId32, "", "32-bit",
                         INT32_MIN, INT32_MAX);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR %" PRId64 " - %" PRId64, "", "64-bit",
                         INT64_MIN, INT64_MAX);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", sefOptionTypeName[kSefOptionUnsignedInt],
                         sefOptionTypeDescription[kSefOptionUnsignedInt]);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR 0 - %" PRIu8, "", "8-bit", UINT8_MAX);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR 0 - %" PRIu16, "", "16-bit", UINT16_MAX);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR 0 - %" PRIu32, "", "32-bit", UINT32_MAX);
    CIHMessageDataNoWrap("%23s\\fB%-10s\\fR\\fIRange:\\fR 0 - %" PRIu64, "", "64-bit", UINT64_MAX);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", sefOptionTypeName[kSefOptionChar],
                         sefOptionTypeDescription[kSefOptionChar]);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", sefOptionTypeName[kSefOptionEnum],
                         sefOptionTypeDescription[kSefOptionEnum]);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", sefOptionTypeName[kSefOptionString],
                         sefOptionTypeDescription[kSefOptionString]);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", "Tuple", CEOTupleDescription);

    CIHMessageDataNoWrap("%4s\\fB%-15s\\fR %s\n.br", "", "Array", CEOArrayDescription);

    CIHMessageDataNoWrap("%s", "");
}

static void printCliOptionsManPage()
{
    CIHMessageDataNoWrap("\\fIGeneral Options\\fR\n.br");
    CIHMessageDataNoWrap("%4s\\fB-h\\fR, \\fB--%-20s\\fR %s\n.br", "", "help",
                         "Prints help and exits");
    CIHMessageDataNoWrap("%31s%-20s %-7s\n.sp", "", "optional", "Flag");
    CIHMessageDataNoWrap("%4s\\fB-V\\fR, \\fB--%-20s\\fR %s\n.br", "", "verbose",
                         "Explains what is being done or shows additional information regarding "
                         "the action being taken");
    CIHMessageDataNoWrap("%31s%-20s %-7s\n.sp", "", "optional", "Flag");
    CIHMessageDataNoWrap("%4s\\fB- \\fR, \\fB--%-20s\\fR %s\n.br", "", "version",
                         "Prints version number and git commit hash and exits");
    CIHMessageDataNoWrap("%31s%-20s %-7s\n.sp", "", "optional", "Flag");
    CIHMessageDataNoWrap("%4s\\fB-w\\fR, \\fB--%-20s\\fR %s\n.br", "", "column-width",
                         "Sets output width (number of characters printed per line)");
    CIHMessageDataNoWrap("%31s%-20s %-7s Default: %-15d\n.sp", "", "optional", "Int", 80);
    CIHMessageDataNoWrap("%4s\\fB- \\fR, \\fB--%-20s\\fR %s\n.br", "", "man-page",
                         "Prints man page for sef-cli and exits");
    CIHMessageDataNoWrap("%31s%-20s %-7s\n.sp", "", "optional", "Flag");

#ifdef SIMULATOR
    CIHMessageDataNoWrap("%4s\\fB-c\\fR, \\fB--%-20s\\fR %s\n.br", "", "simulator-config",
                         "The path to the SEF Simulator's YAML config file");
    CIHMessageDataNoWrap("%31s%-20s %-7s\n.sp", "", "optional", "String");
#endif

    CIHMessageDataNoWrap("%s", "");
}

void DMPPrintManPage()
{
    bool shouldPrintOptions;

    // man page title material
    CIHMessageDataNoWrap(".\\\" Manpage for SEF CLI");
    CIHMessageDataNoWrap(".TH SEF-CLI 1 \"%s\" \"SEF Cli\" \"User Commands\"", CHFCliRelease);

    // name
    CIHMessageDataNoWrap(".TH NAME");
    CIHMessageDataNoWrap("sef-cli \\- control, modify, and manipulate SEF Units using CLI");

    // synopsis
    CIHMessageDataNoWrap(".TH SYNOPSIS");
    CIHMessageDataNoWrap(
        "\\&\\fIsef-cli \\fR\\fB<target-specific-action> <target>\\fR "
        "[target-specific-parameters]");

    // description
    CIHMessageDataNoWrap(".TH DESCRIPTION");
    CIHMessageDataNoWrap("%s\n.br", CHFCliDescription);

    // targets
    CIHMessageDataNoWrap(".TH TARGETS");
    shouldPrintOptions = false;
    CEHForEachEngine(printTargetsManPage, &shouldPrintOptions);

    // actions
    CIHMessageDataNoWrap(".TH ACTIONS");
    CEHForEachEngine(printActionsManPage, NULL);

    CIHMessageDataNoWrap(".TH TYPES");
    printCliTypesManPage();

    // options
    CIHMessageDataNoWrap(".TH OPTIONS");
    printCliOptionsManPage();

    CIHMessageInfoNoWrap("\\fITarget-specific Options:\\fR");
    shouldPrintOptions = true;
    CEHForEachEngine(printTargetsManPage, &shouldPrintOptions);
}
