/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-cli.c
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
#include <SEFAPI.h>
#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef SIMULATOR
#include <SEFSim.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

#include "cli-engine.h"
#include "cli-helper.h"
#include "engine-option.h"
#include "io-helper.h"

#ifndef SEFSDKVersion
#define SEFSDKVersion "unavailable"
#endif

static __attribute((used)) char SdkVersion[] = "@(#) SEF_SDK version " SEFSDKVersion;

int main(int argc, char **argv)
{
    struct sefCliOption cliOptions;
    struct CEHEngineConfig *targetEngine;
    void *engineOptions = NULL;
    void *engineContext = NULL;
    int status;
    int (*actionFunction)(void **, void *);

    // parse and check cli only arguments
    if (CHFParseCliArgs(argc, argv, &cliOptions))
    {
        CHFCleanupCliArgs(&cliOptions);
        return EINVAL;
    }

    // set messaging setting
    if (cliOptions.verbose)
    {
        CIHSetLevel(kInfo);
    }
    else
    {
        CIHSetLevel(kData);
    }

    // set output column width
    CIHSetOutputWidth(cliOptions.columnWidth);

    // print help
    if (cliOptions.help)
    {
        CHFPrintCliHelp();
        CEHPrintAllEngineHelp();

        CHFCleanupCliArgs(&cliOptions);
        return 0;
    }

#ifdef DEVTOOL
    // print devtools
    if (DTFIsDevTool(&cliOptions.devtoolHandle))
    {
        DTFExecuteDevTool(&cliOptions.devtoolHandle);

        CHFCleanupCliArgs(&cliOptions);
        return 0;
    }
#endif

    // print version
    if (cliOptions.version)
    {
        CIHMessageDataNoWrap("SEF CLI version: 2.15");
        CIHMessageData("SEF SDK version %s", SEFSDKVersion);

        CHFCleanupCliArgs(&cliOptions);
        return 0;
    }

#ifndef SIMULATOR
    if (geteuid() && strcasecmp(cliOptions.action, "help") != 0)
    {
        CIHMessageError("Root permissions are required to interact with SEF unit");

        CHFCleanupCliArgs(&cliOptions);
        return EACCES;
    }
#endif

    // route target and action
    status = CEHRouteEngine(cliOptions.target, &targetEngine);
    if (status)
    {
        CHFCleanupCliArgs(&cliOptions);
        return status;
    }

    status = CEHRouteAction(cliOptions.action, targetEngine, &actionFunction);
    if (status)
    {
        CHFCleanupCliArgs(&cliOptions);
        return status;
    }

    // create engine options object
    status = CEOParseEngineOptions(argc, argv, &engineOptions, targetEngine->parsedOptionsStructSize,
                                   targetEngine->options, targetEngine->optionsNum);
    if (status)
    {
        CHFCleanupCliArgs(&cliOptions);
        return status;
    }

#ifdef SIMULATOR
    if (cliOptions.simulatorConfig != NULL)
    {
        int configStatus;

        CIHMessageInfo("Using the simulator config at %s", cliOptions.simulatorConfig);
        configStatus = SEFSimSetDeviceConfig(cliOptions.simulatorConfig);

        if (configStatus)
        {
            CIHMessageError("Error: Was unable to set / parse the simulator config");
            CHFCleanupCliArgs(&cliOptions);
            free(engineOptions);

            return configStatus;
        }
    }
#endif

    // call engine's set up
    if (targetEngine->setupFunction != NULL)
    {
        status = targetEngine->setupFunction(&engineContext, engineOptions);
    }

    // call requested action
    if (status == 0)
    {
        status = (*actionFunction)(&engineContext, engineOptions);
    }

    // call engine's cleanup
    if (targetEngine->cleanupFunction != NULL)
    {
        targetEngine->cleanupFunction(&engineContext, engineOptions);
    }

    // free the engine options
    CHFCleanupCliArgs(&cliOptions);
    CEOFreeEngineOptions(engineOptions, targetEngine->options, targetEngine->optionsNum);

    return status;
}
