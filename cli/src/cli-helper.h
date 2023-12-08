/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * cli-helper.h
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
#ifndef CLI_HELPER_H
#define CLI_HELPER_H

#include <stdbool.h>
#include "devtool/dev-tool.h"

#define CHFCliVersion 1.0
#define CHFCliRelease "2023-5-1"
#define CHFCliDescription                                                                     \
    "sef-cli is a versatile command line tool provided as a means of performing SEF device "  \
    "configuration, administration, and debugging. sef-cli supports many different engines, " \
    "and each engine supports multiple actions."

/**
 * @brief      detailed information about input options
 */
struct sefCliOption
{
    char *cliEvocation; /**< Name of the executing program */
    int verbose;        /**< Should output be verbose */
    int help;           /**< Should help info be printed */
    int version;        /**< Should version info be printed */
    char *target;       /**< Input target name */
    char *action;       /**< Input action name */
    int columnWidth;    /**< Custom column width for the output */
#ifdef DEVTOOL
    struct DTFHandle devtoolHandle; /**< Handle to the devtool helpers */
#endif
#ifdef SIMULATOR
    char *simulatorConfig; /**< Path to the simulator config */
#endif
};

/**
 * @brief       Prints the beginning of SEF-CLI's help, including general use and general options
 **/
void CHFPrintCliHelp();

/**
 * @brief       Frees memory where SEF-CLI arguments and options were stored
 *
 * @param       cliOption       Struct holding current SEF-CLI arguments and options
 **/
void CHFCleanupCliArgs(struct sefCliOption *cliOption);

/**
 * @brief       Parses the user-supplied arguments and options from terminal; displays error message
 *              if target or action are missing
 *
 * @param       argc            argc, passed from main()
 * @param       argv            argv, also passed from main()
 * @param       cliOption       Struct to hold user-supplied SEF-CLI arguments and options
 **/
int CHFParseCliArgs(int argc, char **argv, struct sefCliOption *cliOption);

/**
 * @brief       Returns if the passed in argument is a sef-cli argument
 *
 * @param       arg     The command line arg passed in its original form
 **/
bool CHFIsCliArg(char *arg);

#endif /* CLI_HELPER_H */
