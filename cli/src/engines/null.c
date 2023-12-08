/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * null.c
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

/**
 * This "null" engine is provided as part of SEF-CLI as an
 * example template for creation of additional engines/targets.
 * Furthermore, it serves as an example for how to create and
 * register new actions within existing engines/targets.
 *
 * In short:
 * - one engine source code file exists for each target
 * - each target instance is called with cli input attributes contained in an options structure
 * - each target has target-specific actions (functions) which should be registered in the engine
 * - each action is self-contained and should do its own ENESetup and ENECleanup
 * - in order to interact with a target, its engine must also be registered
 **/

#include <SEFAPI.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>

#include "../cli-engine.h"
#include "../engine-option.h"
#include "../io-helper.h"
#include "../sef-helper.h"

// LCOV_EXCL_START

// function prototypes--these are target-specific actions
static int ENEHelloWorld(void **context, void *options);

// a struct created and populated by SEF-CLI and passed into functions each time they're called; based on engine options declarations
struct ENEOptions
{
    void *pad;
    char *name;
    bool nameGiven;
    uint8_t age;
    bool ageGiven;
    char *code;
    bool codeGiven;
    int codeLength;
};

// engine (target) options declarations
static struct CEOEngineOption options[] = {
    CEOString('n', "name", "The name of the person you want to say hello to", ENEOptions, name, nameGiven),
    CEOUInt('-', "age", "The age of the person you want to say hello to", k8, ENEOptions, age, ageGiven),
    {.shortName = 'c',
     .longName = "code",
     .description = "",
     .defaultVal = NULL,
     .type = kSefOptionChar,
     .structOffset = offsetof(struct ENEOptions, code),
     .structGivenOffset = offsetof(struct ENEOptions, codeGiven),
     .structLengthOffset = offsetof(struct ENEOptions, codeLength),
     .IsArray = true,
     .IsTuple = false}};

// engine setup/cleanup
static int ENESetup(void **engineContext, void *options)
{
    CIHMessageData("Engine Setup");
    return 0;
}

static int ENECleanup(void **engineContext, void *options)
{
    CIHMessageData("Engine Cleanup");
    return 0;
}

// engine functions
static int ENEHelloWorld(void **context, void *options)
{
    char funCode[] = {'w', 'w', 's', 's', 'a', 'd', 'a', 'd', 'b', 'a'};
    struct ENEOptions *no = (struct ENEOptions *)options;

    // helper function to check if an option is given
    // CIHIsNotGiven("name", 'n', no->nameGiven);

    if (no->nameGiven)
    {
        CIHMessageData("Hello %s", no->name);
    }
    else
    {
        CIHMessageData("Hello World");
    }

    if (no->ageGiven)
    {
        CIHMessageData("Your age is %d", no->age);
    }

    if (no->codeGiven && no->codeLength == NELEM(funCode))
    {
        bool isCode = true;
        int i = 0;

        for (i = 0; i < no->codeLength; i++)
        {
            isCode = isCode && no->code[i] == funCode[i];
        }

        if (isCode)
        {
            /* clang-format off */
            CIHMessageDataNoWrap("                        :=+*##*+=-.                                                       ");
            CIHMessageDataNoWrap("                 .=  :*@@@##**##@@@#=                                                     ");
            CIHMessageDataNoWrap("                  :#-:##-        -#@@#.    .::..                                          ");
            CIHMessageDataNoWrap("            -*#@@@.=@#:            :@*:=#@@@@@@@@#*-   #@@@@@@@@@@@@@@+  =@@@@@@@@@@@@@*  ");
            CIHMessageDataNoWrap("          =@@@#=--- =@@*.           .-@@@@#*+++*#@@#.  #@@@#**********-  =@@@@*********=  ");
            CIHMessageDataNoWrap("         -@@#        =@@@=           @@@@=        =    #@@@+             =@@@@            ");
            CIHMessageDataNoWrap("      :=*@@@      .#@@@@@@#-        .@@@@*:            #@@@+             =@@@@            ");
            CIHMessageDataNoWrap("    +@@@#**+        #@@@@#=+=        +@@@@@@#*=-.      #@@@#=========:   =@@@@........    ");
            CIHMessageDataNoWrap("   #@@+.             +@@@@+           .+#@@@@@@@@@#-   #@@@@@@@@@@@@@+   =@@@@@@@@@@@@-   ");
            CIHMessageDataNoWrap("  =@@-             =+=#@@@@#              .:=*#@@@@@#  #@@@*:::::::::.   =@@@@********.   ");
            CIHMessageDataNoWrap("  +@@:              -#@@@@@@#                   -@@@@= #@@@+             =@@@@            ");
            CIHMessageDataNoWrap("  :@@#.               =@@@= ..        =-        .#@@@= #@@@+             =@@@@            ");
            CIHMessageDataNoWrap("   -@@@*-.............. *@@. ......  +@@@##*++*#@@@@*  #@@@###########+  =@@@@            ");
            CIHMessageDataNoWrap("     =#@@@@@@@@@@@@@@@@#.-#@.=@@@@@@*-=*#@@@@@@@@#*:   #@@@@@@@@@@@@@@#  =@@@@            ");
            CIHMessageDataNoWrap("        .::::::::::::::::  -* :::::::::   .::::.                                          ");
            CIHMessageDataNoWrap("                             .                                                            ");
            /* clang-format on */
        }
    }

    return 0;
}

// engine action declarations
static struct CEHEngineAction actions[] = {
    newEngineAction("hello", "Say hello to your friend", &ENEHelloWorld)};

// engine declaration
static struct CEHEngineConfig config =
    newEngineWithSetup("null", "A good start", ENEOptions, options, actions, &ENESetup, &ENECleanup);

// registering the engine
static void sefEngineInit superblockInit()
{
    // To register the engine, the following line would need to be un-commented:
    // CEHRegisterEngine(&config);
    config = config;    // used to suppress warning
}

// LCOV_EXCL_STOP
