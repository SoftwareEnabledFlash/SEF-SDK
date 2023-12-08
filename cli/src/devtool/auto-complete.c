/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * auto-complete.c
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
#include "auto-complete.h"

#include "cli-engine.h"
#include "engine-option.h"
#include "io-helper.h"

static void printActionAutoComplete(struct CEHEngineConfig *engine, void *data)
{
    int i;

    for (i = 0; i < engine->actionsNum; i++)
    {
        CIHMessageData("opts=\"${opts} %s\"", engine->actions[i].name);
    }
}

static void printTargetAutoComplete(struct CEHEngineConfig *engine, void *data)
{
    int i;

    for (i = 0; i < engine->actionsNum; i++)
    {
        CIHMessageData("if [[ ${action} == \"%s\" ]] ; then", engine->actions[i].name);
        CIHMessageData("opts=\"${opts} %s\"", engine->name);
        CIHMessageData("fi");
    }
}

static void printOptionAutoComplete(struct CEHEngineConfig *engine, void *data)
{
    int i;
    CIHMessageData("%s)", engine->name);

    for (i = 0; i < engine->optionsNum; i++)
    {
        CIHMessageData("opts=\"${opts} --%s\"", engine->options[i].longName);

        if (engine->options[i].shortName != '-')
        {
            CIHMessageData("opts=\"${opts} -%c\"", engine->options[i].shortName);
        }
    }
    CIHMessageData(";;");
}

void DACPrintAutoComplete()
{
    CIHMessageData("_sef-cli ()");
    CIHMessageData("{");

    CIHMessageData("local opts cur");
    CIHMessageData("cur=\"${COMP_WORDS[COMP_CWORD]}\"");
    CIHMessageData("opts=\"\"");

    // print actions
    CIHMessageData("if [[ ${COMP_CWORD} -eq 1 ]] ; then");
    CEHForEachEngine(printActionAutoComplete, NULL);
    CIHMessageData("fi");

    // print action based target
    CIHMessageData("if [[ ${COMP_CWORD} -eq 2 ]] ; then");

    CIHMessageData("local action");
    CIHMessageData("action=\"${COMP_WORDS[1]}\"");

    CEHForEachEngine(printTargetAutoComplete, NULL);

    CIHMessageData("fi");

    // print options
    CIHMessageData("if [[ ${COMP_CWORD} -gt 2 ]] ; then");

    CIHMessageData("local target");
    CIHMessageData("target=\"${COMP_WORDS[2]}\"");

    CIHMessageData("case \"$target\" in");
    CEHForEachEngine(printOptionAutoComplete, NULL);
    CIHMessageData("esac");

    CIHMessageData("fi");

    CIHMessageData("COMPREPLY=( $(compgen -W \"${opts}\" -- ${cur}) )");
    CIHMessageData("return 0");

    CIHMessageData("}");

    CIHMessageData("complete -F _sef-cli sef-cli");
}
