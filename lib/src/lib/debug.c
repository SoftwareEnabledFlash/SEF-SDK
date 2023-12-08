/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * debug.c
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

#ifdef SEFLIB_DEBUG_OUT_FILE_ENABLE

#include "debug.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

STATIC char saveFileNameBase[][64] = {"DebugSaveCreateVdData",      "DebugSaveCreateQosdData",
                                      "DebugSaveGetVdStaticInfo",   "DebugSaveGetVdDynamicInfo",
                                      "DebugSaveGetQosdInfo",       "DebugSaveCopyData",
                                      "DebugSaveGetLogUserAddrList"};

void DebugSaveInfoFile(void *pData, uint32_t size, enum DebugSaveFileType type)
{
    const char *pEnvPrefix;
    char *pFileName;
    static uint32_t saveCount[kDebugSaveFileTypeMax] = {0};

    SEF_ASSERT(type < kDebugSaveFileTypeMax);

    pEnvPrefix = (const char *)getenv("SEFLIB_DBG_OUT_PREFIX");
    if (pEnvPrefix)
    {
        pFileName = (char *)malloc(strlen(pEnvPrefix) + strlen(saveFileNameBase[type]) + 32);
        sprintf(pFileName, "%s%s%04d.bin", pEnvPrefix, saveFileNameBase[type], saveCount[type]);
    }
    else
    {
        pFileName = (char *)malloc(strlen(saveFileNameBase[type]) + 32);
        sprintf(pFileName, "%s%04d.bin", saveFileNameBase[type], saveCount[type]);
    }

    if (pFileName)
    {
        FILE *fp = fopen(pFileName, "w+");
        if (fp)
        {
            fwrite(pData, size, 1, fp);
            fprintf(stderr, "%s %d %d\n", __FILE__, __LINE__, io_thread->efd, io_thread->wfd);
            fclose(fp);
            saveCount[type]++;
        }
        free(pFileName);
    }
}

#else

#endif /* SEFLIB_DEBUG_OUT_FILE_ENABLE */
