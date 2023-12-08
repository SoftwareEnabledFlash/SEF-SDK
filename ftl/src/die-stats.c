/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * die-stats.c
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
#include "die-stats.h"

#include <SEFAPI.h>
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>    // snprintf
#include <string.h>
#include <strings.h>

#include "config.h"
#include "log-manager.h"
#include "sef-utils.h"
#include "utils/instrumentation.h"

struct DieStats
{
    SEFHandle sefHandle;
    SEFQoSHandle qosHandle;
    const struct SEFInfo* sefInfo;
    void* hInst;
    //  ssize_t adus_per_die;
    ssize_t die_page_adus;
    ssize_t numDie;
    uint64_t* readDie;
    uint64_t* writeDie;
#if CC_HEATMAP
    ssize_t numHeat;
    uint64_t* readHeat;
    uint64_t* writeHeat;
#endif
};

int dumpCounters(uint64_t* values, ssize_t numValues, char* out, ssize_t size)
{
    int i, n, totalSize = 0;

    for (i = 0; i < numValues - 1 && size > 0; ++i)
    {
        n = snprintf(NULL, 0, "%" PRIu64 ",", *values);
        if (size < n)
        {
            return totalSize;
        }

        n = snprintf(out, size, "%" PRIu64 ",", *values);
        out += n;
        size -= n;
        totalSize += n;
        values++;
    }

    n = snprintf(NULL, 0, "%" PRIu64 "\n", *values);
    if (size < n)
    {
        return totalSize;
    }

    n = snprintf(out, size, "%" PRIu64 "\n", *values);
    totalSize += n;

    return totalSize;
}

void dieMapAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    DieStatsHandle dieStats = arg;

    if (strcmp(verb, "help") == 0)
    {
        snprintf(out, size, "* diemap <read|write>: prints per die read or write counters\n");
        return;
    }
    char* row = strtok_r(NULL, " \t", savePtr);
    if (row && strcasecmp(row, "read") == 0)
    {
        dumpCounters(dieStats->readDie, dieStats->numDie, out, size);
    }
    else if (row && strcasecmp(row, "write") == 0)
    {
        dumpCounters(dieStats->writeDie, dieStats->numDie, out, size);
    }
    else if (row)
    {
        snprintf(out, size, "Expected 'diemap read' or 'diemap write', not 'diemap %s'\n", row);
    }
    else
    {
        int n;
        n = snprintf(NULL, 0, "Read: ");
        if (size < n)
        {
            return;
        }

        n = snprintf(out, size, "Read: ");
        out += n;
        size -= n;

        n = dumpCounters(dieStats->readDie, dieStats->numDie, out, size);
        out += n;
        size -= n;

        n = snprintf(NULL, 0, "Write: ");
        if (size < n)
        {
            return;
        }

        n = snprintf(out, size, "Write: ");
        out += n;
        size -= n;

        dumpCounters(dieStats->writeDie, dieStats->numDie, out, size);
    }
}

#if CC_HEATMAP
void heatMapAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    DieStatsHandle dieStats = arg;

    if (strcmp(verb, "help") == 0)
    {
        snprintf(out, size,
                 "* heatmap <read|write>: prints per superblock read or write counters\n");
        return;
    }
    char* row = strtok_r(NULL, " \t", savePtr);
    if (row && strcasecmp(row, "read") == 0)
    {
        dumpCounters(dieStats->readHeat, dieStats->numHeat, out, size);
    }
    else if (row && strcasecmp(row, "write") == 0)
    {
        dumpCounters(dieStats->writeHeat, dieStats->numHeat, out, size);
    }
    else if (row)
    {
        snprintf(out, size, "Expected 'heatmap read' or 'heatmap write', not 'heatmap %s'\n", row);
    }
    else
    {
        snprintf(out, size, "Expected an argument of 'read' or 'write'\n");
    }
}
#endif

void DieStatsCleanup(DieStatsHandle dieStats)
{
    if (dieStats == NULL)
    {
        return;
    }
    INSUnRegisterAction(dieStats->hInst, "heatmap");
    INSUnRegisterAction(dieStats->hInst, "diemap");
    SUfree(dieStats);
}

int DieStatsInit(SEFQoSHandle qosHandle,
                 struct SEFADUsize aduSize,
                 void* hInst,
                 DieStatsHandle* dieStats_out,
                 LogHandle logHandle)
{
    SEFHandle sefHandle =
        SEFGetHandle(SEFGetQoSHandleProperty(qosHandle, kSefPropertyUnitNumber).intVal);
    const struct SEFInfo* sefInfo = SEFGetInformation(sefHandle);
    ssize_t numDie = sefInfo->numBanks * sefInfo->numChannels;
    DieStatsHandle dieStats;
    ssize_t dieStatsSize = sizeof(*dieStats) + 2 * numDie * sizeof(*dieStats->readDie);
#if CC_HEATMAP
    ssize_t numHeat = numDie * sefInfo->numBlocks;
    dieStatsSize += 2 * numHeat * sizeof(*dieStats->readHeat);
#endif

    // alloc memory for counters
    *dieStats_out = NULL;
    dieStats = SUzalloc(dieStatsSize);

    if (dieStats == NULL)
    {
        LogError(logHandle, "Failed to allocate memory for die stats");
        return -ENOSPC;
    }

    dieStats->sefHandle = sefHandle;
    dieStats->qosHandle = qosHandle;
    dieStats->sefInfo = sefInfo;
    dieStats->hInst = hInst;
    int adus_per_page = sefInfo->pageSize / aduSize.data;
    dieStats->die_page_adus = sefInfo->numPlanes * adus_per_page;
    // dieStats->adus_per_die = sefInfo->numPlanes * sefInfo->numPages * adus_per_page;
    dieStats->numDie = numDie;
    dieStats->readDie = (void*)(dieStats + 1);
    dieStats->writeDie = dieStats->readDie + numDie;
    // register action
    INSRegisterAction(hInst, "diemap", dieMapAction, dieStats);
#if CC_HEATMAP
    dieStats->numHeat = numHeat;
    dieStats->readHeat = dieStats->writeDie + numDie;
    dieStats->writeHeat = dieStats->readHeat + numHeat;

    // check math
    assert((char*)(dieStats->writeHeat + numHeat) - (char*)dieStats == dieStatsSize);

    // register action
    INSRegisterAction(hInst, "heatmap", heatMapAction, dieStats);
#endif

    *dieStats_out = dieStats;
    return 0;
}

void dieStatsIndexes(DieStatsHandle dieStats,
                     const struct SEFDieList* vdDieList,
                     uint32_t block,
                     uint32_t adu,
                     ssize_t* pDieIndex,
                     ssize_t* pHeatIndex)
{
    // todo: only supports full width superpages (SB must include all dies in the VD)
    ssize_t vdDieIndex =
        (adu / dieStats->die_page_adus) % (vdDieList->numDies);    // vd relative die index
    uint16_t dieIndex = vdDieList->dieIDs[vdDieIndex];
    // ssize_t sefChannel = dieIndex % dieStats->sefInfo->numChannels;
    // ssize_t sefBank = dieIndex / dieStats->sefInfo->numChannels;
    ssize_t heatIndex = block + dieIndex * dieStats->sefInfo->numBlocks;

    *pDieIndex = dieIndex;
    *pHeatIndex = heatIndex;
}

void DieStatsWrite(DieStatsHandle dieStats,
                   const struct SEFDieList* vdDieList,
                   struct SEFFlashAddress* flashAddress,
                   ssize_t numAddr)
{
    int i;

    for (i = 0; i < numAddr; i++)
    {
        ssize_t dieIndex = dieStats->numDie;
#if CC_HEATMAP
        ssize_t heatIndex = dieStats->numHeat;
#else
        ssize_t heatIndex = 0;
#endif
        uint32_t block;
        uint32_t adu;

        if (SEFParseFlashAddress(dieStats->qosHandle, flashAddress[i], NULL, &block, &adu).error == 0)
        {
            dieStatsIndexes(dieStats, vdDieList, block, adu, &dieIndex, &heatIndex);

#if CC_HEATMAP
            assert(heatIndex < dieStats->numHeat);
            ++dieStats->writeHeat[heatIndex];
#endif
            assert(dieIndex < dieStats->numDie);
            ++dieStats->writeDie[dieIndex];
        }
    }
}

void DieStatsRead(DieStatsHandle dieStats,
                  const struct SEFDieList* vdDieList,
                  struct SEFFlashAddress flashAddress,
                  int numReads)
{
    uint32_t block;
    uint32_t adu;

    if (SEFParseFlashAddress(dieStats->qosHandle, flashAddress, NULL, &block, &adu).error == 0)
    {
        int i;
        for (i = 0; i < numReads; i++, adu++)
        {
            ssize_t dieIndex = dieStats->numDie;
#if CC_HEATMAP
            ssize_t heatIndex = dieStats->numHeat;
#else
            ssize_t heatIndex = 0;
#endif

            dieStatsIndexes(dieStats, vdDieList, block, adu, &dieIndex, &heatIndex);

#if CC_HEATMAP
            assert(heatIndex < dieStats->numHeat);
            ++dieStats->readHeat[heatIndex];
#endif
            assert(dieIndex < dieStats->numDie);
            ++dieStats->readDie[dieIndex];
        }
    }
}
