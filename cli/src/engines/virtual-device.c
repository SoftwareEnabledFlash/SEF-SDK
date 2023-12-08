/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * virtual-device.c
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
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../cli-engine.h"
#include "../engine-option.h"
#include "../io-helper.h"
#include "../sef-helper.h"
#include "../utils/str-builder.h"
#include "../utils/str-error.h"

// prototypes
static int EVDPrintList(void **context, void *options);

static int EVDPrintInfo(void **context, void *options);

static int EVDDelete(void **context, void *options);

static int EVDCreate(void **context, void *options);

// engine options
struct EVDOptions
{
    void *pad;
    uint16_t sefUnitIndex;
    bool sefUnitIndexGiven;
    uint16_t virtualDeviceId;
    bool virtualDeviceIdGiven;
    uint16_t *channelNum;
    bool channelNumGiven;
    int channelNumLength;
    uint16_t *bankNum;
    bool bankNumGiven;
    int bankNumLength;
    uint16_t *repeatNum;
    bool repeatNumGiven;
    int repeatNumLength;
    uint32_t numPSLCSuperBlock;
    bool numPSLCSuperBlockGiven;
    uint32_t maxTimePerSuspend;
    bool maxTimePerSuspendGiven;
    uint32_t minTimeUntilSuspend;
    bool minTimeUntilSuspendGiven;
    uint32_t maxSuspendInterval;
    bool maxSuspendIntervalGiven;
    uint16_t *dieMap;
    bool dieMapGiven;
    int dieMapLength;
    union CEOTuple *superBlock;
    bool superBlockGiven;
    int superBlockLength;
    union CEOTuple *readWeight;
    bool readWeightGiven;
    int readWeightLength;
    int TilingStrategy;
    bool verbose;
    bool force;
};

static struct CEOEngineOption options[] = {
    CEOUInt('s', "sef-index", "The index of the target SEF Unit", k16, EVDOptions, sefUnitIndex, sefUnitIndexGiven),
    CEOUInt('v',
            "virtual-device-id",
            "The ID for the target Virtual Device",
            k16,
            EVDOptions,
            virtualDeviceId,
            virtualDeviceIdGiven),
    CEOUInt('-',
            "num-pslc-super-block",
            "The number of pSLC super blocks to set",
            k32,
            EVDOptions,
            numPSLCSuperBlock,
            numPSLCSuperBlockGiven),
    CEOUInt('-',
            "suspend-max-time-per",
            "Read will issued within this time.  When 0, the number of read command is unlimited. "
            "When larger than the device defined maximum value, the device defined value is used.",
            k32,
            EVDOptions,
            maxTimePerSuspend,
            maxTimePerSuspendGiven),
    CEOUInt('-',
            "suspend-min-time-until",
            "A suspend starts when the cumulated weights of pending reads exceeds this value.  "
            "When 0, the suspend will start immediately for a pending read.",
            k32,
            EVDOptions,
            minTimeUntilSuspend,
            minTimeUntilSuspendGiven),
    CEOUInt('-',
            "suspend-max-interval",
            "Suspend starts for a pending read command after this interval since the last resume "
            "(or start programming/erase).  When larger than the device defined minimum value, the "
            "device defined value is used.",
            k32,
            EVDOptions,
            maxSuspendInterval,
            maxSuspendIntervalGiven),
    CEOUIntArray('-',
                 "channel-num",
                 "A list of channels to be used to describe Virtual Device(s)",
                 k16,
                 EVDOptions,
                 channelNum,
                 channelNumGiven,
                 channelNumLength),
    CEOUIntArray('-',
                 "bank-num",
                 "A list of banks to be used to describe Virtual Device(s)",
                 k16,
                 EVDOptions,
                 bankNum,
                 bankNumGiven,
                 bankNumLength),
    CEOUIntArray('-',
                 "repeat-num",
                 "A list of numbers of times each configuration should be repeated",
                 k16,
                 EVDOptions,
                 repeatNum,
                 repeatNumGiven,
                 repeatNumLength),
    CEOUIntArray('-',
                 "die-map",
                 "A list of IDs locating Virtual Devices in a SEF Unit ordered by die",
                 k16,
                 EVDOptions,
                 dieMap,
                 dieMapGiven,
                 dieMapLength),
    CEOTupleArray(
        '-',
        "super-block",
        "A list of tuples denoting the Virtual Device ID and number of dies per Super Block",
        kSefOptionUnsignedInt,
        EVDOptions,
        superBlock,
        superBlockGiven,
        superBlockLength),
    CEOTupleArray(
        '-',
        "read-weight",
        "A list of tuples of Default weights for read operations for each possible read queue",
        kSefOptionUnsignedInt,
        EVDOptions,
        readWeight,
        readWeightGiven,
        readWeightLength),
    CEOEnumDefault('-',
                   "tiling-strategy",
                   "The strategy to be used to populate the remaining empty space in a SEF Unit",
                   "kStrict",
                   "kStrict,kFill",
                   EVDOptions,
                   TilingStrategy),
    CEOFlag('V', "verbose", "Show additional information regarding the action being taken", EVDOptions, verbose),
    CEOFlag('f', "force", "Never prompt before removing or creating Virtual Device", EVDOptions, force),
};

// engine setup/cleanup
struct engineData
{
    int numSefUnits;
    SEFHandle sefHandle;
    const struct SEFInfo *sefInfo;
};

static int EVDSetup(void **engineContext, void *options)
{
    struct engineData *data;
    struct SEFStatus status;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // allocate engine data
    *engineContext = malloc(sizeof(struct engineData));
    if (!(*engineContext))
    {
        return EINTR;
    }

    data = *engineContext;

    // init sef library
    status = SEFLibraryInit();
    if (status.error)
    {
        CIHMessageError("Error: Was unable to init the SEF Library");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFLibraryInit", status.error));
        return EIO;
    }

    if (!status.info)
    {
        CIHMessageError("Error: Was unable to find any SEF Units");
        return ENODEV;
    }

    data->numSefUnits = status.info;

    if (vdo->sefUnitIndexGiven)
    {
        if (CIHIsOutOfRange("sef-index", 's', vdo->sefUnitIndex, 0, data->numSefUnits - 1))
        {
            return EINVAL;
        }

        // get the handle for the SEF Unit
        data->sefHandle = SEFGetHandle(vdo->sefUnitIndex);
        if (!data->sefHandle)
        {
            CIHMessageError("Error: Was unable to get SEF Unit %u handle", vdo->sefUnitIndex);
            return EINVAL;
        }

        // get sef unit information
        data->sefInfo = SEFGetInformation(data->sefHandle);
        if (!data->sefInfo)
        {
            CIHMessageError("Error: Was unable to get SEF Unit %d information", vdo->sefUnitIndex);
            return EIO;
        }
    }

    return 0;
}

static int EVDCleanup(void **engineContext, void *options)
{
    if (*engineContext != NULL)
    {
        free(*engineContext);
    }

    SEFLibraryCleanup();

    return 0;
}

// local functions
/**
 * @brief       Prints the virtual device configuration
 **/
static void EVDPrintMap(struct SEFVirtualDeviceConfig **vdConfig, int vdCount, const struct SEFInfo *sefInfo)
{
    uint16_t i, j;
    char *dieMapInfo = calloc(sefInfo->numChannels * 5 + 1, sizeof(char));
    struct SEFVirtualDeviceID *dieMap =
        calloc(sefInfo->numChannels * sefInfo->numBanks, sizeof(struct SEFVirtualDeviceID));

    for (i = 0; i < vdCount; i++)
    {
        for (j = 0; j < vdConfig[i]->dieList.numDies; j++)
        {
            dieMap[vdConfig[i]->dieList.dieIDs[j]] = vdConfig[i]->virtualDeviceID;
        }
    }

    CIHMessageInfo("Die Map:");
    for (i = 0; i < sefInfo->numBanks; i++)
    {
        for (j = 0; j < sefInfo->numChannels; j++)
        {
            struct SEFVirtualDeviceID vID = dieMap[i * sefInfo->numChannels + j];
            sprintf(&dieMapInfo[j * 5], "%-5d", vID.id);
        }

        dieMapInfo[sefInfo->numChannels * 5] = '\0';
        CIHMessageInfo("%s", dieMapInfo);
    }

    free(dieMapInfo);
    free(dieMap);
}

/**
 * @brief       Checks if the proposed location for the virtual device is valid
 *
 * A virtual device location is valid if the dies are unused and the virtual device fits
 **/
static bool EVDIsValidDie(uint16_t channelNum,
                          uint16_t bankNum,
                          uint16_t startChannel,
                          uint16_t StartBank,
                          uint64_t *sefDieMap,
                          const struct SEFInfo *sefInfo,
                          struct SEFVirtualDeviceConfig *vdConfig,
                          uint16_t vdId)
{
    uint16_t channel, bank, i;

    if (bankNum + StartBank > sefInfo->numBanks || channelNum + startChannel > sefInfo->numChannels)
    {
        return false;
    }

    // check placement availablity
    for (bank = StartBank; bank < (bankNum + StartBank); bank++)
    {
        for (channel = startChannel; channel < (startChannel + channelNum); channel++)
        {
            uint64_t index = (channel * sefInfo->numBanks + bank) / 64;
            uint64_t mask = (uint64_t)1 << ((channel * sefInfo->numBanks + bank) % 64);

            if (sefDieMap[index] & mask)
            {
                return false;
            }
        }
    }

    // update local data
    i = 0;
    vdConfig->virtualDeviceID.id = vdId;
    vdConfig->superBlockDies = 0;
    vdConfig->numReadQueues = 0;
    vdConfig->dieList.numDies = channelNum * bankNum;
    for (bank = StartBank; bank < (bankNum + StartBank); bank++)
    {
        for (channel = startChannel; channel < (startChannel + channelNum); channel++)
        {
            uint64_t index = (channel * sefInfo->numBanks + bank) / 64;
            uint64_t mask = (uint64_t)1 << ((channel * sefInfo->numBanks + bank) % 64);

            sefDieMap[index] |= mask;
            vdConfig->dieList.dieIDs[i] = channel + sefInfo->numChannels * bank;
            i++;
        }
    }

    return true;
}

/**
 * @brief       Generates a Virtual Device map based on the input requirements and populates the required objects
 **/
static int EVDTiledDies(struct EVDOptions *vdo,
                        struct SEFVirtualDeviceConfig **vdConfig[0],
                        const struct SEFInfo *sefInfo)
{
    int i;
    uint16_t *vdRepeatNum;
    uint16_t neededDieNum, vdNeeded, dieNum, emptyDieNum, vdCount;
    uint64_t *sefDieMap;

    // validate inputs
    neededDieNum = 0;
    vdNeeded = 0;
    for (i = 0; i < vdo->repeatNumLength; i++)
    {
        vdNeeded += vdo->repeatNum[i];
        neededDieNum += vdo->channelNum[i] * vdo->bankNum[i] * vdo->repeatNum[i];
    }

    dieNum = sefInfo->numChannels * sefInfo->numBanks;    // maximum number of Virtual Devices (VD Id)
    if (CIHIsOutOfRange("virtual-device-id", 'v', vdo->virtualDeviceId, 1, dieNum))
    {
        return 0;
    }

    if (vdo->virtualDeviceId + vdNeeded > dieNum + 1)
    {
        CIHMessageError("Error: Virtual Device Id can not be bigger than number of dies %u", dieNum);
        return 0;
    }

    if (neededDieNum > dieNum)
    {
        CIHMessageError(
            "Error: The requested configuration is bigger than the maximum number of dies");
        return 0;
    }

    if (vdo->TilingStrategy == 0 && dieNum != neededDieNum && !vdo->force &&
        !CIHGetBoolInput(
            "The requested configuration does not fill-up the device; Should Continue?"))
    {
        return 0;
    }

    // prep objects
    vdCount = 0;
    emptyDieNum = dieNum;
    sefDieMap = calloc(((sefInfo->numChannels * sefInfo->numBanks + 63) / 64), sizeof(uint64_t));
    (*vdConfig) = malloc(sizeof(struct SEFVirtualDeviceConfig) * dieNum);

    vdRepeatNum = calloc(vdo->repeatNumLength, sizeof(uint16_t));
    memcpy(vdRepeatNum, vdo->repeatNum, vdo->repeatNumLength * sizeof(uint16_t));

    // place virtual devices
    for (i = 0; i < dieNum && emptyDieNum && vdCount < vdNeeded; i++)
    {
        uint16_t channel, bank, j, selectMax, isValidConfig;
        int32_t selectedIndex;

        // find best Virtul Device
        selectMax = 0;
        selectedIndex = -1;
        for (j = 0; j < vdo->repeatNumLength; j++)
        {
            if (vdRepeatNum[j] == 0 || vdo->bankNum[j] * vdo->channelNum[j] > emptyDieNum)
            {
                continue;
            }

            uint16_t temp = vdo->bankNum[j] * vdo->channelNum[j];
            if (selectMax < temp)
            {
                selectMax = temp;
                selectedIndex = j;
            }
        }

        assert(selectedIndex > -1);

        // create vdConfig
        (*vdConfig)[i] =
            malloc(sizeof(struct SEFVirtualDeviceConfig) +
                   sizeof(uint16_t) * vdo->bankNum[selectedIndex] * vdo->channelNum[selectedIndex]);

        isValidConfig = 0;
        for (channel = 0; channel < sefInfo->numChannels && !isValidConfig; channel++)
        {
            for (bank = 0; bank < sefInfo->numBanks && !isValidConfig; bank++)
            {
                isValidConfig = EVDIsValidDie(
                    vdo->channelNum[selectedIndex], vdo->bankNum[selectedIndex], channel, bank,
                    sefDieMap, sefInfo, (*vdConfig)[i], vdo->virtualDeviceId + vdCount);
            }
        }

        emptyDieNum -= vdo->channelNum[selectedIndex] * vdo->bankNum[selectedIndex];
        vdRepeatNum[selectedIndex]--;

        if (isValidConfig)
        {
            vdCount++;
        }
        else
        {
            for (j = 0; j < vdo->repeatNumLength; j++)
            {
                if (vdRepeatNum[j] > 0)
                {
                    CIHMessageError(
                        "Error: Was unable to find the optimal placement for the requested "
                        "configuration");

                    free(sefDieMap);
                    free(vdRepeatNum);
                    free(*vdConfig);
                    return 0;
                }

                vdRepeatNum[j] = -1;
            }
        }
    }

    // free memeory
    free(sefDieMap);
    free(vdRepeatNum);

    return vdCount;
}

/**
 * @brief       Validates and populates the required objects based on the requested die map
 **/
static int EVDDieMap(struct EVDOptions *vdo,
                     struct SEFVirtualDeviceConfig **vdConfig[0],
                     const struct SEFInfo *sefInfo)
{
    uint16_t vdMaxCount, vdCount, i;

    vdMaxCount = sefInfo->numChannels * sefInfo->numBanks;

    if (vdMaxCount != vdo->dieMapLength)
    {
        CIHMessageError(
            "Error: The die map array count %d does not match the number of dies in the device %u",
            vdo->dieMapLength, vdMaxCount);
        return 0;
    }

    for (i = 0; i < vdMaxCount; i++)
    {
        if (vdo->dieMap[i] > vdMaxCount)
        {
            CIHMessageError("Error: Virtual Device Id can not be bigger than number of dies %u",
                            vdMaxCount);
            return 0;
        }
    }

    // create an array
    vdCount = 0;
    (*vdConfig) = malloc(sizeof(struct SEFVirtualDeviceConfig *) * vdMaxCount);

    for (i = 0; i < vdo->dieMapLength; i++)
    {
        uint16_t j, numDies, dieIndex, virtualDeviceId;
        bool isNewVirtualDevice = true;

        // check if already created
        virtualDeviceId = vdo->dieMap[i];
        for (j = 0; j < vdCount && isNewVirtualDevice; j++)
        {
            if ((*vdConfig)[j]->virtualDeviceID.id == virtualDeviceId)
            {
                isNewVirtualDevice = false;
            }
        }

        // Skip vd0 for an unallocated die
        if (virtualDeviceId == 0)
        {
            continue;
        }

        if (!isNewVirtualDevice)
        {
            continue;
        }

        // get number of dies
        numDies = 0;
        for (j = 0; j < vdo->dieMapLength; j++)
        {
            if (vdo->dieMap[j] == virtualDeviceId)
            {
                numDies++;
            }
        }

        // populate virtual device config
        dieIndex = 0;
        (*vdConfig)[vdCount] =
            malloc(sizeof(struct SEFVirtualDeviceConfig) + sizeof(uint16_t) * numDies);
        (*vdConfig)[vdCount]->superBlockDies = 0;
        (*vdConfig)[vdCount]->numReadQueues = 0;
        (*vdConfig)[vdCount]->virtualDeviceID.id = virtualDeviceId;
        (*vdConfig)[vdCount]->dieList.numDies = numDies;
        for (j = 0; j < vdo->dieMapLength; j++)
        {
            if (vdo->dieMap[j] == virtualDeviceId)
            {
                (*vdConfig)[vdCount]->dieList.dieIDs[dieIndex] = j;
                dieIndex++;
            }
        }

        vdCount++;
    }
    return vdCount;
}

static void printRecreateCommand(struct engineData *data,
                                 struct EVDOptions *vdo,
                                 struct SEFVirtualDeviceInfo *vdInfoList,
                                 struct SEFVirtualDeviceList *vdList)
{
    int i, currentLen;
    struct StrBuilder createCommand;
    char *createCommandOut;
    struct SEFVirtualDeviceID *dieMap;
    uint16_t *vdNumDiesList;
    struct SEFStatus status;
    bool isConfigured;

    StrBldInit(&createCommand, 50 + 6 * data->sefInfo->numChannels * data->sefInfo->numBanks);

    StrBldAppendFormat(&createCommand, "sef-cli create virtual-device --sef-index %u",
                       vdo->sefUnitIndex);

    // allocate space for dieMap
    vdNumDiesList = calloc(vdList->numVirtualDevices, sizeof(uint16_t));
    dieMap = calloc(data->sefInfo->numChannels * data->sefInfo->numBanks,
                    sizeof(struct SEFVirtualDeviceID));

    // get dieMap list
    for (i = 0; i < vdList->numVirtualDevices; i++)
    {
        struct SEFDieList *dieList;
        uint16_t j, numDies;

        // get virtual device die
        numDies = 0;
        dieList = NULL;
        do
        {
            int dieListSize;

            if (dieList != NULL)
            {
                numDies = dieList->numDies;
                free(dieList);
            }

            dieListSize = struct_size(dieList, dieIDs, numDies);
            dieList = malloc(dieListSize);

            status = SEFGetDieList(data->sefHandle, vdList->virtualDeviceID[i], dieList, dieListSize);
            if (status.error)
            {
                CIHMessageError("Error: Was unable to get list of Virtual Device %u die list",
                                vdList->virtualDeviceID[i].id);
                CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetDieList", status.error));

                free(dieList);
                free(dieMap);
                free(vdNumDiesList);
                return;
            }

        } while (numDies != dieList->numDies);

        // create Virtual Device DieMap
        vdNumDiesList[i] = dieList->numDies;
        for (j = 0; j < numDies; j++)
        {
            dieMap[dieList->dieIDs[j]] = vdList->virtualDeviceID[i];
        }

        free(dieList);
    }

    // add diemap option to the command
    StrBldAppendFormat(&createCommand, " --die-map \"%u", dieMap[0].id);
    for (i = 1; i < data->sefInfo->numChannels * data->sefInfo->numBanks; i++)
    {
        StrBldAppendFormat(&createCommand, " %u", dieMap[i].id);
    }
    StrBldAppend(&createCommand, "\"");

    // add config options read-weight
    currentLen = StrBldGetLen(&createCommand);
    StrBldAppend(&createCommand, " --read-weight=\"");
    isConfigured = false;

    for (i = 0; i < vdList->numVirtualDevices; i++)
    {
        uint16_t j;

        // add read-weights to the string
        for (j = 0; j < SEFMaxReadQueues; j++)
        {
            if (vdInfoList[i].readWeights[j] != data->sefInfo->minReadWeight)
            {
                StrBldAppendFormat(&createCommand, "%u:%u ", vdList->virtualDeviceID[i].id,
                                   vdInfoList[i].readWeights[j]);

                isConfigured = true;
            }
        }
    }

    // remove added option if there was no extra config
    if (isConfigured)
    {
        StrBldAppend(&createCommand, "\"");
    }
    else
    {
        StrBldDelete(&createCommand, currentLen, SIZE_MAX);
    }

    // add config options super-block
    currentLen = StrBldGetLen(&createCommand);
    StrBldAppend(&createCommand, " --super-block=\"");
    isConfigured = false;

    for (i = 0; i < vdList->numVirtualDevices; i++)
    {
        if (vdInfoList[i].superBlockDies != vdNumDiesList[i])
        {
            StrBldAppendFormat(&createCommand, "%u:%u ", vdList->virtualDeviceID[i].id,
                               vdInfoList[i].superBlockDies);
        }
    }

    // remove added option if there was no extra config
    if (isConfigured)
    {
        StrBldAppend(&createCommand, "\"");
    }
    else
    {
        StrBldDelete(&createCommand, currentLen, SIZE_MAX);
    }

    // To avoid the -Wformat-security warning and possible attack vectors
    createCommandOut = StrBldToString(&createCommand);
    CIHMessageInfo("Recreate Command:");
    CIHMessageInfo("%s", createCommandOut);

    // cleanup
    StrBldCleanup(&createCommand);
    free(createCommandOut);
    free(dieMap);
    free(vdNumDiesList);
}

// engine functions
static int EVDPrintList(void **context, void *options)
{
    int i, returnVal;
    uint16_t numVirtualDevices;
    struct SEFStatus status;
    struct SEFVirtualDeviceList *vdList;
    struct SEFVirtualDeviceInfo *vdInfoList;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    if (CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven))
    {
        return EINVAL;
    }

    // get list of the the created virtual devices
    numVirtualDevices = 0;
    vdList = NULL;
    do
    {
        int vdListSize;

        if (vdList != NULL)
        {
            numVirtualDevices = vdList->numVirtualDevices;
            free(vdList);
        }

        vdListSize = struct_size(vdList, virtualDeviceID, numVirtualDevices);
        vdList = malloc(vdListSize);

        status = SEFListVirtualDevices(data->sefHandle, vdList, vdListSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of Virtual Devices");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFListVirtualDevices", status.error));

            free(vdList);
            return status.error;
        }

    } while (numVirtualDevices != vdList->numVirtualDevices);

    if (vdList->numVirtualDevices == 0)
    {
        CIHMessageData("No Virtual Devices were found on SEF Unit %u", vdo->sefUnitIndex);
        free(vdList);
        return 0;
    }

    vdInfoList = calloc(vdList->numVirtualDevices, sizeof(struct SEFVirtualDeviceInfo));

    // print table header
    CIHMessageDataNoWrap("Virtual Device %-8s %-13s %-13s %-10s %-10s", "ID", "Flash Cap",
                         "Flash Avail", "SB Cap", "SB Dies");

    // print virtual device list
    returnVal = 0;
    for (i = 0; i < vdList->numVirtualDevices; i++)
    {
        // get virtual device information
        status = SEFGetVirtualDeviceInformation(data->sefHandle, vdList->virtualDeviceID[i],
                                                &vdInfoList[i], sizeof(struct SEFVirtualDeviceInfo));
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get Virtual Device %u information",
                            vdList->virtualDeviceID[i].id);
            CIHMessageInfo("Detailed Error: %s",
                           SEFStrError("SEFGetVirtualDeviceInformation", status.error));

            returnVal = status.error;
            continue;
        }

        CIHMessageDataNoWrap("Virtual Device %-8d %-13" PRIu64 " %-13" PRIu64 " %-10d %-10d",
                             vdList->virtualDeviceID[i].id, vdInfoList[i].flashCapacity,
                             vdInfoList[i].flashAvailable, vdInfoList[i].superBlockCapacity,
                             vdInfoList[i].superBlockDies);
    }

    CIHMessageData("Virtual Device Count %u", vdList->numVirtualDevices);

    // print SEF Recreate Command
    if (vdo->verbose)
    {
        printRecreateCommand(data, vdo, vdInfoList, vdList);
    }

    // free objects
    free(vdInfoList);
    free(vdList);

    return returnVal;
}

static int EVDDraw(void **context, void *options)
{
    uint16_t numVirtualDevices, i, returnStatus;
    struct SEFStatus status;
    struct SEFVirtualDeviceConfig **vdConfig;
    struct SEFVirtualDeviceList *vdList;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    if (CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven))
    {
        return EINVAL;
    }

    // get list of the the available virtual devices
    numVirtualDevices = 0;
    vdList = NULL;
    do
    {
        int vdListSize;

        if (vdList != NULL)
        {
            numVirtualDevices = vdList->numVirtualDevices;
            free(vdList);
        }

        vdListSize = struct_size(vdList, virtualDeviceID, numVirtualDevices);
        vdList = malloc(vdListSize);

        status = SEFListVirtualDevices(data->sefHandle, vdList, vdListSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of Virtual Devices");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFListVirtualDevices", status.error));

            free(vdList);
            return status.error;
        }

    } while (numVirtualDevices != vdList->numVirtualDevices);

    if (vdList->numVirtualDevices == 0)
    {
        CIHMessageData("No Virtual Devices were found on SEF Unit %u", vdo->sefUnitIndex);
        free(vdList);

        return 0;
    }

    returnStatus = 0;
    vdConfig = calloc(numVirtualDevices, sizeof(struct SEFVirtualDeviceConfig *));

    for (i = 0; i < vdList->numVirtualDevices; i++)
    {
        uint16_t numDies;

        // get virtual device die
        numDies = 0;
        vdConfig[i] = NULL;
        do
        {
            int dieListSize, vdConfigSize;

            if (vdConfig[i] != NULL)
            {
                numDies = vdConfig[i]->dieList.numDies;
                free(vdConfig[i]);
            }

            vdConfigSize = sizeof(struct SEFVirtualDeviceConfig) + numDies * sizeof(uint16_t);
            dieListSize = sizeof(struct SEFDieList) + numDies * sizeof(uint16_t);
            vdConfig[i] = malloc(vdConfigSize);
            vdConfig[i]->virtualDeviceID = vdList->virtualDeviceID[i];

            status = SEFGetDieList(data->sefHandle, vdList->virtualDeviceID[i],
                                   &vdConfig[i]->dieList, dieListSize);
            if (status.error)
            {
                CIHMessageError("Error: Was unable to get list of Virtual Device %u die list",
                                vdList->virtualDeviceID[i].id);
                CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetDieList", status.error));

                returnStatus = status.error;
            }

        } while (numDies != vdConfig[i]->dieList.numDies);
    }

    // force verbose mode to print the map
    CIHSetLevel(kInfo);

    // print the map
    EVDPrintMap(vdConfig, numVirtualDevices, data->sefInfo);

    // free memory
    for (i = 0; i < numVirtualDevices; i++)
    {
        free(vdConfig[i]);
    }
    free(vdConfig);
    free(vdList);

    return returnStatus;
}

static void printUsageInfo(SEFHandle sefHandle, struct SEFVirtualDeviceID VirtualDeviceId)
{
    struct SEFStatus status;
    SEFVDHandle vdHandle;
    struct SEFVirtualDeviceUsage vdUsage;

    status = SEFOpenVirtualDevice(sefHandle, VirtualDeviceId, NULL, NULL, &vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open Virtual Device %u", VirtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenVirtualDevice", status.error));

        return;
    }

    status = SEFGetVirtualDeviceUsage(vdHandle, &vdUsage);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get Virtual Device %u usage information",
                        VirtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetVirtualDeviceUsage", status.error));
        return;
    }

    CIHMessageData("eraseCount: %u", vdUsage.eraseCount);
    CIHMessageData("numUnallocatedSuperBlocks: %u", vdUsage.numUnallocatedSuperBlocks);
    CIHMessageData("numSuperBlocks: %u", vdUsage.numSuperBlocks);
    CIHMessageData("numUnallocatedPSLCSuperBlocks: %u", vdUsage.numUnallocatedPSLCSuperBlocks);
    CIHMessageData("numPSLCSuperBlocks: %u", vdUsage.numPSLCSuperBlocks);
    CIHMessageData("averagePEcount: %u", vdUsage.averagePEcount);
    CIHMessageData("maxPEcount: %u", vdUsage.maxPEcount);
    CIHMessageData("patrolCycleTime: %u", vdUsage.patrolCycleTime);

    status = SEFCloseVirtualDevice(vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close the Virtual Device %u", VirtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseVirtualDevice", status.error));
        return;
    }
}

static int EVDPrintInfo(void **context, void *options)
{
    int i, isInputInvalid;
    uint16_t numDies, numQosDomains;
    struct SEFStatus status;
    struct SEFVirtualDeviceID VirtualDeviceId;
    struct SEFVirtualDeviceInfo *vdInfo = NULL;
    struct SEFDieList *vdDieList;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    isInputInvalid = 0;
    isInputInvalid += CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven);
    isInputInvalid += CIHIsNotGiven("virtual-device-id", 'v', vdo->virtualDeviceIdGiven);

    if (isInputInvalid)
    {
        return EINVAL;
    }

    // check if virtual device id is valid
    VirtualDeviceId.id = vdo->virtualDeviceId;
    if (!CSHIsVirtualDeviceIdValid(data->sefHandle, VirtualDeviceId))
    {
        CIHMessageError("Error: The Virtual Device Id %u is invalid", VirtualDeviceId.id);
        return ENODEV;
    }

    // get Virtual Device information
    vdInfo = NULL;
    numQosDomains = 0;
    do
    {
        int vdInfoSize;

        if (vdInfo != NULL)
        {
            numQosDomains = vdInfo->QoSDomains.numQoSDomains;
            free(vdInfo);
        }

        vdInfoSize = sizeof(struct SEFVirtualDeviceInfo) + sizeof(struct SEFQoSDomainID) * numQosDomains;
        vdInfo = malloc(vdInfoSize);

        status = SEFGetVirtualDeviceInformation(data->sefHandle, VirtualDeviceId, vdInfo, vdInfoSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get Virtual Device %d Information",
                            VirtualDeviceId.id);
            CIHMessageInfo("Detailed Error: %s",
                           SEFStrError("SEFGetVirtualDeviceInformation", status.error));
            free(vdInfo);
            return status.error;
        }

    } while (numQosDomains != vdInfo->QoSDomains.numQoSDomains);

    // get die list
    numDies = 0;
    vdDieList = NULL;
    do
    {
        int vdDieListSize;

        if (vdDieList != NULL)
        {
            numDies = vdDieList->numDies;
            free(vdDieList);
        }

        vdDieListSize = sizeof(struct SEFDieList) + sizeof(uint16_t) * numDies;
        vdDieList = malloc(vdDieListSize);

        status = SEFGetDieList(data->sefHandle, VirtualDeviceId, vdDieList, vdDieListSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get Virtual Device %u die information",
                            VirtualDeviceId.id);
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetDieList", status.error));

            free(vdInfo);
            free(vdDieList);
            return status.error;
        }

    } while (numDies != vdDieList->numDies);

    // print virtual device information
    CIHMessageData("VirtualDeviceId: %u", VirtualDeviceId.id);
    CIHMessageData("flashCapacity: %" PRIu64, vdInfo->flashCapacity);
    CIHMessageData("flashAvailable: %" PRIu64, vdInfo->flashAvailable);
    CIHMessageData("pSLCFlashCapacity: %" PRIu64, vdInfo->pSLCFlashCapacity);
    CIHMessageData("pSLCFlashAvailable: %" PRIu64, vdInfo->pSLCFlashAvailable);
    CIHMessageData("superBlockCapacity: %u", vdInfo->superBlockCapacity);
    CIHMessageData("pSLCSuperBlockCapacity: %u", vdInfo->pSLCSuperBlockCapacity);
    CIHMessageData("maxOpenSuperBlocks: %u", vdInfo->maxOpenSuperBlocks);
    CIHMessageData("numPSLCSuperBLocks: %u", vdInfo->numPSLCSuperBLocks);
    CIHMessageData("suspendConfig:");
    CIHMessageData("* maxTimePerSuspend: %u", vdInfo->suspendConfig.maxTimePerSuspend);
    CIHMessageData("* minTimeUntilSuspend: %u", vdInfo->suspendConfig.minTimeUntilSuspend);
    CIHMessageData("* maxSuspendInterval: %u", vdInfo->suspendConfig.maxSuspendInterval);
    CIHMessageData("superBlockDies: %u", vdInfo->superBlockDies);
    CIHMessageData("aduOffsetBitWidth: %u", vdInfo->aduOffsetBitWidth);
    CIHMessageData("superBlockIdBitWidth: %u", vdInfo->superBlockIdBitWidth);

    CIHMessageData("Read Queues Weights (%u):", vdInfo->numReadQueues);
    for (i = 0; i < vdInfo->numReadQueues; i++)
    {
        CIHMessageData("* Read Weight %u", vdInfo->readWeights[i]);
    }

    if (!vdInfo->QoSDomains.numQoSDomains)
    {
        CIHMessageData("QoS Domains: None");
    }
    else
    {
        CIHMessageData("QoS Domains (%u):", vdInfo->QoSDomains.numQoSDomains);
        for (i = 0; i < vdInfo->QoSDomains.numQoSDomains; i++)
        {
            CIHMessageData("* QoS Domain %u", vdInfo->QoSDomains.QoSDomainID[i].id);
        }
    }

    CIHMessageData("Dies (%u):", vdDieList->numDies);
    for (i = 0; i < vdDieList->numDies; i++)
    {
        CIHMessageData("* Die %u", vdDieList->dieIDs[i]);
    }

    // print usage in verbose
    if (vdo->verbose)
    {
        printUsageInfo(data->sefHandle, VirtualDeviceId);
    }
    free(vdDieList);
    free(vdInfo);

    return 0;
}

static int EVDDelete(void **context, void *options)
{
    struct SEFStatus status;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    if (CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven))
    {
        return EINVAL;
    }

    // check for empty sef
    data->sefInfo = SEFGetInformation(data->sefHandle);
    if (!data->sefInfo->numVirtualDevices)
    {
        CIHMessageData("The SEF Unit %u is empty", vdo->sefUnitIndex);
        return 0;
    }

    // check if delete is supported
    if (!(data->sefInfo->supportedOptions & kDeleteVirtualDeviceSupported))
    {
        CIHMessageError("Error: Sef Unit %d does not support deleting Virtaul Devices",
                        vdo->sefUnitIndex);
        return EIO;
    }

    // check for force action
    if (!vdo->force &&
        !CIHGetBoolInput("Are you sure you want to delete all Virtual Devices in SEF Unit %u?",
                         vdo->sefUnitIndex))
    {
        return 0;
    }

    // delete the virtual device
    status = SEFDeleteVirtualDevices(data->sefHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to delete Virtual Devices");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFDeleteVirtualDevices", status.error));

        return status.error;
    }

    CIHMessageInfo("All Virtual Devices in SEF Unit %u were successfully deleted", vdo->sefUnitIndex);

    return 0;
}

static int EVDCreate(void **context, void *options)
{
    int isInvalid, tileInput;
    uint16_t vdCount;
    struct SEFStatus status;
    struct SEFVirtualDeviceConfig **vdConfig;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    isInvalid = 0;
    tileInput = vdo->repeatNumGiven + vdo->bankNumGiven + vdo->channelNumGiven;
    isInvalid += CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven);
    if (tileInput != 0 && tileInput != 3)
    {
        CIHMessageError(
            "Error: --channel-num, --bank-num and --repeat-num are all required and should have "
            "the same length");
        isInvalid++;
    }

    if (tileInput == 3 || !vdo->dieMapGiven)
    {
        isInvalid += CIHIsNotGiven("virtual-device-id", 'v', vdo->virtualDeviceIdGiven);
    }

    if (tileInput == 3 &&
        (vdo->channelNumLength != vdo->bankNumLength || vdo->bankNumLength != vdo->repeatNumLength))
    {
        CIHMessageError(
            "Error: --channel-num, --bank-num and --repeat-num should be the same length");
        isInvalid++;
    }

    if (isInvalid)
    {
        return EINVAL;
    }

    // validate input parameters
    data->sefInfo = SEFGetInformation(data->sefHandle);
    if (data->sefInfo->numVirtualDevices)
    {
        CIHMessageError("Error: SEF Unit %u is not empty; delete created Virtual Devices",
                        vdo->sefUnitIndex);
        return EIO;
    }

    // generate the virtual device config
    if (tileInput == 3)
    {
        vdCount = EVDTiledDies(vdo, &vdConfig, data->sefInfo);
    }
    else if (vdo->dieMapGiven)
    {
        vdCount = EVDDieMap(vdo, &vdConfig, data->sefInfo);
    }
    else
    {
        // fill the sef unit with one single virtual device
        uint16_t channels[] = {data->sefInfo->numChannels};
        uint16_t banks[] = {data->sefInfo->numBanks};
        uint16_t repeatNum[] = {1};

        vdo->channelNum = channels;
        vdo->bankNum = banks;
        vdo->repeatNum = repeatNum;

        vdo->channelNumLength = 1;
        vdo->bankNumLength = 1;
        vdo->repeatNumLength = 1;

        vdCount = EVDTiledDies(vdo, &vdConfig, data->sefInfo);

        vdo->channelNum = NULL;
        vdo->bankNum = NULL;
        vdo->repeatNum = NULL;
    }

    if (!vdCount)
    {
        return 0;
    }

    // populate the Super Block dies
    if (vdo->superBlockGiven)
    {
        int i, j;

        for (i = 0; i < vdCount; i++)
        {
            for (j = 0; j < vdo->superBlockLength; j++)
            {
                if (vdo->superBlock[j].tUint64.a == vdConfig[i]->virtualDeviceID.id)
                {
                    if (vdConfig[i]->dieList.numDies % vdo->superBlock[j].tUint64.b != 0)
                    {
                        CIHMessageError(
                            "Error: invalid argument '--super-block', number of dies in Virtual "
                            "Device is not divisible by super blocks");

                        // free memory
                        for (int i = 0; i < vdCount; i++)
                        {
                            free(vdConfig[i]);
                        }

                        free(vdConfig);

                        return EIO;
                    }

                    vdConfig[i]->superBlockDies = vdo->superBlock[j].tUint64.b;
                }
            }
        }
    }

    // populate the Read Weights
    if (vdo->readWeightGiven)
    {
        int i, j;

        for (i = 0; i < vdCount; i++)
        {
            for (j = 0; j < vdo->readWeightLength; j++)
            {
                if (vdo->readWeight[j].tUint64.a == vdConfig[i]->virtualDeviceID.id)
                {
                    vdConfig[i]->readWeights[vdConfig[i]->numReadQueues] =
                        vdo->readWeight[j].tUint64.b;
                    vdConfig[i]->numReadQueues++;
                }
            }
        }

        // check if all weights are populated
        for (i = 0; i < vdCount; i++)
        {
            if (vdConfig[i]->numReadQueues == 0)
            {
                CIHMessageError(
                    "Warning: The read weight for the Virtual Device %u was not specified",
                    vdConfig[i]->virtualDeviceID.id);

                vdConfig[i]->numReadQueues = 1;
                vdConfig[i]->readWeights[0] = data->sefInfo->minReadWeight;
            }
        }
    }
    else
    {
        // assign default value for all weights
        int i;
        for (i = 0; i < vdCount; i++)
        {
            vdConfig[i]->numReadQueues = 1;
            vdConfig[i]->readWeights[0] = data->sefInfo->minReadWeight;
        }
    }

    // print die map
    if (vdo->verbose)
    {
        EVDPrintMap(vdConfig, vdCount, data->sefInfo);
    }

    // check for force action
    if (!vdo->force && !CIHGetBoolInput("Are you sure you want to create %u Virtual Devices?", vdCount))
    {
        // free memory
        for (int i = 0; i < vdCount; i++)
        {
            free(vdConfig[i]);
        }

        free(vdConfig);

        return 0;
    }

    // create virtual device
    status = SEFCreateVirtualDevices(data->sefHandle, vdCount, vdConfig);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to create Virtual Device for SEF Unit %u", vdo->sefUnitIndex);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCreateVirtualDevices", status.error));

        // free memory
        for (int i = 0; i < vdCount; i++)
        {
            free(vdConfig[i]);
        }

        free(vdConfig);

        return status.error;
    }
    else
    {
        CIHMessageInfo("The Virtual Devices for SEF Unit %u was successfully created",
                       vdo->sefUnitIndex);
    }

    // free memory
    for (int i = 0; i < vdCount; i++)
    {
        free(vdConfig[i]);
    }

    free(vdConfig);

    return 0;
}

static int EVDSetSuspendConfig(void **context, void *options)
{
    int isInputInvalid;
    struct SEFStatus status;
    SEFVDHandle vdHandle;
    struct SEFVirtualDeviceID virtualDeviceId;
    struct SEFVirtualDeviceInfo vdInfo;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    isInputInvalid = 0;
    isInputInvalid += CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven);
    isInputInvalid += CIHIsNotGiven("virtual-device-id", 'v', vdo->virtualDeviceIdGiven);
    if (!vdo->maxTimePerSuspendGiven && !vdo->minTimeUntilSuspendGiven && !vdo->maxSuspendIntervalGiven)
    {
        CIHMessageError(
            "Error: '--suspend-max-time-per', '--suspend-min-time-until' or "
            "'--suspend-max-interval' is required");
        isInputInvalid++;
    }

    if (isInputInvalid)
    {
        return EINVAL;
    }

    // check if virtual device id is valid
    virtualDeviceId.id = vdo->virtualDeviceId;
    if (!CSHIsVirtualDeviceIdValid(data->sefHandle, virtualDeviceId))
    {
        CIHMessageError("Error: The Virtual Device Id %u is invalid", virtualDeviceId.id);
        return ENODEV;
    }

    // check for force action
    if (!vdo->force &&
        !CIHGetBoolInput("Are you sure you want to set the suspend config for Virtual Device %u?",
                         virtualDeviceId.id))
    {
        return 0;
    }

    // open virtual device
    status = SEFOpenVirtualDevice(data->sefHandle, virtualDeviceId, NULL, NULL, &vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open Virtual Device %u", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenVirtualDevice", status.error));

        return status.error;
    }

    // get current suspend config
    status = SEFGetVirtualDeviceInformation(data->sefHandle, virtualDeviceId, &vdInfo,
                                            sizeof(struct SEFVirtualDeviceInfo));
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get Virtual Device %d information", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s",
                       SEFStrError("SEFGetVirtualDeviceInformation", status.error));
        SEFCloseVirtualDevice(vdHandle);
        return status.error;
    }

    // set the new suspend config
    if (vdo->maxTimePerSuspendGiven)
    {
        vdInfo.suspendConfig.maxTimePerSuspend = vdo->maxTimePerSuspend;
    }
    if (vdo->minTimeUntilSuspendGiven)
    {
        vdInfo.suspendConfig.minTimeUntilSuspend = vdo->minTimeUntilSuspend;
    }
    if (vdo->maxSuspendIntervalGiven)
    {
        vdInfo.suspendConfig.maxSuspendInterval = vdo->maxSuspendInterval;
    }

    status = SEFSetVirtualDeviceSuspendConfig(vdHandle, &vdInfo.suspendConfig);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to set Virtual Device %d config", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s",
                       SEFStrError("SEFSetVirtualDeviceSuspendConfig", status.error));
        SEFCloseVirtualDevice(vdHandle);
        return status.error;
    }
    else
    {
        CIHMessageInfo("The suspend configurations for Virtual Devices %u was successfully set",
                       virtualDeviceId.id);
    }

    // close virtual device
    status = SEFCloseVirtualDevice(vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close Virtual Device %u", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseVirtualDevice", status.error));

        return status.error;
    }

    return 0;
}

static int EVDSetPSLC(void **context, void *options)
{
    int isInputInvalid;
    struct SEFStatus status;
    SEFVDHandle vdHandle;
    struct SEFVirtualDeviceID virtualDeviceId;
    struct SEFVirtualDeviceInfo vdInfo;
    struct engineData *data = *context;
    struct EVDOptions *vdo = (struct EVDOptions *)options;

    // validating input parameters
    isInputInvalid = 0;
    isInputInvalid += CIHIsNotGiven("sef-index", 's', vdo->sefUnitIndexGiven);
    isInputInvalid += CIHIsNotGiven("virtual-device-id", 'v', vdo->virtualDeviceIdGiven);
    isInputInvalid += CIHIsNotGiven("num-pslc-super-block", '-', vdo->numPSLCSuperBlockGiven);

    if (isInputInvalid)
    {
        return EINVAL;
    }

    // check if pslc is supported
    if (!(data->sefInfo->supportedOptions & kPSLCSupported))
    {
        CIHMessageError("Error: Sef Unit %d does not support pSLC", vdo->sefUnitIndex);
        return ENOTSUP;
    }

    // check if virtual device id is valid
    virtualDeviceId.id = vdo->virtualDeviceId;
    if (!CSHIsVirtualDeviceIdValid(data->sefHandle, virtualDeviceId))
    {
        CIHMessageError("Error: The Virtual Device Id %u is invalid", virtualDeviceId.id);
        return ENODEV;
    }

    // check for force action
    if (!vdo->force &&
        !CIHGetBoolInput("Are you sure you want to set the PSLC count for Virtual Device %u?",
                         virtualDeviceId.id))
    {
        return 0;
    }

    // open virtual device
    status = SEFOpenVirtualDevice(data->sefHandle, virtualDeviceId, NULL, NULL, &vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open Virtual Device %u", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenVirtualDevice", status.error));

        return status.error;
    }

    // set PSLC size
    status = SEFSetNumberOfPSLCSuperBlocks(vdHandle, vdo->numPSLCSuperBlock);
    if (status.error)
    {
        SEFCloseVirtualDevice(vdHandle);

        CIHMessageError("Error: Was unable to set PSLC Virtual Device %u size", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFSetNumberOfPSLCSuperBlocks", status.error));

        return status.error;
    }
    else
    {
        CIHMessageInfo(
            "The number of pSLC Super Blocks for Virtual Devices %u was successfully set",
            virtualDeviceId.id);
    }

    // get updated suspend config
    status = SEFGetVirtualDeviceInformation(data->sefHandle, virtualDeviceId, &vdInfo,
                                            sizeof(struct SEFVirtualDeviceInfo));
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get Virtual Device %d information", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s",
                       SEFStrError("SEFGetVirtualDeviceInformation", status.error));
        SEFCloseVirtualDevice(vdHandle);
        return status.error;
    }

    // check if enough super blocks were set
    if (vdInfo.numPSLCSuperBLocks < vdo->numPSLCSuperBlock)
    {
        CIHMessageData(
            "Warning: Number of set pSLC super blocks were less than the requested amount, Was "
            "only able to set %d super blocks as pSLC",
            vdInfo.numPSLCSuperBLocks);
    }

    // close virtual device
    status = SEFCloseVirtualDevice(vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close Virtual Device %u", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseVirtualDevice", status.error));

        return status.error;
    }

    return 0;
}

// engine declarations
static struct CEHEngineAction actions[] = {
    newEngineAction("list",
                    "Print list of created Virtual Devices. Commands to recreate same "
                    "configuration is printed in verbose",
                    &EVDPrintList),
    newEngineAction("draw", "Print a map of created Virtual Devices", &EVDDraw),
    newEngineAction("info",
                    "Print a Virtual Device's information; Prints usage data in verbose mode",
                    &EVDPrintInfo),
    newEngineAction("create", "Create a new Virtual Device", &EVDCreate),
    newEngineAction("delete",
                    "Delete Virtual Device. Virtual Device can only be deleted if it is empty",
                    &EVDDelete),
    newEngineAction("set-pslc", "Sets the number of pSLC super blocks for a virtual device", &EVDSetPSLC),
    newEngineAction("set-suspend-config",
                    "Sets the suspend configuration for a Virtual Device",
                    &EVDSetSuspendConfig),
};

static struct CEHEngineConfig config = newEngineWithSetup("virtual-device",
                                                          "Perform actions toward Virtual Devices",
                                                          EVDOptions,
                                                          options,
                                                          actions,
                                                          &EVDSetup,
                                                          &EVDCleanup);

// registering the engine
static void sefEngineInit virtualDeviceInit()
{
    CEHRegisterEngine(&config);
}
