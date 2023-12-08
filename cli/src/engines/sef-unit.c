/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-unit.c
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
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "../cli-engine.h"
#include "../engine-option.h"
#include "../io-helper.h"
#include "../utils/str-error.h"

// prototypes
static int ESUPrintList(void **context, void *options);

static int ESUPrintInfo(void **context, void *options);

// engine options
struct ESUOptions
{
    void *pad;
    uint16_t sefUnitIndex;
    bool sefUnitIndexGiven;
    bool verbose;
};

static struct CEOEngineOption options[] = {
    CEOUInt('s', "sef-index", "The index of the target SEF Unit", k16, ESUOptions, sefUnitIndex, sefUnitIndexGiven),
    CEOFlag(
        'V', "verbose", "show additional information regarding the action being taken", ESUOptions, verbose)};

// engine setup/cleanup
struct engineData
{
    int numSefUnits;
};

static int ESUSetup(void **engineContext, void *engineOptions)
{
    struct SEFStatus status;
    struct engineData *data;

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

    return 0;
}

static int ESUCleanup(void **engineContext, void *engineOptions)
{
    if (*engineContext != NULL)
    {
        free(*engineContext);
    }

    SEFLibraryCleanup();

    return 0;
}

// engine functions
static int ESUPrintList(void **context, void *options)
{
    int returnVal, i;
    struct engineData *data = *context;

    // print table header
    CIHMessageDataNoWrap("SEF Unit %-8s %-14s %-14s %-14s %-10s %-10s", "Index", "Vendor",
                         "FW Version", "HW Version", "Channels", "Banks");

    returnVal = 0;
    for (i = 0; i < data->numSefUnits; i++)
    {
        SEFHandle sefHandle;
        const struct SEFInfo *info;

        // get sef handle
        sefHandle = SEFGetHandle(i);
        if (!sefHandle)
        {
            CIHMessageError("Error: Was unable to access SEF Unit %d", i);
            returnVal = EIO;
            continue;
        }

        // get sef unit information
        info = SEFGetInformation(sefHandle);
        if (!info)
        {
            CIHMessageError("Error: Was unable to get SEF Unit %d information", i);
            returnVal = EIO;
            continue;
        }

        CIHMessageDataNoWrap("SEF Unit %-8d %-14.8s %-14.8s %-14.8s %-10d %-10d", i, info->vendor,
                             info->FWVersion, info->HWVersion, info->numChannels, info->numBanks);
    }

    CIHMessageDataNoWrap("SEF Unit Count %d", data->numSefUnits);

    return returnVal;
}

static int ESUPrintInfo(void **context, void *options)
{
    int i;
    const struct SEFInfo *info;
    struct engineData *data = *context;
    struct ESUOptions *suo = options;
    SEFHandle sefHandle;

    // validating input parameters
    if (CIHIsNotGiven("sef-index", 's', suo->sefUnitIndexGiven))
    {
        return EINVAL;
    }

    if (CIHIsOutOfRange("sef-index", 's', suo->sefUnitIndex, 0, data->numSefUnits - 1))
    {
        return EINVAL;
    }

    // get sef unit handle
    sefHandle = SEFGetHandle(suo->sefUnitIndex);
    if (!sefHandle)
    {
        CIHMessageError("Error: Was unable to access SEF Unit %d", suo->sefUnitIndex);
        return EIO;
    }

    // get sef unit information
    info = SEFGetInformation(sefHandle);
    if (!info)
    {
        CIHMessageError("Error: Was unable to get SEF Unit %d information", suo->sefUnitIndex);
        return EIO;
    }

    // print SEF unit information
    CIHMessageData("name: %s", info->name);
    CIHMessageData("vendor: %8s", info->vendor);
    CIHMessageData("serialNumber: %8s", info->serialNumber);
    CIHMessageData("FWVersion: %8s", info->FWVersion);
    CIHMessageData("HWVersion: %8s", info->HWVersion);

    CIHMessageData("supported Options:");
    if (info->supportedOptions & kFragmentedSupported)
    {
        CIHMessageData("* kFragmentedSupported");
    }
    if (info->supportedOptions & kPackedSupported)
    {
        CIHMessageData("* kPackedSupported");
    }
    if (info->supportedOptions & kPerfectSupported)
    {
        CIHMessageData("* kPerfectSupported");
    }
    if (info->supportedOptions & kEncryptionSupported)
    {
        CIHMessageData("* kEncryptionSupported");
    }
    if (info->supportedOptions & kHostSerialNumberSupported)
    {
        CIHMessageData("* kHostSerialNumberSupported");
    }
    if (info->supportedOptions & kCopyUserAddressRangeSupported)
    {
        CIHMessageData("* kCopyUserAddressRangeSupported");
    }
    if (info->supportedOptions & kCopyFlashAddressListSupported)
    {
        CIHMessageData("* kCopyFlashAddressListSupported");
    }
    if (info->supportedOptions & kSuperBlockSupported)
    {
        CIHMessageData("* kSuperBlockSupported");
    }
    if (info->supportedOptions & kInDriveGCSupported)
    {
        CIHMessageData("* kInDriveGCSupported");
    }
    if (info->supportedOptions & kVirtualSSDSupported)
    {
        CIHMessageData("* kVirtualSSDSupported");
    }
    if (info->supportedOptions & kAutomaticSupported)
    {
        CIHMessageData("* kAutomaticSupported");
    }
    if (info->supportedOptions & kHostControlledSupported)
    {
        CIHMessageData("* kHostControlledSupported");
    }
    if (info->supportedOptions & kFastestSupported)
    {
        CIHMessageData("* kFastestSupported");
    }
    if (info->supportedOptions & kTypicalSupported)
    {
        CIHMessageData("* kTypicalSupported");
    }
    if (info->supportedOptions & kLongSupported)
    {
        CIHMessageData("* kLongSupported");
    }
    if (info->supportedOptions & kHeroicSupported)
    {
        CIHMessageData("* kHeroicSupported");
    }
    if (info->supportedOptions & kStableLatencySupported)
    {
        CIHMessageData("* kStableLatencySupported");
    }
    if (info->supportedOptions & kIdleTimeSupported)
    {
        CIHMessageData("* kIdleTimeSupported");
    }
    if (info->supportedOptions & kStopSupported)
    {
        CIHMessageData("* kStopSupported");
    }
    if (info->supportedOptions & kMixedDefectManagementSupported)
    {
        CIHMessageData("* kMixedDefectManagmentSupported");
    }
    if (info->supportedOptions & kPSLCSupported)
    {
        CIHMessageData("* kPSLCSupported");
    }
    if (info->supportedOptions & kDeleteVirtualDeviceSupported)
    {
        CIHMessageData("* kDeleteVirtualDeviceSupported");
    }

    CIHMessageData("unitNumber: %d", info->unitNumber);
    CIHMessageData("APIVersion: %d", info->APIVersion);
    CIHMessageData("supportedOptions: %" PRIu64, info->supportedOptions);
    CIHMessageData("maxQoSDomains: %d", info->maxQoSDomains);
    CIHMessageData("maxRootPointers: %d", info->maxRootPointers);
    CIHMessageData("maxPlacementIDs: %d", info->maxPlacementIDs);
    CIHMessageData("maxOpenSuperBlocks: %d", info->maxOpenSuperBlocks);
    CIHMessageData("numReadQueues: %d", info->numReadQueues);
    CIHMessageData("numVirtualDevices: %d", info->numVirtualDevices);
    CIHMessageData("numQoSDomains: %d", info->numQoSDomains);
    CIHMessageData("numBanks: %d", info->numBanks);
    CIHMessageData("numChannels: %d", info->numChannels);
    CIHMessageData("numPlanes: %d", info->numPlanes);
    CIHMessageData("pageSize: %d", info->pageSize);
    CIHMessageData("numPages: %d", info->numPages);
    CIHMessageData("numBlocks: %d", info->numBlocks);
    CIHMessageData("totalBandWidth: %d", info->totalBandWidth);
    CIHMessageData("readTime: %d", info->readTime);
    CIHMessageData("programTime: %d", info->programTime);
    CIHMessageData("eraseTime: %d", info->eraseTime);
    CIHMessageData("minReadWeight: %d", info->minReadWeight);
    CIHMessageData("minWriteWeight: %d", info->minWriteWeight);
    CIHMessageData("openExpirationPeriod: %d", info->openExpirationPeriod);
    CIHMessageData("ADUsize(%d):", info->numADUSizes);
    CIHMessageData("* data:meta");
    for (i = 0; i < info->numADUSizes; i++)
    {
        CIHMessageData("* %d:%d", info->ADUsize[i].data, info->ADUsize[i].meta);
    }

    return 0;
}

// engine declarations
static struct CEHEngineAction actions[] = {
    newEngineAction("list",
                    "Print list of available SEF Units. Additional info is printed in verbose mode",
                    &ESUPrintList),
    newEngineAction("info", "Prints SEF Unit's information", &ESUPrintInfo)};

static struct CEHEngineConfig config = newEngineWithSetup(
    "sef-unit", "Perform actions toward mounted sef units", ESUOptions, options, actions, &ESUSetup, &ESUCleanup);

// registering the engine
static void sefEngineInit sefUnitInit()
{
    CEHRegisterEngine(&config);
}
