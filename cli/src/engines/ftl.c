/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * ftl.c
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
#include <errno.h>
#include "SEFAPI.h"
#include "sef-utils.h"
#define CC_BACKUP_RESTORE 1    // Required, because we can't include config.h
#include <inttypes.h>
#include <stddef.h>
#include "sef-block-module.h"

#include "../cli-engine.h"
#include "../engine-option.h"
#include "../io-helper.h"
#include "../sef-helper.h"
#include "../utils/str-error.h"

// prototypes
static int ESBPrintInfo(void **context, void *options);

static int ESBConfigure(void **context, void *options);

static int ESBCheck(void **context, void *options);

// engine options
struct ESBOptions
{
    void *pad;
    uint16_t sefUnitIndex;
    bool sefUnitIndexGiven;
    uint16_t qosDomainId;
    bool qosDomainIdGiven;
    uint64_t *label;
    bool labelGiven;
    int labelLength;
    uint8_t overprovisioning;
    bool shouldRepair;
    bool force;
    uint16_t numDomains;
};

static struct CEOEngineOption options[] = {
    CEOUInt('s', "sef-index", "The index of the target SEF Unit", k16, ESBOptions, sefUnitIndex, sefUnitIndexGiven),
    CEOUInt('q', "qos-domain-id", "The ID for the target QoS Domain", k16, ESBOptions, qosDomainId, qosDomainIdGiven),
    CEOUIntArray(
        'l', "label", "The label for the target QoS Domain", k64, ESBOptions, label, labelGiven, labelLength),
    CEOUIntDefault('-', "num-domains", "Number of domains for device", k16, "1", ESBOptions, numDomains),
    CEOUIntDefault(
        '-', "overprovisioning", "Percent of overprovisioning (e.g., 20 for 20%%)", k8, "0", ESBOptions, overprovisioning),
    CEOFlag('-',
            "should-repair",
            "Should the Block Module be repaired if problems are found. Might result in losing "
            "placement data",
            ESBOptions,
            shouldRepair),
    CEOFlag('f', "force", "Never prompt before formating or configuring SDK", ESBOptions, force),
};

// engine setup/cleanup
static int ESBSetup(void **engineContext, void *engineOptions)
{
    int isInvalid, i, maxNumDomains;
    SEFHandle sefHandle;
    struct SEFStatus status;
    struct ESBOptions *sdko = (struct ESBOptions *)engineOptions;

    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', sdko->sefUnitIndexGiven);
    if (!sdko->qosDomainIdGiven && !sdko->labelGiven)
    {
        CIHMessageError("Error: '--qos-domain-id' (`-q`) or '--label' (`-l`) option is required");
        isInvalid++;
    }

    if (sdko->labelGiven && sdko->labelLength > 2)
    {
        CIHMessageError("Error: '--label' (`-l`) option exceeds the required size of 16 bytes");
        isInvalid++;
    }

    if (isInvalid)
    {
        return EINVAL;
    }

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

    if (CIHIsOutOfRange("sef-index", 's', sdko->sefUnitIndex, 0, status.info))
    {
        return ENODEV;
    }

    // get the handle for the SEF Unit
    sefHandle = SEFGetHandle(sdko->sefUnitIndex);
    if (!sefHandle)
    {
        CIHMessageError("Error: The SEF Unit Index was invalid");
        return EINVAL;
    }

    // get qos domain id if label provided
    if (sdko->labelGiven && !sdko->qosDomainIdGiven)
    {
        struct SIFLabel label;

        label.data[0] = sdko->label[0];
        label.data[1] = sdko->labelLength == 2 ? sdko->label[1] : 0;

        status = CSHGetQosDomainId(sefHandle, label);
        if (status.error)
        {
            return status.error;
        }

        sdko->qosDomainId = status.info;
        sdko->qosDomainIdGiven = true;
    }

    maxNumDomains = SEFGetInformation(sefHandle)->numChannels * SEFGetInformation(sefHandle)->numBanks;
    if (CIHIsOutOfRange("num-domains", '-', sdko->numDomains, 1, maxNumDomains))
    {
        return EINVAL;
    }

    // validate qos domains ids
    for (i = 0; i < sdko->numDomains; i++)
    {
        struct SEFQoSDomainID brDomainId;

        brDomainId.id = sdko->qosDomainId + i;
        if (!CSHIsQosDomainIdValid(sefHandle, brDomainId))
        {
            CIHMessageError("Error: The QoS Domain ID (%d) is invalid", brDomainId.id);
            return EINVAL;
        }
    }

    return 0;
}

static int ESBCleanup(void **engineContext, void *engineOptions)
{
    // clean up sef library
    SEFLibraryCleanup();

    return 0;
}

// engine functions
static int ESBPrintInfo(void **context, void *options)
{
    struct SEFStatus status;
    struct SEFBlockInfo blockInfo;
    struct SEFQoSDomainID qosDomainId;
    struct ESBOptions *sdko = (struct ESBOptions *)options;
    struct SEFBlockOption blockOption = {.delayMount = true};
    SEFBlockHandle blockHandle;

    qosDomainId.id = sdko->qosDomainId;
    status = SEFBlockInit(sdko->sefUnitIndex, qosDomainId, &blockOption, &blockHandle);
    if (status.error)
    {
        switch (status.error)
        {
            case -EBADF:
                CIHMessageError("The FTL was not shutdown cleanly; Consider running Check FTL");
                break;

            case -ENOENT:
                CIHMessageError(
                    "The QoS Domain Id %d isn't configured with Block Module; You may need to "
                    "configure it",
                    sdko->qosDomainId);
                break;

            default:
                CIHMessageError("Was unable to get SEF Block Info");
                break;
        }

        return EIO;
    }
    SEFBlockGetInfo(blockHandle, &blockInfo);
    SEFBlockCleanup(&blockHandle);

    // print SEF SDK information
    CIHMessageData("aduSize: %d", blockInfo.aduSize.data);
    CIHMessageData("numPlacementIDs: %d", blockInfo.numPlacementIDs);
    CIHMessageData("superBlockSize: %d", blockInfo.superBlockSize);
    CIHMessageData("superPageSize: %d", blockInfo.superPageSize);
    CIHMessageData("capacity: %" PRIu64, blockInfo.capacity);
    CIHMessageData("overprovisioning: %d", blockInfo.overprovisioning);
    CIHMessageData("numDomains: %d", blockInfo.numDomains);

    // print SEF SDK Recreate Command
    CIHMessageInfo("Recreate Command:");
    CIHMessageInfo("sef-cli configure sdk -s %d -q %d --overprovisioning %d", sdko->sefUnitIndex,
                   sdko->qosDomainId, blockInfo.overprovisioning);

    return 0;
}

static int ESBConfigure(void **context, void *options)
{
    struct SEFStatus status;
    struct SEFBlockConfig blockConfig = {0};
    struct SEFQoSDomainID qosDomainId;
    struct ESBOptions *sdko = (struct ESBOptions *)options;

    if (CIHIsOutOfRange("overprovisioning", '-', sdko->overprovisioning, 0, 80))
    {
        return EINVAL;
    }

    // populate the config object
    blockConfig.overprovisioning = sdko->overprovisioning;
    blockConfig.numDomains = sdko->numDomains;

    // check for force action
    if (!sdko->force && !CIHGetBoolInput("Are you sure you want to configure the SEF SDK?"))
    {
        return 0;
    }

    // configure the device
    qosDomainId.id = sdko->qosDomainId;
    status = SEFBlockConfig(sdko->sefUnitIndex, qosDomainId, &blockConfig);
    if (status.error)
    {
        switch (status.error)
        {
            case -ENOTBLK:
                CIHMessageError("The SDK requires one root pointer to configure the device");
                break;

            case -EEXIST:
                CIHMessageError(
                    "The QoS Domain ID %d is already configured, can not reconfigure a "
                    "pre-configured device",
                    sdko->qosDomainId);
                break;

            case -ENOSPC:
                CIHMessageError(
                    "The QoS Domain ID %d is too small for the required reserve; %d ADUs are "
                    "required",
                    sdko->qosDomainId, status.info);
                break;

            default:
                CIHMessageError("Was unable to configure SEF Block");
                break;
        }

        return status.error;
    }

    CIHMessageInfo("The QoS Domain %d was successfully configured by SDK", sdko->qosDomainId);

    return 0;
}

static int ESBCheck(void **context, void *options)
{
    struct SEFStatus status;
    struct SEFQoSDomainID qosDomainId;
    struct ESBOptions *sdko = (struct ESBOptions *)options;
    struct SEFBlockOption blockOption = {.delayMount = true};
    SEFBlockHandle blockHandle;

    if (CIHIsOutOfRange("overprovisioning", '-', sdko->overprovisioning, 0, 80))
    {
        return EINVAL;
    }

    // populate the config object
    qosDomainId.id = sdko->qosDomainId;

    // check the sef block module
    status = SEFBlockInit(sdko->sefUnitIndex, qosDomainId, &blockOption, &blockHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to repair the SEF SDK");
        return status.error;
    }
    status = SEFBlockCheck(blockHandle, sdko->shouldRepair);
    SEFBlockCleanup(&blockHandle);
    if (sdko->shouldRepair)
    {
        if (status.error)
        {
            CIHMessageError("Error: Was unable to repair the SEF SDK");
            return status.error;
        }

        CIHMessageInfo("The SEF SDK was repaired");
    }
    else
    {
        if (status.error)
        {
            CIHMessageData(
                "The SEF SDK is broken and needs repair; please consider passing in the "
                "should-repair flag");
            return status.error;
        }

        CIHMessageInfo("The SEF SDK was not broken and does not need repair");
    }

    return 0;
}

// engine declarations
static struct CEHEngineAction actions[] = {
    newEngineAction("info", "Prints the SEF Block Module's information/configuration", &ESBPrintInfo),
    newEngineAction("configure", "Configure the SEF Block Module", &ESBConfigure),
    newEngineAction("check", "Check the SEF Block Module's health, and repair if broken", &ESBCheck),
};

static struct CEHEngineConfig config = newEngineWithSetup(
    "ftl", "Perform actions toward SEF Reference FTL", ESBOptions, options, actions, &ESBSetup, &ESBCleanup);

// registering the engine
static void sefEngineInit sefSDKInit()
{
    CEHRegisterEngine(&config);
}
