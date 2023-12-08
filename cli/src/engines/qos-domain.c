/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * qos-domain.c
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
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <regex.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include "SEFAPI.h"
#include "sef-utils.h"

#include "../cli-engine.h"
#include "../engine-option.h"
#include "../io-helper.h"
#include "../sef-helper.h"
#include "../utils/str-builder.h"
#include "../utils/str-error.h"

#define BackupVersion 1.2
#define BackupGUID \
    0x43, 0x30, 0x12, 0x4a, 0xf, 0x4b, 0x20, 0x21, 0xb8, 0xce, 0x43, 0xef, 0x83, 0xe6, 0xd8, 0x2d

// prototypes
static int EQDPrintList(void **context, void *options);

static int EQDPrintInfo(void **context, void *options);

static int EQDDelete(void **context, void *options);

static int EQDCreate(void **context, void *options);

static int EQDFormat(void **context, void *options);

static int EQDResize(void **context, void *options);

static int EQDBackup(void **context, void *options);

static int EQDRestore(void **context, void *options);

static int EQDLabel(void **context, void *options);

// engine options
struct EQDOptions
{
    void *pad;
    uint16_t sefUnitIndex;
    bool sefUnitIndexGiven;
    uint16_t qosDomainId;
    bool qosDomainIdGiven;
    uint64_t *label;
    bool labelGiven;
    int labelLength;
    uint16_t virtualDeviceId;
    bool virtualDeviceIdGiven;
    uint64_t flashCapacity;
    bool flashCapacityGiven;
    uint8_t flashCapacityPercent;
    uint64_t pSLCFlashCapacity;
    uint64_t flashQuota;
    uint64_t pSLCFlashQuota;
    union CEOTuple ADUsize;
    int api;
    int recovery;
    char *encryption;
    bool encryptionGiven;
    uint16_t numPlacementIDs;
    uint16_t maxOpenSuperBlocks;
    int defectStrategy;
    uint8_t readQueue;
    uint16_t programWeight;
    bool programWeightGiven;
    uint16_t eraseWeight;
    bool eraseWeightGiven;
    char *filepath;
    bool filepathGiven;
    char *filePrefix;
    union CEOTuple *allowList;
    bool allowListGiven;
    int allowListLength;
    union CEOTuple *blockList;
    bool blockListGiven;
    int blockListLength;
    bool force;
    bool forceUnsafe;
    bool verbose;
    bool relabel;
};

static struct CEOEngineOption options[] = {
    CEOUInt('s', "sef-index", "The index of the target SEF Unit", k16, EQDOptions, sefUnitIndex, sefUnitIndexGiven),
    CEOUInt('q', "qos-domain-id", "The ID for the target QoS Domain", k16, EQDOptions, qosDomainId, qosDomainIdGiven),
    CEOUIntArray(
        'l', "label", "The label for the target QoS Domain", k64, EQDOptions, label, labelGiven, labelLength),
    CEOUInt('v',
            "virtual-device-id",
            "The ID for the target Virtual Device",
            k16,
            EQDOptions,
            virtualDeviceId,
            virtualDeviceIdGiven),
    CEOUInt('-', "flash-capacity", "Number of required/reserved ADUs", k64, EQDOptions, flashCapacity, flashCapacityGiven),
    CEOUIntDefault(
        '-',
        "flash-capacity-percent",
        "Percentage of the Virtual Device that should be used for the QoS Domain capacity",
        k8,
        "100",
        EQDOptions,
        flashCapacityPercent),
    CEOUIntDefault(
        '-', "pslc-capacity", "Number of required/reserved pSLC adus", k64, "0", EQDOptions, pSLCFlashCapacity),
    CEOUIntDefault(
        '-',
        "flash-quota",
        "Number of ADUs that can be allocated; When the flashQuota is less than the flashCapacity, "
        "it will be set to the flashCapacity.",
        k64,
        "1",
        EQDOptions,
        flashQuota),
    CEOUIntDefault('-',
                   "pslc-quota",
                   "Number of ADUs that can be allocated; When the pSLC flashQuota is less than "
                   "the pSLC flashCapacity, "
                   "it will be set to the pslc flashCapacity.",
                   k64,
                   "0",
                   EQDOptions,
                   pSLCFlashQuota),
    CEOTupleDefault(
        '-',
        "adu-size",
        "Size of Atomic Data Unit (ADU) in data and metadata in bytes. Must be one of the values "
        "supported by this SEF unit.",
        "4096:16",
        kSefOptionUnsignedInt,
        EQDOptions,
        ADUsize),
    CEOEnumDefault('-',
                   "api",
                   "Specifies the API Identifier for this QoS domain",
                   "kSuperBlock",
                   "kSuperBlock,kInDriveGC,kVirtualSSD",
                   EQDOptions,
                   api),
    CEOEnumDefault('-',
                   "recovery",
                   "Specifies the recovery mode for this QoS domain",
                   "kAutomatic",
                   "kAutomatic,kHostControlled",
                   EQDOptions,
                   recovery),
    CEOEnumDefault('-',
                   "defect-strategy",
                   "Defect management strategy for the QoS Domain",
                   "kPerfect",
                   "kPacked,kFragmented,kPerfect",
                   EQDOptions,
                   defectStrategy),
    CEOString(
        '-', "encryption-key", "Specifies the QoS Domain encryption key", EQDOptions, encryption, encryptionGiven),
    CEOUIntDefault('-',
                   "num-placement-id",
                   "The maximum number of Placement IDs that can be placed on the QoS domain.",
                   k16,
                   "1",
                   EQDOptions,
                   numPlacementIDs),
    CEOUIntDefault('-',
                   "max-open-super-blocks",
                   "The maximum number super blocks that can be open in a QoS domain.",
                   k16,
                   "0",
                   EQDOptions,
                   maxOpenSuperBlocks),
    CEOUIntDefault(
        '-', "read-queue", "Default scheduling queue for user read commands", k8, "0", EQDOptions, readQueue),
    CEOUInt('-',
            "program-weight",
            "Default scheduling queue for user read commands",
            k16,
            EQDOptions,
            programWeight,
            programWeightGiven),
    CEOUInt('-',
            "erase-weight",
            "Default scheduling queue for user nameless write commands",
            k16,
            EQDOptions,
            eraseWeight,
            eraseWeightGiven),
    CEOFlag('V', "verbose", "show additional information regarding the action being taken", EQDOptions, verbose),
    CEOString(
        '-', "path", "Path to be used for backup or restore of the QoS Domain", EQDOptions, filepath, filepathGiven),
    CEOStringDefault('-', "prefix", "", "backup", EQDOptions, filePrefix),
    CEOTupleArray('-',
                  "allow-list",
                  "A list of user addresses to keep, all other user addresses will be ignored",
                  kSefOptionUnsignedInt,
                  EQDOptions,
                  allowList,
                  allowListGiven,
                  allowListLength),
    CEOTupleArray('-',
                  "block-list",
                  "A list of user addresses to ignore, all other user addresses will be kept",
                  kSefOptionUnsignedInt,
                  EQDOptions,
                  blockList,
                  blockListGiven,
                  blockListLength),
    CEOFlag('-', "relabel", "Will retain the current label of the domain", EQDOptions, relabel),
    CEOFlag('f', "force", "Never prompt before removing or creating QoS Domains", EQDOptions, force),
    CEOFlag('F', "force-unsafe", "Force creation of unsafe QoS Domain configuration", EQDOptions, forceUnsafe)};

// engine setup/cleanup
struct engineData
{
    int numSefUnits;
    SEFHandle sefHandle;
    const struct SEFInfo *sefInfo;
};

static int EQDSetup(void **engineContext, void *options)
{
    struct engineData *data;
    struct SEFStatus status;
    struct EQDOptions *qdo = (struct EQDOptions *)options;

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

    if (qdo->sefUnitIndexGiven)
    {
        if (CIHIsOutOfRange("sef-index", 's', qdo->sefUnitIndex, 0, data->numSefUnits - 1))
        {
            return EINVAL;
        }

        // get the handle for the SEF Unit
        data->sefHandle = SEFGetHandle(qdo->sefUnitIndex);
        if (!data->sefHandle)
        {
            CIHMessageError("Error: Was unable to get SEF Unit %u handle", qdo->sefUnitIndex);
            return EINVAL;
        }

        // get sef unit information
        data->sefInfo = SEFGetInformation(data->sefHandle);
        if (!data->sefInfo)
        {
            CIHMessageError("Error: Was unable to get SEF Unit %d information", qdo->sefUnitIndex);
            return EIO;
        }
    }

    return 0;
}

static int EQDCleanup(void **engineContext, void *options)
{
    if (*engineContext != NULL)
    {
        free(*engineContext);
    }

    SEFLibraryCleanup();

    return 0;
}

static int isIdentifierInvalid(struct EQDOptions *qdo)
{
    if (!qdo->qosDomainIdGiven && !qdo->labelGiven)
    {
        CIHMessageError("Error: '--qos-domain-id' (`-q`) or '--label' (`-l`) option is required");
        return 1;
    }

    if (qdo->labelGiven && qdo->labelLength > 2)
    {
        CIHMessageError("Error: '--label' (`-l`) option exceeds the required size of 16 bytes");
        return 1;
    }

    return 0;
}

static struct SEFStatus populateQosDomainId(struct engineData *data, struct EQDOptions *qdo)
{
    // get qos domain id if label provided
    if (qdo->labelGiven && !qdo->qosDomainIdGiven)
    {
        struct SIFLabel label;
        struct SEFStatus status;

        label.data[0] = qdo->label[0];
        label.data[1] = qdo->labelLength == 2 ? qdo->label[1] : 0;

        status = CSHGetQosDomainId(data->sefHandle, label);
        if (status.error)
        {
            return status;
        }

        qdo->qosDomainId = status.info;
        qdo->qosDomainIdGiven = true;
    }

    return makeStatusOk();
}

static void printRecreateCommand(struct engineData *data,
                                 struct EQDOptions *qdo,
                                 struct SEFQoSDomainList *qosList,
                                 struct SEFQoSDomainInfo *qosInfoList)
{
    struct StrBuilder createCommand;
    char *createCommandOut;
    uint16_t i;

    StrBldInit(&createCommand, qosList->numQoSDomains * 75);

    for (i = 0; i < qosList->numQoSDomains; i++)
    {
        StrBldAppendFormat(&createCommand,
                           "sef-cli create qos-domain --sef-index %u --virtual-device %u "
                           "--flash-capacity %" PRIu64,
                           qdo->sefUnitIndex, qosInfoList[i].virtualDeviceID.id,
                           qosInfoList[i].flashCapacity);

        if (qosInfoList[i].rootPointers[data->sefInfo->maxRootPointers - 2].bits != 0 ||
            qosInfoList[i].rootPointers[data->sefInfo->maxRootPointers - 1].bits != 0)
        {
            StrBldAppendFormat(&createCommand, " --label %" PRIu64 ",%" PRIu64,
                               qosInfoList[i].rootPointers[data->sefInfo->maxRootPointers - 2].bits,
                               qosInfoList[i].rootPointers[data->sefInfo->maxRootPointers - 1].bits);
        }

        if (qosInfoList[i].ADUsize.data != qdo->ADUsize.tUint64.a &&
            qosInfoList[i].ADUsize.meta != qdo->ADUsize.tUint64.b)
        {
            StrBldAppendFormat(&createCommand, " --adu-size %u:%u", qosInfoList[i].ADUsize.data,
                               qosInfoList[i].ADUsize.meta);
        }

        if (qosInfoList[i].api != qdo->api)
        {
            char options[][12] = {"kSuperBlock", "kInDriveGC", "kVirtualSSD"};

            StrBldAppendFormat(&createCommand, " --api \"%s\"", options[qosInfoList[i].api]);
        }

        if (qosInfoList[i].recoveryMode != qdo->recovery)
        {
            char options[][16] = {"kAutomatic", "kHostControlled"};

            StrBldAppendFormat(&createCommand, " --recovery \"%s\"",
                               options[qosInfoList[i].recoveryMode]);
        }

        if (qosInfoList[i].defectStrategy != qdo->defectStrategy)
        {
            char options[][12] = {"kPacked", "kFragmented", "kPerfect"};

            StrBldAppendFormat(&createCommand, " --defect-strategy \"%s\"",
                               options[qosInfoList[i].defectStrategy]);
        }

        if (qosInfoList[i].numPlacementIDs != qdo->numPlacementIDs)
        {
            StrBldAppendFormat(&createCommand, " --num-placement-id %u", qosInfoList[i].numPlacementIDs);
        }

        if (qosInfoList[i].maxOpenSuperBlocks != qdo->maxOpenSuperBlocks)
        {
            StrBldAppendFormat(&createCommand, " --max-open-super-blocks %u",
                               qosInfoList[i].maxOpenSuperBlocks);
        }

        if (qosInfoList[i].defaultReadQueue != qdo->readQueue)
        {
            StrBldAppendFormat(&createCommand, " --read-queue %u", qosInfoList[i].defaultReadQueue);
        }

        if (qosInfoList[i].weights.programWeight != data->sefInfo->minWriteWeight)
        {
            StrBldAppendFormat(&createCommand, " --program-weight %u",
                               qosInfoList[i].weights.programWeight);
        }

        if (qosInfoList[i].weights.eraseWeight != data->sefInfo->minWriteWeight)
        {
            StrBldAppendFormat(&createCommand, " --erase-weight %u", qosInfoList[i].weights.eraseWeight);
        }

        StrBldAppend(&createCommand, "\n");
    }

    CIHMessageInfo("Recreate Command:");

    // To avoid the -Wformat-security warning and possible attack vectors
    createCommandOut = StrBldToString(&createCommand);
    CIHMessageInfo("%s", createCommandOut);

    StrBldCleanup(&createCommand);
    free(createCommandOut);
}

// engine functions
static int EQDPrintList(void **context, void *options)
{
    int i, returnVal;
    uint16_t numQoSDomains;
    struct SEFStatus status;
    struct SEFQoSDomainList *qosList;
    struct SEFQoSDomainInfo *qosInfoList;
    struct EQDOptions *qdo = (struct EQDOptions *)options;
    struct engineData *data = *context;

    if (CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven))
    {
        return EINVAL;
    }

    // get list of the the available qos domains
    numQoSDomains = 0;
    qosList = NULL;
    do
    {
        int qosListSize;

        if (qosList != NULL)
        {
            numQoSDomains = qosList->numQoSDomains;
            free(qosList);
        }

        qosListSize = struct_size(qosList, QoSDomainID, numQoSDomains);
        qosList = malloc(qosListSize);

        status = SEFListQoSDomains(data->sefHandle, qosList, qosListSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of QoS Domains for SEF Unit Index %u",
                            qdo->sefUnitIndex);
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFListQoSDomains", status.error));
            free(qosList);
            return status.error;
        }

    } while (numQoSDomains != qosList->numQoSDomains);

    if (qosList->numQoSDomains == 0)
    {
        CIHMessageData("No QoS Domains were found on SEF Unit Index %u", qdo->sefUnitIndex);
        free(qosList);
        return 0;
    }

    // print table header
    CIHMessageDataNoWrap("QoS Domain %-8s %-8s %-12s %-27s", "ID", "VD ID", "Capacity", "Label");

    qosInfoList = calloc(qosList->numQoSDomains, sizeof(struct SEFQoSDomainInfo));

    // print qos domain list
    returnVal = 0;
    for (i = 0; i < qosList->numQoSDomains; i++)
    {
        // get qos domain information
        status = SEFGetQoSDomainInformation(data->sefHandle, qosList->QoSDomainID[i], &qosInfoList[i]);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get QoS Domain %u information",
                            qosList->QoSDomainID[i].id);
            CIHMessageInfo("Detailed Error: %s",
                           SEFStrError("SEFGetQoSDomainInformation", status.error));

            returnVal = status.error;
            continue;
        }

        CIHMessageDataNoWrap("QoS Domain %-8d %-8d %-12" PRIu64 " [%-12" PRIu64 ",%-12" PRIu64 "]",
                             qosList->QoSDomainID[i].id, qosInfoList[i].virtualDeviceID.id,
                             qosInfoList[i].flashCapacity,
                             qosInfoList[i].rootPointers[data->sefInfo->maxRootPointers - 2].bits,
                             qosInfoList[i].rootPointers[data->sefInfo->maxRootPointers - 1].bits);
    }

    CIHMessageData("QoS Domain Count %u", qosList->numQoSDomains);

    if (qdo->verbose)
    {
        printRecreateCommand(data, qdo, qosList, qosInfoList);
    }

    // free objects
    free(qosList);

    return returnVal;
}

static bool isBAListValid(struct EQDOptions *qdo)
{
    union CEOTuple *list;
    int listLength, i;

    if (!qdo->blockListGiven && !qdo->allowListGiven)
    {
        return true;
    }

    if (qdo->blockListGiven && qdo->allowListGiven)
    {
        CIHMessageError("Error: You can either specify an allow list or block list, not both");
        return false;
    }

    // code simplification
    list = qdo->allowList;
    listLength = qdo->allowListLength;
    if (qdo->blockListGiven)
    {
        list = qdo->blockList;
        listLength = qdo->blockListLength;
    }

    // check for valid range
    for (i = 0; i < listLength; i++)
    {
        if (list[i].tUint64.a > list[i].tUint64.b)
        {
            CIHMessageError("Error: The specified range for allow list or block list is invalid");
            return false;
        }
    }

    return true;
}

static bool isUserAddressSkipped(struct EQDOptions *qdo, struct SEFUserAddress userAddress)
{
    union CEOTuple *list;
    int listLength, i;
    bool skip, isAllowList;

    if (!qdo->blockListGiven && !qdo->allowListGiven)
    {
        return false;
    }

    // code simplification
    isAllowList = true;
    list = qdo->allowList;
    listLength = qdo->allowListLength;
    if (qdo->blockListGiven)
    {
        list = qdo->blockList;
        listLength = qdo->blockListLength;
        isAllowList = false;
    }

    // check if the user address should be skipped
    skip = isAllowList;
    for (i = 0; i < listLength; i++)
    {
        if (userAddress.unformatted >= list[i].tUint64.a &&
            userAddress.unformatted <= list[i].tUint64.b)
        {
            skip = !isAllowList;
            break;
        }
    }

    return skip;
}

static int superBlockCompare(const void *elem1, const void *elem2)
{
    struct SEFSuperBlockInfo *record1 = (struct SEFSuperBlockInfo *)elem1;
    struct SEFSuperBlockInfo *record2 = (struct SEFSuperBlockInfo *)elem2;

    return (record1->eraseOrder - record2->eraseOrder);
}

static struct SEFStatus backupQoSInfo(struct SEFQoSDomainID qosDomainId,
                                      struct SEFQoSDomainInfo *qosInfo,
                                      uint32_t numSuperBlocks,
                                      struct EQDOptions *qdo)
{
    int filePathSize;
    char guid[16] = {BackupGUID};
    char *filePath;
    float versionNum = BackupVersion;
    time_t now;

    now = time(NULL);

    // create file path for backup
    filePathSize = 1 + snprintf(NULL, 0, "%s/%s-info.dat", qdo->filepath, qdo->filePrefix);
    filePath = malloc(filePathSize);

    snprintf(filePath, filePathSize, "%s/%s-info.dat", qdo->filepath, qdo->filePrefix);

    // open file in binary mode
    FILE *write_ptr = fopen(filePath, "wb");
    if (write_ptr == NULL)
    {
        CIHMessageError("Error: Was unable to write to the backup files");
        return makeStatusError(EIO);
    }

    // write data to the file
    fwrite(guid, sizeof(char), 16, write_ptr);
    fwrite(&versionNum, sizeof(float), 1, write_ptr);
    fwrite(&now, sizeof(time_t), 1, write_ptr);
    fwrite(&qosDomainId, sizeof(struct SEFQoSDomainID), 1, write_ptr);
    fwrite(&qosInfo->ADUsize, sizeof(struct SEFADUsize), 1, write_ptr);
    fwrite(&qosInfo->defectStrategy, sizeof(enum SEFDefectManagementMethod), 1, write_ptr);
    fwrite(&numSuperBlocks, sizeof(uint32_t), 1, write_ptr);

    // close file
    fclose(write_ptr);

    // clean memory
    free(filePath);

    return makeStatusOk();
}

static struct SEFStatus backupSuperBlock(SEFQoSHandle qosHandle,
                                         int sbIndex,
                                         struct SEFSuperBlockInfo *sbInfo,
                                         struct SEFADUsize aduSize,
                                         struct EQDOptions *qdo)
{
    int userAddressListSize, filePathSize;
    struct SEFUserAddressList *userAddressList;
    struct SEFFlashAddress flashAddress;
    struct SEFStatus status;
    char *filePath;
    void *fileData, *metaData;

    flashAddress = sbInfo->flashAddress;

    // create file path for backup
    filePathSize = 1 + snprintf(NULL, 0, "%s/%s-%d.dat", qdo->filepath, qdo->filePrefix, sbIndex);
    filePath = malloc(filePathSize);

    snprintf(filePath, filePathSize, "%s/%s-%d.dat", qdo->filepath, qdo->filePrefix, sbIndex);

    // open file in binary mode
    FILE *write_ptr = fopen(filePath, "wb");
    if (write_ptr == NULL)
    {
        CIHMessageError("Error: Was unable to write to the backup files");
        free(filePath);
        return makeStatusError(EIO);
    }

    // get superblock's user address
    userAddressListSize =
        sizeof(struct SEFUserAddressList) + sizeof(struct SEFUserAddress) * sbInfo->writableADUs;
    userAddressList = malloc(userAddressListSize);
    status =
        SEFGetUserAddressList(qosHandle, sbInfo->flashAddress, userAddressList, userAddressListSize);
    if (status.error)
    {
        CIHMessageError("Error: SEFGetUserAddressList failed");
        fclose(write_ptr);
        free(userAddressList);
        free(filePath);
        return status;
    }

    // write super block info
    fwrite(&sbInfo->placementID, sizeof(struct SEFPlacementID), 1, write_ptr);
    fwrite(&sbInfo->type, sizeof(enum SEFSuperBlockType), 1, write_ptr);
    fwrite(&sbInfo->writtenADUs, sizeof(uint32_t), 1, write_ptr);

    // allocate memory
    fileData = calloc(aduSize.data, 1);
    metaData = calloc(aduSize.meta, 1);

    // write ADU data
    for (uint64_t aduOffset = 0; aduOffset < sbInfo->writtenADUs; aduOffset++)
    {
        struct SEFUserAddress userAddress;
        bool skip;

        // init memory
        memset(fileData, 0x00, aduSize.data);
        memset(metaData, 0x00, aduSize.meta);

        // check if userAddress should be skipped
        userAddress = userAddressList->userAddressesRecovery[aduOffset];
        skip = isUserAddressSkipped(qdo, userAddress);

        // read the data.
        if (skip)
        {
            userAddress = SEFUserAddressIgnore;
        }
        else
        {
            struct iovec iovr[] = {{fileData, aduSize.data}};
            status = SEFReadWithPhysicalAddress(qosHandle, flashAddress, 1, iovr, 1, 0, userAddress,
                                                metaData, NULL);
            if (status.error)
            {
                CIHMessageError("Error: Read Failed on Flash Address %lx, User Address %lx",
                                flashAddress.bits, le64toh(userAddress.unformatted));
                CIHMessageInfo("Detailed Error: %s",
                               SEFStrError("SEFReadWithPhysicalAddress1", status.error));

                // close file
                fclose(write_ptr);

                // free memory
                free(userAddressList);
                free(metaData);
                free(fileData);
                free(filePath);

                return status;
            }
        }

        // write data to the file
        fwrite(&userAddress, sizeof(struct SEFUserAddress), 1, write_ptr);
        fwrite(metaData, aduSize.meta, 1, write_ptr);
        fwrite(fileData, aduSize.data, 1, write_ptr);

        flashAddress = SEFNextFlashAddress(qosHandle, flashAddress);
    }

    // close file
    fclose(write_ptr);

    // free memory
    free(userAddressList);
    free(metaData);
    free(fileData);
    free(filePath);

    return makeStatusOk();
}

static int EQDBackup(void **context, void *options)
{
    struct SEFStatus status;
    uint32_t numSuperBlocks, i, sbIndex;
    int isInvalid = 0, isDirectory;
    struct EQDOptions *qdo = (struct EQDOptions *)options;
    SEFQoSHandle qosHandle;
    struct SEFQoSDomainInfo qosInfo;
    struct SEFVirtualDeviceInfo vdInfo;
    struct SEFQoSDomainID qosDomainId;
    struct engineData *data = *context;
    struct SEFSuperBlockInfo *sbInfoList;
    struct SEFSuperBlockList *sbList;
    ssize_t listSize;

    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += CIHIsNotGiven("path", '-', qdo->filepathGiven);
    isInvalid += isIdentifierInvalid(qdo);
    isInvalid += !isBAListValid(qdo);
    if (!isInvalid && access(qdo->filepath, F_OK) == 0)
    {
        CIHMessageError(
            "Error: invalid argument, \"%s\", for option '--path', the backup directory exists",
            qdo->filepath);
        isInvalid++;
    }

    if (isInvalid)
    {
        return EINVAL;
    }

    // Create backup folder
    isDirectory = mkdir(qdo->filepath, 0777);
    if (isDirectory)
    {
        CIHMessageError("Error: Failed to create Backup Directory %s", qdo->filepath);
        return EINVAL;
    }

    // get qos domain id if label provided
    status = populateQosDomainId(data, qdo);
    if (status.error)
    {
        return status.error;
    }

    // check if QoS Domain Id is valid
    qosDomainId.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, qosDomainId))
    {
        CIHMessageError("Error: The QoS Domain ID is invalid");
        return EINVAL;
    }

    status = SEFGetQoSDomainInformation(data->sefHandle, qosDomainId, &qosInfo);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get the QoS Domain %u information", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetQoSDomainInformation", status.error));
        return EIO;
    }

    status = SEFGetVirtualDeviceInformation(data->sefHandle, qosInfo.virtualDeviceID, &vdInfo,
                                            sizeof(struct SEFVirtualDeviceInfo));
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get Virtual Device %u information",
                        qosInfo.virtualDeviceID.id);
        CIHMessageInfo("Detailed Error: %s",
                       SEFStrError("SEFGetVirtualDeviceInformation", status.error));
        return EIO;
    }

    status = SEFOpenQoSDomain(data->sefHandle, qosDomainId, NULL, NULL, NULL, &qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open QoS Domain %u", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenQoSDomain", status.error));
        return EIO;
    }

    // get list of super blocks in Virtual Device
    sbList = NULL;
    listSize = 0;
    status.info = 0;
    do
    {
        if (status.info > listSize)
        {
            free(sbList);
            listSize = status.info;
            sbList = malloc(status.info);
        }

        // get list
        status = SEFGetSuperBlockList(qosHandle, sbList, listSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of super blocks");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetSuperBlockList", status.error));

            free(sbList);
            SEFCloseQoSDomain(qosHandle);
            return status.error;
        }
    } while (status.info);

    // get info for all super blocks
    numSuperBlocks = sbList->numSuperBlocks;
    sbInfoList = malloc(numSuperBlocks * sizeof(struct SEFSuperBlockInfo));
    for (i = 0; i < numSuperBlocks; i++)
    {
        bool isNotClosed = false;
        struct SEFSuperBlockRecord superblockRecord = sbList->superBlockRecords[i];

        if (superblockRecord.state != kSuperBlockClosed)
        {
            CIHMessageError("Error: Cannot backup QoS Domains that have unclosed Super Blocks");
            isNotClosed = true;
        }

        status = SEFGetSuperBlockInfo(qosHandle, superblockRecord.flashAddress, 0, &sbInfoList[i]);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get SuperBlock information");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetSuperBlockInfo", status.error));
        }

        if (status.error || isNotClosed)
        {
            free(sbList);
            free(sbInfoList);
            SEFCloseQoSDomain(qosHandle);

            return isNotClosed ? EPROTONOSUPPORT : status.error;
        }
    }

    // sort super blocks
    qsort(sbInfoList, numSuperBlocks, sizeof(struct SEFSuperBlockInfo), superBlockCompare);

    // free memory
    free(sbList);

    // backup qos domain info
    status = backupQoSInfo(qosDomainId, &qosInfo, numSuperBlocks, qdo);
    if (status.error)
    {
        SEFCloseQoSDomain(qosHandle);
        free(sbInfoList);

        return status.error;
    }

    // backup SuperBlocks
    for (sbIndex = 0; sbIndex < numSuperBlocks; sbIndex++)
    {
        status = backupSuperBlock(qosHandle, sbIndex, &sbInfoList[sbIndex], qosInfo.ADUsize, qdo);
        if (status.error)
        {
            SEFCloseQoSDomain(qosHandle);
            free(sbInfoList);

            return status.error;
        }
    }

    // free memory
    free(sbInfoList);

    // Close QoS Handle
    status = SEFCloseQoSDomain(qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close QoS Domain");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseQoSDomain", status.error));

        return status.error;
    }

    return 0;
}

static struct SEFStatus restoreQoSInfo(struct SEFQoSDomainInfo *qosInfo, struct EQDOptions *qdo)
{
    struct SEFADUsize readADU;
    struct SEFQoSDomainID qId;
    FILE *readPtr;
    enum SEFDefectManagementMethod defectMan;
    time_t time;
    uint32_t numSuperBlocks, i;
    char guid[16], expectedGuid[16] = {BackupGUID};
    float versionNum;
    char *filePath;
    int filePathSize;

    // create file path for backup file
    filePathSize = 1 + snprintf(NULL, 0, "%s/%s-info.dat", qdo->filepath, qdo->filePrefix);
    filePath = malloc(filePathSize);

    snprintf(filePath, filePathSize, "%s/%s-info.dat", qdo->filepath, qdo->filePrefix);

    // open file in binary mode
    readPtr = fopen(filePath, "rb");
    if (readPtr == NULL)
    {
        CIHMessageError("Error: Was unable to write to the backup files");
        free(filePath);
        return makeStatusError(EIO);
    }

    // read data from the file
    fseek(readPtr, 0, SEEK_SET);
    fread(guid, sizeof(char), 16, readPtr);
    for (i = 0; i < 16; i++)
    {
        if (guid[i] != expectedGuid[i])
        {
            CIHMessageError("Error: Backup file didn't match the expected signature");
            fclose(readPtr);
            free(filePath);
            return makeStatusError(EIO);
        }
    }

    fread(&versionNum, sizeof(float), 1, readPtr);
    if (versionNum < BackupVersion)
    {
        if (guid[i] != expectedGuid[i])
        {
            CIHMessageError(
                "Error: The backed up files can't be restored using this version of CLI");
            fclose(readPtr);
            free(filePath);
            return makeStatusError(EIO);
        }
    }

    // read the stored QoS Domain info
    fread(&time, sizeof(time_t), 1, readPtr);
    fread(&qId, sizeof(struct SEFQoSDomainID), 1, readPtr);
    fread(&readADU, sizeof(struct SEFADUsize), 1, readPtr);
    fread(&defectMan, sizeof(enum SEFDefectManagementMethod), 1, readPtr);
    fread(&numSuperBlocks, sizeof(uint32_t), 1, readPtr);

    // close file
    fclose(readPtr);

    // cleanup memory
    free(filePath);

    if (readADU.data != qosInfo->ADUsize.data && readADU.meta != qosInfo->ADUsize.meta &&
        defectMan != qosInfo->defectStrategy)
    {
        CIHMessageError(
            "Error: Backup QoS Domain's settings didn't match the current QoS Domain Settings");
        return makeStatusError(ENOEXEC);
    }

    return makeStatusInfo(numSuperBlocks);
}

static struct SEFStatus restoreSuperBlock(SEFQoSHandle qosHandle,
                                          int sbIndex,
                                          struct SEFADUsize aduSize,
                                          struct EQDOptions *qdo)
{
    char *filePath;
    FILE *readPtr;
    int fileSize, expectedFileSize, filePathSize;
    struct SEFStatus status;
    struct SEFFlashAddress dstAddr = SEFAutoAllocate;
    struct SEFUserAddress userAddress;
    struct iovec iovw[1];
    void *metaData;
    uint32_t i, writtenADUs;
    struct SEFPlacementID placementId;
    enum SEFSuperBlockType superblockType;

    // create file path for backup file
    filePathSize = 1 + snprintf(NULL, 0, "%s/%s-%d.dat", qdo->filepath, qdo->filePrefix, sbIndex);
    filePath = malloc(filePathSize);

    snprintf(filePath, filePathSize, "%s/%s-%d.dat", qdo->filepath, qdo->filePrefix, sbIndex);

    // open file as binary
    readPtr = fopen(filePath, "rb");
    if (readPtr == NULL)
    {
        CIHMessageError("Error: Was unable to open %s", filePath);
        free(filePath);
        return makeStatusError(ENOENT);
    }

    // validate file size
    fseek(readPtr, 0, SEEK_END);
    fileSize = ftell(readPtr);
    expectedFileSize =
        sizeof(struct SEFPlacementID) + sizeof(uint32_t) + sizeof(enum SEFSuperBlockType);
    if (fileSize < expectedFileSize)
    {
        CIHMessageError("Error: Backup file is invalid %s", filePath);
        fclose(readPtr);
        free(filePath);
        return makeStatusError(EIO);
    }

    // read super block info
    fseek(readPtr, 0, SEEK_SET);
    fread(&placementId, sizeof(struct SEFPlacementID), 1, readPtr);
    fread(&superblockType, sizeof(enum SEFSuperBlockType), 1, readPtr);
    fread(&writtenADUs, sizeof(uint32_t), 1, readPtr);

    expectedFileSize += writtenADUs * (aduSize.meta + aduSize.data + sizeof(struct SEFUserAddress));
    if (fileSize != expectedFileSize)
    {
        CIHMessageError("Error: Backup file is invalid %s", filePath);
        fclose(readPtr);
        free(filePath);
        return makeStatusError(EIO);
    }

    // allocate super block if not auto allocate
    if (placementId.id == SEFPlacementIdUnused || superblockType != kForWrite)
    {
        status = SEFAllocateSuperBlock(qosHandle, &dstAddr, superblockType, NULL, NULL);
        if (status.error)
        {
            CIHMessageError("Error: Unable to allocate superblock");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFAllocateSuperBlock", status.error));
            fclose(readPtr);
            free(filePath);
            return status;
        }
    }

    // allocate memory
    metaData = calloc(aduSize.meta, 1);
    iovw[0].iov_len = aduSize.data;
    iovw[0].iov_base = calloc(aduSize.data, 1);

    // read and restore file
    for (i = 0; i < writtenADUs; i++)
    {
        bool skip;
        struct SEFFlashAddress dataAddress;

        fread(&userAddress, sizeof(struct SEFUserAddress), 1, readPtr);
        fread(metaData, aduSize.meta, 1, readPtr);
        fread(iovw[0].iov_base, aduSize.data, 1, readPtr);

        skip = isUserAddressSkipped(qdo, userAddress);
        if (skip)
        {
            userAddress = SEFUserAddressIgnore;
            memset(metaData, 0x00, aduSize.meta);
            memset(iovw[0].iov_base, 0x0, aduSize.data);
        }

        status = CSHWriteSync(qosHandle, dstAddr, placementId, userAddress, 1, iovw, 1, metaData,
                              &dataAddress, NULL, NULL);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to write/restore the backed up data");

            fclose(readPtr);
            free(filePath);
            free(metaData);
            free(iovw[0].iov_base);

            return status;
        }
    }

    // close file
    fclose(readPtr);

    // free memory
    free(filePath);
    free(metaData);
    free(iovw[0].iov_base);

    // close allocated super block
    if (!SEFIsEqualFlashAddress(dstAddr, SEFAutoAllocate))
    {
        status = SEFCloseSuperBlock(qosHandle, dstAddr);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to close superblock");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseSuperBlock", status.error));
            return status;
        }
    }

    return makeStatusOk();
}

static int EQDRestore(void **context, void *options)
{
    struct SEFStatus status;
    struct EQDOptions *qdo = (struct EQDOptions *)options;
    SEFQoSHandle qosHandle;
    struct SEFQoSDomainInfo qinfo;
    struct SEFQoSDomainID qosDomainId;
    int isInvalid;
    uint32_t numSuperBlocks, i;
    struct engineData *data = *context;

    // validating input parameters
    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += isIdentifierInvalid(qdo);
    isInvalid += CIHIsNotGiven("path", '-', qdo->filepathGiven);
    isInvalid += !isBAListValid(qdo);
    if (!isInvalid && access(qdo->filepath, R_OK) != 0)
    {
        CIHMessageError(
            "Error: invalid argument, \"%s\", for option '--path', the directory cannot be "
            "accessed",
            qdo->filepath);
        isInvalid++;
    }

    if (isInvalid)
    {
        return EINVAL;
    }

    // get qos domain id if label provided
    status = populateQosDomainId(data, qdo);
    if (status.error)
    {
        return status.error;
    }

    // check if QoS Domain Id is valid
    qosDomainId.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, qosDomainId))
    {
        CIHMessageError("Error: The QoS Domain ID is invalid");
        return EINVAL;
    }

    status = SEFOpenQoSDomain(data->sefHandle, qosDomainId, NULL, NULL, NULL, &qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open QoS Domain %u", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenQoSDomain", status.error));
        return EIO;
    }

    status = SEFGetQoSDomainInformation(data->sefHandle, qosDomainId, &qinfo);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open QoS Domain %u information", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetQoSDomainInformation", status.error));

        SEFCloseQoSDomain(qosHandle);
        return EIO;
    }

    status = restoreQoSInfo(&qinfo, qdo);
    if (status.error)
    {
        SEFCloseQoSDomain(qosHandle);
        return status.error;
    }

    numSuperBlocks = status.info;
    for (i = 0; i < numSuperBlocks; i++)
    {
        status = restoreSuperBlock(qosHandle, i, qinfo.ADUsize, qdo);
        if (status.error)
        {
            SEFCloseQoSDomain(qosHandle);

            return status.error;
        }
    }

    // Close QoS Handle
    status = SEFCloseQoSDomain(qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close QoS Domain");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseQoSDomain", status.error));
        return EIO;
    }

    return 0;
}

static int EQDPrintInfo(void **context, void *options)
{
    int i, isInvalid;
    struct SEFStatus status;
    struct SEFQoSDomainID QoSDomainId;
    struct SEFQoSDomainInfo qosInfo;
    struct engineData *data = *context;
    struct EQDOptions *qdo = (struct EQDOptions *)options;

    // validating input parameters
    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += isIdentifierInvalid(qdo);

    if (isInvalid)
    {
        return EINVAL;
    }

    // get qos domain id if label provided
    status = populateQosDomainId(data, qdo);
    if (status.error)
    {
        return status.error;
    }

    // check if qos domain id is valid
    QoSDomainId.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, QoSDomainId))
    {
        CIHMessageError("Error: The QoS Domain ID %u is invalid", QoSDomainId.id);
        return EINVAL;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(data->sefHandle, QoSDomainId, &qosInfo);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get QoS Domain %u information", QoSDomainId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetQoSDomainInformation", status.error));
        return status.error;
    }

    // print qos domain information
    CIHMessageData("qosDomainId: %u", QoSDomainId.id);
    CIHMessageData("qosDomainLabel: [%" PRIu64 ", %" PRIu64 "]",
                   qosInfo.rootPointers[data->sefInfo->maxRootPointers - 2].bits,
                   qosInfo.rootPointers[data->sefInfo->maxRootPointers - 1].bits);
    CIHMessageData("virtualDeviceID: %u", qosInfo.virtualDeviceID.id);
    CIHMessageData("numPlacementIDs: %u", qosInfo.numPlacementIDs);
    CIHMessageData("maxOpenSuperBlocks: %u", qosInfo.maxOpenSuperBlocks);
    CIHMessageData("encryption: %s", qosInfo.encryption == 0 ? "Enabled" : "Not Enabled");

    switch (qosInfo.recoveryMode)
    {
        case kAutomatic:
            CIHMessageData("recoveryMode: Automatic");
            break;

        case kHostControlled:
            CIHMessageData("recoveryMode: Host Controlled");
            break;
    }

    switch (qosInfo.defectStrategy)
    {
        case kPacked:
            CIHMessageData("deadline: Packed");
            break;

        case kFragmented:
            CIHMessageData("deadline: Fragmented");
            break;

        case kPerfect:
            CIHMessageData("deadline: Perfect");
            break;
    }

    switch (qosInfo.api)
    {
        case kSuperBlock:
            CIHMessageData("api: SuperBlock");
            break;

        case kInDriveGC:
            CIHMessageData("api: In Drive GC");
            break;

        case kVirtualSSD:
            CIHMessageData("api: Virtual SSD");
            break;
    }

    CIHMessageData("flashCapacity: %" PRIu64 "", qosInfo.flashCapacity);
    CIHMessageData("flashQuota: %" PRIu64 "", qosInfo.flashQuota);
    CIHMessageData("flashUsage: %" PRIu64 "", qosInfo.flashUsage);
    CIHMessageData("pSLCFlashCapacity: %" PRIu64 "", qosInfo.pSLCFlashCapacity);
    CIHMessageData("pSLCFlashQuota: %" PRIu64 "", qosInfo.pSLCFlashQuota);
    CIHMessageData("pSLCFlashUsage: %" PRIu64 "", qosInfo.pSLCFlashUsage);
    CIHMessageData("ADUSize:");
    CIHMessageData("* Data: %u", qosInfo.ADUsize.data);
    CIHMessageData("* Meta: %u", qosInfo.ADUsize.meta);
    CIHMessageData("superBlockCapacity: %u", qosInfo.superBlockCapacity);
    CIHMessageData("pSLCSuperBlockCapacity: %u", qosInfo.pSLCSuperBlockCapacity);
    CIHMessageData("maxOpenSuperBlocks: %u", qosInfo.maxOpenSuperBlocks);
    CIHMessageData("defectMapSize: %u", qosInfo.defectMapSize);

    switch (qosInfo.deadline)
    {
        case kFastest:
            CIHMessageData("deadline: Fastest");
            break;

        case kTypical:
            CIHMessageData("deadline: Typical");
            break;

        case kLong:
            CIHMessageData("deadline: Long");
            break;

        case kHeroic:
            CIHMessageData("deadline: Heroic");
            break;
    }

    CIHMessageData("defaultReadQueue: %u", qosInfo.defaultReadQueue);
    CIHMessageData("Weights:");
    CIHMessageData("* erase: %u", qosInfo.weights.eraseWeight);
    CIHMessageData("* program: %u", qosInfo.weights.programWeight);

    for (i = 0; i < SEFMaxRootPointer; i++)
    {
        CIHMessageData("* rootPointer (%d): 0x%" PRIx64, i, qosInfo.rootPointers[i].bits);
    }

    return 0;
}

static int EQDDelete(void **context, void *options)
{
    int isInvalid;
    struct SEFQoSDomainID QoSDomainId;
    struct SEFStatus status;
    struct engineData *data = *context;
    struct EQDOptions *qdo = (struct EQDOptions *)options;

    // validating input parameters
    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += isIdentifierInvalid(qdo);

    if (isInvalid)
    {
        return EINVAL;
    }

    // get qos domain id if label provided
    status = populateQosDomainId(data, qdo);
    if (status.error)
    {
        return status.error;
    }

    // check if qos domain id is valid
    QoSDomainId.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, QoSDomainId))
    {
        CIHMessageInfo("Warning: The QoS Domain ID %u does not exist", qdo->qosDomainId);
        return 0;
    }

    // check for force action
    if (!qdo->force &&
        !CIHGetBoolInput("Are you sure you want to delete QoS Domain %u?", qdo->qosDomainId))
    {
        return 0;
    }

    // delete the qos domain
    status = SEFDeleteQoSDomain(data->sefHandle, QoSDomainId);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to delete QoS Domain %u", QoSDomainId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFDeleteQoSDomain", status.error));

        return status.error;
    }

    CIHMessageInfo("The QoS Domain was successfully deleted");

    return 0;
}

static struct SEFQoSDomainCapacity toSEFQoSDomainCapacity(uint64_t flashCapacity, uint64_t flashQuota)
{
    struct SEFQoSDomainCapacity cap = {flashCapacity, flashQuota};

    return cap;
}

static int EQDCreate(void **context, void *options)
{
    SEFVDHandle vdHandle;
    struct SEFStatus status;
    struct SEFQoSDomainID qosDomainId;
    struct SEFVirtualDeviceID virtualDeviceId;
    struct SEFVirtualDeviceInfo vdInfo;
    int isInvalid, aduIndex, i;
    struct SEFWeights weight;
    struct engineData *data = *context;
    struct EQDOptions *qdo = (struct EQDOptions *)options;
    uint64_t domainCapacity;
    struct SEFQoSDomainCapacity cap = {0};
    struct SEFQoSDomainCapacity pSLCcap = {0};

    // validating input parameters
    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += CIHIsNotGiven("virtual-device-id", 'v', qdo->virtualDeviceIdGiven);

    if (isInvalid)
    {
        return EINVAL;
    }

    // validate virtual device id
    virtualDeviceId.id = qdo->virtualDeviceId;
    if (!CSHIsVirtualDeviceIdValid(data->sefHandle, virtualDeviceId))
    {
        CIHMessageError(
            "Error: invalid argument, \"%u\", for option '--virtual-device-id' (`-v`), Virtual "
            "Device doesn't exist",
            qdo->virtualDeviceId);

        return EINVAL;
    }

    // get virtual device information
    status = SEFGetVirtualDeviceInformation(data->sefHandle, virtualDeviceId, &vdInfo,
                                            sizeof(struct SEFVirtualDeviceInfo));
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get Virtual Device %u information", qdo->virtualDeviceId);
        CIHMessageInfo("Detailed Error: %s",
                       SEFStrError("SEFGetVirtualDeviceInformation", status.error));
        return status.error;
    }

    // validate the input parameters
    isInvalid += CIHIsOutOfRange("api", '-', qdo->api, kSuperBlock, kVirtualSSD);
    isInvalid += CIHIsOutOfRange("recovery", '-', qdo->recovery, kAutomatic, kHostControlled);
    isInvalid += CIHIsOutOfRange("defect-strategy", '-', qdo->defectStrategy, kPacked, kPerfect);
    isInvalid += CIHIsOutOfRange("flash-capacity-percent", '-', qdo->flashCapacityPercent, 1, 100);
    isInvalid +=
        CIHIsOutOfRange("pslc-capacity", '-', qdo->pSLCFlashCapacity, 0, vdInfo.pSLCFlashAvailable);
    isInvalid += CIHIsOutOfRange("flash-quota", '-', qdo->flashQuota, 0, vdInfo.flashAvailable);
    isInvalid += CIHIsOutOfRange("pslc-quota", '-', qdo->pSLCFlashQuota, 0, vdInfo.pSLCFlashAvailable);
    isInvalid += CIHIsOutOfRange("num-placement-id", '-', qdo->numPlacementIDs, 0,
                                 data->sefInfo->maxPlacementIDs);
    isInvalid += CIHIsOutOfRange("read-queue", '-', qdo->readQueue, 0, vdInfo.numReadQueues);
    if (qdo->flashCapacityGiven)
    {
        isInvalid +=
            CIHIsOutOfRange("flash-capacity", '-', qdo->flashCapacity, 0, vdInfo.flashAvailable);
    }
    if (qdo->programWeightGiven)
    {
        isInvalid += CIHIsOutOfRange("program-weight", '-', qdo->programWeight, 0, UINT16_MAX);
    }
    if (qdo->eraseWeightGiven)
    {
        isInvalid += CIHIsOutOfRange("erase-weight", '-', qdo->eraseWeight, 0, UINT16_MAX);
    }
    if (qdo->labelGiven && qdo->labelLength > 2)
    {
        CIHMessageError("Error: '--label' (`-l`) option exceeds the required size of 16 bytes");
        isInvalid++;
    }
    if (!qdo->forceUnsafe && (vdInfo.QoSDomains.numQoSDomains > vdInfo.numPSLCSuperBLocks ||
                              vdInfo.QoSDomains.numQoSDomains > vdInfo.superBlockDies))
    {
        CIHMessageError(
            "Error: Number of QoS Domains cannot exceed number of blocks of either flash type");
        CIHMessageError("Number of domains: %u", vdInfo.QoSDomains.numQoSDomains);
        CIHMessageError("Number of super blocks: %u", vdInfo.superBlockDies);
        CIHMessageError("Number of pSLC super blocks: %u", vdInfo.numPSLCSuperBLocks);
        isInvalid++;
    }

    // checking for device support
    if (qdo->encryption && !(data->sefInfo->supportedOptions & kEncryptionSupported))
    {
        CIHMessageError(
            "Error: invalid flag for option '--encryption', Encryption is not supported by this "
            "device");
        isInvalid += 1;
    }

    if (qdo->pSLCFlashCapacity > 0 && !(data->sefInfo->supportedOptions & kPSLCSupported))
    {
        CIHMessageError(
            "Error: invalid flag for option '--pslc-capacity', pSLC is not supported by this "
            "device");
        isInvalid += 1;
    }

    switch (qdo->api)
    {
        case kSuperBlock:
            if (!(data->sefInfo->supportedOptions & kSuperBlockSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--api', The Superblock API is "
                    "not supported by this "
                    "device",
                    qdo->api);
                isInvalid += 1;
            }
            break;

        case kInDriveGC:
            if (!(data->sefInfo->supportedOptions & kInDriveGCSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--api', The In Drive GC API is "
                    "not supported by this "
                    "device",
                    qdo->api);
                isInvalid += 1;
            }
            break;

        case kVirtualSSD:
            if (!(data->sefInfo->supportedOptions & kVirtualSSDSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--api', The Virtual SSD API is "
                    "not supported by this "
                    "device",
                    qdo->api);
                isInvalid += 1;
            }
            break;
    }

    switch (qdo->recovery)
    {
        case kAutomatic:
            if (!(data->sefInfo->supportedOptions & kAutomaticSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--recovery', Automatic Recovery "
                    "is not supported by "
                    "this device",
                    qdo->recovery);
                isInvalid += 1;
            }
            break;

        case kHostControlled:
            if (!(data->sefInfo->supportedOptions & kHostControlledSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--recovery', Host Controlled "
                    "Recovery is not "
                    "supported by this device",
                    qdo->recovery);
                isInvalid += 1;
            }
            break;
    }

    switch (qdo->defectStrategy)
    {
        case kPacked:
            if (!(data->sefInfo->supportedOptions & kPackedSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--defect-strategy', The Packed "
                    "defect strategy is "
                    "not supported by this device",
                    qdo->defectStrategy);
                isInvalid += 1;
            }
            break;

        case kFragmented:
            if (!(data->sefInfo->supportedOptions & kFragmentedSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--defect-strategy', The "
                    "Fragmented defect strategy "
                    "is not supported by this device",
                    qdo->defectStrategy);
                isInvalid += 1;
            }
            break;

        case kPerfect:
            if (!(data->sefInfo->supportedOptions & kPerfectSupported))
            {
                CIHMessageError(
                    "Error: invalid argument, \"%d\", for option '--defect-strategy', The Perfect "
                    "defect strategy is "
                    "not supported by this device",
                    qdo->defectStrategy);
                isInvalid += 1;
            }
            break;
    }

    aduIndex = -1;
    for (i = 0; i < data->sefInfo->numADUSizes; i++)
    {
        if (qdo->ADUsize.tUint64.a == data->sefInfo->ADUsize[i].data &&
            qdo->ADUsize.tUint64.b == data->sefInfo->ADUsize[i].meta)
        {
            aduIndex = i;
        }
    }

    if (aduIndex == -1)
    {
        CIHMessageError("Error: invalid argument, \"%" PRIu64 ":%" PRIu64
                        "\", for option '--adu-size', ADU size is not supported by this device",
                        qdo->ADUsize.tUint64.a, qdo->ADUsize.tUint64.b);
        isInvalid++;
    }

    if (isInvalid)
    {
        return EINVAL;
    }

    // show weight warning
    if (qdo->eraseWeight < data->sefInfo->minWriteWeight)
    {
        CIHMessageData(
            "Warning: The value of the erase weight %u is below the recommended number of %u",
            qdo->eraseWeight, data->sefInfo->minWriteWeight);
    }

    if (qdo->programWeight < data->sefInfo->minWriteWeight)
    {
        CIHMessageData(
            "Warning: The value of the program weight %u is below the recommended number of %u",
            qdo->programWeight, data->sefInfo->minWriteWeight);
    }

    // check for force action
    if (!qdo->force && !CIHGetBoolInput("Are you sure you want to create a QoS Domain?"))
    {
        return 0;
    }

    // get Virtual Device Handle
    status = SEFOpenVirtualDevice(data->sefHandle, virtualDeviceId, NULL, NULL, &vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open Virtual Device %u", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenVirtualDevice", status.error));
        return status.error;
    }

    // prep variables
    weight.programWeight = qdo->programWeightGiven ? qdo->programWeight : data->sefInfo->minWriteWeight;
    weight.eraseWeight = qdo->eraseWeightGiven ? qdo->eraseWeight : data->sefInfo->minWriteWeight;

    if (!qdo->encryptionGiven)
    {
        qdo->encryption = NULL;
    }

    // calculate size of the QoS Domain unless given
    domainCapacity = DIV_ROUND_UP((vdInfo.flashAvailable * qdo->flashCapacityPercent), 100);
    if (qdo->flashCapacityGiven)
    {
        domainCapacity = qdo->flashCapacity;
    }

    // create QoS Domain
    cap = toSEFQoSDomainCapacity(domainCapacity, qdo->flashQuota);
    pSLCcap = toSEFQoSDomainCapacity(qdo->pSLCFlashCapacity, qdo->pSLCFlashQuota);
    status = SEFCreateQoSDomain(vdHandle, &qosDomainId, &cap, &pSLCcap, aduIndex, qdo->api,
                                qdo->defectStrategy, qdo->recovery, qdo->encryption,
                                qdo->numPlacementIDs, qdo->maxOpenSuperBlocks, qdo->readQueue, weight);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to create QoS Domain (%d)", status.error);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCreateQoSDomain", status.error));
        SEFCloseVirtualDevice(vdHandle);
        return status.error;
    }

    CIHMessageInfo("The QoS Domain %u was successfully created", qosDomainId.id);

    if (qdo->labelGiven)
    {
        struct SIFLabel label;
        SEFQoSHandle qosHandle;

        label.data[0] = qdo->label[0];
        label.data[1] = qdo->labelLength == 2 ? qdo->label[1] : 0;

        status =
            SEFOpenQoSDomain(data->sefHandle, qosDomainId, NULL, NULL, qdo->encryption, &qosHandle);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to Open QoS Domain to set the Label");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenQoSDomain", status.error));

            return status.error;
        }

        status = SUSetLabel(qosHandle, label, true);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to set the QoS Domain Label");

            status = SEFCloseQoSDomain(qosHandle);
            status = SEFCloseVirtualDevice(vdHandle);

            return status.error;
        }

        status = SEFCloseQoSDomain(qosHandle);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to close QoS Domain");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseQoSDomain", status.error));

            return status.error;
        }
    }

    // close virtual device
    status = SEFCloseVirtualDevice(vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close Virtual Device");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseVirtualDevice", status.error));

        return status.error;
    }

    return 0;
}

static int EQDFormat(void **context, void *options)
{
    struct SEFStatus status;
    struct SEFQoSDomainInfo qosInfo;
    struct SEFSuperBlockList *sbList;
    struct SEFQoSDomainID qosDomainID;
    SEFQoSHandle qosHandle;
    int i, isInvalid;
    struct engineData *data = *context;
    struct EQDOptions *qdo = (struct EQDOptions *)options;
    ssize_t listSize;

    // validating input parameters
    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += isIdentifierInvalid(qdo);

    if (isInvalid)
    {
        return EINVAL;
    }

    // get qos domain id if label provided
    status = populateQosDomainId(data, qdo);
    if (status.error)
    {
        return status.error;
    }

    // check if qos domain id is valid
    qosDomainID.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, qosDomainID))
    {
        CIHMessageInfo("Warning: The QoS Domain ID %u does not exist", qdo->qosDomainId);
        return 0;
    }

    // check for force action
    if (!qdo->force &&
        !CIHGetBoolInput("Are you sure you want to format QoS Domain %u?", qdo->qosDomainId))
    {
        return 0;
    }

    // get QoS Domain information
    status = SEFGetQoSDomainInformation(data->sefHandle, qosDomainID, &qosInfo);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get QoS Domain %u information", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetQoSDomainInformation", status.error));
        return status.error;
    }

    // get qos domain handle
    status = SEFOpenQoSDomain(data->sefHandle, qosDomainID, NULL, NULL, NULL, &qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open QoS Domain %u", qosDomainID.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenQoSDomain", status.error));
        return status.error;
    }

    // clear all the root pointer
    for (i = 0; i < SEFMaxRootPointer; i++)
    {
        if (!SEFIsNullFlashAddress(qosInfo.rootPointers[i]))
        {
            status = SEFSetRootPointer(qosHandle, i, SEFNullFlashAddress);
            if (status.error)
            {
                CIHMessageError("Error: Was unable to clear root pointers for QoS Domain %u",
                                qdo->qosDomainId);
                CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFSetRootPointer", status.error));

                SEFCloseQoSDomain(qosHandle);
                return status.error;
            }
        }
    }

    // get superblock records
    sbList = NULL;
    listSize = 0;
    status.info = 0;
    do
    {
        if (status.info > listSize)
        {
            free(sbList);
            listSize = status.info;
            sbList = malloc(status.info);
        }

        // get list
        status = SEFGetSuperBlockList(qosHandle, sbList, listSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of super blocks");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetSuperBlockList", status.error));

            free(sbList);
            SEFCloseQoSDomain(qosHandle);

            return status.error;
        }
    } while (status.info);

    // free super blocks
    for (i = 0; i < sbList->numSuperBlocks; i++)
    {
        if (sbList->superBlockRecords[i].state != kSuperBlockClosed)
        {
            CIHMessageError(
                "Error: QoS Domain %u has open Super Blocks; was unable to format the QoS Domain",
                qdo->qosDomainId);

            free(sbList);
            SEFCloseQoSDomain(qosHandle);

            return -EBUSY;
        }

        status = SEFReleaseSuperBlock(qosHandle, sbList->superBlockRecords[i].flashAddress);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to format data from QoS Domain %u", qdo->qosDomainId);

            free(sbList);
            SEFCloseQoSDomain(qosHandle);

            return status.error;
        }
    }

    // free list
    free(sbList);

    // relabel qos domain if set
    if (qdo->relabel)
    {
        struct SIFLabel label;

        // label the qos domain
        label.data[0] = qdo->label[0];
        label.data[1] = qdo->labelLength == 2 ? qdo->label[1] : 0;

        status = SUSetLabel(qosHandle, label, true);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to label QoS Domain %u", qdo->qosDomainId);
            return status.error;
        }
    }

    // close the qos domain
    status = SEFCloseQoSDomain(qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close QoS Domain");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseQoSDomain", status.error));
        return status.error;
    }

    CIHMessageInfo("QoS Domain %u was successfully formatted", qdo->qosDomainId);

    return 0;
}

static int EQDResize(void **context, void *options)
{
    int isInvalid;
    struct SEFStatus status;
    struct SEFQoSDomainInfo qosInfo;
    struct SEFQoSDomainID qosDomainId;
    struct SEFVirtualDeviceID virtualDeviceId;
    struct SEFQoSDomainCapacity cap;
    enum SEFSuperBlockType sbType;
    SEFVDHandle vdHandle;
    struct engineData *data = *context;
    struct EQDOptions *qdo = (struct EQDOptions *)options;

    // validating input parameters
    isInvalid = 0;
    isInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInvalid += isIdentifierInvalid(qdo);

    if (!qdo->flashCapacityGiven && qdo->pSLCFlashCapacity == 0)
    {
        CIHMessageError("Error: '--%s' or '--%s' option required", "flash-capacity",
                        "pslc-capacity");
        isInvalid++;
    }

    if (isInvalid)
    {
        return EINVAL;
    }

    // get qos domain id if label provided
    status = populateQosDomainId(data, qdo);
    if (status.error)
    {
        return status.error;
    }

    // verify qos domain id is valid
    qosDomainId.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, qosDomainId))
    {
        CIHMessageError("Error: The QoS Domain ID %u is invalid", qdo->qosDomainId);
        return EINVAL;
    }

    // check for force action or user confirmation
    if (!qdo->force && !CIHGetBoolInput("Are you sure you want to resize this QoS Domain?"))
    {
        return 0;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(data->sefHandle, qosDomainId, &qosInfo);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get QoS Domain %u information", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFGetQoSDomainInformation", status.error));

        return status.error;
    }

    virtualDeviceId.id = qosInfo.virtualDeviceID.id;

    // get Virtual Device Handle
    status = SEFOpenVirtualDevice(data->sefHandle, virtualDeviceId, NULL, NULL, &vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open Virtual Device %u", virtualDeviceId.id);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenVirtualDevice", status.error));
        return status.error;
    }

    // get the super block type
    sbType = kForWrite;
    cap.flashCapacity = qdo->flashCapacity;
    cap.flashQuota = qdo->flashQuota;

    if (qdo->pSLCFlashCapacity != 0)
    {
        sbType = kForPSLCWrite;
        cap.flashCapacity = qdo->pSLCFlashCapacity;
        cap.flashQuota = qdo->pSLCFlashQuota;
    }

    // resize QoS Domain to new size
    status = SEFSetQoSDomainCapacity(vdHandle, qosDomainId, sbType, &cap);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to resize QoS Domain %u", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFSetQoSDomainCapacity", status.error));
        SEFCloseVirtualDevice(vdHandle);
        return status.error;
    }

    // close virtual device
    status = SEFCloseVirtualDevice(vdHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close Virtual Device");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseVirtualDevice", status.error));
        return status.error;
    }

    CIHMessageInfo("QoS Domain %u was successfully resized", qdo->qosDomainId);

    return 0;
}

static int EQDLabel(void **context, void *options)
{
    int isInputInvalid;
    struct SEFStatus status;
    struct SEFQoSDomainID qosDomainId;
    struct SIFLabel label;
    SEFQoSHandle qosHandle;
    struct engineData *data = *context;
    struct EQDOptions *qdo = (struct EQDOptions *)options;

    // validating input parameters
    isInputInvalid = 0;
    isInputInvalid += CIHIsNotGiven("sef-index", 's', qdo->sefUnitIndexGiven);
    isInputInvalid += CIHIsNotGiven("qos-domain-id", 'q', qdo->qosDomainIdGiven);
    isInputInvalid += CIHIsNotGiven("label", 'l', qdo->labelGiven);

    if (qdo->labelGiven && qdo->labelLength > 2)
    {
        CIHMessageError("Error: '--label' (`-l`) option exceeds the required size of 16 bytes");
        isInputInvalid++;
    }

    if (isInputInvalid)
    {
        return EINVAL;
    }

    // verify qos domain id is valid
    qosDomainId.id = qdo->qosDomainId;
    if (!CSHIsQosDomainIdValid(data->sefHandle, qosDomainId))
    {
        CIHMessageError("Error: The QoS Domain ID %u is invalid", qdo->qosDomainId);
        return EINVAL;
    }

    // get the current label
    status = SUGetLabel(data->sefHandle, qosDomainId, &label);
    if (status.error)
    {
        CIHMessageError("Error: was unable to get the QoS Domain label");
        return status.error;
    }

    if (label.data[0] != 0 && label.data[1] != 0 && !qdo->force &&
        !CIHGetBoolInput("Are you sure you want to relabel this QoS Domain?"))
    {
        return 0;
    }

    // open the qos domain
    status = SEFOpenQoSDomain(data->sefHandle, qosDomainId, NULL, NULL, NULL, &qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to open QoS Domain %u", qdo->qosDomainId);
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFOpenQoSDomain", status.error));
        return EIO;
    }

    // label the qos domain
    label.data[0] = qdo->label[0];
    label.data[1] = qdo->labelLength == 2 ? qdo->label[1] : 0;

    status = SUSetLabel(qosHandle, label, true);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to label QoS Domain %u", qdo->qosDomainId);
        return status.error;
    }

    // close the qos doamin
    status = SEFCloseQoSDomain(qosHandle);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to close QoS Domain");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseQoSDomain", status.error));

        return status.error;
    }

    CIHMessageInfo("QoS Domain %u was successfully labeled", qdo->qosDomainId);

    return 0;
}

// engine declarations
static struct CEHEngineAction actions[] = {
    newEngineAction("list",
                    "Print list of available QoS Domains. Commands to recreate same configuration "
                    "is printed in verbose",
                    &EQDPrintList),
    newEngineAction("info", "Prints QoS Domain's information", &EQDPrintInfo),
    newEngineAction("create", "create a new QoS Domain", &EQDCreate),
    newEngineAction(
        "delete", "Delete QoS Domain. All data stored in QoS Domain will be removed", &EQDDelete),
    newEngineAction("format",
                    "Format QoS Domain. All data stored in QoS Domain will be removed, but the QoS "
                    "Domain will not be removed",
                    &EQDFormat),
    newEngineAction("backup", "Backup QoS Domain's stored data to a set of files", &EQDBackup),
    newEngineAction("restore", "Restore a set of backed up files to the QoS Domain", &EQDRestore),
    newEngineAction("resize", "Change size of existing QoS Domain", &EQDResize),
    newEngineAction("label", "Set the label for an existing QoS Domain", &EQDLabel)};

static struct CEHEngineConfig config = newEngineWithSetup(
    "qos-domain", "Perform actions toward QoS Domains", EQDOptions, options, actions, &EQDSetup, &EQDCleanup);

// registering the engine
static void sefEngineInit qosDomainInit()
{
    CEHRegisterEngine(&config);
}
