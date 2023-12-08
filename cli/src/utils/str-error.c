/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * str-error.c
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
#include "str-error.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define FunctionMapNew(N, C, ...)             \
    {                                         \
        N, C, (struct ReturnMap[])__VA_ARGS__ \
    }

struct ReturnMap
{
    int errorNum;
    int stringIndex;
};

struct FunctionMap
{
    const char *functionName;
    int returnMapNum;
    struct ReturnMap *returnMap;
};

static char *const sefErrors[] = {
    "The info member returns the number of units",
    "The info field is the library's reference count.",
    "The SEF Library was not initialized",
    "This function cannot be called on a callback thread",
    "The SEF Handle is not valid",
    "The function parameter is not valid; info returns the parameter index that is not valid",
    "info field returns the minimum buffer size if the buffer is insufficient or NULL; otherwise, "
    "0",
    "You don't have the needed permission to perform this operation",
    "The number of pSLC super blocks has been set",
    "No space is available for pSLC super blocks",
    "The function parameter is not valid; info returns the parameter index that is not valid.",
    "The Virtual Device Handle is not valid",
    "The Virtual Device Handle is not open",
    "The suspend configuration has been set",
    "The library was unable to allocate needed structures. status.info is set to the type of "
    "capacity that caused the failure (0 for kForWrite, 1 for kForPSLCWrite, 2 for QoSD max)",
    "The Virtual Device does not have enough space",
    "The QoS Domain handle is not valid",
    "The QoS Domain Handle is not open",
    "The SEF handle is not valid",
    "SEFQoSDomainInfo was successfully returned.",
    "The super block is checked",
    "At least one QoS Domain exists",
    "The Virtual Device is in use and not all the handles are closed",
    "The QoS Domain ID is not valid",
    "The QoS Domain is in use and not all the handles are closed",
    "The Virtual Device handle is not valid",
    "The Virtual Device handle is not open",
    "The QoS Domain Id is not valid",
    "The library was unable to allocate needed structures",
    "The Virtual Device is already open",
    "The QoS Domain is already open",
    "The Flash Address is not valid",
    "The info member contains number of ADUs in allocated super block",
    "The QoS Domain is out of space",
    "The super block is was closed or was already closed",
    "the info member contains: Source list indication referenced non-closed super blocks "
    "(kCopyNonClosedSuperBlock), Destination super block has defective planes "
    "(kCopyDestinationDefectivePlanes), Read error was detected on source "
    "(kCopyReadErrorOnSource), Data that is out of User Address range is detected "
    "(kCopyFilteredUserAddresses), Filled addressChangeInfo array and stopped the copy "
    "(kCopyFilledAddressChangeInfo), Destination super block was filled/closed "
    "(kCopyClosedDestination), Consumed entire source bitmap or list (kCopyConsumedSource)"};

#define sefFunctionNum 38

static struct FunctionMap sefFunctions[] = {
    FunctionMapNew("SEFAllocateSuperBlock",
                   5,
                   {{0, 32}, {-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {-ENOSPC, 33}}),
    FunctionMapNew("SEFCheckSuperBlock", 4, {{0, 20}, {-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFCloseQoSDomain", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EWOULDBLOCK, 3}}),
    FunctionMapNew("SEFCloseSuperBlock", 4, {{0, 34}, {-ENODEV, 16}, {-EPERM, 17}, {-EFAULT, 31}}),
    FunctionMapNew("SEFCloseVirtualDevice", 3, {{-ENODEV, 25}, {-EPERM, 12}, {-EWOULDBLOCK, 3}}),
    FunctionMapNew("SEFCreateQoSDomain", 4, {{-ENODEV, 11}, {-EPERM, 12}, {-EINVAL, 5}, {-ENOMEM, 14}}),
    FunctionMapNew("SEFCreateVirtualDevices", 3, {{-ENODEV, 4}, {-EINVAL, 5}, {-EACCES, 7}}),
    FunctionMapNew("SEFDeleteQoSDomain", 4, {{-ENODEV, 18}, {-EINVAL, 23}, {-EACCES, 7}, {-EBUSY, 24}}),
    FunctionMapNew("SEFDeleteVirtualDevices",
                   5,
                   {{-ENODEV, 4}, {-EINVAL, 5}, {-EACCES, 7}, {-ENOTEMPTY, 21}, {-EBUSY, 22}}),
    FunctionMapNew("SEFFlushSuperBlock", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFGetCheckList", 4, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetDieList", 3, {{-ENODEV, 4}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetQoSDomainInformation", 3, {{-ENODEV, 18}, {-EINVAL, 5}, {0, 19}}),
    FunctionMapNew("SEFGetRefreshList", 4, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetReuseList", 4, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetSuperBlockInfo", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFGetSuperBlockList", 4, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetUserAddressList", 4, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetVirtualDeviceInformation", 3, {{-ENODEV, 4}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFGetVirtualDeviceUsage", 2, {{-ENODEV, 11}, {-EPERM, 12}}),
    FunctionMapNew("SEFLibraryCleanup", 3, {{0, 1}, {-ENODEV, 2}, {-EWOULDBLOCK, 3}}),
    FunctionMapNew("SEFLibraryInit", 1, {{0, 0}}),
    FunctionMapNew("SEFListQoSDomains", 3, {{-ENODEV, 4}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFListVirtualDevices", 3, {{-ENODEV, 4}, {-EINVAL, 5}, {0, 6}}),
    FunctionMapNew("SEFNamelessCopy", 4, {{0, 35}, {-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFOpenQoSDomain",
                   5,
                   {{-ENODEV, 18}, {-EINVAL, 5}, {-EACCES, 7}, {-ENOMEM, 28}, {-EALREADY, 30}}),
    FunctionMapNew("SEFOpenVirtualDevice",
                   5,
                   {{-ENODEV, 18}, {-EINVAL, 5}, {-EACCES, 7}, {-ENOMEM, 28}, {-EALREADY, 29}}),
    FunctionMapNew("SEFReadWithPhysicalAddress", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFReleaseSuperBlock", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EFAULT, 31}}),
    FunctionMapNew("SEFResetEncryptionKey", 3, {{-ENODEV, 25}, {-EPERM, 26}, {-EINVAL, 27}}),
    FunctionMapNew(
        "SEFSetNumberOfPSLCSuperBlocks", 4, {{0, 8}, {-ENODEV, 4}, {-ENOSPC, 9}, {-EINVAL, 10}}),
    FunctionMapNew(
        "SEFSetQoSDomainCapacity", 4, {{-ENODEV, 11}, {-EPERM, 12}, {-EINVAL, 5}, {-ENOSPC, 15}}),
    FunctionMapNew("SEFSetQoSHandleProperty", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFSetReadDeadline", 2, {{-ENODEV, 16}, {-EPERM, 17}}),
    FunctionMapNew("SEFSetRootPointer", 3, {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}}),
    FunctionMapNew("SEFSetVirtualDeviceSuspendConfig", 3, {{0, 13}, {-ENODEV, 4}, {-EINVAL, 10}}),
    FunctionMapNew("SEFSetWeights", 2, {{-ENODEV, 16}, {-EPERM, 17}}),
    FunctionMapNew("SEFWriteWithoutPhysicalAddress",
                   4,
                   {{-ENODEV, 16}, {-EPERM, 17}, {-EINVAL, 5}, {-ENOSPC, 33}})};

int FunctionMapCompare(const void *a, const void *b)
{
    const struct FunctionMap *aMap = a;
    const struct FunctionMap *bMap = b;

    return strcmp(aMap->functionName, bMap->functionName);
}

char *SEFStrError(char *functionName, int errorNum)
{
    struct FunctionMap *foundFunction, tmpFunction;

    // bsearch the functions list
    foundFunction = NULL;
    tmpFunction.functionName = functionName;
    foundFunction = (struct FunctionMap *)bsearch(&tmpFunction, sefFunctions, sefFunctionNum,
                                                  sizeof(struct FunctionMap), FunctionMapCompare);

    // find the return string
    if (foundFunction != NULL)
    {
        int j;
        struct ReturnMap *map = foundFunction->returnMap;

        for (j = 0; j < foundFunction->returnMapNum; j++)
        {
            if (errorNum == map[j].errorNum || errorNum * -1 == map[j].errorNum)
            {
                return sefErrors[map[j].stringIndex];
            }
        }
    }

    // return default error text if not found
    return strerror(errorNum);
}
