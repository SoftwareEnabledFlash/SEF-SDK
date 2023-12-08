/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-utils.c
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
#include "sef-utils.h"

#include <errno.h>
#include <stddef.h>

#include "config.h"

SEFHandle SUGetHandle(SEFQoSHandle qosHandle)
{
    int unit = SEFGetQoSHandleProperty(qosHandle, kSefPropertyUnitNumber).intVal;

    return SEFGetHandle(unit);
}

const struct SEFInfo *SUGetInformation(SEFQoSHandle qosHandle)
{
    SEFHandle sefHandle = SUGetHandle(qosHandle);

    return SEFGetInformation(sefHandle);
}

struct SEFStatus SUGetSuperBlockDies(SEFQoSHandle qosHandle)
{
    struct SEFVirtualDeviceInfo vdInfo;
    struct SEFVirtualDeviceID vdId;
    struct SEFStatus status;
    SEFHandle hSef;

    hSef = SUGetHandle(qosHandle);
    vdId = SEFGetQoSHandleProperty(qosHandle, kSefPropertyVirtualDeviceID).vdID;
    status = SEFGetVirtualDeviceInformation(hSef, vdId, &vdInfo, sizeof(vdInfo));
    if (status.error == 0)
    {
        status.info = vdInfo.superBlockDies;
    }
    return status;
}

struct SEFStatus SUGetDieList(SEFHandle hSef, struct SEFVirtualDeviceID vid, struct SEFDieList **pDieList)
{
    int ret_n = 0;
    int req_n = 0;
    struct SEFDieList *dieList = NULL;
    struct SEFStatus status;

    *pDieList = NULL;
    do
    {
        int size = struct_size(dieList, dieIDs, ret_n);

        req_n = ret_n;
        SUfree(dieList);
        dieList = SUzalloc(size);
        status = SEFGetDieList(hSef, vid, dieList, size);
        ret_n = dieList->numDies;
    } while (status.error != 0 || ret_n > req_n);

    if (status.error == 0)
    {
        *pDieList = dieList;
    }
    else
    {
        SUfree(dieList);
    }

    return status;
}

struct SEFStatus SUGetLabel(SEFHandle sefHandle, struct SEFQoSDomainID qosId, struct SIFLabel *label)
{
    struct SEFStatus status;
    struct SEFQoSDomainInfo qosDomainInfo;
    const struct SEFInfo *sefInfo;

    // get sef info and check compatibility
    sefInfo = SEFGetInformation(sefHandle);
    if (sefInfo == NULL)
    {
        return SUMakeStatusError(-ENODEV);
    }

    if (sefInfo->maxRootPointers < 2)
    {
        return SUMakeStatusError(-EPROTONOSUPPORT);
    }

    // get current label for the qos domain
    status = SEFGetQoSDomainInformation(sefHandle, qosId, &qosDomainInfo);
    if (status.error)
    {
        return status;
    }

    label->data[0] = qosDomainInfo.rootPointers[sefInfo->maxRootPointers - 2].bits;
    label->data[1] = qosDomainInfo.rootPointers[sefInfo->maxRootPointers - 1].bits;

    return SUMakeStatusOk();
}

struct SEFStatus SUGetQosDomainId(SEFHandle sefHandle,
                                  struct SIFLabel label,
                                  struct SEFQoSDomainList *list,
                                  int bufferSize)
{
    struct SEFStatus status;
    uint16_t i, numQosDomains, numLabeledQoSDomains, bufferCount;
    struct SEFQoSDomainList *qosList;

    // validate arguments
    if (list != NULL && bufferSize < sizeof(struct SEFQoSDomainList) && bufferSize != 0)
    {
        return SUMakeStatusError(-EINVAL);
    }

    // calculate caller buffer capacity
    bufferCount =
        list ? (bufferSize - sizeof(struct SEFQoSDomainList)) / sizeof(struct SEFQoSDomainID) : 0;

    // get list of the the available qos domains
    numQosDomains = 0;
    qosList = NULL;
    do
    {
        int qosListSize;

        if (qosList != NULL)
        {
            numQosDomains = qosList->numQoSDomains;
            SUfree(qosList);
        }

        qosListSize = sizeof(struct SEFQoSDomainList) + numQosDomains * sizeof(struct SEFQoSDomainID);
        qosList = SUmalloc(qosListSize);

        status = SEFListQoSDomains(sefHandle, qosList, qosListSize);
        if (status.error)
        {
            SUfree(qosList);
            return status;
        }

    } while (numQosDomains != qosList->numQoSDomains);

    // get list of qos domains with the label
    numLabeledQoSDomains = 0;
    for (i = 0; i < numQosDomains; i++)
    {
        struct SIFLabel qosLabel;

        status = SUGetLabel(sefHandle, qosList->QoSDomainID[i], &qosLabel);
        if (status.error)
        {
            SUfree(qosList);
            return status;
        }

        if (qosLabel.data[0] == label.data[0] && qosLabel.data[1] == label.data[1])
        {
            numLabeledQoSDomains++;

            if (bufferCount >= numLabeledQoSDomains)
            {
                list->QoSDomainID[numLabeledQoSDomains - 1] = qosList->QoSDomainID[i];
            }
        }
    }

    if (list != NULL)
    {
        list->numQoSDomains = numLabeledQoSDomains;
    }

    // free memory
    SUfree(qosList);

    if (bufferCount < numLabeledQoSDomains)
    {
        return SUMakeStatusInfo(sizeof(struct SEFQoSDomainList) +
                                numLabeledQoSDomains * sizeof(struct SEFQoSDomainID));
    }

    return SUMakeStatusOk();
}

struct SEFStatus SUSetLabel(SEFQoSHandle qosHandle, struct SIFLabel label, int override)
{
    struct SIFLabel qosLabel;
    struct SEFStatus status;
    SEFHandle sefHandle;
    struct SEFQoSDomainID qosId;
    struct SEFFlashAddress flashAddress[2];

    // get sef handle from qos domain handle
    sefHandle = SUGetHandle(qosHandle);

    // get qos domain id from qos domain handle
    qosId = SEFGetQoSHandleProperty(qosHandle, kSefPropertyQoSDomainID).qosID;

    // get current label for the qos domain
    status = SUGetLabel(sefHandle, qosId, &qosLabel);
    if (status.error)
    {
        return status;
    }

    // get sef info and check compatibility
    const struct SEFInfo *sefInfo = SEFGetInformation(sefHandle);
    if (sefInfo == NULL)
    {
        return SUMakeStatusError(-ENODEV);
    }

    if (sefInfo->maxRootPointers < 2)
    {
        return SUMakeStatusError(-EPROTONOSUPPORT);
    }

    // check if qos domain is already labeled
    if ((qosLabel.data[0] != 0 || qosLabel.data[1] != 0) && !override)
    {
        return SUMakeStatusError(-EEXIST);
    }

    // set label root pointers
    flashAddress[0].bits = label.data[0];
    flashAddress[1].bits = label.data[1];
    status = SEFSetRootPointer(qosHandle, sefInfo->maxRootPointers - 2, flashAddress[0]);
    if (status.error)
    {
        return status;
    }

    status = SEFSetRootPointer(qosHandle, sefInfo->maxRootPointers - 1, flashAddress[1]);
    if (status.error)
    {
        return status;
    }

    return SUMakeStatusOk();
}

struct SEFFlashAddress SUFlashAddressSuperBlock(SEFQoSHandle qosHandle,
                                                struct SEFFlashAddress flashAddress)
{
    uint32_t blockNumber;
    struct SEFQoSDomainID qosDomainId;

    // generate the superblock address
    SEFParseFlashAddress(qosHandle, flashAddress, &qosDomainId, &blockNumber, NULL);
    return SEFCreateFlashAddress(qosHandle, qosDomainId, blockNumber, 0);
}
