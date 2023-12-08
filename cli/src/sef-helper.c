/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-helper.c
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
#include "sef-helper.h"

#include <SEFAPI.h>
#include <errno.h>
#include <pthread.h>

#include "io-helper.h"
#include "utils/sef-event.h"
#include "utils/str-error.h"

struct pthread_event
{
    pthread_cond_t cond;
    pthread_mutex_t mutex;
};

bool CSHIsQosDomainIdValid(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainId)
{
    uint16_t numQoSDomains, i;
    struct SEFStatus status;
    struct SEFQoSDomainList *qosList;

    // basic check
    if (QoSDomainId.id < 0)
    {
        return false;
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

        status = SEFListQoSDomains(sefHandle, qosList, qosListSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of QoS Domains for SEF Unit Index");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFListQoSDomains", status.error));
            free(qosList);
            return false;
        }

    } while (numQoSDomains != qosList->numQoSDomains);

    // check if qos domain id exists
    for (i = 0; i < qosList->numQoSDomains; i++)
    {
        if (qosList->QoSDomainID[i].id == QoSDomainId.id)
        {
            free(qosList);
            return true;
        }
    }

    free(qosList);

    return false;
}

bool CSHIsVirtualDeviceIdValid(SEFHandle sefHandle, struct SEFVirtualDeviceID VirtualDeviceId)
{
    uint16_t numVirtualDevices, i;
    struct SEFStatus status;
    struct SEFVirtualDeviceList *vdList;

    // basic check
    if (VirtualDeviceId.id < 1)
    {
        return false;
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

        status = SEFListVirtualDevices(sefHandle, vdList, vdListSize);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to get list of Virtual Devices");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFListVirtualDevices", status.error));

            free(vdList);
            return false;
        }

    } while (numVirtualDevices != vdList->numVirtualDevices);

    // check if qos domain id exists
    for (i = 0; i < vdList->numVirtualDevices; i++)
    {
        if (vdList->virtualDeviceID[i].id == VirtualDeviceId.id)
        {
            free(vdList);
            return true;
        }
    }

    free(vdList);

    return false;
}

struct SEFStatus CSHGetQosDomainId(SEFHandle sefHandle, struct SIFLabel label)
{
    int qosListSize;
    struct SEFStatus status;
    struct SEFQoSDomainList *qosList;
    uint16_t qosDomainId;

    qosListSize = sizeof(struct SEFQoSDomainList) + sizeof(struct SEFQoSDomainID);
    qosList = (struct SEFQoSDomainList *)malloc(qosListSize);

    status = SUGetQosDomainId(sefHandle, label, qosList, qosListSize);
    if (status.error)
    {
        CIHMessageError("Error: Was unable to get the associated QoS Domain");
        free(qosList);
        return status;
    }

    if (qosList->numQoSDomains == 0)
    {
        CIHMessageError("Error: Was unable to find any associated QoS Domains with given label");
        free(qosList);
        return makeStatusError(ENXIO);
    }

    if (qosList->numQoSDomains > 1)
    {
        CIHMessageError("Error: More than one QoS Domain is associated with the given label");
        free(qosList);
        return makeStatusError(EBADF);
    }

    qosDomainId = qosList->QoSDomainID[0].id;

    free(qosList);

    return makeStatus(0, qosDomainId);
}

void SEFWriteSyncComp(struct SEFCommonIOCB *iocb)
{
    struct sefEvent *done = iocb->param1;
    SefEventSet(done);
}

struct SEFStatus CSHWriteSync(SEFQoSHandle qosHandle,
                              struct SEFFlashAddress flashAddress,
                              struct SEFPlacementID placementID,
                              struct SEFUserAddress userAddress,
                              uint32_t numADU,
                              const struct iovec *iov,
                              uint16_t iovCnt,
                              const void *metadata,
                              struct SEFFlashAddress *permanentAddresses,
                              uint32_t *distanceToEndOfSuperBlock,
                              const struct SEFWriteOverrides *overrides)
{
    struct SEFWriteWithoutPhysicalAddressIOCB writeIocb;
    struct sefEvent done = SEFEVENT_AUTORESET_INITIALIZER;

    writeIocb.flashAddress = flashAddress;
    writeIocb.placementID = placementID;
    writeIocb.userAddress = userAddress;
    writeIocb.numADU = numADU;
    writeIocb.iov = iov;
    writeIocb.iovcnt = iovCnt;
    writeIocb.tentativeAddresses = permanentAddresses;
    writeIocb.metadata = metadata;
    writeIocb.common.flags = 0;
    writeIocb.common.param1 = &done;
    writeIocb.common.complete_func = &SEFWriteSyncComp;
    if (overrides != NULL)
    {
        writeIocb.overrides.programWeight = overrides->programWeight;
    }

    SEFWriteWithoutPhysicalAddressAsync(qosHandle, &writeIocb);

    SefEventWait(&done);

    if (distanceToEndOfSuperBlock != NULL)
    {
        *distanceToEndOfSuperBlock = writeIocb.distanceToEndOfSuperBlock;
    }

    return writeIocb.common.status;
}
