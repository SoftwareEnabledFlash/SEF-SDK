/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef-notification.c
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
#include "sef-notification.h"

#include <SEFAPI.h>
#include <pthread.h>
#include <stdint.h>

#include "config.h"
#include "flash-translation.h"
#include "garbage-collection.h"
#include "log-manager.h"
#include "superblock.h"

static SEFQoSHandle getQoSHandleFromId(FTLContext *ctxt, struct SEFQoSDomainID qosId)
{
    int index = qosId.id - ctxt->ssbState->dstate->domainId.id;

    if (qosId.id < ctxt->ssbState->dstate->domainId.id)
    {
        return NULL;
    }
    if (index >= ctxt->ssbState->num_qos)
    {
        return NULL;
    }
    return ctxt->ssbState->dstate[index].qosHandle;
}

void HandleSEFNotification(void *context, struct SEFQoSNotification event)
{
    FTLContext *ctxt = (FTLContext *)context;
    struct SSBParsedFLA pfa;

    if (!ctxt->initialized || ctxt->timeToDie)
    {    // ctxt resources are being constructed/released, can't use ctxt
        // missing a kAddressUpdate will corrupt data
        assert(event.type != kAddressUpdate);
        if (ctxt->initialized && event.type == kSuperBlockStateChanged)
        {    // persistence is running
            LogTrace(ctxt->logHandle, "closed: %" PRIx64, event.changedFlashAddress.bits);
            if (event.writtenADUs == 0)
            {
                SEFQoSHandle hQos = getQoSHandleFromId(ctxt, event.QoSDomainID);
                LogTrace(ctxt->logHandle, "releasing %" PRIx64 "because it is empty",
                         event.changedFlashAddress.bits);
                SEFReleaseSuperBlock(hQos, event.changedFlashAddress);
            }
        }
        else
        {
            LogDebug(ctxt->logHandle, "Lost notification type %d", event.type);
        }
        return;
    }

    switch (event.type)
    {
        case kAddressUpdate:
            if (SEFGetUserAddressMeta(event.changedUserAddress) != SEF_USR_META_TAG)
            {
                LogError(ctxt->logHandle,
                         "Ignoring bad address update of UA:0x%lx NFA:0x%lx OFA:0x%lx",
                         le64toh(event.changedUserAddress.unformatted), event.newFlashAddress.bits,
                         event.oldFlashAddress.bits);
                break;
            }
            SFTUpdate(ctxt, SEFGetUserAddressLba(event.changedUserAddress), event.oldFlashAddress,
                      event.newFlashAddress);
            break;
        case kSuperBlockStateChanged:
            LogTrace(ctxt->logHandle, "closing 0x%" PRIx64 " with 0x%x written out of 0x%x ADUs",
                     event.changedFlashAddress.bits, event.writtenADUs, event.numADUs);
            SSBParseFlashAddress(ctxt, event.changedFlashAddress, &pfa);
            SSBClosed(ctxt, &pfa, event.writtenADUs, event.numADUs);
            break;
        case kUnreadableData:
            LogError(ctxt->logHandle, "Unreadable data for address 0x%lx - shutting down",
                     event.unreadableFlashAddress.bits);
            GCSetFatalError(ctxt->gctx, SUMakeStatus(-EIO, 0));
            break;
        case kRequirePatrol: {
            int index = event.QoSDomainID.id - SSBGetQoSID(ctxt->ssbState->dstate).id;

            if (index >= 0 && index < ctxt->ssbState->num_qos)
            {
                SSBSetPatrol(&ctxt->ssbState->dstate[index], true);
            }
        }
        break;
        case kRequireMaintenance:
            SSBMarkForMaintenance(ctxt, event.maintenanceFlashAddress);
            break;
        case kBufferRelease: {
            struct SEFBlockNotify notify = {
                .type = kSefBlockNotifyBufferRelease, .iov = event.iov, .iovcnt = event.iovcnt};

            ctxt->notifyFunc(notify, ctxt->notifyContext);
            break;
        }
        default:
            LogError(ctxt->logHandle, "Unknown notification type %d - shutting down", event.type);
            GCSetFatalError(ctxt->gctx, SUMakeStatus(-EINVAL, event.type));
            break;
    }
}
