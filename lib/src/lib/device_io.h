/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * device_io.h
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

#ifndef __DEVICE_IO_H__
#define __DEVICE_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "common.h"

struct ReqToComplete;
struct SEFHandle_;

#define VD_AEN 0xd04006
#define SB_AEN 0xd34006

struct nvme_sef_notification
{
    uint32_t aen_result;
    union
    {
        struct
        {
            uint32_t qosd;
            uint32_t sbid;
            uint32_t nvadu;
            uint16_t state;
        };
        struct
        {
            uint16_t vdid;
            uint8_t cwarn;
        };
    };
};

/**
 *  @brief	 Command type for requesting SEF Driver to issue commands
 */
enum IoctlCommandType {
    kIoctlAdminCommand,     //!< Admin command
    kIoctlIoCommand,        //!< I/O (NVM) command
    kIoctlIoFusedCommand    //!< Fused command
};

/*
 * function prototype
 */
int DeviceIoInit(int num_threads);
int DeviceIoWaitResponse(struct ReqToComplete **pComplete);
int DeviceIoPreDestroy(void);
int DeviceIoCleanup(void);
int DeviceIoOpenFile(char *pDeviceName);
void DeviceIoCloseFile(int fd);
int DeviceIoRequest(int fd,
                    enum IoctlCommandType type,
                    struct ReqToComplete **pCommands,
                    size_t numCommands,
                    bool bSync);
int DeviceIoSbStateChangeRequest(int fd, int index);
int DeviceIoSbStateChangeCancel(int fd);
int DeviceIoNotifyCreate(pthread_t *threadId,
                         void (*callback)(struct SEFHandle_ *pSef, struct nvme_sef_notification *event));
void DeviceIoNotifyShutdown();
int DeviceIoQoSDevOpen(int unit, int qosId);
int DeviceIoIsIoThread();

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_IO_H__ */
