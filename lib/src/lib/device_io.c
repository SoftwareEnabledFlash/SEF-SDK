/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * device_io.c
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

#define MODULE_NAME_SELECT "DEVI/O"

#include "device_io.h"

#include "common.h"
#include "device_info.h"
#include "io_thread_pool.h"
#include "liburing.h"
#include "request.h"
#include "sef_cmd.h"
#include "ulog.h"

#include <fcntl.h>
#include <libudev.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/queue.h>
#include <sys/stat.h>
#include <time.h>

int g_uring_queue_size = 1024;    // The number of sqe entries in the uring
int g_evt = -1;                   // eventfd to shutdown DeviceIoNotifyProc

#define URING_QSIZE g_uring_queue_size
#define URING_FLAGS 0    // Flags for uring creation, use defaults

#if CC_USE_FUSED    // Requires a driver that supports IOSQE_IO_FUSE
#define FUSED_FLAGS (IOSQE_IO_LINK | IOSQE_IO_FUSE)
#else
#define FUSED_FLAGS 0
#endif

/*
 * We use one staticly constructed UringCtx for
 * async i/o, with two locks, one for submission,
 * one for completion
 */
static struct io_thread_pool *thread_pool;
static struct io_thread *sync_thread;

static inline void io_uring_prep_passthrough(struct io_uring_sqe *sqe, int fd, unsigned code)
{
    io_uring_prep_rw(IORING_OP_URING_CMD, sqe, fd, NULL, 0, 0);
    sqe->cmd_op = code;
}

/**
 *  @brief	Initializes device_io class
 *  @return	0: success; -1: ioctl error
 *  @details	Opens common (device-independent) device file of SEF Driver.
 *  @details	Asks SEF Driver to prepare async information, retrieves it, and retains in class.
 */
int DeviceIoInit(int num_threads)
{
    int ret = 0;

    ret = io_thread_create(URING_QSIZE, &sync_thread);
    if (ret)
    {
        return ret;
    }

    /*
     * Set up the UringCtx
     */
    thread_pool = io_thread_pool_create(num_threads, URING_QSIZE);
    if (!thread_pool)
    {
        ULOG_ERROR("failed to create io_thread_pool()\n");
        goto delete_sync;
    }

    return 0;

    io_thread_pool_delete(thread_pool);
delete_sync:
    io_thread_delete(sync_thread);
    sync_thread = NULL;
    thread_pool = NULL;
    return ret;
}

/**
 *  @brief	Asks SEF Driver to prepare for deletion of asynchronous information
 *  @return	0: success; -1: ioctl error
 */
int DeviceIoPreDestroy(void)
{
    if (!thread_pool || !sync_thread)
    {
        return -EBADFD;
    }

    return 0;
}

/**
 *  @brief	Asks SEF Driver to delete asynchronous information
 *  @return	0: success; -1: ioctl error
 */
STATIC int DeviceIoDestroy(void)
{
    int ret;

    if (!sync_thread || !thread_pool)
    {
        return -1;
    }

    io_thread_delete(sync_thread);
    sync_thread = NULL;

    ret = io_thread_pool_delete(thread_pool);
    thread_pool = NULL;

    DeviceIoNotifyShutdown();

    return ret;
}

/**
 *  @brief	Finalizes device_io class
 *  @return	0: success; -1: ioctl error
 *  @details	Asks SEF Driver to delete asynchronous information.
 *  @details	Closes common (device-independent) device file of SEF Driver.
 */
int DeviceIoCleanup(void)
{
    int ret;

    ret = DeviceIoDestroy();
    return ret;
}

/**
 *  @brief	Opens a file
 *  @param	[in] pDeviceName: pointer to filename
 *  @return	fd: file descriptor
 */
int DeviceIoOpenFile(char *pDeviceName)
{
    int fd;

    SEF_ASSERT(pDeviceName != NULL);
    fd = open(pDeviceName, O_RDWR);
    return fd;
}

/**
 *  @brief	Closes a file
 *  @param	[in] fd: file descriptor
 *  @return	None
 */
void DeviceIoCloseFile(int fd)
{
    close(fd);
}

static int TimeElapsed(atomic_uint_least64_t *past, atomic_int *count, int duration_sec)
{
    struct timespec ts;
    uint64_t now_ms;
    uint64_t expiry_ms;
    uint64_t val;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    now_ms = (ts.tv_sec) * 1000 + (ts.tv_nsec / 1000000);
    expiry_ms = now_ms - (1000 * duration_sec);
    val = atomic_load(past);
    while (val <= expiry_ms)
    {
        if (atomic_compare_exchange_weak(past, &val, now_ms))
        {
            return atomic_exchange(count, 1);
        }
    }
    atomic_fetch_add(count, 1);
    return 0;
}

// status is CQE.status and result is CQE.DW0/DW1
static int DeviceIoComplete(void *p, int status, uint64_t result)
{
    struct ReqToComplete *pReq = p;
    pReq->status = status;
    pReq->result = result;
    if (pReq->retryCnt)
    {
        static atomic_uint_least64_t lastLogTime = 0;
        static atomic_int callCount = 1;

        int nCalls = TimeElapsed(&lastLogTime, &callCount, CC_EBUSY_LOG_TIME_LIMIT_SEC);

        if (nCalls)
        {
            ULOG_NOTICE("Command %s retried %d times - %d messages suppressed\n",
                        GetCmdName(pReq->type, &pReq->submit), pReq->retryCnt, nCalls - 1);
        }
    }
    pReq->CommandComplete(pReq);
    return 0;
}

static void DeviceIoPrep(struct io_uring_sqe *sqe, void *p)
{
    struct ReqToComplete *pReq = p;
    unsigned int cmd;
    if (pReq->submit.opcode == SEF_NVMOPC_NAMELESSWRITE || pReq->submit.opcode == SEF_NVMOPC_READ)
    {
        cmd = NVME_URING_CMD_IO_VEC;
    }
    else
    {
        cmd = NVME_URING_CMD_IO;
    }
    io_uring_prep_passthrough(sqe, pReq->io_request.fd, cmd);
    // nvme_passthru_cmd64 looks like a nvme_uring_cmd, only longer
    memcpy(&sqe->cmd, &pReq->submit, sizeof(struct nvme_uring_cmd));
    if (pReq->io_request.fused)
    {
        sqe->flags = FUSED_FLAGS;
    }
}

struct sync_complete
{
    atomic_uint_fast64_t refcnt;
    sem_t sem;
};

struct sync_req_complete
{
    struct sync_complete *sync;
    struct ReqToComplete *req;
};

static int DeviceIoCompleteSync(void *p, int status, uint64_t result)
{
    struct sync_req_complete *complete = p;
    complete->req->status = status;
    complete->req->result = result;
    if (atomic_fetch_sub(&complete->sync->refcnt, 1) == 1)
    {
        sem_post(&complete->sync->sem);
    }
    return 0;
}

/**
 *  @brief	Asks SEF Driver to issue a command
 *  @param	[in] fd: device file descriptor
 *  @param	[in] type: command type
 *  @param	[in] pCommandParam: pointer to parameter according to itype
 *  @return	0: success; -1: ioctl error
 *  @details	Asks SEF Driver according to command type
 *  @details	- For kIoctlAdminCommand: type of pCommandParam is struct sef_submit
 *  @details	- For kIoctlIoCommand: type of pCommandParam is struct sef_submit
 *  @details	- For kIoctlIoFusedCommand: type of pCommandParam is struct sef_submit_fused
 */
int DeviceIoRequest(int fd,
                    enum IoctlCommandType type,
                    struct ReqToComplete **pCommands,
                    size_t numCommands,
                    bool bSync)
{
    int ret = -1;

    SEF_ASSERT(numCommands <= 2);
    SEF_ASSERT(pCommands);
    SEF_ASSERT(pCommands[0] != NULL && (numCommands == 1 || pCommands[1]));
    SEF_ASSERT((type == kIoctlAdminCommand) || (type == kIoctlIoCommand) ||
               (type == kIoctlIoFusedCommand));

    if (bSync)
    {
        /*
         * For sync i/o, use ioctl to not risk causing lockup by blocking all
         * completion threads
         */
        int code;

#ifndef UNIT_TEST
        SEF_ASSERT(type == kIoctlAdminCommand || type == kIoctlIoCommand);
#endif
        if (type == kIoctlAdminCommand)
        {
            code = NVME_IOCTL_ADMIN64_CMD;
        }
        else if (type == kIoctlIoCommand)
        {
            code = NVME_IOCTL_IO64_CMD;
        }
        else
        {
            struct sync_complete sync = {2};
            sem_init(&sync.sem, 0, 0);

            struct sync_req_complete completes[2] = {{&sync, pCommands[0]}, {&sync, pCommands[1]}};

            pCommands[0]->io_request.prep = DeviceIoPrep;
            pCommands[0]->io_request.user_data = &completes[0];
            pCommands[0]->io_request.complete = DeviceIoCompleteSync;
            pCommands[0]->io_request.fd = fd;
            pCommands[0]->io_request.fused = 0;

            if (numCommands == 2)
            {
                pCommands[1]->io_request.prep = DeviceIoPrep;
                pCommands[1]->io_request.user_data = &completes[2];
                pCommands[1]->io_request.complete = DeviceIoCompleteSync;
                pCommands[1]->io_request.fd = fd;
                pCommands[0]->io_request.fused = &pCommands[1]->io_request;
            }

            ret = io_thread_submit(sync_thread, &pCommands[0]->io_request);
            if (ret)
            {
                return ret;
            }

            sem_wait(&sync.sem);

            return 0;
        }
        ret = ioctl(fd, code, &pCommands[0]->submit);
        if (ret)
        {
            ULOG_ERROR("RET=%d\n", ret);
        }
        pCommands[0]->status = ret;
        pCommands[0]->result = pCommands[0]->submit.result;
        return ret >= 0 ? 0 : ret;
    }

    pCommands[0]->io_request.prep = DeviceIoPrep;
    pCommands[0]->io_request.user_data = pCommands[0];
    pCommands[0]->io_request.complete = DeviceIoComplete;
    pCommands[0]->io_request.fd = fd;
    pCommands[0]->io_request.fused = 0;

    if (numCommands == 2)
    {
        pCommands[1]->io_request.prep = DeviceIoPrep;
        pCommands[1]->io_request.user_data = pCommands[1];
        pCommands[1]->io_request.complete = DeviceIoComplete;
        pCommands[1]->io_request.fd = fd;
        pCommands[0]->io_request.fused = &pCommands[1]->io_request;
    }

    if (pCommands[0]->syncOverride)
    {
        ret = io_thread_submit(sync_thread, &pCommands[0]->io_request);
    }
    else
    {
        ret = io_thread_pool_submit(thread_pool, &pCommands[0]->io_request);
    }

    if (ret)
    {
        ULOG_ERROR("Async submit failed: %d", ret);
    }
    return ret;
}

struct SEFHandle_ *getSefHandleByName(const char *name)
{
    uint32_t i;

    for (i = 0; i < DeviceInfoGetNumSefUnits(); i++)
    {
        const char *devName;

        devName = DeviceInfoGetDevicePath(i);
        if (strcmp(name, devName) == 0)
        {
            return DeviceInfoGetSefHandle(i);
        }
    }
    return NULL;
    ;
}

uint32_t getUDevProp32(struct udev_device *dev, const char *name, uint32_t default_value)
{
    const char *val = udev_device_get_property_value(dev, name);

    return val ? strtol(val, NULL, 0) : default_value;
}

STATIC void *DeviceIoNotifyProc(void *arg)
{
    void (*callback)(struct SEFHandle_ *pSef, struct nvme_sef_notification *event) = arg;
    struct udev *udev;
    struct udev_monitor *mon;
    int fd;

    udev = udev_new();
    mon = udev_monitor_new_from_netlink(udev, "udev");
    if (!mon)
    {
        ULOG_ERROR("Unable to open netlink monitor: %d\n", errno);
        goto free_dev;
    }
    udev_monitor_filter_add_match_subsystem_devtype(mon, "sef", NULL);
    udev_monitor_enable_receiving(mon);    // ** what was missing
    fd = udev_monitor_get_fd(mon);
    for (;;)
    {
        struct pollfd ufd[] = {{.fd = fd, .events = POLLIN}, {.fd = g_evt, .events = POLLIN}};
        struct nvme_sef_notification event;
        struct udev_device *dev;
        struct SEFHandle_ *pSef;
        uint32_t nvme_aen;

        while (poll(ufd, 2, -1) == -1)
        {
            if (errno != EINTR)
            {
                break;
            }
        }
        if (ufd[1].revents & POLLIN)
        {
            break;
        }
        dev = udev_monitor_receive_device(mon);
        if (!dev)
        {
            continue;
        }
        const char *sef_aen = udev_device_get_property_value(dev, "SEF_AEN");
        if (!sef_aen)
        {
            udev_device_unref(dev);
            continue;
        }
        const char *dev_name = udev_device_get_devnode(dev);
        nvme_aen = strtol(sef_aen, NULL, 0);

        event.aen_result = nvme_aen;
        switch (nvme_aen)
        {
            case VD_AEN:
                event.vdid = strtol(udev_device_get_property_value(dev, "SEF_VDID"), NULL, 0);
                event.cwarn = strtol(udev_device_get_property_value(dev, "SEF_CRIT"), NULL, 0);
                ULOG_NOTICE("VD AEN Event received: vd %d %s\n", event.vdid, dev_name);
                break;
            case SB_AEN:
                event.qosd = getUDevProp32(dev, "SEF_QOSD", 0);
                event.sbid = getUDevProp32(dev, "SEF_SBID", 0);
                event.state = getUDevProp32(dev, "SEF_SBST", 0);
                event.nvadu = getUDevProp32(dev, "SEF_NVADU", 0);
                ULOG_NOTICE("SB AEN Event received: qos %d %s\n", event.qosd, dev_name);
                break;
            default:
                continue;    // an event we don't care about
        }
        pSef = getSefHandleByName(dev_name);
        if (pSef)
        {
            callback(pSef, &event);    // Notify thread creator w/ registered callback
        }
        else
        {
            ULOG_ERROR("NULL SEF handle for device %s\n", dev_name);
        }
        udev_device_unref(dev);
    }
    udev_monitor_unref(mon);
free_dev:
    udev_unref(udev);

    ULOG_INFORMATION("notification thread exiting\n");

    return NULL;
}

/**
 *  @brief	Create a completion handler thread
 *  @param	[out] threadId: handle to created thread
 *  @param	[in] caller defined handler for async device notifications
 *  @return	0: success; -errno on thread creation failure
 */
int DeviceIoNotifyCreate(pthread_t *threadId,
                         void (*callback)(struct SEFHandle_ *pSef, struct nvme_sef_notification *event))
{
    int ret;

    if (!callback)
    {
        return -EINVAL;
    }
    if (g_evt != -1)
    {
        ULOG_ERROR("Notify thread is already running\n");
        return -EINVAL;
    }
    g_evt = eventfd(0, EFD_NONBLOCK);
    ret = -EBADF;
    if (g_evt == -1)
    {
        ULOG_ERROR("Failed to create notify event fd\n");
        ret = -errno;
    }
    else
    {
        ret = pthread_create(threadId, NULL, DeviceIoNotifyProc, callback);
    }
    if (ret)
    {
        close(g_evt);
        g_evt = -1;
    }
    return ret;
}

void DeviceIoNotifyShutdown()
{
    if (g_evt != -1)
    {
        eventfd_write(g_evt, 1);
        pthread_join(DeviceInfoGetSystemInfo()->asyncNotifyThreadId, NULL);
        close(g_evt);
        g_evt = -1;
    }
}

int DeviceIoIsIoThread()
{
    return io_thread_is();
}
