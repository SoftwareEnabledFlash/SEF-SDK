/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * device_info.c
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

#define MODULE_NAME_SELECT "DEVINF"

#include "device_info.h"

#include "common.h"
#include "request.h"
#include "ulog.h"

#include <dirent.h>
#include <errno.h>
#include <linux/types.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 *  @brief	 Cause of suspension
 */
enum SuspendCause {
    kSuspendedByOpen = 1,    //!< Suspended because SB is open
    kSuspendedByWrite = 2    //!< Suspended because SB is being written
};

STATIC struct SystemInfo systemInfo;    //!< System information table
STATIC struct SeflibInfo seflibInfo;    //!< SEF Library information table

STATIC uint32_t DeviceInfoKeyFromSbId(uint32_t sbId)
{
    return sbId + 1;
}

/**
 *  @brief	Initializes device_info class
 *  @param	[in] numSefUnits: number of SEF Units
 *  @param	[in] ppDeviceName: pointer to device name list
 *  @retval	0: success
 *  @retval	-1: memory allocation error
 *  @details	Initializes system information table and SEF Library information table.
 *  @details	On error, allocated memory area is released before returning.
 */
int DeviceInfoInit(int numSefUnits, char **ppDeviceName)
{
    SEF_ASSERT(numSefUnits > 0);
    SEF_ASSERT(ppDeviceName != NULL);

    seflibInfo.ppSefHandle = calloc(numSefUnits, sizeof(*seflibInfo.ppSefHandle));
    if (seflibInfo.ppSefHandle == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        return -1;
    }
    seflibInfo.numSefUnits = numSefUnits;
    seflibInfo.ppDeviceName = ppDeviceName;

    return 0;
}

/**
 *  @brief	Finalizes device_info class
 *  @return	None
 *  @details	Releases system information table and SEF Library information table
 */
void DeviceInfoCleanup(void)
{
    free(seflibInfo.ppSefHandle);
    for (int i = 0; i < seflibInfo.numSefUnits; i++)
    {
        free(seflibInfo.ppDeviceName[i]);
    }
    free(seflibInfo.ppDeviceName);

    memset(&systemInfo, 0, sizeof(systemInfo));
    memset(&seflibInfo, 0, sizeof(seflibInfo));
}

/**
 *  @brief	Creates SEF handle
 *  @param	[in] index: index to SEF Unit
 *  @param	[in] numVds: maximum number of Virtual Devices in the device to create SEF handle
 *  @param	[in] numQosds: maximum number of QoS Domains in the device to create SEF handle
 *  @param	[in] numAduSizes: number of elements in ADUsize[] in struct SEFInfo
 *  @param	[out] ppSefHandle: pointer for returning SEF handle
 *  @retval	0: success
 *  @retval	-1: memory allocation error
 *  @details	Allocates and initializes areas for:
 *  @details	- SEF handle
 *  @details	- SEF Unit Info in SEF handle
 *  @details	- VD handle table in SEF handle
 *  @details	- QoSD handle table in SEF handle
 *  @details	- ReqToComplete in SEF handle for SB State Change notification
 *  @details	On error, allocated areas are released before returning.
 */
int DeviceInfoCreateSefHandle(uint32_t index,
                              uint16_t numVds,
                              uint16_t numQosds,
                              uint16_t numAduSizes,
                              struct SEFHandle_ **ppSefHandle)
{
    struct SEFHandle_ *pSefHandle;
    struct SEFVDHandle_ **ppVdHandle;
    struct SEFQoSHandle_ **ppQosHandle;
    struct ReqToComplete *pReq2Cmp;

    SEF_ASSERT(index < seflibInfo.numSefUnits);
    SEF_ASSERT(numVds != 0);
    SEF_ASSERT(numQosds != 0);
    SEF_ASSERT(ppSefHandle != NULL);

    pSefHandle = (struct SEFHandle_ *)calloc(1, sizeof(*pSefHandle));
    if (pSefHandle == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        return -1;
    }

    pSefHandle->pSefInfo = (struct SEFInfo *)calloc(
        1, sizeof(*(pSefHandle->pSefInfo)) + sizeof(struct SEFADUsize) * numAduSizes);
    if (pSefHandle->pSefInfo == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        goto FreeSefHandle;
    }

    ppVdHandle = (struct SEFVDHandle_ **)calloc(numVds + 1, sizeof(*ppVdHandle));
    if (ppVdHandle == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        goto FreeSefInfo;
    }

    ppQosHandle = (struct SEFQoSHandle_ **)calloc(numQosds + 1, sizeof(*ppQosHandle));
    if (ppQosHandle == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        goto FreeVdHandle;
    }
    pSefHandle->numQosds = numQosds + 1;

    pReq2Cmp = (struct ReqToComplete *)calloc(1, sizeof(*pReq2Cmp));
    if (pReq2Cmp == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        goto FreeQosHandle;
    }

    pSefHandle->pReq2CmpSbStateChange = pReq2Cmp;
    pSefHandle->numProcessingRequests = 0;
    pSefHandle->deviceId = index;
    pSefHandle->ppVdHandle = ppVdHandle;
    pSefHandle->ppQosHandle = ppQosHandle;
    pthread_mutex_init(&pSefHandle->mutex, NULL);
    pthread_mutex_init(&pSefHandle->vd_mutex, NULL);
    *ppSefHandle = pSefHandle;
    return 0;

FreeQosHandle:
    free(ppQosHandle);
FreeVdHandle:
    free(ppVdHandle);
FreeSefInfo:
    free(pSefHandle->pSefInfo);
FreeSefHandle:
    free(pSefHandle);

    return -1;
}

int DeviceInfoSetSefHandle(struct SEFHandle_ *pSefHandle)
{
    struct SEFHandle_ *old_value = NULL;
    if (atomic_compare_exchange_strong(&seflibInfo.ppSefHandle[pSefHandle->deviceId], &old_value,
                                       pSefHandle))
    {
        return 0;
    }
    return -1;
}

/**
 *  @brief	Deletes SEF handle
 *  @param	[in] pSefHandle: pointer to SEF handle
 *  @return	None
 *  @details	Releases areas for:
 *  @details	- SEF handle
 *  @details	- SEF Unit Info in SEF handle
 *  @details	- VD handle table in SEF handle
 *  @details	- QoSD handle table in SEF handle
 *  @details	- ReqToComplete in SEF handle for SB State Change notification
 */
void DeviceInfoDeleteSefHandle(struct SEFHandle_ *pSefHandle)
{
    int ret;

    SEF_ASSERT(pSefHandle != NULL);
    SEF_ASSERT(pSefHandle->pSefInfo != NULL);
    SEF_ASSERT(pSefHandle->ppVdHandle != NULL);
    SEF_ASSERT(pSefHandle->ppQosHandle != NULL);
    SEF_ASSERT(pSefHandle->pReq2CmpSbStateChange != NULL);

    ret = pthread_mutex_destroy(&pSefHandle->vd_mutex);
    if (ret != 0)
    {
        // logging only
        ULOG_ERROR("failed to pthread_mutex_destroy()\n");
    }
    ret = pthread_mutex_destroy(&pSefHandle->mutex);
    if (ret != 0)
    {
        // logging only
        ULOG_ERROR("failed to pthread_mutex_destroy()\n");
    }

    free(pSefHandle->nvmeUnit);

    free(pSefHandle->ppQosHandle);

    free(pSefHandle->ppVdHandle);

    free(pSefHandle->pSefInfo);

    free(pSefHandle->pReq2CmpSbStateChange);

    atomic_store(&seflibInfo.ppSefHandle[pSefHandle->deviceId], NULL);

    free(pSefHandle);
}

/**
 *  @brief	Retrieves SEF handle address
 *  @param	[in] index: index to SEF Unit
 *  @return	Pointer to SEF handle
 */
struct SEFHandle_ *DeviceInfoGetSefHandle(uint32_t index)
{
    SEF_ASSERT(index < seflibInfo.numSefUnits);
    return atomic_load(&seflibInfo.ppSefHandle[index]);
}

/**
 *  @brief	Creates VD handle
 *  @param	[in] pSefHandle: pointer to SEF handle
 *  @param	[in] vdId: Virtual Device ID
 *  @param	[out] ppVdHandle: pointer for returning VD handle
 *  @return	0: success; -1: memory allocation error
 *  @details	Allocates area for VD handle and associates it to parent table
 */
int DeviceInfoCreateVdHandle(struct SEFHandle_ *pSefHandle, uint16_t vdId, struct SEFVDHandle_ **ppVdHandle)
{
    struct SEFVDHandle_ *pVdHandle;

    SEF_ASSERT(pSefHandle != NULL);
    SEF_ASSERT(vdId <= (pSefHandle->pSefInfo->numChannels * pSefHandle->pSefInfo->numBanks));
    SEF_ASSERT(ppVdHandle != NULL);

    pVdHandle = (struct SEFVDHandle_ *)calloc(1, sizeof(*pVdHandle));
    if (pVdHandle == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        return -ENOMEM;
    }
    pVdHandle->vdId = vdId;
    pVdHandle->pSefHandle = pSefHandle;
    pSefHandle->ppVdHandle[vdId] = pVdHandle;

    *ppVdHandle = pVdHandle;

    return 0;
}

/**
 *  @brief	Deletes VD handle
 *  @param	[in] pVdHandle: pointer to VD handle
 *  @return	None
 *  @details	Releases area for VD handle
 */
void DeviceInfoDeleteVdHandle(struct SEFVDHandle_ *pVdHandle)
{
    SEF_ASSERT(pVdHandle != NULL);

    pVdHandle->pSefHandle->ppVdHandle[pVdHandle->vdId] = NULL;
    free(pVdHandle->pDieList);
    free(pVdHandle);
}

/**
 *  @brief	Retrieves VD handle
 *  @param	[in] pSefHandle: pointer to SEF handle
 *  @param	[in] vdId: Virtual Device ID
 *  @return	Pointer to VD handle
 */
struct SEFVDHandle_ *DeviceInfoGetVdHandle(struct SEFHandle_ *pSefHandle, uint16_t vdId)
{
    SEF_ASSERT(pSefHandle != NULL);
    SEF_ASSERT(vdId <= (pSefHandle->pSefInfo->numChannels * pSefHandle->pSefInfo->numBanks));
    return pSefHandle->ppVdHandle[vdId];
}

STATIC int SysReadFileQosInt(int devIdx, int qosIdx, const char *field)
{
    FILE *f;
    const char *devName = DeviceInfoGetDeviceName(devIdx);
    char path[64];
    int ret = -1;
    snprintf(path, sizeof(path), "/sys/class/sef/%s/%sn%d/%s", devName, devName, qosIdx, field);
    f = fopen(path, "r");
    if (!f)
    {
        return -1;
    }
    if (fscanf(f, "%d", &ret) != 1)
    {
        ret = -1;
    }
    fclose(f);
    return ret;
}

int DeviceIoQoSDevOpen(int unit, int qosId)
{
    DIR *pDir;
    struct dirent *pDirent;
    const char *devName = DeviceInfoGetDeviceName(unit);
    char path[64];
    int fd = -1;
    bool found = false;
    int qosIdx;
    int devIdx;

    snprintf(path, sizeof(path), "/sys/class/sef/%s", devName);

    pDir = opendir(path);
    // Search itself caused an error
    if (!pDir)
    {
        ULOG_ERROR("failed to opendir()\n");
        return -1;
    }

    while ((pDirent = readdir(pDir)) != NULL)
    {
        if (sscanf(pDirent->d_name, "sef%dn%d", &devIdx, &qosIdx) == 2)
        {
            if (SysReadFileQosInt(unit, qosIdx, "id") == qosId)
            {
                found = true;
                break;
            }
        }
    }

    if (!found)
    {
        ULOG_ERROR("unable to find qos device() %d\n", qosId);
        goto error_dir;
    }

    snprintf(path, sizeof(path), "/dev/sef%dn%d", devIdx, qosIdx);
    fd = DeviceIoOpenFile(path);

error_dir:
    closedir(pDir);
    return fd;
}

/**
 *  @brief	Creates QoSD handle
 *  @param	[in] pSefHandle: pointer to SEF handle
 *  @param	[in] vdId: Virtual Device ID
 *  @param	[in] qosId: QoS Domain ID
 *  @param	[out] ppQosHandle: pointer for returning QoSD handle
 *  @return	0: success; -1: memory allocation error
 *  @details	Allocates area for QoSD handle and associates it to parent table
 */
int DeviceInfoCreateQosHandle(struct SEFHandle_ *pSefHandle,
                              uint16_t vdId,
                              uint16_t qosId,
                              uint16_t nPID,
                              struct SEFQoSHandle_ **ppQosHandle)
{
    struct SEFQoSHandle_ *pQosHandle;
    size_t qosHandleSize;
    int i, fd, ret = -1;

    SEF_ASSERT(pSefHandle != NULL);
    SEF_ASSERT(qosId <= (pSefHandle->pSefInfo->maxQoSDomains + 1));
    SEF_ASSERT(ppQosHandle != NULL);

    fd = DeviceIoQoSDevOpen(pSefHandle->deviceId, qosId);
    if (fd < 0)
    {
        ULOG_ERROR("unable to open qos device(): %d\n", fd);
        return fd;
    }

    qosHandleSize = sizeof(*pQosHandle) + sizeof(pQosHandle->pidState[0]) * nPID;
    pQosHandle = (struct SEFQoSHandle_ *)calloc(1, qosHandleSize);
    if (pQosHandle == NULL)
    {
        ULOG_ERROR("failed to calloc()\n");
        goto error_fd;
    }
    pQosHandle->sbs_info = LHTcreate();
    pQosHandle->qosId = qosId;
    pQosHandle->pSefHandle = pSefHandle;
    pQosHandle->vdId = vdId;
    pSefHandle->ppQosHandle[qosId] = pQosHandle;
    pQosHandle->deviceFd = fd;
#ifdef USE_READ_CACHE
    pQosHandle->cache = LHTcreate();
#endif
    for (i = 0; i < nPID; i++)
    {
        FPWCInit(&pQosHandle->pidState[i].numProcessingFAR);
    }

    *ppQosHandle = pQosHandle;

    pthread_mutex_init(&pQosHandle->mutex, NULL);

    return 0;

error_fd:
    close(fd);

    return ret;
}

/**
 *  @brief	Deletes QoSD handle
 *  @param	[in] pQosHandle: pointer to QoSD handle
 *  @return	None
 *  @details	Releases area for QoSD handle
 */
void DeviceInfoDeleteQosHandle(struct SEFQoSHandle_ *pQosHandle)
{
    int ret;

    SEF_ASSERT(pQosHandle != NULL);

    ret = pthread_mutex_destroy(&pQosHandle->mutex);
    if (ret != 0)
    {
        // logging only
        ULOG_ERROR("failed to pthread_mutex_destroy()\n");
    }

    if (pQosHandle->deviceFd > 0)
    {
        close(pQosHandle->deviceFd);
    }
    pQosHandle->pSefHandle->ppQosHandle[pQosHandle->qosId] = NULL;

    DeviceInfoSbCleanup(pQosHandle);
    if (pQosHandle->sbs_info)
    {
        LHTdestroy(pQosHandle->sbs_info);
    }
#ifdef USE_READ_CACHE
    LHTdestroy(pQosHandle->cache);
#endif
    for (int i = 0; i < pQosHandle->numPlacementIds; i++)
    {
        ret = FPWCCleanup(&pQosHandle->pidState[i].numProcessingFAR);
        SEF_ASSERT(!ret);
    }

    free(pQosHandle);
}

/**
 *  @brief	Returns address of QoS Domain handle
 *  @param	[in] pSefHandle: pointer to SEF handle
 *  @param	[in] qosId: QoS Domain ID
 *  @return	Pointer to QoSD handle
 */
struct SEFQoSHandle_ *DeviceInfoGetQosHandle(struct SEFHandle_ *pSefHandle, uint16_t qosId)
{
    SEF_ASSERT(pSefHandle != NULL);
    SEF_ASSERT(qosId <= (pSefHandle->pSefInfo->maxQoSDomains + 1));
    return pSefHandle->ppQosHandle[qosId];
}

/**
 *  @brief	Returns address of system information
 *  @return	Pointer to system information
 */
struct SystemInfo *DeviceInfoGetSystemInfo(void)
{
    return &systemInfo;
}

/**
 *  @brief	Returns address of device filename stored in SEF Library information
 *  @param	[in] index: index to SEF Unit
 *  @return	Pointer to device filename
 */
char *DeviceInfoGetDeviceName(uint32_t index)
{
    SEF_ASSERT(index < seflibInfo.numSefUnits);
    return seflibInfo.ppDeviceName[index] ? seflibInfo.ppDeviceName[index] + 5 : NULL;
}

/**
 *  @brief	Returns address of device file path stored in SEF Library information
 *  @param	[in] index: index to SEF Unit
 *  @return	Pointer to device path
 */
char *DeviceInfoGetDevicePath(uint32_t index)
{
    SEF_ASSERT(index < seflibInfo.numSefUnits);
    return seflibInfo.ppDeviceName[index];
}

/**
 *  @brief	Returns number of SEF Units stored in SEF Library information
 *  @return	Number of SEF Units
 */
int DeviceInfoGetNumSefUnits(void)
{
    return seflibInfo.numSefUnits;
}

void DeviceInfoDeletePendingIf(struct SEFQoSHandle_ *pQosHandle, struct sefSlistNode *head)
{
    struct sefSlistNode *node = SefSlistPopAllIf(&pQosHandle->sbs_to_delete, head);
    while (node)
    {
        struct sefSlistNode *next = node->next;
        struct SbWriteInformation *sb_info = (void *)node;
        sem_destroy(&sb_info->closed);
        free(node);
        node = next;
    }
}

void DeviceInfoSbCleanup(struct SEFQoSHandle_ *pQosHandle)
{
    struct sefSlistNode *node = SefSlistPopAll(&pQosHandle->sbs_to_delete);
    while (node)
    {
        struct sefSlistNode *next = node->next;
        free(node);
        node = next;
    }
}

struct SbWriteInformation *DeviceInfoNewSbInfo(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    // Add newly
    struct SbWriteInformation *sb_info = calloc(1, sizeof(*sb_info));
    if (sb_info)
    {
        sb_info->sbId = sbId;
        sb_info->refcnt = 1;
        sem_init(&sb_info->closed, 0, 0);
        SefSlistInitialize(&sb_info->listHeadByOpen);
        SefSlistInitialize(&sb_info->listHeadByWriting);
    }
    return sb_info;
}

void DeviceInfoReleaseSb(struct SEFQoSHandle_ *pQosHandle, struct SbWriteInformation *sbInfo)
{
    // Release the reference to sbInfo
    struct sefSlistNode *head = SefSlistGetHead(&pQosHandle->sbs_to_delete);
    uint32_t src = atomic_fetch_sub(&sbInfo->refcnt, 1);
    int32_t arc = atomic_fetch_sub(&pQosHandle->sbs_ht_accesses, 1);
    SEF_ASSERT(arc > 0);

    // if sbInfo's refcount went to zero
    // push sbInfo on the pending delete list
    if (src == 1)
    {
        SefSlistPush(&pQosHandle->sbs_to_delete, &sbInfo->link);
    }

    // If ht's current user count went to zero
    // no one is in between a hash table get and
    // a sbInfo refcnt increment
    if (arc == 1)
    {
        DeviceInfoDeletePendingIf(pQosHandle, head);
    }
}

struct SbWriteInformation *DeviceInfoGetSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId, bool create)
{
    uint32_t key = DeviceInfoKeyFromSbId(sbId);
    struct SbWriteInformation *sb_info = NULL;
    atomic_fetch_add(&pQosHandle->sbs_ht_accesses, 1);
get:
    sb_info = LHTget(pQosHandle->sbs_info, key);
    if (sb_info)
    {
        uint32_t rc = atomic_load(&sb_info->refcnt);
        do
        {
            if (rc == 0)
            {
                break;
            }
        } while (!atomic_compare_exchange_weak(&sb_info->refcnt, &rc, rc + 1) && rc != 0);

        // If rc is 0 sb_info was dereferenced to zero between get from hashtable
        // and refcnt increment attempt

        if (rc == 0)
        {
            sb_info = NULL;
        }
    }
    if (!sb_info)
    {
        if (create)
        {
            sb_info = DeviceInfoNewSbInfo(pQosHandle, sbId);
            if (sb_info)
            {
                // Increment active users
                sb_info->refcnt++;
                struct SbWriteInformation *old_sb_info =
                    LHTputIfEmpty(pQosHandle->sbs_info, key, sb_info);
                if (old_sb_info)
                {
                    SEF_ASSERT(sb_info != LHTget(pQosHandle->sbs_info, key));
                    free(sb_info);
                    goto get;
                }
            }
        }

        if (!sb_info)
        {
            struct sefSlistNode *head = SefSlistGetHead(&pQosHandle->sbs_to_delete);
            int32_t arc = atomic_fetch_sub(&pQosHandle->sbs_ht_accesses, 1);
            SEF_ASSERT(arc > 0);
            if (arc == 1)
            {
                DeviceInfoDeletePendingIf(pQosHandle, head);
            }
        }
    }
    return sb_info;
}

void DeviceInfoAddSb(struct SEFQoSHandle_ *pQosHandle, struct SbWriteInformation *sbInfo)
{
    uint32_t key = DeviceInfoKeyFromSbId(sbInfo->sbId);

    sbInfo->refcnt = 1;
    // Increment active users
    atomic_fetch_add(&pQosHandle->sbs_ht_accesses, 1);
    struct SbWriteInformation *old_sb_info = LHTput(pQosHandle->sbs_info, key, sbInfo);
    SEF_ASSERT(!old_sb_info);

    // No new users can get a ref to old_sb_info at this point
    uint32_t src = old_sb_info ? atomic_fetch_sub(&old_sb_info->refcnt, 1) : 0;
    int32_t arc = atomic_fetch_sub(&pQosHandle->sbs_ht_accesses, 1);
    SEF_ASSERT(arc > 0);
    if (src == 1 && arc == 1)
    {
        // If both counters are zero, no one else can get hold ref
        // to old_sb_info, so it can be deleted
        free(old_sb_info);
    }
    else if (src == 1)
    {
        // Some was still accessing the ht, could be in between
        // a hashtable get and a sbInfo ref increment, so defer
        // deletion
        SefSlistPush(&pQosHandle->sbs_to_delete, &old_sb_info->link);
    }
}

void DeviceInfoDeleteSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    uint32_t key = DeviceInfoKeyFromSbId(sbId);
    struct SbWriteInformation *sbInfo;
    atomic_fetch_add(&pQosHandle->sbs_ht_accesses, 1);
    sbInfo = LHTdelete(pQosHandle->sbs_info, key);
    // No new possible pointers to sbInfo at this
    // point, but someone could still be between
    // getting it from the hashtable and incrementing
    // the reference
    if (sbInfo)
    {
        uint32_t src = atomic_fetch_sub(&sbInfo->refcnt, 1);
        int32_t arc = atomic_fetch_sub(&pQosHandle->sbs_ht_accesses, 1);
        SEF_ASSERT(arc > 0);
        if (src == 1 && arc == 1)
        {
            // If both counters are zero, no one else can get hold ref
            // to sbInfo, so it can be deleted
            free(sbInfo);
        }
        else if (src == 1)
        {
            // Some was still accessing the ht, could be in between
            // a hashtable get and an sbInfo ref increment, so defer
            // deletion
            SefSlistPush(&pQosHandle->sbs_to_delete, &sbInfo->link);
        }
    }
    else
    {
        struct sefSlistNode *head = SefSlistGetHead(&pQosHandle->sbs_to_delete);
        int32_t arc = atomic_fetch_sub(&pQosHandle->sbs_ht_accesses, 1);
        SEF_ASSERT(arc > 0);
        if (arc == 1)
        {
            DeviceInfoDeletePendingIf(pQosHandle, head);
        }
    }
}

/**
 *  @brief	Registers specified SB to management list as Opened
 *  @param	[in] pQosHandle: pointer to QoSD handle
 *  @param	[in] sbId: Super Block ID
 *  @retval	0: success (can be already registered)
 *  @retval	-1: memory allocation error
 *  @pre    QoSD handle Mutex must be locked
 */
int DeviceInfoAddOpenInfoForSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    struct SbWriteInformation *sb_info = LHTget(pQosHandle->sbs_info, DeviceInfoKeyFromSbId(sbId));

    SEF_ASSERT(pQosHandle != NULL);

    if (sb_info)
    {
        ULOG_INFORMATION("already exists sbId=%u\n", sbId);
        return 0;
    }

    sb_info = DeviceInfoNewSbInfo(pQosHandle, sbId);
    DeviceInfoAddSb(pQosHandle, sb_info);
    return 0;
}

/**
 *  @brief	Updates management area of specified SB
 *  @param	[in]  sb_info: pointer to sb write info
 *  @param	[in]  refAdj: how much to adjust the SB numIO count by (1 or -1)
 *  @param  [in]  numADUs: how much to add to writtenADUs
 *  @param	[out] pState: write status of SB
 */
void DeviceInfoUpdateWriteInfoForSb(struct SbWriteInformation *sb_info,
                                    int refAdj,
                                    int numADUs,
                                    enum SbWriteState *pState)
{
    uint32_t oldVal;
    uint32_t adus = 0;
    union SbInfoAtomicState newState;

    SEF_ASSERT(sb_info != NULL);

    if (numADUs)
    {
        adus = numADUs + atomic_fetch_add(&sb_info->writtenAdus, numADUs);
    }
    oldVal = atomic_load(&sb_info->state.val);
    do
    {
        newState.val = oldVal;
        SEF_ASSERT(refAdj > 0 || newState.numIO >= -refAdj);
        SEF_ASSERT(newState.state < kSbInfoNotifying);
        newState.numIO += refAdj;
        if (newState.numIO == 0 && newState.state == kSbInfoClosePending)
        {
            newState.state = kSbInfoNotifying;
        }
    } while (!atomic_compare_exchange_weak(&sb_info->state.val, &oldVal, newState.val));

    if (newState.numIO != 0)
    {
        *pState = kSbWriting;
    }
    else if (newState.state == kSbInfoNotifying)
    {
        *pState = kSbAllWritten;
    }
    else
    {
        *pState = kSbNoWriting;
    }

    ULOG_DEBUG("Update count for SB sbId=%u, state=%u, numIO=%u, refAdj=%u numADUs=%u adus=%u\n",
               sb_info->sbId, newState.state, newState.numIO, refAdj, numADUs, adus);
}

/**
 *  @brief	Updates SbWriteInformation information for SB to prepare for close notification
 *  @param	[in]  sb_info: pointer to sbWriteInfortion to move to closed/closing
 *  @param	[in]  pNotify: SBStateChange notification information
 *  @param  [out] pState: Write state, either kSbWriting, kSbAllWritten or kSbWriteClosed
 *                        If writing sb_info will be close pending, if all written,
 *                        notifying otherwise no action is to be taken.
 *  @retval	0: success
 */
int DeviceInfoUpdateSbClosed(struct SbWriteInformation *sb_info,
                             struct SEFQoSNotification *pNotify,
                             enum SbWriteState *pState)
{
    uint32_t oldVal;
    union SbInfoAtomicState newState;
    uint32_t sbId = sb_info->sbId;

#if CC_IGNORE_EXTRA_CLOSE
    // This is not ideal, there's a race with setting sb_info->notificationData
    // If this occurs, the asserts below will trigger.  FW needs to be fixed
    // or a more robust double close detection implemented
    if (sb_info->notificationData.type != 0)
    {
        ULOG_NOTICE("Extra SB Close for sbId=%u ignored\n", sbId);
        *pState = kSbWriteClosed;    // already closing/closed
        return 0;
    }
#endif
    SEF_ASSERT(sb_info != NULL);
    SEF_ASSERT(sb_info->notificationData.type == 0);
    SEF_ASSERT(pNotify->type == kSuperBlockStateChanged || pNotify->type == kUnreadableData);

    sb_info->notificationData = *pNotify;

    oldVal = atomic_load(&sb_info->state.val);
    do
    {
        newState.val = oldVal;
        SEF_ASSERT(newState.state == kSbInfoOpen);
        if (newState.numIO == 0)
        {
            newState.state = kSbInfoNotifying;
        }
        else
        {
            newState.state = kSbInfoClosePending;
        }
    } while (!atomic_compare_exchange_weak(&sb_info->state.val, &oldVal, newState.val));
    // if state became close pending, another thread may have closed it and
    // deleted it.  Currently sb_info is ref counted, but we could change that.
    sb_info = NULL;
    if (newState.numIO == 0)
    {
        *pState = kSbAllWritten;
    }
    else
    {
        *pState = kSbWriting;
    }
    ULOG_DEBUG("SB Closed sbId=%u as state=%u\n", sbId, newState.state);

    return 0;
}

/**
 *  @brief	Returns management area of specified SB
 *  @param	[in]  pQosHandle: pointer to QoSD handle
 *  @param	[in]  sbId: Super Block ID
 *  @return	Pointer to SB management area
 *  @pre    QoSD handle Mutex must be locked
 *  @note    Management area returned is not deleted from management area list
 */
struct SbWriteInformation *DeviceInfoGetSbWriteInfoList(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    SEF_ASSERT(0);
    return LHTget(pQosHandle->sbs_info, DeviceInfoKeyFromSbId(sbId));
}

/**
 *  @brief	Deletes management area of specified SB from list and returns it
 *  @param	[in]  pQosHandle: pointer to QoSD handle
 *  @param	[in]  sbId: Super Block ID
 *  @return	Pointer to SB management area
 *  @pre    QoSD handle Mutex must be locked
 */
struct SbWriteInformation *DeviceInfoPopSbWriteInfoList(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    SEF_ASSERT(pQosHandle != NULL);

    return LHTdelete(pQosHandle->sbs_info, DeviceInfoKeyFromSbId(sbId));
}

/**
 *  @brief	Add to suspended response list in management area of specified SB
 *  @param	[in]  pQosHandle: pointer to QoSD handle
 *  @param	[in]  sbId: Super Block ID
 *  @param	[in]  pReq2Cmp: request information and completion information
 *  @param	[in]  cause: cause of suspension
 *  @pre    QoSD handle Mutex must be locked
 *  @pre    Management area of target SB must be allocated
 */
STATIC void AddSuspendList(struct SEFQoSHandle_ *pQosHandle,
                           uint32_t sbId,
                           struct ReqToComplete *pReq2Cmp,
                           enum SuspendCause cause)
{
    struct SbWriteInformation *sb_info;

    SEF_ASSERT(pQosHandle != NULL);

    sb_info = DeviceInfoGetSb(pQosHandle, sbId, true);
    SEF_ASSERT(sb_info != NULL);
    SEF_ASSERT((cause == kSuspendedByOpen) || (cause == kSuspendedByWrite));
    switch (cause)
    {
        case kSuspendedByOpen:
            SefSlistPush(&sb_info->listHeadByOpen, &pReq2Cmp->io_request.link);
            break;
        case kSuspendedByWrite:
            SefSlistPush(&sb_info->listHeadByWriting, &pReq2Cmp->io_request.link);
            break;
        default:
            // not reachable
            SEF_ASSERT(0);
            break;
    }
    DeviceInfoReleaseSb(pQosHandle, sb_info);
    return;
}

/**
 *  @brief	Add to suspended-by-open response list in management area of specified SB
 *  @param	[in]  pQosHandle: pointer to QoSD handle
 *  @param	[in]  sbId: Super Block ID
 *  @param	[in]  pReq2Cmp: request information and completion information
 *  @pre    QoSD handle Mutex must be locked
 *  @pre    Management area of target SB must be allocated
 */
void DeviceInfoAddSuspendListByOpen(struct SEFQoSHandle_ *pQosHandle,
                                    uint32_t sbId,
                                    struct ReqToComplete *pReq2Cmp)
{
    AddSuspendList(pQosHandle, sbId, pReq2Cmp, kSuspendedByOpen);
}

/**
 *  @brief	Add to suspended-by-writing response list in management area of specified SB
 *  @param	[in]  pQosHandle: pointer to QoSD handle
 *  @param	[in]  sbId: Super Block ID
 *  @param	[in]  pReq2Cmp: request information and completion information
 *  @pre    QoSD handle Mutex must be locked
 *  @pre    Management area of target SB must be allocated
 */
void DeviceInfoAddSuspendListByWriting(struct SEFQoSHandle_ *pQosHandle,
                                       uint32_t sbId,
                                       struct ReqToComplete *pReq2Cmp)
{
    AddSuspendList(pQosHandle, sbId, pReq2Cmp, kSuspendedByWrite);
}

/**
 *  @brief	Check whether specified SB is opened
 *  @param	[in] pQosHandle: pointer to QoSD handle
 *  @param	[in] sbId: Super Block ID
 *  @retval	true:  opened
 *  @retval	false: not opened
 *  @pre    QoSD handle Mutex must be locked
 */
bool DeviceInfoIsOpenedSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    struct SbWriteInformation *sb_info;
    SEF_ASSERT(pQosHandle != NULL);
    bool open = false;

    sb_info = DeviceInfoGetSb(pQosHandle, sbId, false);

    if (sb_info)
    {
        union SbInfoAtomicState state;
        state.val = atomic_load(&sb_info->state.val);
        open = (state.state == kSbInfoOpen);
        DeviceInfoReleaseSb(pQosHandle, sb_info);
    }
    return open;
}

/**
 *  @brief	Check whether specified SB is closed
 *
 *  There is no open SB_INFO, or if there is, the notify is about to be called
 *  or has been called and the SB_INFO in the process of being deleted.
 *
 *  @param	[in] pQosHandle: pointer to QoSD handle
 *  @param	[in] sbId: Super Block ID
 *  @retval	true:  opened
 *  @retval	false: not opened
 *  @pre    QoSD handle Mutex must be locked
 */
bool DeviceInfoIsClosedSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    struct SbWriteInformation *sb_info;
    SEF_ASSERT(pQosHandle != NULL);
    bool closed = true;

    sb_info = DeviceInfoGetSb(pQosHandle, sbId, false);

    if (sb_info)
    {
        union SbInfoAtomicState state;

        state.val = atomic_load(&sb_info->state.val);
        closed = (state.state == kSbInfoNotified || state.state == kSbInfoNotifying);
        DeviceInfoReleaseSb(pQosHandle, sb_info);
    }
    return closed;
}

/**
 *  @brief	Check whether specified SB is closing
 *
 *  This is not the same as !DeviceInfoIsOpenedSb() since both return false
 *  when sb is unknown (sb_info[sbid] is NULL).
 *
 *  @param	[in] pQosHandle: pointer to QoSD handle
 *  @param	[in] sbId: Super Block ID
 *  @retval	true:  opened
 *  @retval	false: not opened
 *  @pre    QoSD handle Mutex must be locked
 */
bool DeviceInfoIsClosingSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    struct SbWriteInformation *sb_info;
    SEF_ASSERT(pQosHandle != NULL);
    bool closing = false;

    sb_info = DeviceInfoGetSb(pQosHandle, sbId, false);

    if (sb_info)
    {
        union SbInfoAtomicState state;

        state.val = atomic_load(&sb_info->state.val);
        closing = (state.state != kSbInfoOpen && state.state != kSbInfoNotified);
        DeviceInfoReleaseSb(pQosHandle, sb_info);
    }
    return closing;
}

/**
 *  @brief	Check whether specified SB is being written
 *  @param	[in] pQosHandle: pointer to QoSD handle
 *  @param	[in] sbId: Super Block ID
 *  @retval	true:  being written
 *  @retval	false: not being written
 *  @pre    QoSD handle Mutex must be locked
 */
bool DeviceInfoIsWritingSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId)
{
    struct SbWriteInformation *sb_info;
    SEF_ASSERT(pQosHandle != NULL);
    union SbInfoAtomicState state = {};

    sb_info = DeviceInfoGetSb(pQosHandle, sbId, false);

    if (sb_info)
    {
        state.val = atomic_load(&sb_info->state.val);
        DeviceInfoReleaseSb(pQosHandle, sb_info);
    }
    return (sb_info && state.numIO);
}
