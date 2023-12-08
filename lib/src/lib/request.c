/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * request.c
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

#define MODULE_NAME_SELECT "SEFREQ"

#include "request.h"

#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "sef_api.h"
#include "sef_cmd.h"

extern int DeviceToSefError(int err);

#define DEVICE_FILE_PATH_MAX \
    (5 + 265)    //!< Maximum path length of device file (/dev/sef%d)
                 //!< 5:   device ("/dev/")
                 //!< 256: file (number of array elements of struct dirent.d_name[])

#define WAIT_COMPLETE_CHECK_INTERVAL_SEC \
    (1)    //!< Interval between request completion checks: to be specified to sleep
#define WAIT_COMPLETE_LOG_INTERVAL_COUNT \
    (5)    //!< Interval between request completion wait log outputs: number of sleeps

/**
 *  @brief	Content of SQ information log output
 *  @param	[in] pReq2Cmp: request information and completion information
 */
#define SQINFO_LOG(pReq2Cmp)                                                          \
    "%s:0x%x,info=0x%lx,sync=%d,SQ[01]=%08x,SQ[10]=%08x %08x %08x %08x %08x %08x\n",  \
        GetCmdName((pReq2Cmp)->type, &(pReq2Cmp)->submit), (pReq2Cmp)->submit.opcode, \
        (uint64_t)(pReq2Cmp), (pReq2Cmp)->bSync, (pReq2Cmp)->submit.nsid,             \
        (pReq2Cmp)->submit.cdw10, (pReq2Cmp)->submit.cdw11, (pReq2Cmp)->submit.cdw12, \
        (pReq2Cmp)->submit.cdw13, (pReq2Cmp)->submit.cdw14, (pReq2Cmp)->submit.cdw15

/**
 *  @brief	Content of CQ information log output
 *  @param	[in] pReq2Cmp: request information and completion information
 */
#define CQINFO_LOG(pReq2Cmp)                                                                 \
    "%s:0x%x,info=0x%lx,status=0x%x,CQ[00]=%08x,CQ[01]=%08x\n",                              \
        GetCmdName((pReq2Cmp)->type, &(pReq2Cmp)->submit), (pReq2Cmp)->submit.opcode,        \
        (uint64_t)(pReq2Cmp), (pReq2Cmp)->status, (uint32_t)(pReq2Cmp)->result & 0xFFFFFFFF, \
        (uint32_t)(((pReq2Cmp)->result >> 32) & 0xFFFFFFFFUL)

/**
 *  @brief	Sets completion information in IOCB
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    Sets ERROR and DONE information in IOCB; calls complete_func.
 *  @note    If pIocb in pReq2Cmp is NULL, do nothing
 */
void CompleteIocbProc(struct ReqToComplete *pReq2Cmp)
{
    if (pReq2Cmp->pIocb == NULL)
    {
        // If IOCB is NULL, quit without doing anything
        return;
    }

    // first error received is the error used.  For the case where there is a
    // race among errors, last writer wins.
    if (pReq2Cmp->status != 0 && pReq2Cmp->pIocb->common.status.error == 0)
    {
        pReq2Cmp->pIocb->common.status.error = DeviceToSefError(pReq2Cmp->status);
    }

    int32_t rc = atomic_fetch_sub((atomic_int_least32_t *)&pReq2Cmp->pIocb->common.reserved, 1);
    SEF_ASSERT(rc > 0);
    if (rc == 1)
    {
        void (*complete_func)(struct SEFCommonIOCB *) = pReq2Cmp->pIocb->common.complete_func;
        pReq2Cmp->pIocb->common.flags |= kSefIoFlagDone;

        if (complete_func != NULL)
        {
            complete_func(&pReq2Cmp->pIocb->common);
        }
    }
}

/**
 *  @brief	Processes completion from driver
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @details	If bSync of Req2Cmp is true, semaphore is released.
 *  @details	If async, pSyncSemaphore is assumed to be allocated and its memory is released.
 *  @details	If async, memory for pReq2Cmp is released.
 */
void Complete(struct ReqToComplete *pReq2Cmp)
{
    SEF_ASSERT(pReq2Cmp != NULL);
    SEF_ASSERT(pReq2Cmp->pSyncSemaphore != NULL);

    ULOG_CQINFO(CQINFO_LOG(pReq2Cmp));
    if (pReq2Cmp->status == -EBUSY || pReq2Cmp->status == -EWOULDBLOCK)
    {
        ULOG_DEBUG("EBUSY received\n");
        int ret = RequestResubmit(pReq2Cmp);

        if (ret == 0)
        {
            return;
        }
        pReq2Cmp->status = ret;
    }
    else if (pReq2Cmp->status == -ECANCELED)
    {
        return;
    }

    if (pReq2Cmp->status != 0)
    {
        ULOG_ERROR(SQINFO_LOG(pReq2Cmp));
        ULOG_ERROR(CQINFO_LOG(pReq2Cmp));
    }
    else
    {
        // Suspend check of completion is done only for normal result
        if (pReq2Cmp->CheckSuspendFunc != NULL)
        {
            if (pReq2Cmp->CheckSuspendFunc(pReq2Cmp))
            {
                return;
            }
        }
    }

    /*
     * If result check function is registered, call it
     * CheckResult sets DONE and ERROR information in IOCB and calls user completion callback
     */
    if (pReq2Cmp->CheckResultExt != NULL)
    {
        pReq2Cmp->CheckResultExt(pReq2Cmp);
    }

    // Increment semaphore for sync operation
    if (pReq2Cmp->bSync)
    {
        int tmp = sem_post(pReq2Cmp->pSyncSemaphore);
        if (tmp != 0)
        {
            ULOG_ERROR("failed to sem_post()\n");
        }
        // Release RefInfo here if async
    }
    else
    {
        // Release memory
        free(pReq2Cmp->pSyncSemaphore);
        free(pReq2Cmp);
    }
}

/**
 *  @brief	Issues request to driver
 *  @param	[in] pSefHandle: SEF handle (for retrieving device node)
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	0: success; otherwise: command issue failed
 *  @note    If bSync of Req2Cmp is true, pSyncSemaphore of Req2Cmp is used to wait for completion
 */
int Request(int fd, struct ReqToComplete *pReq2Cmp)
{
    int ret;
    bool bSync = pReq2Cmp->bSync;
    struct ReqToComplete *submit[1] = {pReq2Cmp};

    SEF_ASSERT(pReq2Cmp->pSyncSemaphore != NULL);

    pReq2Cmp->fd = fd;
    pReq2Cmp->fused = NULL;
    ULOG_SQINFO(SQINFO_LOG(pReq2Cmp));
    for (;;)
    {
        ret = DeviceIoRequest(fd, pReq2Cmp->type, submit, 1, bSync);
        if (ret != -EBUSY)
        {
            break;
        }
        ULOG_DEBUG("retrying DeviceIoRequest\n");
        usleep(1000);
    }

    if (ret >= 0 && bSync && pReq2Cmp->CommandComplete)
    {
        if (pReq2Cmp->retryCnt)
        {
            ULOG_NOTICE("Command %s retried %d times\n",
                        GetCmdName(pReq2Cmp->type, &pReq2Cmp->submit), pReq2Cmp->retryCnt);
        }
        pReq2Cmp->CommandComplete(pReq2Cmp);
    }

    if (ret >= 0 && bSync)
    {
        int tmp = sem_wait(pReq2Cmp->pSyncSemaphore);
        if (tmp != 0)
        {
            ULOG_ERROR("failed to sem_wait()\n");
        }
    }
    return ret;
}

/**
 *  @brief	Issues request to driver (for Fused commands)
 *  @param	[in] pSefHandle: SEF handle (for retrieving device node)
 *  @param	[in] pReq2Cmp1st: (Fused 1st) request information and completion information
 *  @param	[in] pReq2Cmp2nd: (Fused 2nd) request information and completion information
 *  @return	0: success; otherwise: command issue failed
 */
int RequestFusedWrite(int fd, struct ReqToComplete *pReq2Cmp1st, struct ReqToComplete *pReq2Cmp2nd)
{
    int ret;
    struct ReqToComplete *submits[2] = {pReq2Cmp1st, pReq2Cmp2nd};
    enum IoctlCommandType type = pReq2Cmp1st->type;

    SEF_ASSERT(pReq2Cmp1st->pSyncSemaphore != NULL);
    SEF_ASSERT(pReq2Cmp2nd->pSyncSemaphore != NULL);
    SEF_ASSERT(pReq2Cmp1st->type == kIoctlIoFusedCommand);
    //    SEF_ASSERT(pReq2Cmp2nd->type == kIoctlIoFusedCommandSecond);

    pReq2Cmp1st->fd = fd;
    pReq2Cmp2nd->fd = fd;
    pReq2Cmp1st->fused = pReq2Cmp2nd;
    pReq2Cmp2nd->fused = NULL;
    pReq2Cmp1st->status = 0;
    pReq2Cmp2nd->status = 0;

    ULOG_SQINFO(SQINFO_LOG(pReq2Cmp1st));
    ULOG_SQINFO(SQINFO_LOG(pReq2Cmp2nd));

    for (;;)
    {
        ret = DeviceIoRequest(fd, type, submits, 2, false);
        if (ret != -EBUSY)
        {
            break;
        }
        ULOG_DEBUG("retrying fused DeviceIoRequest\n");
        usleep(1000);
    }
    return ret;
}

/**
 *  @brief	Reissues request to driver
 *  @param	[in] pReq2Cmp: Request information and completion information
 *  @return	0: success; otherwise: command issue failed
 */
int RequestResubmit(struct ReqToComplete *pReq2Cmp)
{
    int ret;
    int fd = pReq2Cmp->fd;

    ULOG_WTCMD_INF("Retrying...\n");

    if (pReq2Cmp->retryCnt++ > MAX_REQ_RETRY)
    {
        return -ENODEV;
    }

    // When CC_USE_FUSED is 0 fused cmds are still of type
    // kIoctlIoFusedCommand so the fused retry logic is used.
    // if the code is changed to submit as type kIoctlIoCommand,
    // this if can be removed.
    //
    // When not fused both 1st and 2nd of the pair fail with
    // a -EWOULDBLOCK error (so the 2nd command resubmits) instead of
    // -ECANCELED (so it's ignored).  This skip submitting the 2nd and
    // lets it be submitted with the 1st as a pair to keep ordering.  Note
    // the 1st and 2nd of a pair can complete in either order.  This makes
    // a big assumption that if one fails, they both will.  The driver
    // will fail both if one fails while preparing a request.
#if HONOR_FUSED_ON_RESUBMIT
    if (pReq2Cmp->type == kIoctlIoFusedCommand && !pReq2Cmp->fused)
    {
        ULOG_DEBUG("Not issuing 2nd fused %p\n", pReq2Cmp);
        return 0;
    }
    usleep(1000);
    if (pReq2Cmp->fused)
    {
        ret = RequestFusedWrite(fd, pReq2Cmp, pReq2Cmp->fused);
    }
    else
    {
        ret = Request(fd, pReq2Cmp);
    }
#else
    usleep(1000);
    if (pReq2Cmp->fused || pReq2Cmp->type == kIoctlIoFusedCommand)
    {
        pReq2Cmp->fused = NULL;
        pReq2Cmp->type = kIoctlIoCommand;
    }
    ret = Request(fd, pReq2Cmp);
#endif

    return ret;
}

/**
 *  @brief	Returns size of buffer area that IOV structure points to
 *  @param	[in] pIov: target IOV structures
 *  @param	[in] iovCnt: elements of IOV structures
 *  @param	[in] iovOffs: offset of IOV structures
 *  @return	Size of buffer area
 */
size_t GetIovBuffLen(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs)
{
    size_t iovLen = 0;
    for (uint16_t i = 0; i < iovCnt; i++)
    {
        iovLen += pIov[i].iov_len;
    }
    if (iovLen <= iovOffs)
    {
        iovLen = 0;
    }
    else
    {
        iovLen -= iovOffs;
    }
    return iovLen;
}

/**
 *  @brief	Returns whether buffer area that IOV structure points to is smaller than specified size
 *  @param	[in] pIov: target IOV structures
 *  @param	[in] iovCnt: elements of IOV structures
 *  @param	[in] iovOffs: offset of IOV structures
 *  @param	[in] len: size
 *  @return	true: smaller than size, false: equal to size or larger
 */
bool IsShortageIovLen(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, size_t len)
{
    size_t iovLen;
    iovLen = GetIovBuffLen(pIov, iovCnt, iovOffs);
    if ((iovLen < len) || (iovLen == 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 *  @brief	Retrieves position of IOV structure that specified offset points to
 *  @param	[in] pIov: target IOV structures
 *  @param	[in] iovCnt: elements of IOV structures
 *  @param	[in] iovOffs: offset of IOV structures
 *  @param	[out] pIndex: index of offset position
 *  @param	[out] pOffs: offset from base address that index of offset position points to
 *  @return	None
 *  @attention    Errors such as iovOffs exceeding pIov buffer area are not checked; must be checked in advance
 */
void GetIovOffsPosition(
    const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, uint16_t *pIndex, size_t *pOffs)
{
    uint16_t currentIdx = 0;
    size_t currentOffs = 0;
    size_t previousOffs = 0;

    for (uint16_t i = 0; i < iovCnt; i++)
    {
        if (currentOffs > iovOffs)
        {
            break;
        }
        previousOffs = currentOffs;
        currentOffs += pIov[currentIdx].iov_len;
        currentIdx++;
    }
    *pIndex = currentIdx - 1;
    *pOffs = iovOffs - previousOffs;
}

/**
 *  @brief	Copies data from non-contiguous areas pointed by IOV structure to a contigous memory area
 *  @param	[in] pIov: copy source IOV structures
 *  @param	[in] iovCnt: elements of IOV structures
 *  @param	[in] iovOffs: offset of IOV structures
 *  @param	[in] copyLen: copy length
 *  @param	[out] pBuff: copy destination memory address
 *  @return	None
 *  @attention    Errors such as copyLen exceeding pIov buffer area are not checked; must be checked in advance
 */
void CopyIovToBuff(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, size_t copyLen, void *pBuff)
{
    uint16_t iovTopIndex;
    size_t iovCopyOffs;
    size_t remainCopySize;
    uint8_t *pDst = (uint8_t *)pBuff;

    GetIovOffsPosition(pIov, iovCnt, iovOffs, &iovTopIndex, &iovCopyOffs);
    remainCopySize = copyLen;

    for (uint16_t iovIndex = iovTopIndex; (iovIndex < iovCnt) && (remainCopySize > 0); iovIndex++)
    {
        char *addr = pIov[iovIndex].iov_base;
        size_t copySize = pIov[iovIndex].iov_len - iovCopyOffs;
        if (remainCopySize < copySize)
        {
            copySize = remainCopySize;
        }

        memcpy(pDst, addr + iovCopyOffs, copySize);
        pDst += copySize;
        remainCopySize -= copySize;
        iovCopyOffs = 0;
    }
    SEF_ASSERT(remainCopySize == 0);
}

/**
 *  @brief	Copies data from a contigous memory area to non-contigous regions pointed by IOV structure
 *  @param	[out] pIov: copy destination IOV structure
 *  @param	[in] iovCnt: elements of IOV structures
 *  @param	[in] iovOffs: offset of IOV structures
 *  @param	[in] copyLen: copy length
 *  @param	[in] pBuff: copy source memory address
 *  @return	None
 *  @attention    User must release returned memory area using free()
 *  @attention    Errors such as copyLen exceeding pIov buffer area are not checked; must be checked in advance
 */
void CopyIovFromBuff(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, size_t copyLen, void *pBuff)
{
    uint16_t iovTopIndex;
    size_t iovCopyOffs;
    size_t remainCopySize;
    uint8_t *pSrc = (uint8_t *)pBuff;

    GetIovOffsPosition(pIov, iovCnt, iovOffs, &iovTopIndex, &iovCopyOffs);
    remainCopySize = copyLen;

    for (uint16_t iovIndex = iovTopIndex; (iovIndex < iovCnt) && (remainCopySize > 0); iovIndex++)
    {
        char *addr = pIov[iovIndex].iov_base;
        size_t copySize = pIov[iovIndex].iov_len - iovCopyOffs;
        if (remainCopySize < copySize)
        {
            copySize = remainCopySize;
        }

        memcpy(addr + iovCopyOffs, pSrc, copySize);

        pSrc += copySize;
        remainCopySize -= copySize;
        iovCopyOffs = 0;
    }
    SEF_ASSERT(remainCopySize == 0);
}

int CloneIov(const struct iovec *src, int32_t iovcnt, size_t offset, size_t dataSize, struct iovec **pdest)
{
    int i, count = 0;
    if (!iovcnt)
    {
        *pdest = NULL;
        return 0;
    }
    struct iovec *dest = calloc(iovcnt, sizeof(struct iovec));
    if (!dest)
    {
        return -ENOMEM;
    }
    for (i = 0; i < iovcnt; ++i)
    {
        if (!dataSize)
        {
            break;
        }
        size_t len = src[i].iov_len;
        uint8_t *base = src[i].iov_base;
        if (len < offset)
        {
            offset -= len;
            continue;
        }
        if (offset)
        {
            len -= offset;
            base += offset;
            offset = 0;
        }
        if (dataSize < len)
        {
            len = dataSize;
        }

        dest[count].iov_base = base;
        dest[count].iov_len = len;
        ++count;
        dataSize -= len;
    }

    if (dataSize)
    {
        free(dest);
        return -EINVAL;
    }

    *pdest = dest;

    return count;
}

#if CC_EMU_VALIDATION
// If emulated, check support
// "nqn.2019-08.org.qemu:QEMU KIC SEF V1.11.0.sn120197"
// param is nvme<id>, not sef unit
// return true if not qemu or is compatible
STATIC int IsCompatQemuDev(int id)
{
    char path[128];
    char subnqn[257] = "";
    unsigned int mjv, mnv, ifv;
    int numParsed;
    FILE *fh;

    sprintf(path, "/sys/class/nvme/nvme%d/subsysnqn", id);
    fh = fopen(path, "r");
    if (fh)
    {
        char *p;
        if (!fgets(subnqn, sizeof(subnqn), fh))
        {
            subnqn[0] = '\0';
        }
        fclose(fh);
        p = strchr(subnqn, '\n');
        if (p)
        {
            *p = '\0';
        }
    }

    // Likely emulated that linux is making up the subsysnqn for Kioxia
    if (strstr(subnqn, "nqn.2014.08.org.nvmexpress:1e0f1e0f") == subnqn)
    {
        // it's a SEF device that isn't publishing it's own subnqn
        ULOG_NOTICE("SEF device nqn is not org.qemu but is linux generated: '%s'\n", subnqn);
        return 1;
    }
    numParsed = sscanf(subnqn, "nqn.2019-08.org.qemu:QEMU KIC SEF V%u.%u.%u.sn", &mjv, &mnv, &ifv);
    if (numParsed == 3)    // it's the emulated device
    {
        if (mjv != SEF_CS_MJ_VER || mnv != SEF_CS_MN_VER || ifv != SEF_QEMU_IF_VER)
        {
            ULOG_ERROR("SEF device nqn: '%s' is not SEF V%d.%d.%d\n", subnqn, SEF_CS_MJ_VER,
                       SEF_CS_MN_VER, SEF_QEMU_IF_VER);
            return 0;
        }
    }
    return 1;
}
#endif

/**
 *  @brief	Filter function for scandir() (detects /dev/sef%d)
 *  @param	[out] pDirent
 *  @return	0: does not apply; 1: applies
 */
STATIC int ScanSefUnitFilter(const struct dirent *pDirent)
{
    int id;

    if (pDirent->d_name[0] == '.')
    {
        return 0;
    }

    if (sscanf(pDirent->d_name, "sef%d", &id) == 1)
    {
#if CC_EMU_VALIDATION
        return IsCompatQemuDev(id);
#else
        return 1;
#endif
    }
    return 0;
}

/**
 *  @brief	Retrieves path list of SEF devices that SEF driver recognizes (/dev/sef%d)
 *  @param	[out] pNum: number of devices (list entries)
 *  @param	[out] ppplist: SEF device list (two-dimensional array of [pNum][DEVICE_FILE_PATH_MAX])
 *  @retval	0: success
 *  @retval	-1: memory allocation error
 *  @details	This function allocates memory area for SEF device list and releases it in case of an error.
 *  @note    User must release memory area for SEF device list using free().
 */
int GetSefUnitList(int *pNum, char ***pppList)
{
    int ret = -1;
    struct dirent **ppCtrls;
    char **ppList = NULL;
    int sefUnitNum = 0;
    int detectNum;

    SEF_ASSERT(pppList != NULL);

    detectNum = scandir("/sys/class/sef", &ppCtrls, ScanSefUnitFilter, alphasort);
    // Search itself caused an error
    if (detectNum < 0)
    {
        ULOG_NOTICE("No sef devices found %d\n", detectNum);
        ret = 0;
        goto Exit;
    }

    // Searched but no device attached (handled as a normal case)
    if (detectNum == 0)
    {
        ret = 0;
        goto FreeCtrls;
    }

    ppList = (char **)calloc(detectNum, sizeof(*ppList));
    if (!ppList)
    {
        ULOG_ERROR("failed to calloc()\n");
        goto FreeCtrlsList;
    }

    // Allocate memory area for each PATH, copy and register to list
    for (int i = 0; i < detectNum; i++)
    {
        char pTmp[DEVICE_FILE_PATH_MAX];
        snprintf(pTmp, DEVICE_FILE_PATH_MAX, "/dev/%s", ppCtrls[i]->d_name);
        ppList[i] = strdup(pTmp);
        if (!ppList[i])
        {
            ULOG_ERROR("failed to strdup() str=%s\n", pTmp);
            goto FreeCtrlsList;
        }
        sefUnitNum++;
    }

    ret = 0;

FreeCtrlsList:
    for (int i = 0; i < detectNum; i++)
    {
        free(ppCtrls[i]);
    }
FreeCtrls:
    free(ppCtrls);

    // Memory allocation failed during copy of each device's PATH
    if (sefUnitNum < detectNum)
    {
        for (int i = 0; i < sefUnitNum; i++)
        {
            free(ppList[i]);
        }
        if (ppList != NULL)
        {
            free(ppList);
            ppList = NULL;
        }
    }
    else
    {
        ULOG_INFORMATION("detected sef unit num=%d\n", detectNum);
    }
Exit:
    *pNum = sefUnitNum;
    *pppList = ppList;
    return ret;
}

/**
 *  @brief	Allocates buffer area for device I/O
 *  @param	[in] size: size of buffer area
 *  @param	[in] bZero: whether to zero-clear buffer area (does not zero-clear if false)
 *  @return	Non-NULL: address of buffer area; NULL: allocation failed
 *  @note    Memory area is aligned to alignment information in SystemInfo
 *  @attention    User must execute free() if non-NULL is returned
 */
void *AllocateDeviceBuffer(uint32_t size, bool bZero)
{
    int ret;
    void *pBuff;
    struct SystemInfo *pSystemInfo;

    pSystemInfo = DeviceInfoGetSystemInfo();
    ret = posix_memalign(&pBuff, pSystemInfo->transferDataAlign, size);
    if (ret != 0)
    {
        ULOG_ERROR("failed to posix_memalign() ret=%d, align=%u, size=%u\n", ret,
                   pSystemInfo->transferDataAlign, size);
        return NULL;
    }
    else
    {
        if (bZero)
        {
            memset(pBuff, 0, size);
        }
        return pBuff;
    }
}

/**
 *  @brief	Allocates all memory areas for information used for issuing requests (without data)
 *  @param	[out] ppReq2Cmp: memory area for request information and completion information
 *  @param	[out] ppIocb: memory area for IOCB
 *  @param	[out] ppParam: memory area of arbitrary size (as large as paramSize)
 *  @param	[in] paramSize: size of memory to allocate to ppParam
 *  @param	[out] ppSem: memory area for semaphore
 *  @retval	0: success
 *  @retval	-1: memory allocation error
 *  @details	If allocation fails, all memory areas are released
 *  @note    NULL can be specified to skip allocation
 *  @note    All allocated memory areas are zero-cleared
 */
int AllocateReqInfo(struct ReqToComplete **ppReq2Cmp,
                    union RequestIocb **ppIocb,
                    void **ppParam,
                    int paramSize,
                    sem_t **ppSem)
{
    if (ppReq2Cmp)
    {
        *ppReq2Cmp = (struct ReqToComplete *)calloc(1, sizeof(**ppReq2Cmp));
        if (*ppReq2Cmp == NULL)
        {
            ULOG_ERROR("failed to calloc()\n");
            return -1;
        }
    }
    if (ppIocb)
    {
        *ppIocb = (union RequestIocb *)calloc(1, sizeof(**ppIocb));
        if (*ppIocb == NULL)
        {
            ULOG_ERROR("failed to calloc()\n");
            goto FreeReq2Cmp;
        }
    }
    if (ppParam)
    {
        *ppParam = calloc(1, paramSize);
        if (*ppParam == NULL)
        {
            ULOG_ERROR("failed to calloc()\n");
            goto FreeIocb;
        }
    }
    if (ppSem)
    {
        *ppSem = (sem_t *)calloc(1, sizeof(**ppSem));
        if (*ppSem == NULL)
        {
            ULOG_ERROR("failed to calloc()\n");
            goto FreeParam;
        }
    }

    return 0;

FreeParam:
    if (ppParam)
    {
        free(*ppParam);
    }
FreeIocb:
    if (ppIocb)
    {
        free(*ppIocb);
    }
FreeReq2Cmp:
    if (ppReq2Cmp)
    {
        free(*ppReq2Cmp);
    }

    return -1;
}

/**
 *  @brief	Allocates all memory areas for information used for issuing requests (with data)
 *  @param	[out] ppReq2Cmp: memory area for request information and completion information
 *  @param	[out] ppIocb: memory area for IOCB
 *  @param	[out] ppParam: memory area of arbitrary size (as large as paramSize)
 *  @param	[in] paramSize: size of memory to allocate to ppParam
 *  @param	[out] ppSem: memory area for semaphore
 *  @param	[out] ppDevBuff: memory area for device buffer
 *  @param	[in] buffSize: size of memory to allocate to ppDevBuff (must match buffCnt)
 *  @retval	0: success
 *  @retval	-1: memory allocation error
 *  @details	If allocation of memory area for device buffer fails, all other memory areas are released
 *  @note    NULL can be specified to skip allocation
 *  @note    Allocated memory areas other than that for device buffer are zero-cleared
 */
int AllocateReqInfoExt(struct ReqToComplete **ppReq2Cmp,
                       union RequestIocb **ppIocb,
                       void **ppParam,
                       int paramSize,
                       sem_t **ppSem,
                       void **ppDevBuff,
                       int buffSize)
{
    int ret;

    ret = AllocateReqInfo(ppReq2Cmp, ppIocb, ppParam, paramSize, ppSem);
    if (ret != 0)
    {
        return -1;
    }

    if (ppDevBuff)
    {
        *ppDevBuff = AllocateDeviceBuffer(buffSize, false);
        if (*ppDevBuff == NULL)
        {
            ULOG_ERROR("failed to AllocateDeviceBuffer()\n");
            goto FreeReqInfo;
        }
    }

    return 0;

FreeReqInfo:
    if (ppSem)
    {
        free(*ppSem);
    }
    if (ppParam)
    {
        free(*ppParam);
    }
    if (ppIocb)
    {
        free(*ppIocb);
    }
    if (ppReq2Cmp)
    {
        free(*ppReq2Cmp);
    }

    return -1;
}

/**
 *  @brief	Waits all uncompleted requests in the QoSD to complete
 *  @param	[in] pQosd: pQosd handle
 *  @return	false: timed out; true: completions received
 *  @pre    QoSD handle Mutex must be locked (lock is kept even if function aborts)
 */
void WaitAllRequestsComplete(struct SEFQoSHandle_ *pQosd)
{
    int logWaitCount = 0;
    uint32_t numRequest;
    uint32_t numNlw;
    uint32_t nowNumProcessing;     //!< Current NumRequest + NumNlw
    uint32_t prevNumProcessing;    //!< Last NumRequest + NumNlw

    numRequest = atomic_load(&pQosd->numProcessingRequests);
    numNlw = atomic_load(&pQosd->numProcessingNlw);
    nowNumProcessing = numRequest + numNlw;
    prevNumProcessing = nowNumProcessing;

    // If request other than NamelessWrite remains, output log
    if (numRequest != 0)
    {
        ULOG_NOTICE("not completed requests other than NamelessWrite remains now [request=%u]\n",
                    numRequest);
    }

    // Wait uncompleted requests to complete
    while (nowNumProcessing)
    {
        // Unlock Mutex to accept completion
        pthread_mutex_unlock(&pQosd->mutex);

        // If number of current requests exceeds that of last requests, output log
        if (prevNumProcessing < nowNumProcessing)
        {
            ULOG_NOTICE("not completed requests increased [prev=%u, now=%u]\n", prevNumProcessing,
                        nowNumProcessing);
        }

        // Output log with a regular interval
        if (logWaitCount > WAIT_COMPLETE_LOG_INTERVAL_COUNT)
        {
            ULOG_NOTICE(
                "wait for requests to be completed... [qosd id=%u, remain : nlw=%u other=%u]\n",
                pQosd->qosId, numNlw, numRequest);
            logWaitCount = 0;
        }

        sleep(WAIT_COMPLETE_CHECK_INTERVAL_SEC);

        // Lock Mutex and check again
        pthread_mutex_lock(&pQosd->mutex);
        numRequest = atomic_load(&pQosd->numProcessingRequests);
        numNlw = atomic_load(&pQosd->numProcessingNlw);
        prevNumProcessing = nowNumProcessing;
        nowNumProcessing = numRequest + numNlw;
        logWaitCount++;
        // free(a);
    }
}

/**
 *  @brief	Waits all uncompleted requests in Sef to complete
 *  @param	[in] pSef: Sef handle
 */
void WaitAllRequestsCompleteForSef(struct SEFHandle_ *pSef)
{
    int logWaitCount = 0;
    uint16_t numRequest;

    numRequest = atomic_load(&pSef->numProcessingRequests);
    if (numRequest != 0)
    {
        ULOG_NOTICE("not completed requests of sef remains now [request=%u]\n", numRequest);
    }

    // Wait uncompleted requests to complete
    while (numRequest)
    {
        // Output log with a regular interval
        if (logWaitCount > WAIT_COMPLETE_LOG_INTERVAL_COUNT)
        {
            ULOG_NOTICE("wait for requests to be completed... [sef device id=%u, remain=%u]\n",
                        pSef->deviceId, numRequest);
            logWaitCount = 0;
        }

        sleep(WAIT_COMPLETE_CHECK_INTERVAL_SEC);

        numRequest = atomic_load(&pSef->numProcessingRequests);
        logWaitCount++;
    }
}
