/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_api.c
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

#define MODULE_NAME_SELECT "SEFAPI"

#include "sef_api.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>    // PRIx64
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "SEFAPI.h"
#include "common.h"
#include "debug.h"
#include "device_info.h"
#include "device_io.h"
#include "request.h"
#include "sef-slist.h"
#include "sef_cmd.h"

#ifndef SEFSDKVersion
#define SEFSDKVersion "unavailable"
#endif

static __attribute((used)) char SdkVersion[] = "@(#) SEF_SDK version " SEFSDKVersion;

#define SEFINFO_ADU_NUM       (1)       //!< Number of ADU sizes retained in struct SEFInfo
#define SEFINFO_ADU_SIZE      (4096)    //!< ADU size retained in struct SEFInfo
#define SEFINFO_ADU_META_SIZE (16)      //!< Number of metadata bytes per ADU
#define MAX(a, b)             (((a) > (b)) ? (a) : (b))
#define MIN(a, b)             (((a) < (b)) ? (a) : (b))
#define SBLIST_MAX_LEN(pQos)                                                    \
    MIN((pQos)->pSefHandle->pSefInfo->numBlocks *(pQos)->ndie / (pQos)->sbSize, \
        (pQos)->maxBufSize)    //!< Maximum number of descriptors that can be retrieved by
                               //!< SuperBlockList (maximum number of SBs) capped by MDTS
#define NLC_SFLA_MASK_FOR_QWORD \
    ((uint64_t)(0x3F))    //!< Mask of SFLA bit location from start of bitmap for Bitmap-format NLC
                          //!< (SEFAPI: 6bit, command: 5bit)

#define READ_CMD_MAX_ADU (0x10000)    //!< Maximum number of ADUs for Read command to a device

#define FAR_AND_NLW_CMD_MAX_ADU \
    (0x10000)    //!< Maximum number of ADUs for FAR and NLW commands to a device

#define NLC_CMD_READ_ERROR \
    (0xFFFFFFFFFFFFFFFF)    //!< Indicates that read of copy source ADU caused an error during NLC
#define NLC_CMD_DEFECTIVE_PLANE \
    (0xFFFFFFFFFFFFFFFE)    //!< Indicates that copy destination was a Defective Plane during NLC
#define NLC_CMD_CSTS_MASK \
    (0x0000001F)    //!< Mask for Copy Status (CSTS) of CQ DW0 of NLC command (all bits)
#define NLC_CMD_CSTS_MASK_WO_CONSUMED \
    (0x0000001E)    //!< Mask for Copy Status (CSTS) of CQ DW0 of NLC command (except for kCopyConsumedSource bit)

#define LIST_NLC_CMD_MAX_FLA \
    (0x10000)    //!< Maximum number of FLAs that can be specified in List-format NLC command
#define LIST_NLC_CMD_OUT_OF_RANGE \
    (0xFFFFFFFFFFFFFFFF)    //!< Indicates filtered out during List-format NLC

#define SBLIST_LOG_CMD_LIST_END (0xFFFFFFFF)    //!< Indicates end of SBList

/**
 *  @brief	Rounds up size to DWORD
 *  @param	[in] pSef: SEF handle
 *  @return	Size rounded up to DWORD
 */
#define TO_UPPER_DWORD_SIZE(size) ((((size) + 3) >> 2) << 2)
#define TO_UPPER_QWORD_SIZE(size) ((((size) + 7) >> 2) << 2)

/**
 *  @brief	Retrieves size of Defective Plane Bitmap
 *  @param	[in] pSef: SEF handle
 *  @param	[in] pQos: QosD handle
 *  @param	[in] unitNum:
 *  @return	Size of Defective Plane Bitmap
 */
#define GET_DEFECT_PL_BMP_SIZE(pSef, pQos, bitNum) \
    (((pSef)->pSefInfo->numPlanes * (pQos)->sbSize + ((bitNum)-1)) / (bitNum))

/**
 *  @brief	Returns whether QoS Domain ID is out of valid range
 *  @param	[in] id: QoS Domain ID
 *  @param	[in] max_qosd: maximum QoS Domains per VD
 *  @return	true: out of valid range; false: within valid range
 *  @note	As QoS Domain ID 0 and 1 cannot be used and only 2 or larger is valid, ID equal to
 * maximum + 1 or smaller is valid
 */
#define IS_INVALID_QOSD_ID(id, max_qosd) ((((id) < 1) || ((id) > ((max_qosd)))) ? true : false)

/**
 *  @brief	Calculates number of valid ADUs in SB
 *  @param	[in] pSef: SEF handle
 *  @param	[in] sbSize: number of die in SB
 *  @param	[in] defectPlaneNum: number of Defective Planes in SB
 *  @return	number of ADUs
 */
#define CALC_ADU_NUM_IN_SB_BASE(pSef, sbSize, defectPlaneNum)                                 \
    ((((sbSize) * (pSef)->numPlanesPerDie) - (defectPlaneNum)) * (pSef)->pSefInfo->numPages * \
     (pSef)->numADUsPerPlane)

/**
 *  @brief	Calculates number of valid ADUs in SB
 *  @param	[in] pSef: SEF handle
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] defectPlaneNum: number of Defective Planes in SB
 *  @return	number of ADUs
 */
#define CALC_ADU_NUM_IN_SB(pSef, pQos, defectPlaneNum) \
    CALC_ADU_NUM_IN_SB_BASE(pSef, (pQos)->sbSize, defectPlaneNum)

/**
 *  @brief	Calculates number of SB Records that buffer can store
 *  @param	[in] pBuffer: pointer to buffer (with struct SEFSuperBlockRecord member)
 *  @param	[in] bufferSize: buffer size
 *  @return	Number of SB Records
 */
#define CALC_SAVE_SB_RECORD_MAX(pBuffer, bufferSize) \
    ((bufferSize) < sizeof(*pBuffer))                \
        ? 0                                          \
        : (((bufferSize) - sizeof(*(pBuffer))) / sizeof((pBuffer)->superBlockRecords[0]))

/**
 *  @brief	Calculates size of buffer for holding numSuperBlocks super block records
 *  @param  [in] pBuffer: pointer to buffer (with struct SEFSuperBlockRecord member)
 *  @param	[in] numSuperBlocks: number of super blocks to store in buffer
 *  @return	Size of buffer in bytes
 */
#define CALC_SAVE_SB_BUFFER_SIZE(pBuffer, numSuperBlocks) \
    (sizeof(*(pBuffer)) + sizeof(struct SEFSuperBlockRecord) * (numSuperBlocks))

/**
 *  @brief  Calculates number of SEFUserAddress elements that buffer can store
 *  @param  [in] pBuffer: pointer to buffer (with struct SEFUserAddress member)
 *  @param  [in] bufferSize: buffer size
 *  @return Buffer capacity for SEFUserAddress elements
 */
#define CALC_SAVE_UA_RECOVERY_MAX(pBuffer, bufferSize) \
    (((bufferSize) - sizeof(*(pBuffer))) / sizeof((pBuffer)->userAddressesRecovery[0]))

/**
 *  @brief	Calculates size of SEFUserAddressRecord for holding recovery info for sbADUs
 *  @param  [in] pBuffer: pointer to buffer (with struct SEFUserAddress member)
 *  @param	[in] numADUs: number of ADUs in a super block
 *  @return	Size of buffer in bytes
 */
#define CALC_SAVE_UA_BUFFER_SIZE(pBuffer, numADUs) \
    (sizeof(*(pBuffer)) + sizeof(struct SEFUserAddress) * (numADUs))

/**
 *  @brief  Calculates number of die IDs a SEFDieList can store
 *  @param  [in] pBuffer: pointer to buffer (with dieIDs array member)
 *  @param  [in] bufferSize: buffer size
 *  @return Buffer capacity for dieIDs elements
 */
#define CALC_SAVE_DIE_LIST_MAX(pBuffer, bufferSize) \
    (((bufferSize) - sizeof(*(pBuffer))) / sizeof((pBuffer)->dieIDs[0]))

/**
 *  @brief	Calculates size of SEFDieList for holding numDies die IDs
 *  @param  [in] pBuffer: pointer to buffer (with dieIDs array member)
 *  @param	[in] numDies: number of dies in a super block
 *  @return	Size of buffer in bytes
 */
#define CALC_SAVE_DIE_LIST_BUFFER_SIZE(pBuffer, numDies) \
    (sizeof(*(pBuffer)) + sizeof(uint16_t) * (numDies))

/**
 *  @brief	Increments Request counter (except that for NamelessWrite) in QoS handle
 *  @param	[in] pQos: QoSDmain handle
 *  @return	None
 */
#define QOSD_REQUEST_COUNT_INC(pQos)                                           \
    do                                                                         \
    {                                                                          \
        uint32_t numReq = atomic_fetch_add(&(pQos)->numProcessingRequests, 1); \
        ULOG_DEBUG("REQUEST_COUNT_INC == %d\n", numReq + 1);                   \
    } while (0)

/**
 *  @brief	Decrements Request counter (except that for NamelessWrite) in QoS handle
 *  @param	[in] pQos: QoSDmain handle
 *  @return	None
 */
#define QOSD_REQUEST_COUNT_DEC(pQos)                                           \
    do                                                                         \
    {                                                                          \
        uint32_t numReq = atomic_fetch_sub(&(pQos)->numProcessingRequests, 1); \
        ULOG_DEBUG("REQUEST_COUNT_DEC == %d\n", numReq - 1);                   \
    } while (0)

/**
 *  @brief	Increments Request counter (for NamelessWrite) in QoS handle
 *  @param	[in] pQos: QoSDmain handle
 *  @return	None
 */
#define QOSD_NLW_COUNT_INC(pQos)                                          \
    do                                                                    \
    {                                                                     \
        uint32_t numNlw = atomic_fetch_add(&(pQos)->numProcessingNlw, 1); \
        ULOG_DEBUG("NLW_COUNT_INC == %d\n", numNlw + 1);                  \
    } while (0)

/**
 *  @brief	Decrements Request counter (for NamelessWrite) in QoS handle
 *  @param	[in] pQos: QoSDmain handle
 *  @return	None
 */
#define QOSD_NLW_COUNT_DEC(pQos)                                          \
    do                                                                    \
    {                                                                     \
        uint32_t numNlw = atomic_fetch_sub(&(pQos)->numProcessingNlw, 1); \
        ULOG_DEBUG("NLW_COUNT_DEC == %d\n", numNlw - 1);                  \
    } while (0)

STATIC pthread_mutex_t seflibMutex = /* Protects init/cleanup */
    PTHREAD_MUTEX_INITIALIZER;
STATIC int seflibInitCounter = 0;    /* Number of calls to SEFLibraryInit() */

STATIC void CheckResultNamelessWrite(struct ReqToComplete *pReq2Cmp);

// devPath may optionally have /dev infront
STATIC FILE *SysOpenDevice(const char *devPath, const char *field)
{
    char path[64];
    const char *devName = strncmp(devPath, "/dev/", 5) == 0 ? devPath + 5 : devPath;
    if (memcmp(devPath, "/sys", 4) == 0)
    {
        snprintf(path, sizeof(path), "%s/%s", devPath, field);
    }
    else
    {
        snprintf(path, sizeof(path), "/sys/class/sef/%s/%s", devName, field);
    }
    FILE *f = fopen(path, "r");
    if (!f)
    {
        ULOG_ERROR("Unable to open %s: %d\n", path, errno);
    }
    return f;
}

STATIC long long SysReadFileDeviceInt(const char *devName, const char *field)
{
    FILE *f = SysOpenDevice(devName, field);
    long long ret = -1;
    char s[32];

    if (!f)
    {
        return ret;
    }
    if (fgets(s, sizeof(s), f))
    {
        char *p;

        ret = strtoll(s, &p, 0);
        if (*p != '\n')
        {
            ret = -1;
        }
    }
    fclose(f);
    return ret;
}

STATIC char *SysReadFileDeviceStr(const char *devName, const char *field)
{
    FILE *f = SysOpenDevice(devName, field);
    char s[4096];
    char *p = s;

    if (!f)
    {
        return NULL;
    }
    *s = '\0';
    if (fgets(s, sizeof(s), f))
    {
        p = strchr(s, '\0');
    }
    fclose(f);
    if (p - s && p[-1] == '\n')
    {
        p[-1] = '\0';
    }
    else
    {
        return NULL;
    }
    return strdup(s);
}

/*
 * This is explicitly for fixed size character fields and not strings.
 * The returned value in s blank padded and is not null terminated
 */
STATIC void SysReadFileDeviceFixed(const char *devName, const char *field, char *s, int len)
{
    char *tmp = SysReadFileDeviceStr(devName, field);
    char ns[len + 1];

    // blank pad to the end
    snprintf(ns, len + 1, "%*s", len, tmp);
    memcpy(s, ns, len);
    free(tmp);
}

int DeviceToSefError(int err)
{
    switch (err)
    {
        case -EINVAL:
            ULOG_ERROR("Device inval\n");
            SEF_ASSERT(0);
            return err;
        case 0:
        case -EACCES:
        case -EPERM:
        case -ENOMEM:
        case -EWOULDBLOCK:
        case -EBUSY:
        case -ETIMEDOUT:
        case -ENODEV:
        case -EOVERFLOW:
            return err;
        default:
            return -EIO;
    }
}

static bool DoZeroCopyRead(struct SEFReadWithPhysicalAddressIOCB *iocb)
{
    int i;
    if (iocb->iovOffset == 0)
    {
        for (i = 0; i < iocb->iovcnt; ++i)
        {
            if (((__u64)iocb->iov[0].iov_base & 0x3) != 0)
            {
                return false;
            }
        }
    }
    return true;
}

static bool DoZeroCopyWrite(struct SEFWriteWithoutPhysicalAddressIOCB *iocb)
{
    if (iocb->common.flags & kSefIoFlagNotifyBufferRelease)
    {
        int i;
        for (i = 0; i < iocb->iovcnt; ++i)
        {
            if (((__u64)iocb->iov[i].iov_base & 0x3) != 0)
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

static void NotifyMemoryAvailable(struct SEFQoSHandle_ *qosHandle, const struct iovec *iovec, int iovcnt)
{
    if (qosHandle->notifyFunc)
    {
        struct SEFQoSNotification notify;
        notify.type = kBufferRelease;
        notify.iov = iovec;
        notify.iovcnt = iovcnt;
        qosHandle->notifyFunc(qosHandle->pContext, notify);
    }
}

uint32_t GetNlwid(struct SEFQoSHandle_ *pQos)
{
    uint32_t nlwid;
    while ((nlwid = atomic_fetch_add(&pQos->nlwid, 1)) == 0) {}
    return nlwid;
}

STATIC uint32_t MaxAduCount(SEFQoSHandle pQoS)
{
    size_t maxBufSize = pQoS->maxBufSize;
    // struct SEFInfo *pSefInfo = pQoS->pSefHandle->pSefInfo;
    uint32_t aduSize = GET_ADU_SIZE(pQoS->pSefHandle);
    // size_t numADU = ((maxBufSize - (maxBufSize % aduSize*pSefInfo->numPlanes*pSefInfo->pageSize))/aduSize);
    size_t numADU = maxBufSize / aduSize;

    // MaxAduCount() used for read and write
    static_assert(FAR_AND_NLW_CMD_MAX_ADU == READ_CMD_MAX_ADU, "max read != max write");

    return numADU < FAR_AND_NLW_CMD_MAX_ADU ? numADU : FAR_AND_NLW_CMD_MAX_ADU;
}

STATIC size_t MaxAcrCount(SEFQoSHandle pQoS)
{
    return ((pQoS->maxBufSize - sizeof(struct SEFAddressChangeRequest)) /
            sizeof_member(struct SEFAddressChangeRequest, addressUpdate[0]));
}

struct SEFFlashAddress SEFNextFlashAddress(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress)
{
    if (qosHandle->defectStrategy != kFragmented)
    {
        return (struct SEFFlashAddress){htole64(le64toh(flashAddress.bits) + 1)};
    }
    return SEFNullFlashAddress;
}

struct PidNotifyData
{
    struct sefSlistNode link;
    struct SEFQoSHandle_ *pQoS;    // NULL causes PidNotifyCloseThread to exit
    struct SEFPlacementID pid;
    uint32_t sbid;
    uint32_t nadu;    // num adus in sb
};

struct PidCloseNotifyState
{
    struct sefSlist list;       // list of PidNotifyData
    struct sefEvent newWork;    // could use futex2 instead?
};

static struct PidCloseNotifyState gPCNS = {.newWork = SEFEVENT_AUTORESET_INITIALIZER};
static pthread_t gPidFlushWorker;

/**
 *  @brief	Substitutes for the processing of SEFLibraryCleanup().
 *  @retval	0: ended normally
 *  @retval	-1: ended abnormally
 *  @details	Terminates threads, deletes device information, and releases its area
 */
STATIC int SeflibCleanup(void)
{
    int numSefUnits;
    int ret = -1;
    int tmp;

    // Retrieve number of SEFUnits
    numSefUnits = (int)DeviceInfoGetNumSefUnits();
    if (numSefUnits == 0)
    {
        // Return as is because number of devices = 0 is not an error
        return 0;
    }

    DeviceIoNotifyShutdown();
    // Delete SEF handle (before deleting threads in order to return error if SEF handle has an open VD or QoSD handle)
    for (int i = 0; i < numSefUnits; i++)
    {
        struct SEFHandle_ *pSefHandle = DeviceInfoGetSefHandle(i);
        if (pSefHandle == NULL)
        {
            continue;
        }

        // close open virtual device handles
        tmp = 0;
        while (atomic_load(&pSefHandle->numOpenVds) > 0)
        {
            if (pSefHandle->ppVdHandle[tmp] != NULL)
            {
                SEFCloseVirtualDevice(pSefHandle->ppVdHandle[tmp]);
            }

            tmp++;
        }

        // close open QoS Domain handles
        tmp = 0;
        while (pSefHandle->numOpenAllQosds > 0 && tmp < pSefHandle->numQosds)
        {
            if (pSefHandle->ppQosHandle[tmp] != NULL)
            {
                struct SEFStatus status;

                status = SEFCloseQoSDomain(pSefHandle->ppQosHandle[tmp]);
                if (status.error != 0)
                {
                    ULOG_ERROR("Failed to close domain %d - (%d)\n", tmp, status.error);
                }
            }

            tmp++;
        }

        // Wait number of requests being processed in SEF handle to be 0
        if (pSefHandle->numOpenAllQosds == 0)
        {
            WaitAllRequestsCompleteForSef(pSefHandle);
        }
        else
        {
            ULOG_ERROR("Not all domains closed - not waiting for i/o to complete\n");
        }

        DeviceIoCloseFile(pSefHandle->deviceFd);

        DeviceInfoDeleteSefHandle(pSefHandle);
    }

    if (gPidFlushWorker)
    {
        struct PidNotifyData pnd = {};

        if (SefSlistPush(&gPCNS.list, &pnd.link))
        {
            SefEventSet(&gPCNS.newWork);
        }

        pthread_join(gPidFlushWorker, NULL);
    }

    // Notify SEF Driver to prepare for deletion
    tmp = DeviceIoPreDestroy();
    if (tmp != 0)
    {
        ULOG_ERROR("failed to DeviceIoPreDestroy()\n");
        ret = DeviceToSefError(tmp);
        goto ExitProc;
    }

    // Notify SEF Driver to delete
    tmp = DeviceIoCleanup();
    if (tmp != 0)
    {
        ULOG_ERROR("failed to DeviceIoCleanup()\n");
        ret = DeviceToSefError(tmp);
        goto ExitProc;
    }

    DeviceInfoCleanup();

    ret = 0;

ExitProc:
    return ret;
}

uint32_t EnumStrIntList(const char *strl, void (*fnc)(int i, long long v, void *c), void *ctx)
{
    const char *p = strl ?: "";
    uint32_t index = 0;

    while (*p)
    {
        char *end;

        long long val = strtoll(p, &end, 0);
        if (end == p)
        {
            break;
        }
        fnc(index, val, ctx);
        index++;
        if (*end == ',')
        {
            end++;
        }
        p = end;
    }
    return index;
}

static void EnumDomainListCB(int index, long long val, void *ctx)
{
    struct SEFQoSDomainList *qlist = ctx;

    if (index < qlist->numQoSDomains)
    {
        qlist->QoSDomainID[index].id = val;
    }
}

static void StrToDomainList(const char *strl, struct SEFQoSDomainList *qlist, uint32_t numMaxQd)
{
    qlist->numQoSDomains = numMaxQd;
    qlist->numQoSDomains = EnumStrIntList(strl, EnumDomainListCB, qlist);
}

static void EnumRFListCB(int index, long long val, void *ctx)
{
    uint32_t *rflist = ctx;

    if (index < SEFMaxReadQueues)
    {
        rflist[index] = val;
    }
}

static void EnumRWListCB(int index, long long val, void *ctx)
{
    uint16_t *rwlist = ctx;

    if (index < SEFMaxReadQueues)
    {
        rwlist[index] = val;
    }
}

static void StrToRFList(const char *strl, uint32_t *rflist)
{
    EnumStrIntList(strl, EnumRFListCB, rflist);
}

static void StrToRWList(const char *strl, uint16_t *rwlist)
{
    EnumStrIntList(strl, EnumRWListCB, rwlist);
}

static void EnumDieListCB(int index, long long val, void *ctx)
{
    struct SEFDieList *dlist = ctx;

    if (index < dlist->numDies)
    {
        dlist->dieIDs[index] = val;
    }
}

static void StrToDieList(const char *strl, struct SEFDieList *dlist, uint32_t numMaxDie)
{
    dlist->numDies = numMaxDie;
    dlist->numDies = EnumStrIntList(strl, EnumDieListCB, dlist);
}

static void EnumVDListCB(int index, long long val, void *ctx)
{
    struct SEFVirtualDeviceList *vlist = ctx;

    if (index < vlist->numVirtualDevices)
    {
        vlist->virtualDeviceID[index].id = val;
    }
}

static void StrToVDList(const char *strl, struct SEFVirtualDeviceList *vlist, uint32_t numMaxVd)
{
    vlist->numVirtualDevices = numMaxVd;
    vlist->numVirtualDevices = EnumStrIntList(strl, EnumVDListCB, vlist);
}

static void EnumRootListCB(int index, long long val, void *ctx)
{
    struct SEFFlashAddress *rlist = ctx;

    if (index < SEFMaxRootPointer)
    {
        rlist[index].bits = val;
    }
}

static void StrToRootList(const char *strl, struct SEFFlashAddress *rlist)
{
    EnumStrIntList(strl, EnumRootListCB, rlist);
}

uint32_t BuildSupportedOptions(const char *pDeviceName)
{
    /* Mask of bits that match between supported options and device capabilities
     *   00 kFragmentedSupported
     *   01 kPackedSupported
     *   02 kPerfectSupported
     *   05 kCopyUserAddressRangeSupported
     *   06 kCopyFlashAddressListSupported
     * Bits that don't match
     *   kPSLCSupported
     *   kDeleteVirtualDeviceSupported
     */
    uint32_t devCap = SysReadFileDeviceInt(pDeviceName, "capability");
    uint32_t ctratt = SysReadFileDeviceInt(pDeviceName, "ctratt");
    uint32_t mask = 0x406f;
    uint32_t ret = devCap & mask;

    if (ctratt & (1 << 13))
    {
        ret |= kDeleteVirtualDeviceSupported;
    }

    ret |= kSuperBlockSupported;
    // These have no mapping from the device with CS 1.13
    ret |= kAutomaticSupported;
    ret |= kFastestSupported;
    ret |= kTypicalSupported;
    ret |= kLongSupported;
    ret |= kHeroicSupported;
    ret |= kStopSupported;

    return ret;
}

/**
 *  @brief	Sets the values of the Sef Unit Information to the SefHandle.
 *  @param	[in] pSefUnitInfoBuff: Buffer for storing values of Sef Unit Information
 *  @param	[out] pSEFHandle_: Handle to the SEF Unit
 *  @return	none
 */
STATIC int SetSefUnitInfo(struct SEFHandle_ *pSEFHandle_, bool insert)
{
    struct SEFInfo *pInfo = pSEFHandle_->pSefInfo;
    const char *pDeviceName = pInfo->name;
    struct SEFQoSDomainList qlist;
    struct SEFVirtualDeviceList vlist;
    char *str;

    // Retain device information
    pSEFHandle_->numPlanesPerDie =
        SysReadFileDeviceInt(pDeviceName, "num_plane");    // pSefUnitInfoBuff->NPL + 1;
    pSEFHandle_->numADUsPerPlane =
        SysReadFileDeviceInt(pDeviceName, "num_adus");    // pSefUnitInfoBuff->NADU + 1;
    memset(&(pSEFHandle_->pSefInfo->HWVersion[0]), 0, (sizeof(char) * 8));    // fixed to 0
    pInfo->supportedOptions = BuildSupportedOptions(pDeviceName);
    pInfo->maxQoSDomains = SysReadFileDeviceInt(pDeviceName, "max_qosd");
    //        pSefUnitInfoBuff->MAXQOSD - 1;    // Maximum ID number - 1 is the upper limit (1 cannot be used)
    pInfo->maxRootPointers =
        SysReadFileDeviceInt(pDeviceName, "max_rootp");    // pSefUnitInfoBuff->NRP + 1;
    pInfo->maxPlacementIDs =
        SysReadFileDeviceInt(pDeviceName, "max_placement");    // pSefUnitInfoBuff->NPLID + 1;
    pInfo->numReadQueues = SysReadFileDeviceInt(pDeviceName, "num_rf");    // pSefUnitInfoBuff->NFMQ + 1;
    pInfo->maxOpenSuperBlocks = SysReadFileDeviceInt(pDeviceName, "max_osb");

    // @todo These are the only dynamic values, plus the missing prop read
    //       queues in  use, they should be the only thing read.  All others should
    //       be read only once.
    str = SysReadFileDeviceStr(pDeviceName, "qosd_list");
    StrToDomainList(str, &qlist, 0);
    free(str);
    pInfo->numQoSDomains = qlist.numQoSDomains;    // pSefUnitInfoBuff->NQOSD;
    str = SysReadFileDeviceStr(pDeviceName, "vd_list");
    StrToVDList(str, &vlist, 0);
    free(str);
    pInfo->numVirtualDevices = vlist.numVirtualDevices;    // pSefUnitInfoBuff->NVD;
    pInfo->APIVersion = SEFAPIVersion;
    pInfo->numChannels = SysReadFileDeviceInt(pDeviceName, "num_ch");    // pSefUnitInfoBuff->NCH + 1;
    pInfo->numBanks = SysReadFileDeviceInt(pDeviceName, "num_bank");
    pInfo->numPlanes = SysReadFileDeviceInt(pDeviceName, "num_plane");    // pSefUnitInfoBuff->NPL + 1;
    pInfo->numADUSizes = SEFINFO_ADU_NUM;
    pInfo->numBlocks = SysReadFileDeviceInt(pDeviceName, "num_blocks");    // pSefUnitInfoBuff->NBLK + 1;
    pInfo->numPages = SysReadFileDeviceInt(pDeviceName, "num_page");    // pSefUnitInfoBuff->NPAG + 1;
    // This make no sense - it implies unit info NADU is in units of ADU,
    // but ADU size is not a device prop, it's a domain property.  Likely,
    // is in ADUSize[0] ADU units (aka 4k).
    pInfo->pageSize = SysReadFileDeviceInt(pDeviceName, "num_adus") *
                      SEFINFO_ADU_SIZE;    // (pSefUnitInfoBuff->NADU + 1) * SEFINFO_ADU_SIZE;
    pInfo->totalBandWidth = 0;             // fixed to 0
    pInfo->readTime = 0;                   // fixed to 0
    pInfo->programTime = 0;                // fixed to 0
    pInfo->eraseTime = 0;                  // fixed to 0
    pInfo->minReadWeight = 32;             // fixed to 32
    pInfo->minWriteWeight = 256;           // fixed to 256
    pInfo->openExpirationPeriod = SysReadFileDeviceInt(pDeviceName, "eop");    // pSefUnitInfoBuff->EOP;
    // This is supposed to be an array of supported sizes that come from the
    // device. Actual data/ meta size is a domain prop and can't be selected
    // with CS 1.13.
    pInfo->numADUSizes = 1;
    pInfo->ADUsize[0].data = SEFINFO_ADU_SIZE;
    pInfo->ADUsize[0].meta = SEFINFO_ADU_META_SIZE;
    pInfo->ADUsize[0].reserved = 0;

    // no eop, use 5 mins
    pSEFHandle_->writeTimeout = (1 + (pInfo->openExpirationPeriod ?: 300)) * 1000;

    /*
     * Initialization of pSEFHandle_->numOpenVds moved to GetHandle to avoid being cleared in GetInformation
     */

    // Log output
    {
        char vendorId[9] = {'\0'};
        char serialNo[21] = {'\0'};
        char fwRev[9] = {'\0'};
        char hwRev[9] = {'\0'};
        memcpy(&vendorId[0], pInfo->vendor, sizeof(pInfo->vendor));
        memcpy(&serialNo[0], pInfo->serialNumber, sizeof(pInfo->serialNumber));
        memcpy(&fwRev[0], pInfo->FWVersion, sizeof(pInfo->FWVersion));
        memcpy(&hwRev[0], pInfo->HWVersion, sizeof(pInfo->HWVersion));

        ULOG_GETINFO_DATA("--SEF Unit Information--\n");
        ULOG_GETINFO_DATA("  vendor=%s\n", vendorId);
        ULOG_GETINFO_DATA("  serialNumber=%s\n", serialNo);
        ULOG_GETINFO_DATA("  FWVersion=%s\n", fwRev);
        ULOG_GETINFO_DATA("  HWVersion=%s\n", hwRev);
        ULOG_GETINFO_DATA("  maxQoSDomains=%u\n", pInfo->maxQoSDomains);
        ULOG_GETINFO_DATA("  maxRootPointers=%u\n", pInfo->maxRootPointers);
        ULOG_GETINFO_DATA("  supportedOptions=0x%lx\n", pInfo->supportedOptions);
        ULOG_GETINFO_DATA("  maxPlacementIDs=%u\n", pInfo->maxPlacementIDs);
        ULOG_GETINFO_DATA("  numReadQueues=%u\n", pInfo->numReadQueues);
        ULOG_GETINFO_DATA("  numVirtualDevices=%u\n", pInfo->numVirtualDevices);
        ULOG_GETINFO_DATA("  numQoSDomains=%u\n", pInfo->numQoSDomains);
        ULOG_GETINFO_DATA("  APIVersion=0x%x\n", pInfo->APIVersion);
        ULOG_GETINFO_DATA("  numBanks=%u\n", pInfo->numBanks);
        ULOG_GETINFO_DATA("  numChannels=%u\n", pInfo->numChannels);
        ULOG_GETINFO_DATA("  numPlanes=%u\n", pInfo->numPlanes);
        ULOG_GETINFO_DATA("  numADUSizes=%u\n", pInfo->numADUSizes);
        ULOG_GETINFO_DATA("  numBlocks=%u\n", pInfo->numBlocks);
        ULOG_GETINFO_DATA("  numPages=%u\n", pInfo->numPages);
        ULOG_GETINFO_DATA("  pageSize=%u\n", pInfo->pageSize);
        ULOG_GETINFO_DATA("  totalBandWidth=%u\n", pInfo->totalBandWidth);
        ULOG_GETINFO_DATA("  readTime=%u\n", pInfo->readTime);
        ULOG_GETINFO_DATA("  programTime=%u\n", pInfo->programTime);
        ULOG_GETINFO_DATA("  eraseTime=%u\n", pInfo->eraseTime);
        ULOG_GETINFO_DATA("  minReadWeight=%u\n", pInfo->minReadWeight);
        ULOG_GETINFO_DATA("  minWriteWeight=%u\n", pInfo->minWriteWeight);
        ULOG_GETINFO_DATA("  openExpirationPeriod=%u\n", pInfo->openExpirationPeriod);
        ULOG_GETINFO_DATA("  ADUsize[0]=%u\n", pInfo->ADUsize[0].data);

        ULOG_GETINFO_DATA("--SEF Handle Information--\n");
        ULOG_GETINFO_DATA("  deviceId=%u\n", pSEFHandle_->deviceId);
        ULOG_GETINFO_DATA("  numPlanesPerDie=%u\n", pSEFHandle_->numPlanesPerDie);
        ULOG_GETINFO_DATA("  numADUsPerPlane=%u\n", pSEFHandle_->numADUsPerPlane);
        ULOG_GETINFO_DATA("------------------------\n");
    }

    if (insert)
    {
        return DeviceInfoSetSefHandle(pSEFHandle_);
    }

    return 0;
}

/**
 *  @brief	Retrieves command type from command name
 *  @param	[in] pCmdName: command name (CMD_XXX macro defined in sef_cmd.h is specified)
 *  @param	[out] pType: command type to issue at IOCTL()
 *  @return	None
 *  @note    First letter of command name is for identifying command type
 */
STATIC void GetIoctlCmdType(const char *pCmdName, enum IoctlCommandType *pType)
{
    switch (pCmdName[0])
    {
        case 'A':
            *pType = kIoctlAdminCommand;
            break;
        case 'I':
            *pType = kIoctlIoCommand;
            break;
        case 'F':
            *pType = kIoctlIoFusedCommand;
            break;
        default:
            break;
    }
}

/**
 *  @brief	Issues command
 *  @param	[in] pSef: SEF handle
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pCmdName: command name (CMD_XXX macro defined in sef_cmd.h is specified)
 *  @param	[in] pCmdInfo: command information (if ppData is non-NULL, this function overwrites addr)
 *  @param	[out] pCmdStat: command execution result
 *  @param	[in] pIocb: IOCB information of SEFAPI (pIocb in Req2Cmp)
 *  @param	[in] pParam: arbitrary parameter (pParam in Req2Cmp)
 *  @param	[in] CheckResultExt: callback to check result (CheckResultExt in Req2Cmp)
 *  @param	[in] CheckSuspendFunc: callback to check suspend (CheckSuspendFunc in Req2Cmp)
 *  @param	[out] ppData: Received data from device
 *  @param	[in] size: Data receive buffer size
 *  @return	0: success; -1: command issuing failed
 *  @note    This function returns success even if execution result of command is an error.
 *  @note    This function allocates memory to Req2Cmp, Sem, and DevBuff (if ppData is not NULL) for request information.
 *  @note    If ppData is not NULL, this function overwrites addr in pCmdInfo because it allocates memory for DevBuff (ppData).
 *  @note    For synchronous operation, this function releases memory of Req2Cmp and Sem; for async,
 *           Complete() releases them (this function releases them if command issuing fails).
 *  @note    Caller must release DevBuff (ppData) or call CheckResultExt() to release it.
 *  @note    However, this function releases DevBuff (ppData) if command issuing failed.
 *  @note    Besides, for synchronous operation, this function releases DevBuff (ppData) in case of command error.
 *  @note    pIocb, pParam, CheckResultExt, and CheckSuspendFunc are set to Req2Cmp as they are.
 *  @note    CheckResultExt is expected to be set when command response check needs a field other than StatusField.
 */
STATIC int IssueCmd(int fd,
                    bool bSync,
                    const char *pCmdName,
                    struct nvme_passthru_cmd64 *pCmdInfo,
                    uint32_t *pCmdStat,
                    union RequestIocb *pIocb,
                    void *pParam,
                    void (*CheckResultExt)(struct ReqToComplete *pReq2Cmp),
                    bool (*CheckSuspendFunc)(struct ReqToComplete *pReq2Cmp),
                    void **ppData,
                    uint32_t size)
{
    struct RequestInfoExt req;
    int ret = -1;

    // Allocate memory for request
    req.ppDevBuff = ppData;
    req.buffSize = size;
    ret = AllocateReqInfoExt(&req.pReq2Cmp, NULL, NULL, 0, &req.pSem, req.ppDevBuff, req.buffSize);
    if (ret != 0)
    {
        ret = -ENOMEM;
        goto ExitProc;
    }

    if (req.ppDevBuff)
    {
        pCmdInfo->addr = (__u64)(*req.ppDevBuff);
    }

    // Create request information
    req.pReq2Cmp->pIocb = pIocb;
    GetIoctlCmdType(pCmdName, &req.pReq2Cmp->type);
    memcpy(&req.pReq2Cmp->submit, pCmdInfo, sizeof(req.pReq2Cmp->submit));
    req.pReq2Cmp->bSync = bSync;
    req.pReq2Cmp->syncOverride = false;
    req.pReq2Cmp->pSyncSemaphore = req.pSem;
    req.pReq2Cmp->param = pParam;
    req.pReq2Cmp->CommandComplete = Complete;
    req.pReq2Cmp->CheckResultExt = CheckResultExt;
    req.pReq2Cmp->CheckSuspendFunc = CheckSuspendFunc;

    // Request command issuing
    ret = Request(fd, req.pReq2Cmp);
    if (ret != 0)
    {
        ret = DeviceToSefError(ret);
        goto FreeDevBuff;
    }

    ret = 0;
    if (bSync)
    {
        pCmdInfo->result = req.pReq2Cmp->result;
        if (pCmdStat != NULL)
        {
            *pCmdStat = DeviceToSefError(req.pReq2Cmp->status);
            if (req.pReq2Cmp->status != 0)
            {
                goto FreeDevBuff;
            }
        }

        // For synchronous operation, memory for ReqInfo is released here
        goto FreeReqInfo;
    }
    else
    {
        // For async, ReqInfo memory release is skipped because Complete releases it
        goto ExitProc;
    }

FreeDevBuff:
    if (req.ppDevBuff)
    {
        free(*req.ppDevBuff);
        *req.ppDevBuff = 0;
    }
FreeReqInfo:
    free(req.pReq2Cmp);
    free(req.pSem);
ExitProc:
    return ret;
}

/**
 *  @brief	Issues command without data transfer
 *  @note    Just a wrapper function of IssueCmd
 *  @see    IssueCmd
 */
STATIC int IssueNoDataCmd(int fd,
                          bool bSync,
                          const char *pCmdName,
                          struct nvme_passthru_cmd64 *pCmdInfo,
                          uint32_t *pCmdStat,
                          union RequestIocb *pIocb,
                          void *pParam,
                          void (*CheckResultExt)(struct ReqToComplete *pReq2Cmp))
{
    return IssueCmd(fd, bSync, pCmdName, pCmdInfo, pCmdStat, pIocb, pParam, CheckResultExt, NULL,
                    NULL, 0);
}

/**
 *  @brief	Issues command with data transfer (host to controller)
 *  @note    Just a wrapper function of IssueCmd
 *  @note    Though IssueSendDataCmd() allocates DevBuff internally, caller must allocate it for send data configuration.
 *  @note    Because allocated buffer address is set to pCmdInfo, request can be issued in the same way as IssueNoDataCmd.
 *  @see    IssueCmd
 */
STATIC int IssueSendDataCmd(int fd,
                            bool bSync,
                            const char *pCmdName,
                            struct nvme_passthru_cmd64 *pCmdInfo,
                            uint32_t *pCmdStat,
                            union RequestIocb *pIocb,
                            void *pParam,
                            void (*CheckResultExt)(struct ReqToComplete *pReq2Cmp))
{
    return IssueCmd(fd, bSync, pCmdName, pCmdInfo, pCmdStat, pIocb, pParam, CheckResultExt, NULL,
                    NULL, 0);
}

/**
 *  @brief	Issues command with data transfer (controller to host)
 *  @note    Just a wrapper function of IssueCmd
 *  @see    IssueCmd
 */
STATIC int IssueRecvDataCmd(int fd,
                            bool bSync,
                            const char *pCmdName,
                            struct nvme_passthru_cmd64 *pCmdInfo,
                            uint32_t *pCmdStat,
                            union RequestIocb *pIocb,
                            void *pParam,
                            void (*CheckResultExt)(struct ReqToComplete *pReq2Cmp),
                            void **ppData,
                            uint32_t size)
{
    return IssueCmd(fd, bSync, pCmdName, pCmdInfo, pCmdStat, pIocb, pParam, CheckResultExt, NULL,
                    ppData, size);
}

/**
 *  @brief	Structure of parameters retained for responding to Release Super Block
 *  @note    Set to param of Req2Cmp at creating request
 */
struct ReleaseSbParam
{
    struct SEFQoSHandle_ *pQos;    //!< QoSD handle
    uint32_t sbId;                 //!< Super Block ID specified at request
};

/**
 *  @brief	Responds to Release Super Block
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
STATIC void CheckResultReleaseSb(struct ReqToComplete *pReq2Cmp)
{
    struct ReleaseSbParam *pParam = (struct ReleaseSbParam *)pReq2Cmp->param;

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        goto ExitProc;
    }

    uint32_t sbId = pReq2Cmp->result & UINT32_MAX;
    ULOG_SBCMD_DBG("sbid=0x%04x\n", sbId);

    if (pParam->sbId != sbId)
    {
        ULOG_ERROR("mismatched sb id!! req=0x%04x, ret=0x%04x\n", pParam->sbId, sbId);
        pReq2Cmp->status = REQUEST_MISMATCH_SBID;
    }

ExitProc:
    CompleteIocbProc(pReq2Cmp);

    QOSD_REQUEST_COUNT_DEC(pParam->pQos);

    free(pParam);
}

/**
 *  @brief	Internal processing for SEFReleaseSuperBlock and SEFReleaseSuperBlockAsync
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Release Super Block
 *  @details	Area of arbitrary parameter for responding to Release Super Block is allocated.
 *  @details	This function releases arbitrary parameter area if command issuing failed.
 *  @note    CheckResultExt must release arbitrary parameter area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info: fixed to 0
 */
STATIC struct SEFStatus ReleaseSbProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common)
{
    struct SEFReleaseSuperBlockIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct nvme_passthru_cmd64 cmd;
    struct ReleaseSbParam *pParam;
    int ret;
    uint32_t sbid;

    QOSD_REQUEST_COUNT_INC(pQos);

    // Check parameter
    if (pQos->qosId != GetQosdId(iocb->flashAddress))
    {
        ULOG_ERROR("argument error!! flashAddress=0x%016lx\n", iocb->flashAddress.bits);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    sbid = GetSbId(pQos, iocb->flashAddress);
    bool bOpned = !DeviceInfoIsClosedSb(pQos, sbid);
    if (bOpned)
    {
        ULOG_ERROR("still open the sb=%u\n", sbid);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Prepare parameters
    pParam = (struct ReleaseSbParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }

    pParam->pQos = pQos;
    pParam->sbId = sbid;

    iocb->common.reserved = 1;

    // Release SB
    SEFNvmSuperBlockManagement(&cmd, NULL, 0, SEF_NVMOPE_SBMANAGEMENT_FREE, pQos->qosId,
                               pParam->sbId, NULL, 0);
    ret = IssueNoDataCmd(pQos->deviceFd, bSync, CMD_I_SBMNG_FREE, &cmd, NULL,
                         (union RequestIocb *)iocb, pParam, CheckResultReleaseSb);
    if (ret != 0)
    {
        goto FreeParam;
    }

    if (bSync)
    {
        // For synchronous operation, command result is set
        status = iocb->common.status;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;
    }
    goto ExitProc;

FreeParam:
    free(pParam);    // Released only in case of an error; otherwise, CheckResult releases it
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(
        pQos);    // Decremented only in case of an error; otherwise, CheckResult decrements it
ExitProc:
    return status;
}

/**
 *  @brief	Structure of parameters retained for responding to Allocate Super Block
 *  @note    Set to param of Req2Cmp at creating request
 */
struct AllocateSbParam
{
    struct SEFQoSHandle_ *pQos;    //!< QoSD handle
};

/**
 *  @brief	Responds to Allocate Super Block
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
STATIC void CheckResultAllocateSb(struct ReqToComplete *pReq2Cmp)
{
    struct AllocateSbParam *pParam = (struct AllocateSbParam *)pReq2Cmp->param;
    struct SEFAllocateSuperBlockIOCB *pAllocSbIocb = &pReq2Cmp->pIocb->allocateSb;
    int ret;

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        goto ExitProc;
    }

    uint32_t sbId = pReq2Cmp->result & UINT32_MAX;
    ULOG_SBCMD_DBG("sbid=0x%04x\n", sbId);

    ret = DeviceInfoAddOpenInfoForSb(pParam->pQos, sbId);
    if (ret != 0)
    {
        ULOG_ERROR("failed to add sb(0x%04x) info to qosId(%u)\n", sbId, pParam->pQos->qosId);
        pReq2Cmp->status = REQUEST_MEMORY_ERROR; /* Error response */
        goto ExitProc;
    }

    // Retrieve FlashAddress from QoS Domain ID and Super Block ID
    pAllocSbIocb->flashAddress = GetFlashAddress(pParam->pQos, pParam->pQos->qosId, sbId);

    // Calculate number of ADUs in the allocated SB
    pAllocSbIocb->common.status.info = pReq2Cmp->result >> 32;
    ULOG_SBCMD_INF("sb valid adu num=%d(0x%x)\n", pAllocSbIocb->common.status.info,
                   pAllocSbIocb->common.status.info);

ExitProc:
    CompleteIocbProc(pReq2Cmp);

    QOSD_REQUEST_COUNT_DEC(pParam->pQos);

    free(pParam);
}

/**
 *  @brief	Internal processing for SEFAllocateSuperBlock and SEFAllocateSuperBlockAsync
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Allocate Super Block
 *  @details	Area of arbitrary parameter for responding to Allocate Super Block is allocated.
 *  @details	This function releases arbitrary parameter area if command issuing failed.
 *  @note	CheckResultExt must release arbitrary parameter area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info:
 *          number of valid ADUs in allocated SB (for async, fixed to 0)
 */
STATIC struct SEFStatus AllocateSbProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common)
{
    struct SEFAllocateSuperBlockIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct nvme_passthru_cmd64 cmd;
    struct AllocateSbParam *pParam;
    int ret;
    bool override = !!(common->flags & kSefIoFlagOverride);
    struct SEFAllocateOverrides *overrides = override ? &iocb->overrides : NULL;
    uint8_t op;

    QOSD_REQUEST_COUNT_INC(pQos);

    // Prepare parameters
    pParam = (struct AllocateSbParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }
    pParam->pQos = pQos;

    iocb->common.reserved = 1;

    op = iocb->type == kForPSLCWrite ? SEF_NVMOPE_SBMANAGEMENT_PERASE : SEF_NVMOPE_SBMANAGEMENT_ERASE;
    SEFNvmSuperBlockManagement(&cmd, NULL, 0, op, pQos->qosId, 0xFFFFFFFF, overrides, 0);
    ret = IssueRecvDataCmd(pQos->deviceFd, bSync, CMD_I_SBMNG_ERASE, &cmd, NULL,
                           (union RequestIocb *)iocb, pParam, CheckResultAllocateSb, NULL, 0);
    if (ret != 0)
    {
        goto FreeParam;
    }

    if (bSync)
    {
        // For synchronous operation, command result is set
        status = iocb->common.status;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;
    }
    goto ExitProc;

FreeParam:
    free(pParam);    // Released only in case of an error; otherwise, CheckResult releases it
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(
        pQos);    // Decremented only in case of an error; otherwise, CheckResult decrements it
ExitProc:
    return status;
}

static void completeSuspendedList(struct sefSlist *list)
{
    struct sefSlistNode *node = SefSlistReverse(SefSlistPopAll(list));
    struct sefSlistNode *next;
    struct ReqToComplete *pReq;

    while (node != NULL)
    {
        pReq = (struct ReqToComplete *)node;
        next = node->next;
        node->next = NULL;
        (*pReq->CommandComplete)(pReq);
        node = next;
    }
}
STATIC void CompleteSuspendedWrites(struct SEFQoSHandle_ *pQos, uint32_t sbId)
{
    struct SbWriteInformation *sb_info;

    if (atomic_load(&pQos->bClosingFlag))
    {
        return;
    }
    sb_info = DeviceInfoGetSb(pQos, sbId, false);
    if (sb_info)
    {
        completeSuspendedList(&sb_info->listHeadByWriting);
        DeviceInfoReleaseSb(pQos, sb_info);
    }
}

enum ManageResponseOrderFlags {
    kMroCreate = 1,   /**< Create an entry if not already being tracked */
    kMroIncIoRef = 2, /**< Adding a reference, will add numAdus to written ADUs */
    kMroDecIoRef = 4, /**< Removing a reference */
};

/**
 *  @brief	Manages order of responses
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] sbId: target SB ID
 *  @param	[in] numAdus: How much to add to to writtenAdus reported at SB close
 *  @param	[in] pNotify: pointer to SBStateChange notification information
 *  @param	[in] flags: kMroCreate to create an entry if not already being tracked
 *                      kMroIncRef to increase refCount
 *                      kMroDecRef to decrease refCount
 *  @details	If target QoSD is being Closed, do nothing.
 *  @details	Unless notification information pointer is NULL, it is reflected
 *              to management area for flow control.
 *  @details	Suspended operations are executed according to SB state of
 *              result reflected to management area for flow control.
 *  @retval 0           Success
 *  @retval -EINVAL     SB is not open, refAdj != 0 and create was false or
 *                      both inc/dec ref set in flags
 *  @retval -EPERM      Domain handle is invalid (it's closing)
 */
STATIC int ManageResponseOrder(struct SEFQoSHandle_ *pQos,
                               uint32_t sbId,
                               int numADUs,
                               struct SEFQoSNotification *pNotify,
                               unsigned int flags)
{
    struct SbWriteInformation *sb_info;
    enum SbWriteState writeState;
    bool create = !!(flags & kMroCreate);
    int refAdj = 0;

    if (flags & kMroIncIoRef)
    {
        if (flags & kMroDecIoRef)
        {
            return -EINVAL;
        }
        refAdj = 1;
    }
    if (flags & kMroDecIoRef)
    {
        refAdj = -1;
        SEF_ASSERT(!create);    // can only create if not decrementing
        if (create)
        {
            return -EINVAL;
        }
    }
    if (pQos->bClosingFlag)
    {
        return -EPERM;
    }
    SEF_ASSERT(numADUs <= (int)pQos->sbCapacity);
    SEF_ASSERT(-numADUs <= (int)pQos->sbCapacity);

    sb_info = DeviceInfoGetSb(pQos, sbId, create);
    if (!sb_info)
    {
        if (refAdj == 0 && pNotify == NULL)    // nop call to flush closes
        {
            return 0;
        }
        // This will print when a write by SBID is performed against a closed SB
        ULOG_ERROR("failed to %s sb_info: qdId=%u, sbId=%u, refAd=%u pNotify=%p\n",
                   create ? "add" : "get", pQos->qosId, sbId, refAdj, pNotify);
        // SBs opened before library was initialized will trigger this
#if !CC_IGNORE_EXTRA_CLOSE
        SEF_ASSERT(pNotify == NULL);    // Try to catch double close
#endif
        SEF_ASSERT(refAdj != -1);    // I/O completed after closed
        return -EINVAL;
    }

    if (pNotify != NULL)
    {
        SEF_ASSERT(refAdj == 0);
        DeviceInfoUpdateSbClosed(sb_info, pNotify, &writeState);
    }
    else
    {
        DeviceInfoUpdateWriteInfoForSb(sb_info, refAdj, numADUs, &writeState);
    }

    ULOG_DEBUG("SB Write Status qdId=%u, sbId=%u, state=%u\n", pQos->qosId, sbId, writeState);

    if (writeState != kSbWriting)
    {
        completeSuspendedList(&sb_info->listHeadByWriting);
        if (writeState == kSbAllWritten)
        {
            if (pQos->notifyFunc != NULL)
            {
                sb_info->notificationData.writtenADUs = sb_info->writtenAdus;
                pQos->notifyFunc(pQos->pContext, sb_info->notificationData);
            }
            else
            {
                ULOG_NOTICE(
                    "sb state change log not notified. qosd id=%u, fla=0x%016lx, type=%u (because "
                    ": notify=%p)\n",
                    sb_info->notificationData.QoSDomainID.id,
                    sb_info->notificationData.changedFlashAddress.bits,
                    sb_info->notificationData.type, pQos->notifyFunc);
            }
            completeSuspendedList(&sb_info->listHeadByOpen);
            sb_info->state.state = kSbInfoNotified;
            sem_post(&sb_info->closed);
        }
    }
    DeviceInfoReleaseSb(pQos, sb_info);
    if (writeState == kSbAllWritten)
    {
        DeviceInfoDeleteSb(pQos, sbId);
    }

    return 0;
}

STATIC int IsSuperBlockFull(struct SEFQoSHandle_ *pQos, uint32_t sbId, uint32_t sbCap)
{
    struct SbWriteInformation *sb_info;
    int full = 0;

    sb_info = DeviceInfoGetSb(pQos, sbId, false);
    if (sb_info)
    {
        uint32_t writtenAdus = atomic_load(&sb_info->writtenAdus);

        full = (sbCap == writtenAdus);
        DeviceInfoReleaseSb(pQos, sb_info);
    }
    return full;
}

STATIC void *PidCloseNotifyThread(void *arg)
{
    struct PidCloseNotifyState *pcsn = arg;

    for (;;)
    {
        struct sefSlistNode *list;

        SefEventWait(&pcsn->newWork);
        list = SefSlistReverse(SefSlistPopAll(&pcsn->list));
        while (list)
        {
            struct PidNotifyData *pnd = (void *)list;

            list = list->next;
            if (pnd->pQoS == NULL)
            {
                goto Exit;
            }
            else
            {
                struct SEFQoSNotification qosNotify = {};
                struct SEFQoSHandle_ *pQoS = pnd->pQoS;
                uint32_t sbid = pnd->sbid;
                uint16_t pid = pnd->pid.id;
                uint32_t numADUs = pnd->nadu;

                free(pnd);
                pnd = NULL;
                if (!IsSuperBlockFull(pQoS, sbid, numADUs))
                {
                    // wait for all the FARs to complete so the SbState numIO reflects
                    // any outstanding NLWs
                    FPWCFlush(&pQoS->pidState[pid].numProcessingFAR);
                }
                // Set a close notify which will be sent here if numIO is 0 or will
                // be sent when NLWs complete.
                qosNotify.type = kSuperBlockStateChanged;
                qosNotify.changedFlashAddress = GetFlashAddress(pQoS, pQoS->qosId, sbid);
                qosNotify.numADUs = numADUs;
                qosNotify.writtenADUs = 0;    // MRO will set for us
                qosNotify.QoSDomainID.id = pQoS->qosId;
                ManageResponseOrder(pQoS, sbid, 0, &qosNotify, 0);
            }
        }
    }
Exit:
    return NULL;
}

STATIC void FlushCloseNotification(struct SEFQoSHandle_ *pQos, uint32_t sbid)
{
    struct SbWriteInformation *sb_info = DeviceInfoGetSb(pQos, sbid, false);

    if (sb_info == NULL)
    {
        return;
    }
    sem_wait(&sb_info->closed);
    DeviceInfoReleaseSb(pQos, sb_info);
}

STATIC void SendCloseNotification(struct SEFQoSHandle_ *pQos,
                                  struct SEFFlashAddress fla,
                                  uint16_t pid,
                                  uint32_t nadu)
{
    struct SEFQoSNotification qosNotify = {};

    qosNotify.type = kSuperBlockStateChanged;
    qosNotify.changedFlashAddress = GetSBAddress(pQos, fla);
    qosNotify.QoSDomainID.id = pQos->qosId;
    qosNotify.writtenADUs = 0;    // MRO will set for us
    qosNotify.numADUs = nadu;

    if (pid < pQos->numPlacementIds)
    {
        struct PidNotifyData *pnd = calloc(1, sizeof(*pnd));

        pnd->pQoS = pQos;
        pnd->pid.id = pid;
        pnd->sbid = GetSbId(pQos, fla);
        pnd->nadu = nadu;

        if (SefSlistPush(&gPCNS.list, &pnd->link))
        {
            SefEventSet(&gPCNS.newWork);
        }
    }
    else
    {
        ManageResponseOrder(pQos, GetSbId(pQos, fla), 0, &qosNotify, 0);
    }
}

/**
 *  @brief	Structure of parameters retained for responding to Close Super Block
 *  @note    Set to param of Req2Cmp at creating request
 */
struct CloseSbParam
{
    struct SEFQoSHandle_ *pQos;    //!< QoSD handle
    uint32_t sbId;                 //!< Super Block ID specified at request
    struct GetLogPageSuperBlockInformation sbInfo;
};

/**
 *  @brief	Responds to Close Super Block
 *  @param	[in] preq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
STATIC void CheckResultCloseSb(struct ReqToComplete *pReq2Cmp)
{
    struct CloseSbParam *pParam = (struct CloseSbParam *)pReq2Cmp->param;
    bool cc = !!(pReq2Cmp->result & (UINT64_C(1) << 63));
    uint32_t sbid = (pReq2Cmp->result & UINT32_MAX);
    struct SEFQoSHandle_ *pQos = pParam->pQos;

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        CompleteIocbProc(pReq2Cmp);
        goto ExitProc;
    }

    ULOG_SBCMD_DBG("sbid=0x%04x cc=%d\n", sbid, cc);

    if (pParam->sbId != sbid)
    {
        ULOG_ERROR("mismatched sb id!! req=0x%04x, ret=0x%04x\n", pParam->sbId, sbid);
        pReq2Cmp->status = REQUEST_MISMATCH_SBID;
        CompleteIocbProc(pReq2Cmp);
        goto ExitProc;
    }

    pReq2Cmp->pIocb->closeSb.common.status.info = pParam->sbInfo.CAP;
    CompleteIocbProc(pReq2Cmp);
    // it's undefined what happens if there are uncompleted FARs on a superblock
    // that's forced closed.

    if (cc)
    {
        SendCloseNotification(pQos, GetFlashAddress(pQos, pQos->qosId, sbid), SEFPlacementIdUnused,
                              pParam->sbInfo.CAP);
    }
ExitProc:
    QOSD_REQUEST_COUNT_DEC(pParam->pQos);

    free(pParam);
}

/**
 *  @brief	Checks whether to suspend Close Super Block response
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @retval	true:  being opened
 *  @retval	false: not being opened (or no need to suspend)
 *  @note	Checks whether to suspend Close Super Block response and registers suspend information
 * in case of suspension
 */
STATIC bool IsSuspendedCloseSb(struct ReqToComplete *pReq2Cmp)
{
    bool bRet = false;
    struct CloseSbParam *pParam = (struct CloseSbParam *)pReq2Cmp->param;
    struct SEFQoSHandle_ *pQos = pParam->pQos;
    uint32_t sbId = pReq2Cmp->result & UINT32_MAX;

    if ((pReq2Cmp->status != 0) || atomic_load(&pQos->bClosingFlag) || (sbId != pParam->sbId))
    {
        goto ExitProc;
    }

    if (DeviceInfoIsWritingSb(pQos, sbId))
    {
        DeviceInfoAddSuspendListByWriting(pQos, sbId, pReq2Cmp);
        // If not writing post add, unsuspend manually
        if (!DeviceInfoIsWritingSb(pQos, sbId))
        {
            CompleteSuspendedWrites(pQos, sbId);
        }
        bRet = true;
        ULOG_SBCMD_DBG("Suspend CloseSb sbid=0x%04x\n", sbId);
    }

ExitProc:

    return bRet;
}

/**
 *  @brief	Internal processing for SEFCloseSuperBlock and SEFCloseSuperBlockAsync
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Close Super Block
 *  @param  [in] suspend: Suspend close until writes complete
 *  @details	Area of arbitrary parameter for responding to Close Super Block is allocated.
 *  @details	This function releases arbitrary parameter area if command issuing failed.
 *  @note    CheckResultExt must release arbitrary parameter area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info: fixed to 0
 */
STATIC struct SEFStatus CloseSbProc(bool bSync,
                                    struct SEFQoSHandle_ *pQos,
                                    struct SEFCommonIOCB *common,
                                    bool suspend)
{
    struct SEFCloseSuperBlockIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct nvme_passthru_cmd64 cmd;
    struct CloseSbParam *pParam;
    int ret;

    QOSD_REQUEST_COUNT_INC(pQos);

    // Check parameter
    if (pQos->qosId != GetQosdId(iocb->flashAddress))
    {
        ULOG_ERROR("argument error!! flashAddress=0x%016lx\n", iocb->flashAddress.bits);
        status.error = -EFAULT;
        goto DecrementProcReq;
    }

    // Prepare parameters
    pParam = (struct CloseSbParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }

    pParam->pQos = pQos;
    pParam->sbId = GetSbId(pQos, iocb->flashAddress);

    iocb->common.reserved = 1;
    // Close SB
    SEFNvmSuperBlockManagement(&cmd, &pParam->sbInfo, sizeof(pParam->sbInfo),
                               SEF_NVMOPE_SBMANAGEMENT_CLOSE, pQos->qosId, pParam->sbId, NULL, 0);
    ret = IssueCmd(pQos->deviceFd, bSync, CMD_I_SBMNG_CLOSE, &cmd, NULL, (union RequestIocb *)iocb,
                   pParam, CheckResultCloseSb, suspend ? IsSuspendedCloseSb : NULL, NULL, 0);
    if (ret != 0)
    {
        goto FreeParam;
    }

    if (bSync)
    {
        // For synchronous operation, command result is set
        status = iocb->common.status;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;
    }
    goto ExitProc;

FreeParam:
    free(pParam);    // Released only in case of an error; otherwise, CheckResult releases it
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(
        pQos);    // Decremented only in case of an error; otherwise, CheckResult decrements it
ExitProc:
    return status;
}

struct SendCloseNotifyParam
{
    struct SEFCloseSuperBlockIOCB iocb;
    struct SEFQoSHandle_ *pQos;
    uint16_t pid;
};

STATIC void SendCloseNotifyComplete(struct SEFCommonIOCB *common)
{
    struct SendCloseNotifyParam *pParam = (void *)common;

    if (pParam->iocb.common.status.error == 0)
    {
        SendCloseNotification(pParam->pQos, pParam->iocb.flashAddress, pParam->pid,
                              pParam->iocb.common.status.info);
    }
    free(pParam);
}

STATIC void SendAcoCloseNotification(struct SEFQoSHandle_ *pQos, struct AcoEntry *acoe, uint16_t pid)
{
    struct SEFFlashAddress fla = {acoe->nfa};
    struct SendCloseNotifyParam *pParam = calloc(1, sizeof(*pParam));
#if CC_ARR_VALIDATION    // LD2 is returning nfa of -1
    if (fla.bits == SEFAutoAllocate.bits)
    {    // FAR should have closed the SB
        ULOG_ERROR("Aco nfa is -1, for ua/ofa %#lx/%#lx using ofa\n", acoe->ua, acoe->ofa);
        fla.bits = acoe->ofa;
    }
#endif

    pParam->pid = pid;
    pParam->pQos = pQos;
    pParam->iocb.flashAddress = fla;
    pParam->iocb.common.param1 = pParam;
    pParam->iocb.common.complete_func = SendCloseNotifyComplete;
    CloseSbProc(false, pQos, &pParam->iocb.common, false);
}

/**
 *  @brief	Structure of parameters retained for responding to Flush Super Block
 *  @note    Set to param of Req2Cmp at creating request
 */
struct FlushSbParam
{
    struct SEFQoSHandle_ *pQos;    //!< QoSD handle
    struct GetLogPageSuperBlockInformation sbInfo;
    bool freeIocb;
};

/**
 *  @brief	Checks whether to suspend Flush Super Block response
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @retval	true:  being written
 *  @retval	false: not being written (or no need to suspend)
 *  @note	Checks whether to suspend Flush Super Block response and registers suspend information
 * in case of suspension
 */
STATIC bool IsSuspendedFlushSb(struct ReqToComplete *pReq2Cmp)
{
    bool bRet = false;
    struct FlushSbParam *pParam = (struct FlushSbParam *)pReq2Cmp->param;
    struct SEFQoSHandle_ *pQos = pParam->pQos;
    uint32_t sbId = pReq2Cmp->result & UINT32_MAX;

    if ((pReq2Cmp->status != 0) || atomic_load(&pQos->bClosingFlag) || (sbId != pParam->sbInfo.SBID))
    {
        goto ExitProc;
    }

    if (DeviceInfoIsWritingSb(pQos, sbId))
    {
        DeviceInfoAddSuspendListByWriting(pQos, sbId, pReq2Cmp);
        // If not writing post add, unsuspend manually
        if (!DeviceInfoIsWritingSb(pQos, sbId))
        {
            CompleteSuspendedWrites(pQos, sbId);
        }
        bRet = true;
        ULOG_SBCMD_DBG("Suspend FlushSb sbid=0x%04x\n", sbId);
    }

ExitProc:
    return bRet;
}

/**
 *  @brief	Responds to Flush Super Block
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
STATIC void CheckResultFlushSb(struct ReqToComplete *pReq2Cmp)
{
    struct FlushSbParam *pParam = (struct FlushSbParam *)pReq2Cmp->param;
    struct FlushSuperBlockIOCB *pIocb = (struct FlushSuperBlockIOCB *)pReq2Cmp->pIocb;
    struct SEFQoSHandle_ *pQos = pParam->pQos;
    uint32_t sbid = pReq2Cmp->result;
    bool cc = false;

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        goto ExitProc;
    }

    cc = !!(pReq2Cmp->result & (UINT64_C(1) << 63));
    ULOG_SBCMD_DBG("sbid=0x%04x cc=%d\n", sbid, cc);

    if (pParam->sbInfo.SBID != sbid)
    {
        ULOG_ERROR("mismatched sb id!! req=0x%04x, ret=0x%04x\n", pParam->sbInfo.SBID, sbid);
        pReq2Cmp->status = REQUEST_MISMATCH_SBID;
    }

    pIocb->distanceToEndOfSuperBlock = pParam->sbInfo.CAP - pParam->sbInfo.ADUPTR;

ExitProc:
    CompleteIocbProc(pReq2Cmp);

    if (cc)    // if it caused sb to close
    {
        SendCloseNotification(pQos, GetFlashAddress(pQos, pQos->qosId, sbid), pParam->sbInfo.PID,
                              pParam->sbInfo.CAP);
        FlushCloseNotification(pQos, sbid);
    }

    QOSD_REQUEST_COUNT_DEC(pParam->pQos);

    if (pParam->freeIocb)
    {
        free(pReq2Cmp->pIocb);
    }
    free(pParam);
}

/**
 *  @brief	Converts SB List/Log DIISBS to API's SEFSuperBlockState
 *  @param	[in] sbsState: DIISBS field from SB List or SB Info log page
 *  @return SEFSuperBlockState for the passed in sbsState
 *  @note	Caller must NULL-check parameters
 */
STATIC enum SEFSuperBlockState DiiSbs2SbState(uint8_t sbState)
{
    enum SEFSuperBlockState state = kSuperBlockOpenedByErase;

    if (sbState & kSbOpenByPlacementID)
    {
        state = kSuperBlockOpenedByPlacementId;
    }
    if (sbState & (kSbClosed | kSbFree))
    {
        state = kSuperBlockClosed;
    }
    return state;
}

/**
 *  @brief	Converts SB List/Log DIISBS to API's SEFSuperBlockType
 *  @param	[in] sbsState: DIISBS field from SB List or SB Info log page
 *  @return SEFSuperBlockType for the passed in sbsState
 *  @note	Caller must NULL-check parameters
 */
STATIC enum SEFSuperBlockType DiiSbs2Type(uint8_t sbState)
{
    enum SEFSuperBlockType type = kForWrite;

    if (sbState & kSbPSLC)
    {
        type = kForPSLCWrite;
    }
    return type;
}

/**
 *  @brief	Retrieves information of SB specified by sbId
 *  @param	[in] deviceFd: File Descriptor to domain NS
 *  @param	[in] vdId: Virtual Device ID
 *  @param	[in] qosId: QoS Domain ID
 *  @param	[in] sbId: Super Block ID
 *  @param  [in] getDefects: True returns defect map in pSbRecord->defects
 *  @param	[out] pSbRecord: Super Block Record
 *  @return	0: SB information successfully retrieved, -1: SB information retrieval failed, -2:
 * retrieved QOSDID in SB information does not match with ID in QoSD handle
 *  @note	Caller must NULL-check parameters
 */
STATIC int GetSuperBlockInfoBase(struct SEFQoSHandle_ *pQoS,
                                 int deviceFd,
                                 uint16_t vdId,
                                 uint16_t qosId,
                                 uint32_t sbId,
                                 bool getDefects,
                                 struct SEFSuperBlockInfo *pSbRecord)
{
    int ret = -1;
    struct nvme_passthru_cmd64 cmd;
    uint32_t infoSize;
    uint32_t cmdStat;
    struct GetLogPageSuperBlockInformation *pSbInfo;
    enum SbState sbState;
    int cmdIssueResult;
    struct SEFHandle_ *pSef = pQoS->pSefHandle;

    infoSize = sizeof(*pSbInfo);
    SEFAdmGetLogPageSBInfo(&cmd, NULL, infoSize, vdId, sbId);
    cmdIssueResult = IssueRecvDataCmd(deviceFd, true, CMD_A_GLOGP_SBINFO, &cmd, &cmdStat, NULL,
                                      NULL, NULL, (void **)&pSbInfo, infoSize);
    if ((cmdIssueResult != 0) || (cmdStat != 0))
    {
        ret = cmdIssueResult ?: cmdStat;
        goto ExitProc;
    }

    if (qosId != pSbInfo->QOSDID)
    {
        ULOG_ERROR("sbId=%u, qosId=%u, pSbInfo->QOSDID=%u, SBS=0x%x\n", sbId, qosId,
                   pSbInfo->QOSDID, pSbInfo->SBS);
        ret = -EINVAL;
        goto FreeSbInfo;
    }

    if (vdId != pSbInfo->VDID)
    {
        ULOG_ERROR("sbId=%u, vdId=%u, pSbInfo->VDID=%u, SBS=0x%x\n", sbId, vdId, pSbInfo->VDID,
                   pSbInfo->SBS);
        ret = -EPROTO;
        goto FreeSbInfo;
    }

    // Output log
    {
        ULOG_GETINFO_DATA("--Get Log Page : Sb Information--\n");
        ULOG_GETINFO_DATA("  SbId=0x%x\n", pSbInfo->SBID);
        ULOG_GETINFO_DATA("  SbState=0x%x\n", pSbInfo->SBS);
        ULOG_GETINFO_DATA("  PlacementId=0x%x\n", pSbInfo->PID);
        ULOG_GETINFO_DATA("  QosdId=0x%x\n", pSbInfo->QOSDID);
        ULOG_GETINFO_DATA("  PeCountIndex=0x%x\n", pSbInfo->PECI);
        ULOG_GETINFO_DATA("  DataIntegrityIndex=0x%x\n", pSbInfo->DII);
        ULOG_GETINFO_DATA("  Capacity=0x%x\n", pSbInfo->CAP);
        ULOG_GETINFO_DATA("  AduPointer=0x%x\n", pSbInfo->ADUPTR);
        ULOG_GETINFO_DATA("  TimeLeft=0x%x\n", pSbInfo->TL);
        ULOG_GETINFO_DATA("  NumberOfDefectivePlanes=0x%x\n", pSbInfo->NDPL);
        ULOG_GETINFO_DATA("  EraseSerialNumber=0x%x\n", pSbInfo->ESN);
        ULOG_GETINFO_DATA("------------------------\n");
    }

    sbState = (enum SbState)pSbInfo->SBS;

    // Set output parameters
    pSbRecord->flashAddress = GetFlashAddress(pQoS, qosId, sbId);
    pSbRecord->eraseOrder = pSbInfo->ESN;
    pSbRecord->writableADUs = pSbInfo->CAP;
    pSbRecord->writtenADUs = pSbInfo->ADUPTR;
    pSbRecord->placementID.id = pSbInfo->PID;
    pSbRecord->PEIndex = pSbInfo->PECI;
    pSbRecord->state = DiiSbs2SbState(pSbInfo->SBS);
    pSbRecord->type = DiiSbs2Type(pSbInfo->SBS);

    if (sbState == kSbClosed)
    {
        if (pSbRecord->writableADUs != pSbRecord->writtenADUs)
        {
            pSbRecord->writtenADUs = pSbRecord->writableADUs;
        }
    }
    else if ((sbState == kSbFree) || (sbState == kSbOffline))
    {
        ULOG_ERROR("super block state is free or offline!! sbid=0x%x, state=0x%x\n", pSbInfo->SBID,
                   pSbInfo->SBS);
        ret = -EPROTO;
        goto FreeSbInfo;
    }

    if (pSbRecord->writableADUs < pSbRecord->writtenADUs)
    {
        ULOG_ERROR(
            "(SbInfo)writableADUs < (SbInfo)writtenADUs sbId=%u, qosId=%u, writableADUs=0x%x, "
            "writtenADUs=0x%x\n",
            sbId, qosId, pSbRecord->writableADUs, pSbRecord->writtenADUs);
        ULOG_ERROR(
            "(SbInfo)NDPL=0x%x, (Sef)numPlanesPerDie=0x%x, "
            "(Sef)numADUsPerPlane=0x%x, (SefInfo)numPages=0x%x\n",
            pSbInfo->NDPL, pSef->numPlanesPerDie, pSef->numADUsPerPlane, pSef->pSefInfo->numPages);
        ret = -EPROTO;
        goto FreeSbInfo;
    }

    ret = 0;

    if (getDefects)
    {
        uint32_t dmSizeBits;

        dmSizeBits = pSef->numPlanesPerDie * pQoS->sbSize;
        memcpy(pSbRecord->defects, pSbInfo->DBM, (7 + dmSizeBits) / 8);
    }

    // Output log
    {
        ULOG_GETINFO_DATA("--SEFSuperBlockRecord--\n");
        ULOG_GETINFO_DATA("  flashAddress=0x%016lx\n", pSbRecord->flashAddress.bits);
        ULOG_GETINFO_DATA("  eraseOrder=0x%x\n", pSbRecord->eraseOrder);
        ULOG_GETINFO_DATA("  writableADUs=0x%x\n", pSbRecord->writableADUs);
        ULOG_GETINFO_DATA("  writtenADUs=0x%x\n", pSbRecord->writtenADUs);
        ULOG_GETINFO_DATA("  placementID=0x%x\n", pSbRecord->placementID.id);
        ULOG_GETINFO_DATA("  PEIndex=0x%x\n", pSbRecord->PEIndex);
        ULOG_GETINFO_DATA("  state=0x%x\n", pSbRecord->state);
        ULOG_GETINFO_DATA("------------------------\n");
    }

FreeSbInfo:
    free(pSbInfo);
ExitProc:
    return ret;
}

/**
 *  @brief	Retrieves information of SB specified by sbId
 *  @param	[in] pQos: QoS Handle
 *  @param	[in] sbId: Super Block ID
 *  @param  [in] getDefects: true, returns defect map in pSbRecord->defects
 *  @param	[out] pSbRecord: Super Block Record
 *  @return	0: SB information successfully retrieved, -1: SB information retrieval failed, -2:
 * retrieved QOSDID in SB information does not match with ID in QoSD handle
 *  @note	Caller must NULL-check parameters
 */
STATIC int GetSuperBlockInfo(struct SEFQoSHandle_ *pQos,
                             uint32_t sbId,
                             bool getDefects,
                             struct SEFSuperBlockInfo *pSbRecord)
{
    return GetSuperBlockInfoBase(pQos, pQos->deviceFd, pQos->vdId, pQos->qosId, sbId, getDefects,
                                 pSbRecord);
}

#if CC_FLUSH_SYNC_COPY
STATIC void FlushCopyComplete(struct SEFCommonIOCB *common);
#endif

/**
 *  @brief	Internal processing for SEFFlushSuperBlock
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Flush Super Block
 *  @param  [in] freeIocb: Free iocb after flush is complete
 *  @param  [in] nlcFlush: True if flushing an NLC block (affects timeout)
 *  @details	Area of arbitrary parameter for responding to Flush Super Block is allocated.
 *  @details	This function releases arbitrary parameter area if command issuing failed.
 *  @note    CheckResultExt must release arbitrary parameter area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info: fixed to 0
 */
STATIC struct SEFStatus FlushSbProc(
    bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common, bool freeIocb, bool nlcFlush)
{
    struct FlushSuperBlockIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct nvme_passthru_cmd64 cmd;
    struct FlushSbParam *pParam;
    int ret;
    bool override = !!(common->flags & kSefIoFlagOverride);
    struct SEFAllocateOverrides *overrides = override ? &iocb->overrides : NULL;
#if CC_FLUSH_SYNC_COPY
    bool noSuspend = common->complete_func == FlushCopyComplete;
#endif

    QOSD_REQUEST_COUNT_INC(pQos);

    // Check parameter
    if (pQos->qosId != GetQosdId(iocb->flashAddress))
    {
        ULOG_ERROR("argument error!! flashAddress=0x%016lx\n", iocb->flashAddress.bits);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Prepare parameters
    pParam = (struct FlushSbParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }

    // Flush SB
    SEFNvmSuperBlockManagement(
        &cmd, &pParam->sbInfo, sizeof(pParam->sbInfo), SEF_NVMOPE_SBMANAGEMENT_FLUSH, pQos->qosId,
        GetSbId(pQos, iocb->flashAddress), overrides, nlcFlush ? pQos->pSefHandle->writeTimeout : 0);
    pParam->freeIocb = freeIocb;
    pParam->pQos = pQos;
#if CC_FLUSH_SYNC_COPY
    ret = IssueCmd(pQos->deviceFd, bSync, CMD_I_SBMNG_FLUSH, &cmd, NULL, (union RequestIocb *)iocb,
                   pParam, CheckResultFlushSb, noSuspend ? NULL : IsSuspendedFlushSb, NULL, 0);
#else
    ret = IssueCmd(pQos->deviceFd, bSync, CMD_I_SBMNG_FLUSH, &cmd, NULL, (union RequestIocb *)iocb,
                   pParam, CheckResultFlushSb, IsSuspendedFlushSb, NULL, 0);
#endif
    if (ret != 0)
    {
        goto FreeParam;
    }

    if (bSync)
    {
        // For synchronous operation, command result is set
        status = iocb->common.status;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;
    }

    goto ExitProc;

FreeParam:
    free(pParam);    // Released only in case of an error; otherwise, CheckResult releases it
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(
        pQos);    // Decremented only in case of an error; otherwise, CheckResult decrements it
ExitProc:
    return status;
}

/**
 *  @brief	Structure of parameters retained for responding to Release Super Block
 *  @note    Set to param of Req2Cmp at creating request
 */
struct ReadParam
{
    struct SEFQoSHandle_ *pQos;    //!< QoSD handle
    uint8_t *pData;                //!< Area for Read Data (set to DPTR)
    struct iovec *iov;
    int32_t iovcnt;
    uint32_t aduOff;
    uint32_t numADU;
};

/**
 *  @brief	Responds to Read
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
void CheckResultRead(struct ReqToComplete *pReq2Cmp)
{
    struct ReadParam *pParam = (struct ReadParam *)pReq2Cmp->param;
    uint32_t aduSize = GET_ADU_SIZE(pParam->pQos->pSefHandle);
    bool isZeroCopy = DoZeroCopyRead(&pReq2Cmp->pIocb->read);

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        goto ExitProc;
    }

    if (!isZeroCopy && pReq2Cmp->pIocb->read.iov)
    {
        // Divide data read to iovs
        CopyIovFromBuff(pReq2Cmp->pIocb->read.iov, pReq2Cmp->pIocb->read.iovcnt,
                        pReq2Cmp->pIocb->read.iovOffset + (pParam->aduOff * aduSize),
                        pParam->numADU * aduSize, pParam->pData);
    }

ExitProc:

    CompleteIocbProc(pReq2Cmp);

    QOSD_REQUEST_COUNT_DEC(pParam->pQos);

    if (!isZeroCopy)
    {
        free(pParam->pData);
    }
    free(pParam->iov);
    free(pParam);
}

/**
 *  @brief	Retrieves Root Pointer from aduOffset of fla
 *  @param	[in] pQos: pointer to QoSD handle
 *  @param	[in] fla : flashAddress including aduOffset
 *  @param	[out] *pRootPtr: retrieved Root Pointer (flashAddress)
 *  @note    Checks range of aduOffset
 *  @retval	 0: successfully retrieved; *pRootPtr is valid
 *  @retval	-1: retrieval failed; check failed or QoSD information retrieval caused an error
 */
STATIC int GetRootPointer(struct SEFQoSHandle_ *pQos,
                          struct SEFFlashAddress fla,
                          struct SEFFlashAddress *pRootPtr)
{
    int ret = -1;
    int cmdRet;
    uint32_t rootPtrIndex;
    uint32_t cmdStat;
    struct SEFHandle_ *pSef = pQos->pSefHandle;
    struct nvme_passthru_cmd64 cmd;

    // Retrieve Root Pointer Index (ADU Offset of FLA)
    rootPtrIndex = GetAduOffset(pQos, fla);
    if (rootPtrIndex >= (uint32_t)pSef->pSefInfo->maxRootPointers)
    {
        ULOG_ERROR("invalid root pointer index!! rootPtrIndex=%u\n", rootPtrIndex);
        ret = -EINVAL;
        goto ExitProc;
    }

    // Retrieve QoSD information
    SEFAdmGetFeatureQoSDManagement(&cmd, NULL, 0, SEF_FEATURE_ROOTPOINTER, rootPtrIndex, pQos->qosId);
    cmdRet =
        IssueNoDataCmd(pQos->deviceFd, true, CMD_A_GFEAT_QOSD_INFO, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((cmdRet != 0) || (cmdStat != 0))
    {
        goto ExitProc;
    }

    // Retrieve Root Pointer
    pRootPtr->bits = cmd.result;

    ret = 0;

ExitProc:
    return ret;
}

struct SEFStatus GetDomainNumSb(struct SEFQoSHandle_ *pQos, uint32_t *numSb)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = pQos->pSefHandle;
    struct SefNsIdentify *pSefNsIdentify = 0;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    uint32_t size = TO_UPPER_DWORD_SIZE(sizeof(*pSefNsIdentify));

    SEFAdmSefNsIdentify(&cmd, pQos->qosId, NULL, size);
    int ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GFEAT_QOSD_INFO, &cmd, &cmdStat, NULL,
                               NULL, NULL, (void **)&pSefNsIdentify, size);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
    }
    else
    {
        status.error = 0;
        *numSb = pSefNsIdentify->sbsta.nsb + pSefNsIdentify->psbsta.nsb;
    }
    if (pSefNsIdentify)
    {
        free(pSefNsIdentify);
    }
    return status;
}

static struct SEFFlashAddress FlashAddressAddOffset(SEFQoSHandle QoSHandle,
                                                    struct SEFFlashAddress flashAddress,
                                                    uint32_t ADUOffset)
{
    struct SEFQoSDomainID QoSDomainID;
    uint32_t blockNumber;
    uint32_t originalADUOffset;

    SEFParseFlashAddress(QoSHandle, flashAddress, &QoSDomainID, &blockNumber, &originalADUOffset);

    originalADUOffset += ADUOffset;
    return SEFCreateFlashAddress(QoSHandle, QoSDomainID, blockNumber, originalADUOffset);
}

/**
 *  @brief	Internal processing for SEFReadWithPhysicalAddress and SEFReadWithPhysicalAddressAsync
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Read
 *  @details	Area of arbitrary parameter for responding to Read is allocated.
 *  @details	This function releases arbitrary parameter area if command issuing failed.
 *  @note    CheckResultExt must release arbitrary parameter area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info: fixed to 0
 */
STATIC struct SEFStatus ReadProcSegment(bool bSync,
                                        struct SEFQoSHandle_ *pQos,
                                        struct SEFCommonIOCB *common,
                                        uint32_t aduOff,
                                        uint32_t numADU)
{
    struct SEFReadWithPhysicalAddressIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = pQos->pSefHandle;
    size_t aduSize = GET_ADU_SIZE(pSef);    // size_t because of math below
    bool isShortage;
    uint32_t size = 0;
    struct ReadParam *pParam;
    struct SEFFlashAddress readFla;
    int ret;
    uint32_t offset = aduOff * aduSize;
    struct RequestInfoExt req;

    struct SEFUserAddress userAddr = iocb->userAddress;
    bool override = !!(common->flags & kSefIoFlagOverride);
    uint32_t readFifo = 0xffffffff;
    uint16_t readWeight = 0;

    QOSD_REQUEST_COUNT_INC(pQos);

    // Check parameters - there must be metadata or iov data
    if (iocb->numADU == 0 || (iocb->metadata == NULL && (iocb->iov == NULL || iocb->iovcnt == 0)))
    {
        ULOG_ERROR("argument error!! numADU=%u, iov=%p, iovcnt=%u metadata=%p\n", iocb->numADU,
                   iocb->iov, iocb->iovcnt, iocb->metadata);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    if (override)
    {
        if (iocb->overrides.readQueue < pQos->nrq)
        {
            readFifo = pQos->read_fifos[iocb->overrides.readQueue];
        }
        readWeight = iocb->overrides.readWeight;
    }

    // Check that total of iovs is equal to or larger than read size
    if (iocb->iov)
    {
        size = numADU * aduSize;
        isShortage = IsShortageIovLen(iocb->iov, iocb->iovcnt, iocb->iovOffset + offset, size);
        if (isShortage)
        {
            size_t len = GetIovBuffLen(iocb->iov, iocb->iovcnt, iocb->iovOffset);
            ULOG_ERROR(
                "iov area is less than data length!! area=%lu, offset=%lu, length=%u, adu num=%u\n",
                len, iocb->iovOffset, size, iocb->numADU);
            status.error = -EINVAL;
            goto DecrementProcReq;
        }
    }

    if ((GetQosdId(iocb->flashAddress) == 0) && (GetSbId(pQos, iocb->flashAddress) == 0))
    {
        // If both QoSDID and SBID are 0, use Root Pointer as read FLA
        ret = GetRootPointer(pQos, iocb->flashAddress, &readFla);
        if (ret != 0)
        {
            status.error = ret;
            goto DecrementProcReq;
        }
    }
    else
    {
        readFla = iocb->flashAddress;
    }

    readFla = FlashAddressAddOffset(pQos, readFla, aduOff);
    if (userAddr.unformatted != SEFUserAddressIgnore.unformatted)
    {
        userAddr = SEFCreateUserAddress(SEFGetUserAddressLba(userAddr) + aduOff,
                                        SEFGetUserAddressMeta(userAddr));
    }

    // Match with QosId
    if (pQos->qosId != GetQosdId(readFla))
    {
        ULOG_ERROR("qosId of fla is unmatch!! qosId=%u, qosId of fla=%u\n", pQos->qosId,
                   GetQosdId(readFla));
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Prepare parameters
    pParam = (struct ReadParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }
    pParam->pQos = pQos;
    pParam->pData = NULL;
    pParam->aduOff = aduOff;
    pParam->numADU = numADU;

    // Read
    void *metadata = NULL;
    if (iocb->metadata)
    {
        metadata = aduOff * GET_META_SIZE(pSef) + (char *)iocb->metadata;
    }

    // Allocate memory for request
    req.ppDevBuff = DoZeroCopyRead(iocb) ? NULL : (void **)&pParam->pData;
    req.buffSize = size;
    ret = AllocateReqInfoExt(&req.pReq2Cmp, NULL, NULL, 0, &req.pSem, req.ppDevBuff, req.buffSize);
    if (ret != 0)
    {
        ret = -ENOMEM;
        goto ExitProc;
    }
    if (DoZeroCopyRead(iocb))
    {
        pParam->iovcnt = CloneIov(iocb->iov, iocb->iovcnt, iocb->iovOffset + aduOff * aduSize,
                                  numADU * aduSize, &pParam->iov);
        if (pParam->iovcnt < 0)
        {
            goto FreeParam;
        }
    }
    else
    {
        pParam->iov = (struct iovec *)calloc(1, sizeof(struct iovec));
        if (!pParam->iov)
        {
            goto FreeParam;
        }

        pParam->iov[0].iov_base = *req.ppDevBuff;
        pParam->iov[0].iov_len = numADU * aduSize;
        pParam->iovcnt = 1;
    }
    SEFNvmReadCommand(&req.pReq2Cmp->submit, pParam->iov, pParam->iovcnt, metadata, pQos->qosId,
                      userAddr, numADU, readFla, readFifo, readWeight);
    req.pReq2Cmp->syncOverride = bSync;
    req.pReq2Cmp->bSync = false;
    req.pReq2Cmp->type = kIoctlIoCommand;
    req.pReq2Cmp->pSyncSemaphore = req.pSem;
    req.pReq2Cmp->param = pParam;
    req.pReq2Cmp->pIocb = (union RequestIocb *)iocb;
    req.pReq2Cmp->CommandComplete = Complete;
    req.pReq2Cmp->CheckResultExt = CheckResultRead;
    req.pReq2Cmp->CheckSuspendFunc = NULL;

    // Request command issuing
    ret = Request(pQos->deviceFd, req.pReq2Cmp);
    if (ret != 0)
    {
        free(req.pReq2Cmp);
        free(req.pSem);
        goto FreeParam;
    }

    status.error = 0;

    goto ExitProc;

FreeParam:
    free(pParam);    // Released only in case of an error; otherwise, CheckResult releases it
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(
        pQos);    // Decremented only in case of an error; otherwise, CheckResult decrements it
ExitProc:
    return status;
}

#ifdef USE_READ_CACHE
bool readFromCache(struct SEFQoSHandle_ *pQos, struct SEFReadWithPhysicalAddressIOCB *iocb, uint32_t aduOff)
{
    void *buffer;
    uint32_t key = (GetSbId(pQos, iocb->flashAddress) << 16) |
                   (GetAduOffset(pQos, iocb->flashAddress) + aduOff);
    pthread_mutex_lock(&pQos->mutex);
    buffer = LHTget(pQos->cache, key);
    if (buffer)
    {
        uint32_t aduSize = GET_ADU_SIZE(pQos->pSefHandle);
        CopyIovFromBuff(iocb->iov, iocb->iovcnt, aduOff * aduSize, aduSize, buffer);
    }
    pthread_mutex_unlock(&pQos->mutex);
    return buffer;
}
#endif

STATIC struct SEFStatus ReadProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common)
{
    struct SEFReadWithPhysicalAddressIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    uint32_t maxAdus = MaxAduCount(pQos);
    int32_t remainingADU = iocb->numADU;
    uint32_t aduOff = 0;
    int i = 0;

    if (iocb->numADU == 0)
    {
        ULOG_ERROR("argument error!! numADU=%u\n", iocb->numADU);
        status.error = -EINVAL;
    }

    SEF_ASSERT(iocb->common.status.error == 0);
    SEF_ASSERT(iocb->common.status.info == 0);
#ifdef USE_READ_CACHE
    struct CacheRun
    {
        uint32_t offset;
        uint32_t length;
    } *cached = calloc(sizeof(struct CacheRun), iocb->numADU);
    SEF_ASSERT(cached);
    uint32_t numRuns = 0;
    uint32_t currentRun = 0;
    while (i < iocb->numADU)
    {
        if (readFromCache(pQos, iocb, i))
        {
            if (cached[currentRun].length == 0)
            {
                cached[currentRun].offset = i;
                numRuns++;
            }
            cached[currentRun].length++;
        }
        else if (cached[currentRun].length)
        {
            currentRun++;
        }
        ++i;
    }
    currentRun = 0;
#endif
    i = 0;
    atomic_store((atomic_int_least32_t *)&iocb->common.reserved, 1);
    while (remainingADU > 0)
    {
        uint32_t numADU = remainingADU > maxAdus ? maxAdus : remainingADU;
#ifdef USE_READ_CACHE
        if (currentRun < numRuns)
        {
            uint32_t cacheOffset = cached[currentRun].offset;
            uint32_t cacheLength = cached[currentRun].length;
            if (aduOff >= cacheOffset && aduOff < cacheOffset + cacheLength)
            {
                aduOff = cacheOffset + cacheLength;
                numADU -= cacheLength;
            }
            if (numADU == 0)
            {
                status.error = 0;
                break;
            }
        }
#endif
        ++i;
        atomic_fetch_add((atomic_int_least32_t *)&iocb->common.reserved, 1);
        status = ReadProcSegment(bSync, pQos, &iocb->common, aduOff, numADU);
        if (status.error)
        {
            if (!bSync)
            {
                atomic_fetch_sub((atomic_int_least32_t *)&iocb->common.reserved, 1);
            }
            break;
        }
        aduOff += numADU;
        remainingADU -= numADU;
    }

    int32_t rc = atomic_fetch_sub((atomic_int_least32_t *)&iocb->common.reserved, 1);
    SEF_ASSERT(rc > 0);
    if (rc == 1)
    {
        void (*complete_func)(struct SEFCommonIOCB *) = iocb->common.complete_func;
        if (status.error && iocb->common.status.error == 0)
        {
            iocb->common.status = status;
        }
        iocb->common.flags |= kSefIoFlagDone;
        if (complete_func)
        {
            complete_func(&iocb->common);
        }
        status.error = 0;
        status.info = 0;
    }

#ifdef USE_READ_CACHE
    free(cached);
#endif
    return status;
}

/**
 *  @brief	Structure of parameters common across Fused commands
 *  @note    Set to param of Req2Cmp at creating request
 */
struct SharedWriteParam
{
    uint32_t numTentativeAddr;    //!< Number of Tentative Addresses received
    uint32_t sbid;                //!< Super block id if writing by sbid (bySb is true)
    uint32_t numUnwrittenADUs;    //!< Number of unwritten ADUs left in the super block, for returning distanceToEndOfSuperBlock
    uint32_t notifyIoVecCnt;
    const struct iovec *notifyIoVec;
    uint8_t *pHeadOfDataBuf;                  //!< Bounce buffer - Used for freeing pData
    uint32_t numADU;                          //!< Number of ADUs (copied from the iocb)
    uint16_t pid;                             //!< PID when writing by PID (bySb is false)
                                              //   else SEFPlacementIDIgnore
    atomic_uint_least16_t numAco;             //!< Number of items in acoList
    atomic_uint_least16_t sendRecvCounter;    //!< Incremented at issuing command
                                              // and decremented at receiving response to
                                              // Fused 2nd command. When decrement makes
                                              // it 0, the response is considered the last
                                              // response to PartialWrite
    bool bSync;                               //!< Sync/async flag
    bool bySb;                                //!< Write by SB (not auto allocate)
    bool pSLC;                                //!< FLA was SEFAutoAllocatePSLC
    sem_t syncSemaphore;                      //!< Semaphore for synchronous write
    struct sefSlist arrList;                  //!< pParams of completed FARs to process
                                              //   when NLWs complete
    struct sefSlist acoList;                  //!< pParams of completed NLWs to process
                                              //   after all NLWs complete
};

/**
 *  @brief	Structure of parameters retained for responding to Write
 *  @note    Set to param of Req2Cmp at creating request
 */
struct WriteParam
{
    struct sefSlistNode arrlink;    //!< Link for SharedWriteParam.arrList
    struct sefSlistNode acolink;    //!< Link for SharedWriteParam.acoList
    struct SEFQoSHandle_ *pQos;     //!< QoSD handle
    struct AddressRecordRequest *pArr;    //!< Area for Address Record Request (set to DPTR of 1st command)
    struct AddressChangeOrder *pAco;    //!< Area for Address Change Order information
    struct iovec *iovec;                //!< Area for Write Data
    uint8_t iovcnt;
    atomic_int refCnt;     //!< Holds a cnt for the FAR and ACO, whoever decrements to 0
                           //!< processes the Address Change Order
    uint32_t nlwStatus;    //!< pReq2Cmp->status from NLW
    int acoStatus;         //!< Status of submit of aco log page
    uint32_t farSn;        //!< FPWC s/n when writing by PID
    bool fua;              //!< True if forceUnitAccess was set for the NLC (padding may be added)
    struct SharedWriteParam *pSharedWriteParam;    //!< Parameter common across Fused commands
    uint32_t aduOff;    //!< Offset into buffer for this segment for writes greater than MaxAduCount()
    uint32_t numADU;              //!< Number of ADU in this segment
    uint32_t numReq;              //!< Number of ADUs returned in FAR
    uint32_t numWritten;          //!< Number of ADUs actually written
    struct SEFUserAddress sua;    //!< Starting User Address
    struct SEFWriteWithoutPhysicalAddressIOCB *pIocb;
};

STATIC void NLCQIocbComplete(struct SEFCommonIOCB *iocb)
{
    sem_post((void *)iocb->param1);
}

static void FarNlwCheckResultFinish(struct WriteParam *pParam);

#ifdef USE_READ_CACHE
bool writeToCache(struct SEFQoSHandle_ *pQos,
                  struct SEFWriteWithoutPhysicalAddressIOCB *iocb,
                  uint64_t bits,
                  uint32_t aduOff)
{
    struct SEFFlashAddress fla = {.bits = bits};
    uint32_t aduSize = GET_ADU_SIZE(pQos->pSefHandle);
    void *buffer = calloc(1, aduSize);
    SEF_ASSERT(buffer);
    if (!buffer)
    {
        return false;
    }
    uint32_t key = (GetSbId(pQos, fla) << 16) | (GetAduOffset(pQos, fla) + aduOff);
    CopyIovToBuff(iocb->iov, iocb->iovcnt, aduOff, aduSize, buffer);
    pthread_mutex_lock(&pQos->mutex);
    void *existing = LHTput(pQos->cache, key, buffer);
    SEF_ASSERT(!existing);
    pthread_mutex_unlock(&pQos->mutex);
    return true;
}

bool deleteFromCache(struct SEFQoSHandle_ *pQos, uint64_t bits)
{
    struct SEFFlashAddress fla = {.bits = bits};
    uint32_t key = (GetSbId(pQos, fla) << 16) | (GetAduOffset(pQos, fla));
    pthread_mutex_lock(&pQos->mutex);
    void *existing = LHTdelete(pQos->cache, key);
    pthread_mutex_unlock(&pQos->mutex);
    if (existing)
    {
        free(existing);
    }
    return existing;
}
#endif

STATIC void ProcessArrList(struct WriteParam *pParam, int refAdj)
{
    // I/O error (empty ARR) with write by sbid
    if (pParam->pArr->nsb == 0 && pParam->pSharedWriteParam->bySb)
    {
        uint32_t sbId = GetSbId(pParam->pQos, pParam->pIocb->flashAddress);

        SEF_ASSERT(refAdj < 0);
        ULOG_NOTICE("Empty arr for SBID 0x%x\n", sbId);
        // Dec refcnt added when the far was issued
        ManageResponseOrder(pParam->pQos, sbId, 0, NULL, kMroDecIoRef);
    }
    for (uint32_t i = 0; i < pParam->pArr->nsb; ++i)
    {
        bool lastSB = (i == pParam->pArr->nsb - 1);
        bool lastSBClosed;
        struct SEFFlashAddress fla = pParam->pArr->sbis[i].sfla;
        uint32_t sbId = GetSbId(pParam->pQos, fla);
        lastSBClosed = (pParam->pArr->nradu == 0);    //<= closeTrigger);

#if CC_ARR_VALIDATION    // for the LD2
        SEF_ASSERT(pParam->pArr->sbis[i].nfla || !lastSBClosed);
        if (!pParam->pArr->sbis[i].nfla)
        {
            ULOG_ERROR("nfla is 0 at %d of %d\n", i, pParam->pArr->nsb);
            continue;
        }
        if (fla.bits == 0)
        {
            ULOG_ERROR("fla is 0 at %d of %d\n", i, pParam->pArr->nsb);
            continue;
        }
        if (GetQosdId(fla) != pParam->pQos->qosId)
        {
            ULOG_ERROR("fla qosd of %d does not match %d at %d of %d", GetQosdId(fla),
                       pParam->pQos->qosId, i, pParam->pArr->nsb);
            continue;
        }
        if (pParam->pSharedWriteParam->bSync && pParam->pSharedWriteParam->bySb)
        {
            uint32_t ioSbId = GetSbId(pParam->pQos, pParam->pIocb->flashAddress);

            if (ioSbId != sbId)
            {
                ULOG_ERROR("ioSbId %#x doesn't match arr sbId %#x, updating!!\n", ioSbId, sbId);
                sbId = ioSbId;
            }
        }
#endif
        SEF_ASSERT(refAdj);    // 0 not valid
        if (refAdj > 0)
        {
            // byPlId has to wait for the far to complete and process the arr
            // to increment the i/o count for each sbid.  bySb did this when the
            // FAR was issued and created the sbInfo when the SB was allocated.
            SEF_ASSERT(!pParam->pSharedWriteParam->bySb);
            ManageResponseOrder(pParam->pQos, sbId, pParam->pArr->sbis[i].nfla, NULL,
                                kMroCreate | kMroIncIoRef);
        }
        else
        {
            // byPlId and bySb use the arr to decrement the reference
            ManageResponseOrder(pParam->pQos, sbId, 0, NULL, kMroDecIoRef);
            if (!lastSB || lastSBClosed)
            {
                uint16_t pid = pParam->pSharedWriteParam->pid;

                SendCloseNotification(pParam->pQos,
                                      GetFlashAddress(pParam->pQos, pParam->pQos->qosId, sbId), pid,
                                      GetAduOffset(pParam->pQos, fla) + pParam->pArr->sbis[i].nfla);
            }
        }
#ifdef USE_READ_CACHE
        int j;
        for (j = 0; j < pParam->pArr->sbis[i].nfla; ++j)
        {
            deleteFromCache(pParam->pQos, pParam->pArr->sbis[i].sfla.bits + j);
        }
#endif
    }
}

/**
 *  @brief	Responds to FlashAddressRequest (1st) of FusedWrite
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @details	Retains TentativeAddress in ARR.
 *  @details	For PartialWrite response, write is retried for remaining data.
 *  @details	At retry, arbitrary parameter area for responding to Write is allocated.
 *  @details	At retry, area for storing ACO temporarily for PartialWrite is allocated (arbitrary
 *              parameter area).
 *  @details	At retry, areas for Req2Cmp, Sem, write data, ARR, and ACO are allocated.
 *  @details	At retry, Complete() releases memory for Req2Cmp and Sem for FusedWrite 1st command
 *              (this function releases them if command issuing fails).
 *  @details	At retry, Complete() releases memory for Req2Cmp and Sem for FusedWrite 2nd command
 *              (this function releases them if command issuing fails).
 *  @details	If command issuing of retry failed, this function releases areas for arbitrary
 *              parameter write data, ARR, and ACO.
 *  @details	If command error is received for retried write, normal operation is performed to
 *              process retained TentativeAddress.
 *  @details	Call to this function for 1st command releases arbitrary parameter area allocated
 *              for retrying write.
 *  @details	For PartialWrite response, ARR area of arbitrary parameter area is reused for
 *              retring write.
 *  @details	For non-PartialWrite response, ARR area of arbitrary parameter area is released.
 *  @details	User completion callback is executed here.
 *  @details	For flow control according to SB Write status, TentativeAddress information in ARR
 *              is stored in temporary area.
 */
STATIC void CheckResultFlashAddressRequest(struct ReqToComplete *pReq2Cmp)
{
    struct WriteParam *pParam = (struct WriteParam *)pReq2Cmp->param;
    struct SharedWriteParam *pSharedParam = pParam->pSharedWriteParam;
    struct SEFFlashAddress *pTentative = pReq2Cmp->pIocb->write.tentativeAddresses;
    uint32_t farSn = pParam->farSn;
    uint16_t pid = pSharedParam->pid;
    int ret;
    struct WriteParam *pRetryParam;
    struct TempAcoBuf *pNewTempAcoBuf = NULL;
    struct RequestInfoExt req[2];
    uint32_t sbId;
    uint32_t writeAdus;
    size_t aduSize = GET_ADU_SIZE(pParam->pQos->pSefHandle);    // size_t because of math below
    uint32_t maxAdus = MaxAduCount(pParam->pQos);
    bool bRetry = false;
    struct SEFUserAddress userAddress;
    uint32_t nextAduOff = pParam->aduOff;
    bool forceUnitAccess = false;
    uint32_t flaNum = pReq2Cmp->result & UINT32_MAX;

    ULOG_WTCMD_INF("pReq2Cmp->status=0x%016x\n", pReq2Cmp->status);

    // For abnormal completion
    pParam->numReq = flaNum;
    if (pReq2Cmp->status != 0)
    {
        SEF_ASSERT(!(pReq2Cmp->status == SEF_PARTIAL_WRITE && pParam->pArr->nsb > 1));
        SEF_ASSERT(pReq2Cmp->status != -EBUSY);    // should be handled by request.c
        SEF_ASSERT(pReq2Cmp->status != -EWOULDBLOCK);
        // If Partial Write, continue with remaining Write
        if (pReq2Cmp->status == SEF_PARTIAL_WRITE)
        {
            SEF_ASSERT(flaNum >= 1);

            ULOG_NOTICE("Partial far: requested = %u nadu written = %d, num sbs used = %d\n",
                        pParam->numADU, flaNum, pParam->pArr->nsb);

            // Check to see if it was partial due to arr filling up
            if (pParam->pArr->nsb == MAX_ARR_SBIS)
            {
                ULOG_NOTICE("Retrying partial far\n");
                nextAduOff += flaNum;
                bRetry = true;
            }
            else if (pSharedParam->bySb)
            {    // requested too much, assumed SB is full
                uint32_t sbId = GetSbId(pParam->pQos, pParam->pIocb->flashAddress);
                int numADUs = pParam->numADU - flaNum;

                // wrote less, adjust by deficit
                ManageResponseOrder(pParam->pQos, sbId, -numADUs, NULL, 0);
            }
            // else domain is full so stop issuing segments
        }
        else
        {
            // For non-partial write errors, contents of arr is undefined.
            // Force it to be be empty.
            pParam->pArr->nsb = 0;
            if (!pSharedParam->bySb)
            {
                ret = FPWCComplete(&pParam->pQos->pidState[pid].numProcessingFAR, farSn);
                SEF_ASSERT(!ret);
            }
            goto ExitProc;
        }
    }
    else
    {
        // Check for normal completion of a segmented write
        if (pSharedParam->numADU > flaNum + nextAduOff)
        {
            nextAduOff += flaNum;
            bRetry = true;
        }
    }

    for (uint32_t i = 0; i < pParam->pArr->nsb; ++i)
    {
        ULOG_WTCMD_INF("arr %d region fla num=%u, fla=0x%016lx, DefectPlane=0x%04x\n", i,
                       pParam->pArr->sbis[i].nfla, pParam->pArr->sbis[i].sfla.bits,
                       pParam->pArr->sbis[i].ndp);
    }

    ULOG_WTCMD_DBG("pCqEntry->far.flaNum=%u, arr->nsb=%d\n", flaNum, pParam->pArr->nsb);
    ULOG_WTCMD_DBG("SharedWriteParam->numTentativeAddr=%u \n", pSharedParam->numTentativeAddr);
    uint32_t Tentativeindex = pParam->aduOff;
    uint32_t nadu = 0;
    for (uint32_t i = 0; i < pParam->pArr->nsb; ++i)
    {
        nadu += pParam->pArr->sbis[i].nfla;
        for (uint32_t j = 0;
             j < pParam->pArr->sbis[i].nfla && Tentativeindex < pReq2Cmp->pIocb->write.numADU; j++)
        {
            pTentative[Tentativeindex++].bits = pParam->pArr->sbis[i].sfla.bits + j;
#ifdef USE_READ_CACHE
            writeToCache(pParam->pQos, &pReq2Cmp->pIocb->write, pParam->pArr->sbis[i].sfla.bits, j);
#endif
        }
    }
    if (nadu != flaNum)
    {
        ULOG_ERROR("Arr num adu (%d) doesn't match command response (%d)\n", nadu, flaNum);
    }
    if (nadu > pReq2Cmp->pIocb->write.numADU)
    {
        ULOG_ERROR("Unexpected result: numAdu written (%d) > numAdu requested (%d)\n", nadu,
                   pReq2Cmp->pIocb->write.numADU);
    }

    if (!pSharedParam->bySb)
    {
        ProcessArrList(pParam, 1);    // inc numIO for each returned SB
    }

    // Update cumulative number of TentativeAddresses
    atomic_fetch_add((atomic_uint *)&pSharedParam->numTentativeAddr, flaNum);

    // Retain for calculating distanceToEndOfSuperBlock
    if (pParam->pArr->nsb)
    {
        pSharedParam->numUnwrittenADUs = pParam->pArr->nradu;
    }

    ULOG_WTCMD_DBG("bRetry=%u \n", bRetry);
    if (bRetry)
    {
        // Create retry command
        QOSD_REQUEST_COUNT_INC(pParam->pQos);
        QOSD_NLW_COUNT_INC(pParam->pQos);

        pRetryParam = (struct WriteParam *)malloc(sizeof(*pRetryParam));
        if (pRetryParam == NULL)
        {
            goto DecrementProcReq;
        }
        pRetryParam->pQos = pParam->pQos;
        pRetryParam->pArr = NULL;
        pRetryParam->pAco = NULL;
        pRetryParam->aduOff = nextAduOff;
        pRetryParam->pIocb = pParam->pIocb;
        atomic_init(&pRetryParam->refCnt, 2);
        // Shift Data location

        // Increment command issue/response receive counter
        atomic_fetch_add(&pSharedParam->sendRecvCounter, 1);

        // Take over SharedWriteParam
        pRetryParam->pSharedWriteParam = pSharedParam;

        // Reuse ARR area rather than allocating newly
        req[0].ppDevBuff = (void **)(&(pRetryParam->pArr));
        req[0].buffSize = sizeof(*(pRetryParam->pArr));
        ret = AllocateReqInfoExt(&req[0].pReq2Cmp, NULL, NULL, 0, &req[0].pSem, req[0].ppDevBuff,
                                 req[0].buffSize);
        if (ret != 0)
        {
            goto FreeTempAcoBuf;
        }
        pRetryParam->pArr->nsb = 0;    // empty arr when arr isn't read

        // Allocate ReqInfo for 2nd
        ret = AllocateReqInfoExt(&req[1].pReq2Cmp, NULL, NULL, 0, &req[1].pSem, 0, 0);
        if (ret != 0)
        {
            goto FreeReqInfo0;
        }

        sbId = pSharedParam->bySb ? GetSbId(pRetryParam->pQos, pReq2Cmp->pIocb->write.flashAddress)
                                  : 0xffffffff;
        // Calculate number of remaining ADUs to write at retry by subtracting cumulative NFLA from
        // number of ADUs to write
        writeAdus = pReq2Cmp->pIocb->write.numADU - pSharedParam->numTentativeAddr;
        if (writeAdus > maxAdus)
        {
            writeAdus = maxAdus;
        }
        else if (pSharedParam->bSync | (pReq2Cmp->pIocb->common.flags & kSefIoFlagCommit))
        {
            forceUnitAccess = true;
        }

        pRetryParam->iovcnt = CloneIov(pParam->pIocb->iov, pParam->pIocb->iovcnt,
                                       pRetryParam->aduOff * aduSize, writeAdus * aduSize,
                                       &pRetryParam->iovec);
        SEF_ASSERT(pRetryParam->iovcnt > 0);

        pRetryParam->fua = forceUnitAccess;
        // Update userAddress
        userAddress = SEFCreateUserAddress(
            SEFGetUserAddressLba(pReq2Cmp->pIocb->write.userAddress) + pSharedParam->numTentativeAddr,
            SEFGetUserAddressMeta(pReq2Cmp->pIocb->write.userAddress));
        pRetryParam->sua = userAddress;
        pRetryParam->numADU = writeAdus;

        struct SEFQoSHandle_ *pQos = pRetryParam->pQos;
        uint32_t nlwid = GetNlwid(pQos);
        bool override = !!(pReq2Cmp->pIocb->common.flags & kSefIoFlagOverride);
        uint16_t programWeight = override ? pReq2Cmp->pIocb->write.overrides.programWeight : 0;
        uint16_t eraseWeight = override ? pReq2Cmp->pIocb->write.overrides.eraseWeight : 0;

        // Create request information for 1st
        req[0].pReq2Cmp->pIocb = pReq2Cmp->pIocb;
        req[0].pReq2Cmp->type = kIoctlIoFusedCommand;
        SEFNvmFlashAddressRequest(
            &req[0].pReq2Cmp->submit, pRetryParam->pArr, sizeof(*(pRetryParam->pArr)), pQos->qosId,
            pSharedParam->bySb ? pReq2Cmp->pIocb->write.flashAddress.bits : SEFAutoAllocate.bits,
            pReq2Cmp->pIocb->write.placementID, eraseWeight, nlwid);
        req[0].pReq2Cmp->bSync = false;
        req[0].pReq2Cmp->syncOverride = pSharedParam->bSync;
        req[0].pReq2Cmp->pSyncSemaphore = req[0].pSem;
        req[0].pReq2Cmp->param = pRetryParam;
        req[0].pReq2Cmp->CommandComplete = Complete;
        req[0].pReq2Cmp->CheckResultExt = CheckResultFlashAddressRequest;
        req[0].pReq2Cmp->CheckSuspendFunc = NULL;

        // Create request information for 2nd
        req[1].pReq2Cmp->pIocb = pSharedParam->bSync ? pReq2Cmp->pIocb : NULL;
        req[1].pReq2Cmp->type = kIoctlIoFusedCommand;
        SEFNvmNamelessWriteCommand(
            &req[1].pReq2Cmp->submit, pRetryParam->iovec, pRetryParam->iovcnt,
            pReq2Cmp->pIocb->write.metadata, pQos->qosId, sbId, pParam->pQos->aduOffsetBitLen,
            userAddress, writeAdus, pReq2Cmp->pIocb->write.placementID, programWeight, nlwid,
            forceUnitAccess, pQos->pSefHandle->writeTimeout, pSharedParam->pSLC);
        req[1].pReq2Cmp->bSync = false;
        req[1].pReq2Cmp->pSyncSemaphore = req[1].pSem;
        req[1].pReq2Cmp->syncOverride = pSharedParam->bSync;
        req[1].pReq2Cmp->param = pRetryParam;
        req[1].pReq2Cmp->CommandComplete = Complete;
        req[1].pReq2Cmp->CheckResultExt = CheckResultNamelessWrite;
        req[1].pReq2Cmp->CheckSuspendFunc = NULL;

        if (pSharedParam->bySb)
        {
            ret = ManageResponseOrder(pQos, sbId, writeAdus, NULL, kMroIncIoRef);
            if (ret)
            {
                goto FreeReqInfo1;
            }
        }
        else
        {
            // instead of starting a new one and completing the current one,
            // pass "ownership" to the next FAR request
            pRetryParam->farSn = farSn;
        }

        // Request command issuing
        ret = RequestFusedWrite(pRetryParam->pQos->deviceFd, req[0].pReq2Cmp, req[1].pReq2Cmp);
        if (ret != 0)
        {
            goto FreeReqInfo1;
        }
    }
    else if (!pSharedParam->bySb)
    {
        ret = FPWCComplete(&pParam->pQos->pidState[pid].numProcessingFAR, farSn);
        SEF_ASSERT(!ret);
    }

    ULOG_WTCMD_DBG("bSync=%u\n", pSharedParam->bSync);
    goto ExitProc;

FreeReqInfo1:
    if (!pSharedParam->bySb)
    {
        ret = FPWCComplete(&pParam->pQos->pidState[pid].numProcessingFAR, farSn);
        SEF_ASSERT(!ret);
    }
    free(req[1].pReq2Cmp);
    free(req[1].pSem);
    free(pRetryParam->pAco);
FreeReqInfo0:
    free(req[0].pReq2Cmp);
    free(req[0].pSem);
FreeTempAcoBuf:
    // May not be allocated if unnecessary
    if (pNewTempAcoBuf != NULL)
    {
        free(pNewTempAcoBuf);
    }
    // DecrementSendRecvCnt:
    //  Revert command issue/response receive counter
    atomic_fetch_sub(&pParam->pSharedWriteParam->sendRecvCounter, 1);
    free(pRetryParam);
DecrementProcReq:
    // Decrement number of requests being processed in QosHandle incremented at retry
    QOSD_REQUEST_COUNT_DEC(pParam->pQos);
    QOSD_NLW_COUNT_DEC(pParam->pQos);
    // NoRetry:
    bRetry = false;

ExitProc:
    if (!bRetry)
    {
        if (pParam->pSharedWriteParam->numTentativeAddr != 0)
        {
            pReq2Cmp->pIocb->write.distanceToEndOfSuperBlock =
                pParam->pSharedWriteParam->numUnwrittenADUs;

            ULOG_INFORMATION("distanceToEndOfSuperBlock=%d\n",
                             pReq2Cmp->pIocb->write.distanceToEndOfSuperBlock);

            // Set number of ADUs for Full or PartialWrite
            pReq2Cmp->pIocb->common.status.info = pParam->pSharedWriteParam->numTentativeAddr;

            // If any data was written, return a good status (partial write)
            pReq2Cmp->status = 0;
        }

        // Call user completion callback at completion of all issued FARs
        CompleteIocbProc(pReq2Cmp);

        // Release area for 1st
    }

    QOSD_REQUEST_COUNT_DEC(pParam->pQos);
    FarNlwCheckResultFinish(pParam);
}

STATIC void UpdateAddressesFromAco(struct WriteParam *pParam)
{
    struct SEFFlashAddress *pPermanent = pParam->pIocb->tentativeAddresses;
    struct AddressChangeOrder *pAco = pParam->pAco;

    // only update full write ACOs
    SEF_ASSERT(pParam->nlwStatus == 0);
    for (uint32_t i = pParam->aduOff, j = 0; i < pParam->numADU && j <= pAco->naco; i++)
    {
        if (pPermanent[i].bits < pAco->acoes[j].ofa)
        {
            continue;
        }

        if (pPermanent[i].bits == pAco->acoes[j].ofa)
        {    // assert it's not a partial write ACO
            SEF_ASSERT(pAco->acoes[j].nfa != SEFAutoAllocate.bits);
            pPermanent[i].bits = pAco->acoes[j++].nfa;
        }
        else    // todo: need to inc j while aco < far addr, assert, or fail the write (to whom?)
        {
            fprintf(stderr, "Unmatched aco entry\n");
        }
    }
}

static void *CalcUserDataAddr(struct WriteParam *pParam, uint32_t aduIndex)
{
    uint32_t aduSize = GET_ADU_SIZE(pParam->pQos->pSefHandle);
    uint64_t vecOffset;
    uint16_t vecIndex;
    uint64_t offset;

    if (aduIndex >= pParam->numADU)
    {
        ULOG_ERROR("ADU index of %d is not less than the write size of %u\n", aduIndex, pParam->numADU);
        return NULL;
    }
    offset = aduSize * aduIndex;
    GetIovOffsPosition(pParam->iovec, pParam->iovcnt, offset, &vecIndex, &vecOffset);
    return pParam->iovec[vecIndex].iov_base + vecOffset;
}

static void *CalcUserDataAddrFromACO(struct WriteParam *pParam, uint32_t index)
{
    struct SEFUserAddress sua = {pParam->pAco->acoes[index].ua};
    uint64_t startLba = SEFGetUserAddressLba(pParam->sua);
    uint64_t acoLba = SEFGetUserAddressLba(sua);
    uint64_t numAdus = acoLba - startLba;

    return CalcUserDataAddr(pParam, numAdus);
}

STATIC void SendAddressNotificationFromAco(struct WriteParam *pParam)
{
    struct AddressChangeOrder *pAco = pParam->pAco;
    SEFQoSHandle pQos = pParam->pQos;
    struct SEFQoSNotification notify;

    notify.type = pParam->nlwStatus ? kUnflushedData : kAddressUpdate;
    notify.QoSDomainID.id = pQos->qosId;
    /*
     * Address Update Notification(1st to 3rd Region)
     */
    for (uint32_t i = 0; i <= pAco->naco; i++)
    {
        if (notify.type == kAddressUpdate)
        {
            notify.changedUserAddress.unformatted = pAco->acoes[i].ua;
            notify.oldFlashAddress.bits = pAco->acoes[i].ofa;
            notify.newFlashAddress.bits = pAco->acoes[i].nfa;
            SEF_ASSERT(pAco->acoes[i].nfa != SEFAutoAllocate.bits);
        }
        else
        {
            notify.unflushedUserAddress.unformatted = pAco->acoes[i].ua;
            notify.userData = CalcUserDataAddrFromACO(pParam, i);
            SEF_ASSERT(pAco->acoes[i].nfa == SEFAutoAllocate.bits);
        }
        pQos->notifyFunc(pQos->pContext, notify);
    }
}

STATIC void SendUnflushedNotification(struct WriteParam *pParam)
{
    SEFQoSHandle pQos = pParam->pQos;
    struct SEFQoSNotification notify;

    notify.type = kUnflushedData;
    notify.QoSDomainID.id = pQos->qosId;
    /*
     * Address Update Notification(1st to 3rd Region)
     */
    for (uint32_t i = pParam->numWritten; i < pParam->numReq; i++)
    {
        struct SEFUserAddress ua;

        ua = SEFCreateUserAddress(SEFGetUserAddressLba(pParam->sua) + pParam->aduOff + i,
                                  SEFGetUserAddressMeta(pParam->sua));
        notify.unflushedUserAddress = ua;
        notify.userData = CalcUserDataAddr(pParam, i);
        pQos->notifyFunc(pQos->pContext, notify);
    }
}

STATIC void SendCloseNotifyFromAco(SEFQoSHandle pQos, struct AddressChangeOrder *pAco, uint16_t pid)
{
    uint32_t psbId = GetSbId(pQos, (struct SEFFlashAddress){pAco->acoes[0].nfa});

    // Send close notification for any closed block, create an entry
    // for opened superblocks.
    for (uint32_t j = 1; j <= pAco->naco; j++)
    {
        uint32_t nsbId = GetSbId(pQos, (struct SEFFlashAddress){pAco->acoes[j].nfa});

        if (nsbId != psbId)
        {
            psbId = nsbId;
            SendAcoCloseNotification(pQos, &pAco->acoes[j - 1], pid);
            ManageResponseOrder(pQos, nsbId, 0, NULL, kMroCreate);
        }
    }
    if (pAco->nadul == 0)
    {
        SendAcoCloseNotification(pQos, &pAco->acoes[pAco->naco], pid);
    }
}

STATIC void ProcessAcoList(struct SharedWriteParam *pSharedWriteParam)
{
    struct WriteParam *pParam;
    uint32_t maxAduOff = 0;
    uint32_t minAduOff = 0xffffffff;

    pParam = SefSlistPopAllAs(&pSharedWriteParam->acoList, struct WriteParam, acolink);
    while (pParam)
    {
        struct AddressChangeOrder *pAco = pParam->pAco;
        bool acoInvalid;

        acoInvalid = (pParam->nlwStatus != 0 && pParam->nlwStatus != SEF_PARTIAL_WRITE);
        acoInvalid |= (pSharedWriteParam->bySb && pParam->nlwStatus == SEF_PARTIAL_WRITE);
        acoInvalid |= (pParam->acoStatus != 0);
        acoInvalid |= (pAco == NULL);

        if (pSharedWriteParam->bSync)
        {
            if (!acoInvalid)
            {
                UpdateAddressesFromAco(pParam);
            }
            else if (pParam->nlwStatus == SEF_PARTIAL_WRITE)
            {
#if CC_ARR_VALIDATION    // LD2 returns non-zero adus left when there aren't any left
                pAco->nadul = 0;
#endif
                ULOG_NOTICE("**Partial Write ACO [%u to %u)\n", pParam->aduOff,
                            pParam->aduOff + pParam->numWritten);
            }

            // Last ACO's num_adus_left is distanceToEndOfSuperBlock
            if (pParam->aduOff >= maxAduOff && pAco)
            {
                pParam->pIocb->distanceToEndOfSuperBlock = pAco->nadul;
                pSharedWriteParam->numUnwrittenADUs = pAco->nadul;
                maxAduOff = pParam->aduOff;
                ULOG_NOTICE("**Max Partial Write ACO %u %u left\n", pParam->aduOff, pAco->nadul);
            }
            // For partial/error writes, find the first ACO and calc how many adus written
            if (pParam->nlwStatus && pParam->aduOff < minAduOff)
            {
                minAduOff = pParam->aduOff;
                SEF_ASSERT(pSharedWriteParam->numTentativeAddr >= pParam->aduOff + pParam->numWritten);
                pSharedWriteParam->numTentativeAddr = pParam->aduOff + pParam->numWritten;
                if (pAco == NULL)
                {
                    pParam->pIocb->distanceToEndOfSuperBlock = 0;
                }
                pParam->pIocb->common.status.info = pParam->aduOff + pParam->numWritten;
                ULOG_NOTICE("**Min Partial Write ACO %u of %u with status 0x%x\n", pParam->aduOff,
                            pParam->numWritten, pParam->nlwStatus);
            }
        }
        else if (pParam->pQos->notifyFunc != NULL)
        {
            if (pAco)
            {
                SendAddressNotificationFromAco(pParam);
            }
            else
            {
                SendUnflushedNotification(pParam);
            }
        }
        // When a defect is hit, only write by plid infers the SB is closed.
        // Write by sbid sends an AEN if that's how the device handles defects.
        if (pParam->nlwStatus == 0 && pAco)
        {    // write by plid hit a defect
            SendCloseNotifyFromAco(pParam->pQos, pAco, pSharedWriteParam->pid);
        }
        free(pAco);
        pParam->pAco = NULL;
        pParam = SefSlistNextAs(&pParam->acolink, struct WriteParam, acolink);
    }    // while
}

STATIC void ProcessWriteParamList(struct WriteParam *pParam)
{
    while (pParam)
    {
        struct WriteParam *next;

        next = SefSlistNextAs(&pParam->arrlink, struct WriteParam, arrlink);
        ProcessArrList(pParam, -1);
        free(pParam->iovec);
        free(pParam->pArr);
        free(pParam);
        pParam = next;
    }
}

STATIC void CompleteNamelessWrite(struct WriteParam *pParam)
{
    struct SharedWriteParam *pSharedWriteParam = pParam->pSharedWriteParam;
    uint16_t sendRecvCounter = atomic_fetch_sub(&pSharedWriteParam->sendRecvCounter, 1);
    struct SEFQoSHandle_ *pQos = pParam->pQos;
    bool lastIO = (sendRecvCounter == 1);
    int ret;

    assert(sendRecvCounter);

    if (lastIO)
    {
        // update/notify changed flash addresses.  Send/queue close notifications
        // for any SB's closed by ACO.
        ProcessAcoList(pSharedWriteParam);

        // Release data area using retained top address
        if (pSharedWriteParam->notifyIoVec)
        {
            NotifyMemoryAvailable(pQos, pParam->iovec, pParam->iovcnt);
        }
        else
        {
            free(pSharedWriteParam->pHeadOfDataBuf);
        }

        // Inform ManageResponseOrder the i/o is complete for which SBID.  Detect
        // if write closed SB and send/queue a close notification.
        ProcessWriteParamList(SefSlistNodeAs(
            SefSlistReverse(SefSlistPopAll(&pSharedWriteParam->arrList)), struct WriteParam, arrlink));
        ProcessArrList(pParam, -1);

        if (pSharedWriteParam->bSync)
        {
            ret = sem_post(&pSharedWriteParam->syncSemaphore);
            if (ret != 0)
            {
                ULOG_ERROR("failed to sem_post()\n");
            }
        }
        else
        {
            free(pSharedWriteParam);
        }
    }
    else
    {
        // ARR's processed once all the NLWs have completed, queue for later
        SefSlistPush(&pSharedWriteParam->arrList, &pParam->arrlink);
    }

    QOSD_NLW_COUNT_DEC(pQos);

    if (lastIO)
    {    // Done by ProcessWriteParamList() for all but the last i/o
        free(pParam->iovec);
        free(pParam->pArr);
        free(pParam);
    }
}

STATIC void CheckResultGetLogAco(struct ReqToComplete *pReq2Cmp)
{
    struct WriteParam *pParam = (struct WriteParam *)pReq2Cmp->param;
    struct SharedWriteParam *pSharedWriteParam = pParam->pSharedWriteParam;

    if (pReq2Cmp->status == 0)
    {
        SefSlistPush(&pSharedWriteParam->acoList, &pParam->acolink);
        atomic_fetch_add(&pSharedWriteParam->numAco, 1);
    }
}

/**
 *  @brief	Post process Far & NLW check result complete
 *  @param	[in] pParam: WriteParam of a pair
 *  @details FAR/NLW can complete in either order.  A ref count is used to know
 *  @details when both are complete so the ACO from the NLW can be processed.
 */
static void FarNlwCheckResultFinish(struct WriteParam *pParam)
{
    struct SharedWriteParam *pSharedWriteParam = pParam->pSharedWriteParam;

    if (atomic_fetch_sub(&pParam->refCnt, 1) > 1)
    {
        return;
    }

    ULOG_WTCMD_DBG(
        "pSharedWriteParam->numTentativeAddr=%u, pSharedWriteParam->numAco=%u, "
        "pSharedWriteParam->sendRecvCounter=%u\n",
        pSharedWriteParam->numTentativeAddr, pSharedWriteParam->numAco,
        pSharedWriteParam->sendRecvCounter);

    CompleteNamelessWrite(pParam);
}

/**
 *  @brief	Responds to NamelessWrite (2nd) of FusedWrite
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @details	Waits response processing of 1st command to complete.
 *  @details	If retry for PartialWrite is ongoing and response is for non-last Fused 2nd command, ACO is retained.
 *  @details	If Write is not Partial or retry for PartialWrite is ongoing and response is to the last Fused 2nd
 *              command, retained ACO is incorporated into notification data for issuing notification.
 *  @details	After copying ACO information, ACO area in arbitrary parameter area is released.
 *  @details	If response is to the last Fused 2nd command, write data area in arbitrary parameter area is released.
 *  @details	If response is to the last Fused 2nd command, arbitrary parameter area is released.
 */
STATIC void CheckResultNamelessWrite(struct ReqToComplete *pReq2Cmp)
{
    struct WriteParam *pParam = (struct WriteParam *)pReq2Cmp->param;
    int ret;
    uint32_t nadu = pReq2Cmp->result & UINT32_MAX;
    uint32_t acoid = pReq2Cmp->result >> 32;

    SEF_ASSERT(pReq2Cmp->status != -EBUSY);    // should be handled by request.c
    SEF_ASSERT(pReq2Cmp->status != -EWOULDBLOCK);
    SEF_ASSERT(pReq2Cmp->status != -ECANCELED);

    ULOG_WTCMD_DBG("acoid=%u, nadu=%u, notifyFunc=0x%p\n", acoid, nadu, pParam->pQos->notifyFunc);

    pParam->nlwStatus = pReq2Cmp->status;
    pParam->numWritten = nadu;
    pParam->acoStatus = 0;

    // Retain ACO information if FLA is changed
    if ((acoid != 0))
    {
        uint32_t acoSize = sizeof(struct AddressChangeOrder) + sizeof(uint64_t) * 3 * nadu;

        SEF_ASSERT(nadu);

        // assume 0 status, full write completed
        pParam->numWritten = pParam->numADU;
        if (pParam->nlwStatus == SEF_PARTIAL_WRITE)
        {    // When there's an acoid and partial write nadu, is num not written
            SEF_ASSERT(pParam->numWritten >= nadu);
            pParam->numWritten -= nadu;
        }
        // Traverse to free ACO area
        ULOG_INFORMATION("Getting aco log page\n");

        struct RequestInfoExt reqInfo;
        reqInfo.ppDevBuff = (void **)&pParam->pAco;
        reqInfo.buffSize = acoSize;
        ret = AllocateReqInfoExt(&reqInfo.pReq2Cmp, NULL, NULL, 0, &reqInfo.pSem, reqInfo.ppDevBuff,
                                 reqInfo.buffSize);
        pParam->acoStatus = ret;
        if (ret != 0)
        {
            pReq2Cmp->status = ret;
            goto ExitProc;
        }

        SEFAdmGetLogPageAco(&reqInfo.pReq2Cmp->submit, pParam->pAco, acoSize, pParam->pQos->qosId,
                            acoid);
        reqInfo.pReq2Cmp->bSync = true;
        reqInfo.pReq2Cmp->syncOverride = false;
        reqInfo.pReq2Cmp->type = kIoctlAdminCommand;
        reqInfo.pReq2Cmp->pSyncSemaphore = reqInfo.pSem;
        reqInfo.pReq2Cmp->param = pParam;
        reqInfo.pReq2Cmp->CommandComplete = Complete;
        reqInfo.pReq2Cmp->CheckResultExt = CheckResultGetLogAco;
        reqInfo.pReq2Cmp->CheckSuspendFunc = NULL;

        // Request command issuing
        ret = Request(pParam->pQos->pSefHandle->deviceFd, reqInfo.pReq2Cmp);
        pParam->acoStatus = ret;
        if (ret != 0 || reqInfo.pReq2Cmp->status)
        {
            free(*reqInfo.ppDevBuff);
            *reqInfo.ppDevBuff = NULL;
            free(reqInfo.pReq2Cmp->pSyncSemaphore);
            free(reqInfo.pReq2Cmp);
            goto ExitProc;
        }
        // check for lost ACO entry by verifying UA matches
        if (pParam->pAco->sua != pParam->sua.unformatted)
        {
            ULOG_ERROR("ACO mismatch - ID %d has UA of %#lx expecting %#lx\n", acoid,
                       pParam->pAco->sua, le64toh(pParam->sua.unformatted));
        }
        free(reqInfo.pReq2Cmp->pSyncSemaphore);
        free(reqInfo.pReq2Cmp);
    }
    else
    {    // Process all NLWs in case one fails to determine how much was written
        SefSlistPush(&pParam->pSharedWriteParam->acoList, &pParam->acolink);
        atomic_fetch_add(&pParam->pSharedWriteParam->numAco, 1);
    }

ExitProc:
    FarNlwCheckResultFinish(pParam);
}

/**
 *  @brief	Internal processing for SEFWriteWithoutPhysicalAddress and
 * SEFWriteWithoutPhysicalAddressAsync
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Write
 *  @details	Arbitrary parameter area for responding to Write is allocated.
 *  @details	Arbitrary parameter area for PartialWrite response to Write is allocated.
 *  @details	Area for storing ACO temporarily for PartialWrite is allocated (arbitrary parameter
 *              area).
 *  @details	This function allocates memory for Req2Cmp, Sem, write data, ARR, and ACO.
 *  @details	Complete() releases memory of Req2Cmp and Sem for FusedWrite 1st command (this
 *              function releases them if command issuing fails).
 *  @details	For synchronous operation, this function releases memory of Req2Cmp and Sem for
 *              FusedWrite 2nd command; for async, Complete() releases them
 *              (this function releases them if command issuing fails).
 *  @details	If command issuing fails, this function releases areas for arbitrary parameter,
 *              write data, ARR, and ACO.
 *  @details	If sync command errored, this function releases areas for write data, ARR, and ACO.
 *  @details	For synchronous operation, this function releases arbitrary parameter area.
 *  @details	For synchronous operation, FusedWrite 2nd command response is awaited.
 *  @note	For async, CheckResultExt of FusedWrite 2nd must release arbitrary parameter area.
 *  @note	CheckResultExt of FusedWrite 1st must release ARR area.
 *  @note	CheckResultExt of FusedWrite 2nd must release write data area and ACO area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info:
 *                                     number of ADUs to write (NFLA in ARR)
 */
STATIC struct SEFStatus WriteProcSegment(bool bSync,
                                         struct SEFQoSHandle_ *pQos,
                                         struct SEFCommonIOCB *common,
                                         uint32_t aduOff,
                                         uint32_t numADU)
{
    struct SEFWriteWithoutPhysicalAddressIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};

    if (pQos == NULL)
    {
        ULOG_ERROR("argument error!! pQos=%p\n", pQos);
        status.error = -ENODEV;
        goto ExitProc;
    }

    struct SEFHandle_ *pSef = pQos->pSefHandle;
    bool isShortage;
    struct RequestInfoExt req[2];
    size_t aduSize = GET_ADU_SIZE(pSef);    // size_t because of math below
    size_t dataSize = numADU * aduSize;
    size_t offset = aduOff * aduSize;
    uint32_t sbId;
    struct WriteParam *pParam;
    int ret;
    uint32_t maxAdus = MaxAduCount(pQos);
    uint32_t writeAdus = numADU > maxAdus ? maxAdus : numADU;
    struct SharedWriteParam *pSharedWriteParam;
    struct SEFUserAddress userAddr = iocb->userAddress;
    bool forceUnitAccess = !(numADU > maxAdus) && (bSync | (common->flags & kSefIoFlagCommit));
    bool pSLC = SEFIsEqualFlashAddress(iocb->flashAddress, SEFAutoAllocatePSLC);
    bool bySb = !SEFIsEqualFlashAddress(iocb->flashAddress, SEFAutoAllocate) && !pSLC;
    bool validPlid = bySb || (iocb->placementID.id < pQos->numPlacementIds);
    uint8_t *data = NULL;

    QOSD_REQUEST_COUNT_INC(pQos);
    QOSD_NLW_COUNT_INC(pQos);

    // bSync can't use a completion routine as the iocb will be updated post
    // completion when ACOs are returned.  This isn't strictly forbidden but
    // the assert is to prevent coding up a situation that could access memory
    // post free or use stale values when there's a media error.
    SEF_ASSERT(common->complete_func == NULL || bSync == false);

    // Check parameters
    if (iocb->numADU == 0 || iocb->iov == NULL || iocb->iovcnt == 0 ||
        iocb->tentativeAddresses == NULL || !validPlid)
    {
        ULOG_ERROR(
            "argument error!! numADU=%u, iov=%p, iovcnt=%u, tentativeAddresses=%p, placementID=%u, "
            "flashAddress=0x%016lx\n",
            iocb->numADU, iocb->iov, iocb->iovcnt, iocb->tentativeAddresses, iocb->placementID.id,
            iocb->flashAddress.bits);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Check parameter (QoSD ID)
    if (bySb && (pQos->qosId != GetQosdId(iocb->flashAddress)))
    {
        ULOG_ERROR("argument error!! flashAddress=0x%016lx\n", iocb->flashAddress.bits);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Check iov area for specified number of ADUs is allocated
    isShortage = IsShortageIovLen(iocb->iov, iocb->iovcnt, offset, dataSize);
    if (isShortage)
    {
        size_t len = GetIovBuffLen(iocb->iov, iocb->iovcnt, offset);
        ULOG_ERROR(
            "iov area is less than data length!! area=%lu, length=%lu, offset=%lu, adu num=%u\n",
            len, dataSize, offset, iocb->numADU);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Check UA
    uint64_t tmpUA = 0xFFFFFFFFFFFFFFFFllu - le64toh(iocb->userAddress.unformatted);
    if (iocb->userAddress.unformatted != SEFUserAddressIgnore.unformatted &&
        (uint64_t)iocb->numADU + aduOff > tmpUA)
    {
        ULOG_ERROR("start user address(SUA) is too big!! SUA=0x%016lx, numADU=0x%x\n",
                   le64toh(iocb->userAddress.unformatted), iocb->numADU);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Prepare parameters
    pParam = (struct WriteParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }
    pParam->pIocb = iocb;
    pParam->pQos = pQos;
    pParam->pArr = NULL;
    pParam->pAco = NULL;
    pParam->aduOff = aduOff;
    pParam->numADU = writeAdus;
    pParam->fua = forceUnitAccess;
    atomic_init(&pParam->refCnt, 2);

    bool doZeroCopy = DoZeroCopyWrite(iocb);
    // Allocate buffer for Write data
    if (doZeroCopy)
    {
        pParam->iovcnt = CloneIov(iocb->iov, iocb->iovcnt, offset, writeAdus * aduSize, &pParam->iovec);
        if (pParam->iovcnt <= 0)
        {
            goto FreeParam;
        }
    }
    else
    {
        data = (uint8_t *)AllocateDeviceBuffer(dataSize, false);
        if (data == NULL)
        {
            goto FreeParam;
        }
        pParam->iovec = calloc(1, sizeof(struct iovec));
        if (pParam->iovec == NULL)
        {
            free(data);
            goto FreeParam;
        }
        pParam->iovec->iov_base = data;
        pParam->iovec->iov_len = writeAdus * aduSize;
        pParam->iovcnt = 1;
        // Copy Write data that iov specifies to Write buffer
        CopyIovToBuff(iocb->iov, iocb->iovcnt, offset, dataSize, data);
    }

    pSharedWriteParam = (struct SharedWriteParam *)calloc(1, sizeof(*pSharedWriteParam));
    if (pSharedWriteParam == NULL)
    {
        goto FreeData;
    }
    if (doZeroCopy)
    {
        pSharedWriteParam->notifyIoVec = iocb->iov;
        pSharedWriteParam->notifyIoVecCnt = iocb->iovcnt;
    }
    else
    {
        pSharedWriteParam->pHeadOfDataBuf = data;
    }
    pSharedWriteParam->bSync = bSync;
    if (bSync)
    {
        ret = sem_init(&pSharedWriteParam->syncSemaphore, 0, 0);
        if (ret != 0)
        {
            ULOG_ERROR("failed to sem_init()\n");
            goto FreeSharedWriteParam;
        }
    }
    pSharedWriteParam->numADU = iocb->numADU;
    atomic_fetch_add(&pSharedWriteParam->sendRecvCounter, 1);
    pSharedWriteParam->bySb = bySb;
    pSharedWriteParam->pSLC = pSLC;
    pParam->pSharedWriteParam = pSharedWriteParam;
    if (bySb)
    {
        sbId = GetSbId(pQos, iocb->flashAddress);
        pSharedWriteParam->pid = SEFPlacementIdUnused;
    }
    else
    {
        sbId = 0xffffffff;
        pSharedWriteParam->pid = iocb->placementID.id;
    }
    pSharedWriteParam->sbid = sbId;

    // Allocate ReqInfo for 1st
    req[0].ppDevBuff = (void **)(&(pParam->pArr));
    req[0].buffSize = sizeof(*(pParam->pArr));
    ret = AllocateReqInfoExt(&req[0].pReq2Cmp, NULL, NULL, 0, &req[0].pSem, req[0].ppDevBuff,
                             req[0].buffSize);
    if (ret != 0)
    {
        goto FreeSharedWriteParam;
    }
    pParam->pArr->nsb = 0;    // empty arr when arr isn't read

    // Allocate ReqInfo for 2nd
    ret = AllocateReqInfoExt(&req[1].pReq2Cmp, NULL, NULL, 0, &req[1].pSem, 0, 0);
    if (ret != 0)
    {
        goto FreeReqInfo0;
    }

    if (userAddr.unformatted != SEFUserAddressIgnore.unformatted)
    {
        userAddr = SEFCreateUserAddress(SEFGetUserAddressLba(userAddr) + aduOff,
                                        SEFGetUserAddressMeta(userAddr));
    }
    pParam->sua = userAddr;

    uint32_t nlwid = GetNlwid(pQos);
    bool override = !!(common->flags & kSefIoFlagOverride);
    uint16_t programWeight = override ? iocb->overrides.programWeight : 0;
    uint16_t eraseWeight = override ? iocb->overrides.eraseWeight : 0;

    // Create request information for 1st
    req[0].pReq2Cmp->pIocb = (union RequestIocb *)iocb;
    req[0].pReq2Cmp->type = kIoctlIoFusedCommand;
    SEFNvmFlashAddressRequest(&req[0].pReq2Cmp->submit, pParam->pArr, sizeof(*(pParam->pArr)),
                              pQos->qosId, bySb ? iocb->flashAddress.bits : SEFAutoAllocate.bits,
                              iocb->placementID, eraseWeight, nlwid);
    req[0].pReq2Cmp->bSync = false;
    req[0].pReq2Cmp->syncOverride = bSync;
    req[0].pReq2Cmp->pSyncSemaphore = req[0].pSem;
    req[0].pReq2Cmp->param = pParam;
    req[0].pReq2Cmp->CommandComplete = Complete;
    req[0].pReq2Cmp->CheckResultExt = CheckResultFlashAddressRequest;
    req[0].pReq2Cmp->CheckSuspendFunc = NULL;

    // Create request information for 2nd
    req[1].pReq2Cmp->pIocb = bSync ? (union RequestIocb *)iocb : NULL;
    req[1].pReq2Cmp->type = kIoctlIoFusedCommand;
    SEFNvmNamelessWriteCommand(&req[1].pReq2Cmp->submit, pParam->iovec, pParam->iovcnt,
                               iocb->metadata, pQos->qosId, sbId, pQos->aduOffsetBitLen, userAddr,
                               writeAdus, iocb->placementID, programWeight, nlwid, forceUnitAccess,
                               pSef->writeTimeout, pSLC);
    req[1].pReq2Cmp->bSync = false;
    req[1].pReq2Cmp->syncOverride = bSync;
    req[1].pReq2Cmp->pSyncSemaphore = req[1].pSem;
    req[1].pReq2Cmp->param = pParam;
    req[1].pReq2Cmp->CommandComplete = Complete;
    req[1].pReq2Cmp->CheckResultExt = CheckResultNamelessWrite;
    req[1].pReq2Cmp->CheckSuspendFunc = NULL;

    if (pSharedWriteParam->bySb)
    {
        ret = ManageResponseOrder(pQos, sbId, writeAdus, NULL, kMroIncIoRef);
    }
    else
    {
        ret = FPWCStart(&pQos->pidState[pSharedWriteParam->pid].numProcessingFAR, &pParam->farSn);
    }
    if (ret)
    {
        status.error = ret;
        goto FreeReqInfo1;
    }

    // Request command issuing
    ret = RequestFusedWrite(pQos->deviceFd, req[0].pReq2Cmp, req[1].pReq2Cmp);
    if (ret != 0)
    {
        goto FreeReqInfo2;
    }

    if (bSync)
    {
        ret = sem_wait(&pSharedWriteParam->syncSemaphore);
        if (ret != 0)
        {
            ULOG_ERROR("failed to sem_wait()\n");
        }

        ret = sem_destroy(&pSharedWriteParam->syncSemaphore);
        SEF_ASSERT(ret == 0);
        (void)ret;

        // For synchronous operation, memory for pSharedParam is released here
        // (different from async because of semaphore)
        free(pSharedWriteParam);

        // For synchronous operation, command result is set

        status = iocb->common.status;

        goto ExitProc;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;

        // For async, ReqInfo memory release is skipped because Complete releases it
        goto ExitProc;
    }

FreeReqInfo2:
    if (!pSharedWriteParam->bySb)
    {
        ret = FPWCComplete(&pQos->pidState[pSharedWriteParam->pid].numProcessingFAR, pParam->farSn);
        SEF_ASSERT(!ret);
    }
FreeReqInfo1:
    free(req[1].pReq2Cmp);
    free(req[1].pSem);
    // free(*(req[1].ppDevBuff));
FreeReqInfo0:
    free(req[0].pReq2Cmp);
    free(req[0].pSem);
    free(*(req[0].ppDevBuff));
FreeSharedWriteParam:
    free(pSharedWriteParam);
FreeData:
    if (!DoZeroCopyWrite(pParam->pIocb))
    {
        free(pParam->iovec[0].iov_base);
    }
    free(pParam->iovec);
FreeParam:
    free(pParam);
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(pQos);
    QOSD_NLW_COUNT_DEC(pQos);
ExitProc:
    return status;
}

STATIC struct SEFStatus WriteProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common)
{
    struct SEFWriteWithoutPhysicalAddressIOCB *iocb = (void *)common;
    if (bSync)
    {
        common->reserved = 1;
        return WriteProcSegment(true, pQos, common, 0, iocb->numADU);
    }
    else
    {
        common->reserved = 1;
        return WriteProcSegment(false, pQos, common, 0, iocb->numADU);
    }
}

/**
 *  @brief	Structure of parameters retained for responding to NLC
 */
struct CopyParam
{
    struct SEFNamelessCopyIOCB *iocb;
    struct SEFQoSHandle_ *pSrcQos;    //!< QoSD handle
    // 1 for all nlc (only one outstanding at a time)
    // +1 for each gcr
    atomic_int refCnt;    //!< resources freed when refCnt hits 0, holds a single ref each for nlc & gcr, even if divided.
    bool bSync;    //!< Whether it is a sync command (Req2Cmp handles all commands as async, so sync
                   //!< information is added
    bool bNLC;     // !< true if at least 1 NLC completed
#if CC_FLUSH_SYNC_COPY
    bool bFlush;    // !< true if flush needs to be issued, false once
                    //    it's been issued.
#endif
    sem_t syncSemaphore;                 // Semaphore for sync
    uint32_t numWritten;                 // when dest closed, numWritten
                                         // Drive segmentation
    uint32_t offset;                     // how many done
    uint32_t count;                      // how many remaining to do
    uint32_t maxCount;                   // how many per seg
                                         // Combined results
    atomic_int numProcessedADUs;         // sum
    atomic_int nextADUOffset;            // max
    atomic_int numReadErrorADUs;         // sum
    atomic_int numADUsLeft;              // min
    atomic_int copyStatus;               // or
    atomic_uint_least64_t lastStatus;    // last non-zero status, upper 32 bits is
                                         // lowest gcrParam->count.  lower 32 bits
                                         // is its status.
};

struct UserAddressRange
{
    struct SEFUserAddress sua;
    uint64_t lr;
};

union CopySource
{
    struct
    {
        struct UserAddressRange uar;
        struct SEFFlashAddress sfla;
        uint64_t bitmap[];    // Device treats as uint32_t
    } bm;
    struct
    {
        struct UserAddressRange uar;
        struct SEFFlashAddress fla[];
    } fal;
};

struct CopyGcrParam
{
    struct CopyParam *parent;
    struct ReqToComplete *pReq2Cmp;
    uint32_t numAcrEntries;
    uint32_t acrSize;
    uint32_t count;    // parent->count value at submit
    struct SEFAddressChangeRequest acr;
};

struct CopyNlcParam
{
    struct CopyParam *parent;
    struct ReqToComplete *pReq2Cmp;
    struct CopyGcrParam *pGcrParam;    // used to retry on EBUSY/EWOULDBLOCK
    uint32_t numSrcEntries;
    uint32_t srcSize;
    bool _1st;
    union CopySource source;
};

STATIC int CopySegBmProc(struct CopyParam *parent);
STATIC int CopySegFalProc(struct CopyParam *parent);

STATIC void AtomicSetMin(atomic_int *pMin, int val)
{
    int min = atomic_load(pMin);

    while (min > val && !atomic_compare_exchange_weak(pMin, &min, val)) {}
}

STATIC void AtomicSetMax(atomic_int *pMax, int val)
{
    int max = atomic_load(pMax);

    while (max < val && !atomic_compare_exchange_weak(pMax, &max, val)) {}
}

// Saves that latest copy status (i.e., with the count that's the lowest
// when the GCR was submitted).
STATIC void CopySaveLastCopyStatus(struct CopyParam *pParam, uint32_t count, uint8_t copyStatus)
{
    uint64_t lastStatus = count;
    uint64_t expected = atomic_load(&pParam->lastStatus);

    lastStatus <<= 32;
    lastStatus |= copyStatus;

    while (count < (expected >> 32))
    {
        if (atomic_compare_exchange_weak(&pParam->lastStatus, &expected, lastStatus))
        {
            return;
        }
    }
}

STATIC void CopyCombineResults(struct CopyGcrParam *pGcrParam)
{
    struct CopyParam *parent = pGcrParam->parent;
    struct SEFNamelessCopyIOCB *iocb = parent->iocb;
    uint32_t numProcessedADUs = pGcrParam->acr.numProcessedADUs;
    uint8_t copyStatus = pGcrParam->acr.copyStatus;
    int index = atomic_fetch_add(&parent->numProcessedADUs, numProcessedADUs);
    int end = index + numProcessedADUs;
    int numToCopy = 0;

    if (end > iocb->numAddressChangeRecords)
    {
        // Ran out of addressChangeInfo entries, didn't run out of src
        // NOTE: parent->numProcessedADUs is now too large, this is fixed up
        //       in CopyCompleteParent after copy is done.
        end = iocb->numAddressChangeRecords;
        copyStatus &= ~kCopyConsumedSource;
        copyStatus |= kCopyFilledAddressChangeInfo;
    }
    if (end > index)
    {
        numToCopy = end - index;
    }

    AtomicSetMin(&parent->numADUsLeft, pGcrParam->acr.numADUsLeft);
    AtomicSetMax(&parent->nextADUOffset, pGcrParam->acr.nextADUOffset);
    atomic_fetch_add(&parent->numReadErrorADUs, pGcrParam->acr.numReadErrorADUs);
    atomic_fetch_or(&parent->copyStatus, copyStatus);
    if (numProcessedADUs)
    {
        CopySaveLastCopyStatus(parent, pGcrParam->count, copyStatus);
    }
    memcpy(iocb->addressChangeInfo->addressUpdate + index, pGcrParam->acr.addressUpdate,
           numToCopy * sizeof(pGcrParam->acr.addressUpdate[0]));
}

// nlc/gcr that dec's parent's refcnt to 0 calls this to complete
// the nlc iocb.
STATIC void CopyCompleteParent(struct CopyParam *pParam, struct SEFStatus status)
{
    struct SEFNamelessCopyIOCB *iocb = pParam->iocb;
    struct SEFAddressChangeRequest *acr = iocb->addressChangeInfo;
    uint8_t copyStatus;
    SEFQoSHandle dstQosHandle = iocb->dstQosHandle;
    struct SEFFlashAddress copyDestination = iocb->copyDestination;
    uint32_t sbid = GetSbId(dstQosHandle, copyDestination);
    uint32_t numWritten;

    // copy over combined results (addressUpdate already done by CopyCombineResults())
    acr->nextADUOffset = atomic_load(&pParam->nextADUOffset);
    acr->numADUsLeft = atomic_load(&pParam->numADUsLeft);
    acr->numReadErrorADUs = atomic_load(&pParam->numReadErrorADUs);
    acr->numProcessedADUs = atomic_load(&pParam->numProcessedADUs);
    // src consumed comes from pParam->lastStatus;
    copyStatus = atomic_load(&pParam->copyStatus) & ~kCopyConsumedSource;
    copyStatus |= atomic_load(&pParam->lastStatus) & kCopyConsumedSource;
    acr->copyStatus = copyStatus;
    if (!status.error)
    {
        status.info = acr->copyStatus;
    }

    // CopyCombineResults may truncate acr results, but doesn't fix up
    // numprocessedADUs, which can be done now that the copy is complete
    if (acr->numProcessedADUs > iocb->numAddressChangeRecords)
    {
        assert(acr->copyStatus & kCopyFilledAddressChangeInfo);
        acr->numProcessedADUs = iocb->numAddressChangeRecords;
    }
    numWritten = acr->numProcessedADUs;

    // complete the i/o
    iocb->common.status = status;
    iocb->common.flags |= kSefIoFlagDone;
    if (iocb->common.complete_func)
    {
        iocb->common.complete_func(&iocb->common);
    }
    pParam->iocb = iocb = NULL;
    acr = NULL;    // For a single segment copy, this is the user's buffer

    if (pParam->bNLC)    // if copy was issued, set num written and remove ref
    {
        ManageResponseOrder(dstQosHandle, sbid, numWritten, NULL, kMroDecIoRef);
    }
    if (copyStatus & kCopyClosedDestination)
    {
        SendCloseNotification(dstQosHandle, copyDestination, SEFPlacementIdUnused, pParam->numWritten);
    }

    QOSD_REQUEST_COUNT_DEC(pParam->pSrcQos);
    if (pParam->pSrcQos != dstQosHandle)
    {
        QOSD_REQUEST_COUNT_DEC(dstQosHandle);
    }

    if (pParam->bSync)
    {
        if (sem_post(&pParam->syncSemaphore) != 0)
        {
            ULOG_ERROR("failed to sem_post()\n");
        }
    }
    else
    {
        free(pParam);
    }
}

STATIC struct SEFStatus CopyProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common);

struct NlcQEntry
{
    TmaDListEntry link;
    struct SEFCommonIOCB *iocb;
    uint32_t sbid;
    enum { kQCopy, kQFlush, kQClose } type;
};

STATIC void StartNextCopy(SEFQoSHandle qosHandle)
{
    struct SEFStatus status;

    do
    {
        struct NlcQEntry *nlcEntry;

        pthread_mutex_lock(&qosHandle->mutex);
        assert(!utl_DListIsEmpty(&qosHandle->nlcQueue));
        nlcEntry = utl_DListPopHeadAs(&qosHandle->nlcQueue, struct NlcQEntry, link);
        free(nlcEntry);
        nlcEntry = utl_DListGetHeadAs(&qosHandle->nlcQueue, struct NlcQEntry, link);
        pthread_mutex_unlock(&qosHandle->mutex);
        status = (struct SEFStatus){0, 0};
        if (nlcEntry)
        {
            switch (nlcEntry->type)
            {
                case kQClose:
                    status = CloseSbProc(false, qosHandle, nlcEntry->iocb, true);
                    break;
                case kQFlush:
                    status = FlushSbProc(false, qosHandle, nlcEntry->iocb, false, true);
                    break;
                case kQCopy:
                    status = CopyProc(false, qosHandle, nlcEntry->iocb);
                    break;
            }
        }
        if (status.error != 0)
        {
            nlcEntry->iocb->status = status;
            nlcEntry->iocb->flags |= kSefIoFlagDone;
            if (nlcEntry->iocb->complete_func != NULL)
            {
                nlcEntry->iocb->complete_func(nlcEntry->iocb);
            }
        }
    } while (status.error != 0);
}

// returns true if queued
STATIC int QueueNLCIocb(SEFQoSHandle qosHandle,
                        struct SEFFlashAddress flashAddress,
                        struct SEFCommonIOCB *iocb,
                        uint8_t type)
{
    struct NlcQEntry *nlcEntry = NULL;
    uint32_t sbid = GetSbId(qosHandle, flashAddress);

    pthread_mutex_lock(&qosHandle->mutex);
    while ((nlcEntry = utl_DListPrevAs(&qosHandle->nlcQueue, nlcEntry, struct NlcQEntry, link)))
    {
        if (nlcEntry->sbid == sbid)
        {
            nlcEntry = malloc(sizeof(*nlcEntry));
            utl_DListInitEntry(&nlcEntry->link);
            nlcEntry->type = type;
            nlcEntry->iocb = iocb;
            nlcEntry->sbid = sbid;
            utl_DListPushTail(&qosHandle->nlcQueue, &nlcEntry->link);
            break;
        }
    }
    pthread_mutex_unlock(&qosHandle->mutex);
    return !!nlcEntry;
}

#if CC_FLUSH_SYNC_COPY
STATIC void FlushCopyComplete(struct SEFCommonIOCB *common)
{
    struct CopyParam *parent = common->param1;
    struct FlushSuperBlockIOCB *flush_iocb = (void *)common;

    if (common->status.error == 0)
    {
        if (flush_iocb->distanceToEndOfSuperBlock == 0)
        {
            atomic_fetch_or(&parent->copyStatus, kCopyClosedDestination);
        }
        AtomicSetMin(&parent->numADUsLeft, flush_iocb->distanceToEndOfSuperBlock);
    }
    if (atomic_fetch_sub(&parent->refCnt, 1) == 1)
    {
        struct SEFStatus status = {};

        CopyCompleteParent(parent, status);
    }
}

STATIC struct SEFStatus FlushCopy(struct CopyParam *parent)
{
    struct FlushSuperBlockIOCB *flush_iocb = calloc(1, sizeof(*flush_iocb));
    struct SEFNamelessCopyIOCB *iocb = parent->iocb;
    struct SEFQoSHandle_ *pQos = iocb->dstQosHandle;
    struct SEFStatus status;

    assert(parent->bFlush);

    flush_iocb->common.reserved = 1;
    flush_iocb->flashAddress = iocb->copyDestination;
    flush_iocb->common.complete_func = FlushCopyComplete;
    flush_iocb->common.param1 = parent;

    parent->bFlush = false;    // Assume will issue
    status = FlushSbProc(false, pQos, &flush_iocb->common, true, true);
    if (status.error != 0)
    {
        parent->bFlush = true;    // Not really issued
    }
    return status;
}
#endif

static bool CopyBitmapEmpty(struct CopyParam *parent)
{
    struct SEFCopySource *source = &parent->iocb->copySource;
    uint32_t i;

    for (i = parent->offset; i < source->arraySize; i++)
    {
        if (source->validBitmap[i])
        {
            return false;
        }
    }
    return true;
}

/**
 *  @brief	Responds to Copy
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here (for async).
 *  @note    Memory for related parameters in Req2Cmp is released here (for async).
 */
STATIC void CheckResultCopy(struct ReqToComplete *pReq2Cmp)
{
    struct SEFNamelessCopyIOCB *iocb = &pReq2Cmp->pIocb->namelessCopy;
    struct CopyNlcParam *pNlcParam = (struct CopyNlcParam *)pReq2Cmp->param;
    struct CopyParam *parent = pNlcParam->parent;
    bool bDivLast = !parent->count;
    bool bErr = false;
    bool bEnd = true;
    int ret = -1;
    uint8_t copyStatus = (pReq2Cmp->result >> 24) & 0xff;
    uint32_t adusLeft = pReq2Cmp->result & 0xFFffff;

    ULOG_CPCMD_DBG("adusLeft=0x%06x, copyStatus=0x%02x\n", adusLeft, copyStatus);

#if CC_DUMP_NLC
    uint32_t bmSize = (pReq2Cmp->submit.cdw10 + 1) / 2;
    uint32_t i;
    ULOG_NOTICE("NLC 0x%x of 0x%x entries to 0x%lx complete; adusLeft=0x%06x, copyStatus=0x%02x\n",
                pReq2Cmp->submit.cdw13, bmSize, pNlcParam->source.bm.sfla.bits, adusLeft, copyStatus);
    for (i = 0; i < bmSize; i += 4)
    {
        uint64_t tmp[4] = {};
        size_t n = bmSize - i < 4 ? bmSize - i : 4;

        memcpy(tmp, &pNlcParam->source.bm.bitmap[i], n * sizeof(tmp[0]));
        ULOG_NOTICE("%06x %016lx %016lx %016lx %016lx\n", i, tmp[0], tmp[1], tmp[2], tmp[3]);
    }
#endif
    // Command error
    if (pReq2Cmp->status != 0)
    {
        SEF_ASSERT(pReq2Cmp->status != -EBUSY);    // should be handled by request.c
        SEF_ASSERT(pReq2Cmp->status != -EWOULDBLOCK);

        bErr = true;
        goto ExitProc;
    }

    if (pNlcParam->_1st)
    {
        uint32_t sbid = GetSbId(iocb->dstQosHandle, iocb->copyDestination);

        ManageResponseOrder(iocb->dstQosHandle, sbid, 0, NULL, kMroIncIoRef);
        parent->bNLC = true;
    }
    // Device returned unexpected CSTS - 3 reasons to stop copying
    if ((copyStatus & (kCopyConsumedSource | kCopyClosedDestination | kCopyFilledAddressChangeInfo)) == 0)
    {
        ULOG_ERROR(
            "unexpected copy status!! consumed src, dst and acr bits are zero, "
            "copyStatus=0x%02x\n",
            copyStatus);
        iocb->common.status.error = DeviceToSefError(pReq2Cmp->status);
        bErr = true;
        goto ExitProc;
    }

    bEnd = bDivLast;
    if (!bEnd && (copyStatus & kCopyClosedDestination))
    {
        bool sourceConsumed = !!(copyStatus & kCopyConsumedSource);
        enum SEFCopySourceType format = iocb->copySource.format;
        uint8_t clearCSMask = ~kCopyConsumedSource;

        // Ending copy early because of full destination, if source consumed,
        // verify the rest of the bit are 0's
        if (format == kBitmap && sourceConsumed && CopyBitmapEmpty(parent))
        {
            clearCSMask = ~0;
        }
        CopySaveLastCopyStatus(parent, 0, copyStatus & clearCSMask);
        bEnd = true;
    }
    ret = 0;
    if (!bEnd)
    {
        if (iocb->copySource.format == kBitmap)
        {
            ret = CopySegBmProc(parent);
        }
        else
        {
            ret = CopySegFalProc(parent);
        }
    }
#if CC_FLUSH_SYNC_COPY
    else if (parent->bFlush)
    {
        ret = FlushCopy(parent).error;
    }
#endif

    bErr = !!ret;

ExitProc:
    // Finalization - If CopySegProc fails, undo the refcnt of what didn't
    //                submit.  If at the end, undo the NLC refcnt
    if (bErr || bEnd)
    {
        if (!parent->bSync)
        {
            StartNextCopy(parent->pSrcQos);
        }
#if CC_FLUSH_SYNC_COPY
        // -1 more if flush wasn't issued but needed to be
        if (atomic_fetch_sub(&parent->refCnt, 1 + parent->bFlush) == 1 + parent->bFlush)
#else
        if (atomic_fetch_sub(&parent->refCnt, 1) == 1)
#endif
        {
            struct SEFStatus status = {.error = bErr ? -1 : 0};
            CopyCompleteParent(parent, status);
        }
    }
    free(pNlcParam);
}

#if CC_ACR_VALIDATION
static void dumpAcr(uint32_t nlcid, struct SEFAddressChangeRequest *acr)
{
    uint32_t i;

    ULOG_NOTICE(
        "ACR from nlc 0x%x: copyStatus=%d nextAdu=%d adusLeft=%d numProc=%d, numReadErr=%d\n",
        nlcid, acr->copyStatus, acr->nextADUOffset, acr->numADUsLeft, acr->numProcessedADUs,
        acr->numReadErrorADUs);
    for (i = 0; i < acr->numProcessedADUs; i++)
    {
        ULOG_NOTICE("%06u: UA:%lx NF:%lx OF:%lx", i, acr->addressUpdate[i].userAddress.unformatted,
                    acr->addressUpdate[i].newFlashAddress.bits,
                    acr->addressUpdate[i].oldFlashAddress.bits);
    }
}
#endif

struct SEFFlashAddress getLastValidAcrFla(struct SEFAddressChangeRequest *acr)
{
    struct SEFFlashAddress lastFLA = SEFAutoAllocate;
    int numADUs = acr->numProcessedADUs;
    int i;

    for (i = numADUs - 1; i >= 0; i--)
    {
        lastFLA = acr->addressUpdate[i].newFlashAddress;
        if (!SEFIsEqualFlashAddress(lastFLA, SEFAutoAllocate))
        {
            break;
        }
    }
    return lastFLA;
}

/**
 *  @brief	Responds to GetCopyResult
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here (for async).
 *  @note    Memory for related parameters in Req2Cmp is released here (for async).
 */
STATIC void CheckResultGetCopyResult(struct ReqToComplete *pReq2Cmp)
{
    struct SEFNamelessCopyIOCB *iocb = &pReq2Cmp->pIocb->namelessCopy;
    struct CopyGcrParam *pGcrParam = (struct CopyGcrParam *)pReq2Cmp->param;
    struct CopyParam *parent = pGcrParam->parent;
    struct SEFAddressChangeRequest *acr = &pGcrParam->acr;
    uint32_t status = pReq2Cmp->result & UINT32_MAX;
    uint32_t numADUs = acr->numProcessedADUs;
    bool bErr = false;

    ULOG_CPCMD_DBG("status=0x%x\n", status);

    // Command error
    if (pReq2Cmp->status != 0)
    {
        bErr = true;
        goto ExitProc;
    }

    // Device returned unexpected CSTS - need 1 of 3 reasons to stop copying
    if ((acr->copyStatus &
         (kCopyConsumedSource | kCopyClosedDestination | kCopyFilledAddressChangeInfo)) == 0)
    {
        ULOG_ERROR(
            "unexpected copy status!! consumed src, dst and acr bits are zero, "
            "copyStatus=0x%02x\n",
            acr->copyStatus);
        iocb->common.status.error = DeviceToSefError(pReq2Cmp->status);
        bErr = true;
        goto ExitProc;
    }

    // GCR failed
    if (iocb->common.status.error)
    {
        bErr = true;
        goto ExitProc;
    }

    if (numADUs)
    {
#if CC_ACR_VALIDATION
        unsigned int i;
        uint64_t lastNewFLA = 0;
        uint64_t lastOldFLA = 0;

        for (i = 0; iocb->copySource.format == kBitmap && i < numADUs; i++)
        {
            if (acr->addressUpdate[i].newFlashAddress.bits == 0)
            {
                ULOG_NOTICE("At %u, new flash address is null\n", i);
                break;
            }
            else if (GetQosdId(acr->addressUpdate[i].newFlashAddress) != GetQosdId(iocb->copyDestination))
            {
                ULOG_NOTICE("At %u new flash address 0x%lx wrong domain %u\n", i,
                            acr->addressUpdate[i].newFlashAddress.bits,
                            GetQosdId(iocb->copyDestination));
                break;
            }
            else if (GetSbId(parent->pSrcQos, acr->addressUpdate[i].newFlashAddress) !=
                     GetSbId(parent->pSrcQos, iocb->copyDestination))
            {
                ULOG_NOTICE("At %u new flash address 0x%lx wrong sbid %u\n", i,
                            acr->addressUpdate[i].newFlashAddress.bits,
                            GetSbId(parent->pSrcQos, iocb->copyDestination));
                break;
            }
            else if (acr->addressUpdate[i].newFlashAddress.bits < lastNewFLA)
            {
                ULOG_NOTICE("At %u, new flash address not increasing 0x%lx to 0x%lx\n", i,
                            acr->addressUpdate[i].newFlashAddress.bits, lastNewFLA);
                break;
            }
            if (acr->addressUpdate[i].oldFlashAddress.bits == 0)
            {
                ULOG_NOTICE("At %u, old flash address is null\n", i);
                break;
            }
            else if (GetQosdId(acr->addressUpdate[i].oldFlashAddress) !=
                     GetQosdId(iocb->copySource.srcFlashAddress))
            {
                ULOG_NOTICE("At %u old flash address 0x%lx wrong domain %u\n", i,
                            acr->addressUpdate[i].oldFlashAddress.bits,
                            GetQosdId(iocb->copySource.srcFlashAddress));
                break;
            }
            else if (GetSbId(parent->pSrcQos, acr->addressUpdate[i].oldFlashAddress) !=
                     GetSbId(parent->pSrcQos, iocb->copySource.srcFlashAddress))
            {
                ULOG_NOTICE("At %u old flash address 0x%lx wrong sbid %u\n", i,
                            acr->addressUpdate[i].oldFlashAddress.bits,
                            GetSbId(parent->pSrcQos, iocb->copySource.srcFlashAddress));
                break;
            }
            else if (acr->addressUpdate[i].oldFlashAddress.bits < lastOldFLA)
            {
                ULOG_NOTICE("At %u, new flash address not increasing 0x%lx to 0x%lx\n", i,
                            acr->addressUpdate[i].oldFlashAddress.bits, lastOldFLA);
                break;
            }

            // Allow 1st UA to be 0 for itest
            if (i && acr->addressUpdate[i].userAddress.unformatted == 0)
            {
                ULOG_NOTICE("At %u user address is null\n", i);
                break;
            }
            lastNewFLA = acr->addressUpdate[i].newFlashAddress.bits;
            lastOldFLA = acr->addressUpdate[i].oldFlashAddress.bits;
        }
        if (iocb->copySource.format == kBitmap && i < numADUs)
        {
            dumpAcr(pReq2Cmp->submit.cdw13, acr);
        }
#endif
        if (acr->copyStatus & kCopyClosedDestination)
        {
            struct SEFFlashAddress lastFLA;

            lastFLA = getLastValidAcrFla(acr);
            if (!SEFIsEqualFlashAddress(lastFLA, SEFAutoAllocate))
            {
                parent->numWritten = GetAduOffset(iocb->dstQosHandle, lastFLA) + 1;
            }
        }
    }
    CopyCombineResults(pGcrParam);

ExitProc:
    // Finalization
    if (atomic_fetch_sub(&parent->refCnt, 1) == 1)
    {
        struct SEFStatus status = {.error = bErr ? -1 : 0};
        CopyCompleteParent(parent, status);
    }
    free(pGcrParam);
}

STATIC struct SEFFlashAddress CalcCopyStartFlashAddress(SEFQoSHandle pQos,
                                                        struct SEFFlashAddress start,
                                                        uint32_t offset)
{
    uint32_t block, aduOffset;
    struct SEFQoSDomainID qosId;

    SEFParseFlashAddress(pQos, start, &qosId, &block, &aduOffset);
    aduOffset += offset ? offset * 64 - (aduOffset % 64) : 0;
    return SEFCreateFlashAddress(pQos, qosId, block, aduOffset);
}

STATIC uint32_t GetNamelessCopyTimeout(SEFQoSHandle pQos)
{
#define NLC_TIMEOUT_BASE         30
#define NLC_BASE_SIZE            (128 * 1024)
#define NLC_TIMEOUT_FACTOR(pQos) ((pQos)->maxBufSize / NLC_BASE_SIZE ?: 1)
    return NLC_TIMEOUT_BASE * 60 * 1000 * NLC_TIMEOUT_FACTOR(pQos);
}

STATIC int CopySubmitSegment(struct CopyNlcParam *pNlcParam, struct CopyGcrParam *pGcrParam)
{
    struct CopyParam *parent = pNlcParam->parent;
    struct SEFNamelessCopyIOCB *iocb = parent->iocb;
    SEFQoSHandle pDstQos = iocb->dstQosHandle;
    struct RequestInfo req[2] = {};
    struct SEFFlashAddress dstFlashAddress;
    struct SEFCopySource copySource;
    uint32_t nlcid;
    uint32_t aduOffset;
    int fua = (parent->count == 0 && iocb->common.flags & kSefIoFlagCommit);
    int ret;
    bool override = !!(iocb->common.flags & kSefIoFlagOverride);
    struct SEFCopyOverrides *overrides = override ? &iocb->overrides : NULL;

    assert(pNlcParam->srcSize <= pDstQos->maxBufSize);
    assert(pGcrParam->acrSize <= pDstQos->maxBufSize);

    pNlcParam->pGcrParam = pGcrParam;    // used to retry on EBUSY
    pGcrParam->count = parent->count;    // remember segement placement
    ret = AllocateReqInfo(&req[0].pReq2Cmp, NULL, NULL, 0, &req[0].pSem);
    if (ret != 0)
    {
        goto FreeReq2Cmp;
    }

    ret = AllocateReqInfo(&req[1].pReq2Cmp, NULL, NULL, 0, &req[1].pSem);
    if (ret != 0)
    {
        goto FreeReq2Cmp;
    }
#if CC_FLUSH_SYNC_COPY
    parent->bFlush = fua;
    fua = 0;
#endif
    pNlcParam->pReq2Cmp = req[0].pReq2Cmp;
    pNlcParam->pReq2Cmp->pIocb = (union RequestIocb *)iocb;
    pNlcParam->pReq2Cmp->type = kIoctlIoFusedCommand;
    pNlcParam->pReq2Cmp->bSync = false;    // sem processing is done here rather than in Request
    pNlcParam->pReq2Cmp->syncOverride =
        parent->bSync;    // sem processing is done here rather than in Request
    pNlcParam->pReq2Cmp->pSyncSemaphore = req[0].pSem;    // Not used
    pNlcParam->pReq2Cmp->param = pNlcParam;
    pNlcParam->pReq2Cmp->CommandComplete = Complete;
    pNlcParam->pReq2Cmp->CheckResultExt = CheckResultCopy;
    pNlcParam->pReq2Cmp->CheckSuspendFunc = NULL;

    pGcrParam->pReq2Cmp = req[1].pReq2Cmp;
    pGcrParam->pReq2Cmp->pIocb = (union RequestIocb *)iocb;
    pGcrParam->pReq2Cmp->type = kIoctlIoFusedCommand;
    pGcrParam->pReq2Cmp->bSync = false;    // sem processing is done here rather than in Request
    pGcrParam->pReq2Cmp->syncOverride =
        parent->bSync;    // sem processing is done here rather than in Request
    pGcrParam->pReq2Cmp->pSyncSemaphore = req[1].pSem;    // Not used
    pGcrParam->pReq2Cmp->param = pGcrParam;
    pGcrParam->pReq2Cmp->CommandComplete = Complete;
    pGcrParam->pReq2Cmp->CheckResultExt = CheckResultGetCopyResult;
    pGcrParam->pReq2Cmp->CheckSuspendFunc = NULL;

    copySource.format = iocb->copySource.format;
    copySource.arraySize = pNlcParam->numSrcEntries;
    nlcid = atomic_fetch_add(&pDstQos->nlcid, 1);
    aduOffset = (pDstQos->aduOffsetBitLen < 32) ? (1U << pDstQos->aduOffsetBitLen) - 1 : UINT32_MAX;
    dstFlashAddress.bits = htole64(le64toh(iocb->copyDestination.bits) | aduOffset);
    SEFNvmNamelessCopy(&pNlcParam->pReq2Cmp->submit, &pNlcParam->source, pNlcParam->srcSize,
                       pDstQos->qosId, &copySource, iocb->filter, overrides, nlcid, dstFlashAddress,
                       GetNamelessCopyTimeout(pDstQos));

    SEFNvmGetCopyResult(&pGcrParam->pReq2Cmp->submit, &pGcrParam->acr, pGcrParam->acrSize,
                        pDstQos->qosId, fua, pGcrParam->numAcrEntries, nlcid,
                        pDstQos->pSefHandle->writeTimeout);

    // Only one NLC outstanding at a time, caller expected to have a ref cnt so
    // we don't have to increment it here but we do for the GCR before we submit
    // the NLC.
#if CC_FLUSH_SYNC_COPY
    // +1 for flush to be issued when this NLC completes
    atomic_fetch_add(&parent->refCnt, 1 + parent->bFlush);
#else
    atomic_fetch_add(&parent->refCnt, 1);
#endif
    // #if CC_USE_FUSED
    ret = RequestFusedWrite(pDstQos->deviceFd, pNlcParam->pReq2Cmp, pGcrParam->pReq2Cmp);
    if (ret == 0)
    {
        return 0;
    }
    else
    {
        // undo count of GCR that wasn't submitted.  Caller will decrement
        // the one for NLC.
#if CC_FLUSH_SYNC_COPY
        // -1 for flush
        if (atomic_fetch_sub(&parent->refCnt, 1 + parent->bFlush) == 1)
#else
        if (atomic_fetch_sub(&parent->refCnt, 1) == 1)
#endif
        {
            struct SEFStatus status = {ret, 0};
            CopyCompleteParent(parent, status);
        }
    }

FreeReq2Cmp:
    if (pNlcParam)
    {
        free(pNlcParam->pReq2Cmp);
    }
    free(pNlcParam);
    if (pGcrParam)
    {
        free(pGcrParam->pReq2Cmp);
    }
    free(pGcrParam);
    return ret;
}

STATIC struct CopyNlcParam *CopyAllocBmNlcParam(struct CopyParam *parent, uint32_t offset, uint32_t count)
{
    struct CopyNlcParam *pNlcParam;
    struct SEFCopySource *source = &parent->iocb->copySource;
    const struct SEFUserAddressFilter *uaf = parent->iocb->filter;
    // SEFAPI is QWORD based and SEF Spec is DWORD based - adjust if bitIndex
    // is into the second DWORD.  It only applies to the first segment.
    int bitIndex = GetAduOffset(parent->pSrcQos, source->srcFlashAddress) % 64;
    int _2ndDWord = (offset == 0 && bitIndex > 31);
    uint32_t dwCount = count * 2 - _2ndDWord;
    uint32_t *bitmap = ((uint32_t *)&source->validBitmap[offset]) + _2ndDWord;
    int srcSize = struct_size(pNlcParam, source.bm.bitmap, count);

    pNlcParam = malloc(srcSize);
    if (pNlcParam)
    {
        pNlcParam->numSrcEntries = count;
        pNlcParam->parent = parent;
        pNlcParam->srcSize = srcSize - offsetof(struct CopyNlcParam, source);
        pNlcParam->source.bm.uar.sua.unformatted = 0;
        pNlcParam->source.bm.uar.lr = 0;
        if (uaf)
        {
            pNlcParam->source.bm.uar.sua = uaf->userAddressStart;
            pNlcParam->source.bm.uar.lr = uaf->userAddressRangeLength;
        }
        pNlcParam->source.bm.sfla =
            CalcCopyStartFlashAddress(parent->pSrcQos, source->srcFlashAddress, offset);
        // fill last dw with 0 for when _2ndDWord is 1
        pNlcParam->source.bm.bitmap[count - 1] = 0;
        memcpy(pNlcParam->source.bm.bitmap, bitmap, dwCount * sizeof(*bitmap));
    }
    return pNlcParam;
}

STATIC struct CopyNlcParam *CopyAllocFalNlcParam(struct CopyParam *parent, uint32_t offset, uint32_t count)
{
    struct CopyNlcParam *pNlcParam;
    struct SEFCopySource *source = &parent->iocb->copySource;
    const struct SEFUserAddressFilter *uaf = parent->iocb->filter;
    const struct SEFFlashAddress *fal = &source->flashAddressList[offset];
    int srcSize = struct_size(pNlcParam, source.fal.fla, count);

    pNlcParam = malloc(srcSize);
    if (pNlcParam)
    {
        pNlcParam->numSrcEntries = count;
        pNlcParam->parent = parent;
        pNlcParam->srcSize = srcSize - offsetof(struct CopyNlcParam, source);
        pNlcParam->source.fal.uar.sua.unformatted = 0;
        pNlcParam->source.fal.uar.lr = 0;
        if (uaf)
        {
            pNlcParam->source.fal.uar.sua = uaf->userAddressStart;
            pNlcParam->source.fal.uar.lr = uaf->userAddressRangeLength;
        }
        memcpy(pNlcParam->source.fal.fla, fal, count * sizeof(*fal));
    }
    return pNlcParam;
}

STATIC struct CopyGcrParam *CopyAllocGcrParam(struct CopyParam *parent, uint32_t count)
{
    struct CopyGcrParam *pGcrParam;
    int paramSize = struct_size(pGcrParam, acr.addressUpdate, count);
    int acrSize = struct_size(&pGcrParam->acr, addressUpdate, count);

    SEF_ASSERT(paramSize > acrSize);
    pGcrParam = calloc(1, paramSize);
    if (pGcrParam)
    {
        pGcrParam->acrSize = acrSize;
        pGcrParam->numAcrEntries = count;
        pGcrParam->parent = parent;
    }
    return pGcrParam;
}

STATIC int CopySegBmProc(struct CopyParam *parent)
{
    uint32_t aduCount = parent->count * 64;
    uint32_t count = (aduCount > parent->maxCount ? parent->maxCount : aduCount) / 64;
    uint32_t offset = parent->offset;
    struct CopyNlcParam *pNlcParam;
    struct CopyGcrParam *pGcrParam;
    uint32_t numACR = count * 64;

    // Segment is first and only, we can honor numAddressChangeRecords.
    if (offset == 0 && numACR > parent->iocb->numAddressChangeRecords)
    {
        numACR = parent->iocb->numAddressChangeRecords;
    }
    pNlcParam = CopyAllocBmNlcParam(parent, offset, count);
    pGcrParam = CopyAllocGcrParam(parent, numACR);
    if (pNlcParam && pGcrParam)
    {
        pNlcParam->_1st = (offset == 0);
        parent->count -= count;
        parent->offset += count;
        return CopySubmitSegment(pNlcParam, pGcrParam);
    }
    free(pNlcParam);
    free(pGcrParam);
    return -ENOMEM;
}

STATIC int CopySegFalProc(struct CopyParam *parent)
{
    uint32_t offset = parent->offset;
    uint32_t count = parent->count > parent->maxCount ? parent->maxCount : parent->count;
    struct CopyNlcParam *pNlcParam;
    struct CopyGcrParam *pGcrParam;

    pNlcParam = CopyAllocFalNlcParam(parent, offset, count);
    pGcrParam = CopyAllocGcrParam(parent, count);
    if (pNlcParam && pGcrParam)
    {
        pNlcParam->_1st = (parent->offset == 0);
        parent->count -= count;
        parent->offset += count;
        return CopySubmitSegment(pNlcParam, pGcrParam);
    }
    free(pNlcParam);
    free(pGcrParam);
    return -ENOMEM;
}

STATIC int CopyBmProc(struct CopyParam *pParam)
{
    size_t maxAcrCount = MaxAcrCount(pParam->pSrcQos);
    pParam->maxCount = (pParam->pSrcQos->maxBufSize - sizeof_member(union CopySource, bm)) /
                       sizeof_member(union CopySource, bm.bitmap[0]);
    if (pParam->maxCount > maxAcrCount)
    {
        pParam->maxCount = maxAcrCount;
    }
    return CopySegBmProc(pParam);
}

STATIC int CopyFalProc(struct CopyParam *pParam)
{
    size_t maxAcrCount = MaxAcrCount(pParam->pSrcQos);

    pParam->maxCount = (pParam->pSrcQos->maxBufSize - sizeof_member(union CopySource, fal)) /
                       sizeof_member(union CopySource, fal.fla[0]);
    if (pParam->maxCount > maxAcrCount)
    {
        pParam->maxCount = maxAcrCount;
    }
    return CopySegFalProc(pParam);
}

/**
 *  @brief	Internal processing for SEFNamelessCopy and SEFNamelessCopyAsync
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Copy
 *  @details	Arbitrary parameter area for responding to Copy is allocated.
 *  @details	For synchronous operation, this function releases arbitrary parameter area.
 *  @details	Area for Req2Cmp, Sem, and DPTR for each division is allocated (if not divided, only
 *              one for each is allocated)
 *  @details	If command issuing of retry failed this function releases areas for arbitrary
 *              parameter, Req2Cmp, Sem, and DPTR.
 *  @note	For async, CheckResultExt must release arbitrary parameter area.
 *  @note	For both sync and async, Complete() must release Req2Cmp and Sem.
 *  @note	CheckResultExt must release DPTR area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info:
 * fixed to 0
 */
STATIC struct SEFStatus CopyProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common)
{
    struct SEFNamelessCopyIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pSrcQos = pQos;
    struct SEFQoSHandle_ *pDstQos = iocb->dstQosHandle;
    struct CopyParam *pParam;
    bool bDstProcReqInc = false;
    int ret;

    // Check parameters
    if ((iocb->dstQosHandle == NULL) || (iocb->addressChangeInfo == NULL) ||
        ((iocb->copySource.format != kBitmap) && (iocb->copySource.format != kList)))
    {
        ULOG_ERROR("argument error!! dstQosHandle=%p, addressChangeInfo=%p, format=%d\n",
                   iocb->dstQosHandle, iocb->addressChangeInfo, iocb->copySource.format);
        status.error = iocb->dstQosHandle == NULL ? -ENODEV : -EINVAL;
        goto ExitProc;
    }

    // Check parameter (copySource)
    if (((iocb->copySource.format == kBitmap) && (iocb->copySource.validBitmap == NULL)) ||
        ((iocb->copySource.format == kList) && (iocb->copySource.flashAddressList == NULL)) ||
        (iocb->copySource.arraySize == 0) || (iocb->numAddressChangeRecords == 0))
    {
        ULOG_ERROR(
            "argument error!! format=%d(bitmap or list is null), arraySize=%u, "
            "numAddressChangeRecords=%u\n",
            iocb->copySource.format, iocb->copySource.arraySize, iocb->numAddressChangeRecords);
        status.error = -EINVAL;
        goto ExitProc;
    }

    // Check parameters (QoS Domain ID)
    if ((iocb->copySource.format == kBitmap) &&
        (pQos->qosId != GetQosdId(iocb->copySource.srcFlashAddress)))
    {
        ULOG_ERROR("QoSD ID error(bitmap)!! qosId=%d,srcFlashAddress=0x%016lx\n", pQos->qosId,
                   iocb->copySource.srcFlashAddress.bits);
        status.error = -EINVAL;
        goto ExitProc;
    }
    if (iocb->copySource.format == kList)
    {
        for (uint32_t i = 0; i < iocb->copySource.arraySize; i++)
        {
            if (pQos->qosId != GetQosdId(iocb->copySource.flashAddressList[i]))
            {
                ULOG_ERROR("QoSD ID error(flaList)!! qosId=%d,flashAddressList[%d]=0x%016lx\n",
                           pQos->qosId, i, iocb->copySource.flashAddressList[i].bits);
                status.error = -EINVAL;
                goto ExitProc;
            }
        }
    }
    if (iocb->dstQosHandle->qosId != GetQosdId(iocb->copyDestination))
    {
        ULOG_ERROR("QoSD ID error!! copyDestination=0x%016lx\n", iocb->copyDestination.bits);
        status.error = -EINVAL;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pSrcQos);

    // If copy source QoSD differs from that of destination, increment number of requests being processed of both QoSDs
    if (pSrcQos->qosId != pDstQos->qosId)
    {
        QOSD_REQUEST_COUNT_INC(pDstQos);
        bDstProcReqInc = true;
    }

    ULOG_CPCMD_DBG("src qosd id=0x%04x, dst qosd id=0x%04x\n", pSrcQos->qosId, pDstQos->qosId);

    // DEBUG_SAVE_INFO_FILE(pContext->data, pContext->dataSize, kDebugSaveFileCopyData);

    // Prepare parameters
    pParam = calloc(1, sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }
    pParam->iocb = iocb;
    pParam->pSrcQos = pSrcQos;
    pParam->count = iocb->copySource.arraySize;
    pParam->bSync = bSync;
    pParam->numADUsLeft = INT32_MAX;
    pParam->lastStatus = (((uint64_t)INT32_MAX) << 32) + kCopyConsumedSource;
    if (bSync)
    {
        ret = sem_init(&pParam->syncSemaphore, 0, 0);
        if (ret != 0)
        {
            ULOG_ERROR("failed to sem_init()\n");
            goto FreeParam;
        }
    }

    atomic_store(&pParam->refCnt, 1);    // for NLC
    if (iocb->copySource.format == kBitmap)
    {
        ret = CopyBmProc(pParam);
    }
    else
    {
        ret = CopyFalProc(pParam);
    }
    if (ret && atomic_fetch_sub(&pParam->refCnt, 1) == 1)
    {
        CopyCompleteParent(pParam, (struct SEFStatus){-1});
        goto FreeParam;
    }

    if (bSync)
    {
        // Wait all divided commands to complete
        ret = sem_wait(&pParam->syncSemaphore);
        if (ret != 0)
        {
            ULOG_ERROR("failed to sem_wait()\n");
        }

        ret = sem_destroy(&pParam->syncSemaphore);
        SEF_ASSERT(ret == 0);

        // For synchronous operation, memory for pParam is released here (different from async because of semaphore)
        free(pParam);

        // For synchronous operation, command result is set
        status = iocb->common.status;

        goto ExitProc;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;

        goto ExitProc;
    }
FreeParam:
    free(pParam);
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(pSrcQos);
    if (bDstProcReqInc)
    {
        QOSD_REQUEST_COUNT_DEC(pDstQos);
    }
ExitProc:
    return status;
}

/**
 *  @brief	Converts Read Deadline type for SEFAPI to Read Deadline type for device
 *  @param	[in] type: Read Deadline type for SEFAPI
 *  @return	Read Deadline type for device
 *  @note    As values for SEFAPI and device do not match, conversion is needed
 */
STATIC enum ReadDeadlineForDev ConvReadDeadlineForDev(enum SEFDeadlineType type)
{
    enum ReadDeadlineForDev typeForDev;

    switch (type)
    {
        case kFastest:
            typeForDev = kReadDeadlineForDevFastest;
            break;
        case kTypical:
            typeForDev = kReadDeadlineForDevTypical;
            break;
        case kLong:
            typeForDev = kReadDeadlineForDevLong;
            break;
        case kHeroic:
            typeForDev = kReadDeadlineForDevHeroic;
            break;
        default:
            typeForDev = kReadDeadlineForDevTypical;
            ULOG_ERROR("read deadline type is invalid!! %d\n", type);
            SEF_ASSERT(0);
    }

    return typeForDev;
}

/**
 *  @brief	Converts Read Deadline type for device to Read Deadline type for SEFAPI
 *  @param	[in] type: Read Deadline type for device
 *  @return	Read Deadline type for SEFAPI
 *  @note    As values for SEFAPI and device do not match, conversion is needed
 */
STATIC enum SEFDeadlineType ConvReadDeadlineForApi(enum ReadDeadlineForDev type)
{
    enum SEFDeadlineType typeForApi;

    switch (type)
    {
        case kReadDeadlineForDevFastest:
            typeForApi = kFastest;
            break;
        case kReadDeadlineForDevTypical:
            typeForApi = kTypical;
            break;
        case kReadDeadlineForDevLong:
            typeForApi = kLong;
            break;
        case kReadDeadlineForDevHeroic:
            typeForApi = kHeroic;
            break;
        default:
            typeForApi = kTypical;
            ULOG_ERROR("read deadline type for device is invalid!! %d\n", type);
            SEF_ASSERT(0);
    }

    return typeForApi;
}

STATIC enum DefectStrategyForDev ConvDefectStrategyForDev(enum SEFDefectManagementMethod type)
{
    enum DefectStrategyForDev typeForDev;

    switch (type)
    {
        case kPacked:
            typeForDev = kDevPacked;
            break;
        case kPerfect:
            typeForDev = kDevPerfect;
            break;
        case kFragmented:
            typeForDev = kDevFragmented;
            break;
        default:
            typeForDev = kDevPerfect;
            ULOG_ERROR("defect strategy %d is invalid!! - mapped to perfect\n", type);
    }

    return typeForDev;
}

STATIC enum SEFDefectManagementMethod ConvDefectStrategyForApi(enum DefectStrategyForDev type)
{
    enum SEFDefectManagementMethod typeForApi;

    switch (type)
    {
        case kDevPacked:
            typeForApi = kPacked;
            break;
        case kDevPerfect:
            typeForApi = kPerfect;
            break;
        case kDevFragmented:
            typeForApi = kFragmented;
            break;
        default:
            typeForApi = kPerfect;
            ULOG_ERROR("defect strategy %d is invalid!! - mapped to perfect\n", type);
    }

    return typeForApi;
}

/**
 *  @brief	Calculates number of Super Block Descriptors in SB list
 *  @param	[in] pSbList: SB list
 *  @param	[in] sbListDescriptorMax: maximum number of descriptors in SB list
 *  @return	Number of Super Block Descriptors
 */
STATIC uint32_t CalcNumSuperBlockDescriptors(struct GetLogPageSuperBlockList *pSbList,
                                             uint32_t sbListDescriptorMax)
{
    uint32_t numSbDescriptors = 0;

    for (uint32_t i = 0; i < sbListDescriptorMax; i++)
    {
        if (pSbList->SuperBlockDescriptor[i].SBID == SBLIST_LOG_CMD_LIST_END)
        {
            break;
        }
        ++numSbDescriptors;
    }

    return numSbDescriptors;
}

/**
 *  @brief	Retrieves SB information based on SB list of listOrder-specified type
 *  @param	[in] pQos: QoS Handle
 *  @param	[in] listOrder: list type
 *  @param	[out] pNumSbs: number of SBs reported in SB list
 *  @param	[out] pSbRecord: SB information of SBs reported in SB list
 *  @param	[in] saveSbRecords: number of SB Records that can be set to pSbRecord
 *  @return	0: SB information successfully retrieved, -1: SB information retrieval failed
 *  @note    Caller must NULL-check parameters
 */
STATIC int GetSuperBlockList(struct SEFQoSHandle_ *pQos,
                             uint32_t listOrder,
                             uint32_t *pNumSbs,
                             struct SEFSuperBlockRecord *pSbRecord,
                             uint32_t saveSbRecords)
{
    int ret = -1;
    struct GetLogPageSuperBlockList *pSbList;
    struct nvme_passthru_cmd64 cmd;
    const char *pCmdName;
    uint32_t listSize;
    uint32_t cmdStat;
    int cmdIssueResult;
    uint32_t sbListDescriptorMax;

    switch (listOrder)
    {
        case SEF_LODR_CLOSE_WEARLEVEL:
            pCmdName = CMD_A_GLOGP_SBLIST_WEARLVCLOSE;
            break;
        case SEF_LODR_CLOSE_REFRESH:
            pCmdName = CMD_A_GLOGP_SBLIST_REFRESHCLOSE;
            break;
        case SEF_LODR_CLOSE_CHECK:
            pCmdName = CMD_A_GLOGP_SBLIST_CHECKCLOSE;
            break;
        case SEF_LODR_OPEN_CLOSED:
            pCmdName = CMD_A_GLOGP_SBLIST_OPENCLOSED;
            break;
        default:
            ULOG_ERROR("list order is invalid!! %d\n", listOrder);
            SEF_ASSERT(0);
            goto ExitProc;
    }

    // Issue command
    sbListDescriptorMax = SBLIST_MAX_LEN(pQos);
    listSize = TO_UPPER_DWORD_SIZE(sizeof(pSbList->SuperBlockDescriptor[0]) *
                                   sbListDescriptorMax);    // Convert to DWORD
    SEFAdmGetLogPageSBList(&cmd, NULL, listSize, pQos->qosId, listOrder, 0);
    cmdIssueResult = IssueRecvDataCmd(pQos->deviceFd, true, pCmdName, &cmd, &cmdStat, NULL, NULL,
                                      NULL, (void **)&pSbList, listSize);
    if ((cmdIssueResult != 0) || (cmdStat != 0))
    {
        ret = cmdIssueResult ?: cmdStat;
        goto ExitProc;
    }

    *pNumSbs = CalcNumSuperBlockDescriptors(pSbList, sbListDescriptorMax);

    // Ceil if maximum number of superBlockRecords is exceeded
    if (saveSbRecords > sbListDescriptorMax)
    {
        saveSbRecords = sbListDescriptorMax;
    }

    for (uint32_t i = 0; i < saveSbRecords; i++)
    {
        struct SuperBlockDescriptor *pDesc = &pSbList->SuperBlockDescriptor[i];

        // Check whether tail of SB list
        if (pDesc->SBID == SBLIST_LOG_CMD_LIST_END)
        {
            break;
        }

        pSbRecord[i].PEIndex = pDesc->PECI;
        pSbRecord[i].state = pDesc->DIISBS;
        pSbRecord[i].flashAddress = GetFlashAddress(pQos, pQos->qosId, pDesc->SBID);
    }

    ret = 0;

    free(pSbList);

ExitProc:
    return ret;
}

static int GetVDInfoLogPage(struct SEFHandle_ *pSef, uint32_t vdID, struct SefLogVdInfo *pVdInfo)
{
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;

    SEFAdmGetLogPageVDInfo(&cmd, pVdInfo, sizeof(*pVdInfo), vdID);
    ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GLOGP_VDINFO, &cmd, &cmdStat, NULL, NULL,
                           NULL, NULL, 0);
    if (!ret && cmdStat)
    {
        ret = cmdStat;
    }
    return ret;
}

static uint32_t GetVDAduDeficit(struct SEFVDHandle_ *pVd)
{
    struct SEFHandle_ *pSef = pVd->pSefHandle;
    struct SEFInfo *sefInfo = pSef->pSefInfo;
    struct SefLogVdInfo vdInfo;
    uint32_t deficit;
    uint32_t result = 0;
    uint32_t sbCap;
    int ret;

    ret = GetVDInfoLogPage(pSef, pVd->vdId, &vdInfo);
    if (ret)
    {
        ULOG_ERROR("Get log page failed - %d\n", ret);
        goto Exit;
    }
    sbCap = (vdInfo.ndie + 1) * sefInfo->numPages * sefInfo->numPlanes;
    sbCap *= sefInfo->pageSize / 4096;
    if (vdInfo.sbsta.nfrsb < vdInfo.ass_qosd)
    {
        deficit = vdInfo.ass_qosd - vdInfo.sbsta.nfrsb;
        result = MAX(result, deficit * sbCap);
    }
    if (vdInfo.psbsta.nfrsb < vdInfo.ass_qosd)
    {
        deficit = vdInfo.ass_qosd - vdInfo.psbsta.nfrsb;
        result = MAX(result, deficit * sbCap);
    }
    if (vdInfo.sbsta.tcap < vdInfo.sbsta.tgcap)
    {
        deficit = vdInfo.sbsta.tgcap - vdInfo.sbsta.tcap;
        result = MAX(result, deficit);
    }
Exit:
    return result;
}

/**
 *  @brief	Issue VD Notifications about critical warnings
 *  @return	None
 */
static void NotifyVdCriticalWarning(struct SEFHandle_ *pSef, struct nvme_sef_notification *event)
{
    struct SEFVDHandle_ *pVd;
    struct SEFVDNotification vdNotify;

    ULOG_INFORMATION("VD critical warning received: vd id=%u, cwarn=0x%x\n", event->vdid, event->cwarn);

    // Lock sef in order to prevent SEFLibraryCleanup from closing vd handles;
    pthread_mutex_lock(&pSef->vd_mutex);
    pVd = DeviceInfoGetVdHandle(pSef, event->vdid);
    if (pVd == NULL)
    {
        pthread_mutex_unlock(&pSef->vd_mutex);
        return;
    }
    SEF_ASSERT(pSef == pVd->pSefHandle);
    if (pVd->notifyFunc == NULL)
    {
        ULOG_NOTICE(
            "vd critical warning not notified. vd id=%u, cwarn=%u "
            "(because : handle=NULL)\n",
            event->vdid, event->cwarn);
        goto Exit;
    }

    vdNotify.virtualDeviceID.id = event->vdid;
    vdNotify.numADUs = GetVDAduDeficit(pVd);
    (*pVd->notifyFunc)(pVd->pContext, vdNotify);
Exit:
    pthread_mutex_unlock(&pSef->vd_mutex);
}

/**
 *  @brief	Issue QoSD Notifications about SuperBlockStateChangeLog
 *  @return	None
 *  @note	Issue kRequirePatrol or kSuperBlockStateChanged according to state
 *          transition type in SuperBlockStateChangeLog
 */
static void NotifySbStateChange(struct SEFHandle_ *pSef, struct nvme_sef_notification *event)
{
    struct SEFQoSHandle_ *pQos;
    struct SEFQoSNotification qosNotify;
    struct SEFFlashAddress tmpFla;

    ULOG_INFORMATION("SB state change received: qosd id=%u, sb id=%u, transition=%u, validADU=%u\n",
                     event->qosd, event->sbid, event->state, event->nvadu);

    // QoSD-ID to notify to cannot be above maximum (checked in advance to avoid
    // passing an illegal value to DeviceInfoGetQosHandle())
    if (IS_INVALID_QOSD_ID(event->qosd, pSef->pSefInfo->maxQoSDomains))
    {
        ULOG_ERROR(
            "qosd id in sb state change log is invalid!! qosd id=%u, sb id=%u, transition=%u, "
            "validADU=%u\n",
            event->qosd, event->sbid, event->state, event->nvadu);
    }

    // Lock sef in order to avoid target Qos being deleted by SEFCloseQoSDomain()
    pthread_mutex_lock(&pSef->mutex);

    /*
     * Do not notify if:
     * - QoSD handle with target QoSD-ID is not Opened
     */
    pQos = DeviceInfoGetQosHandle(pSef, event->qosd);
    if (pQos == NULL)
    {
        ULOG_NOTICE(
            "sb state change log not notified. qosd id=%u, sb id=%u, transition=%u validADU=%u "
            "(because : handle=%p)\n",
            event->qosd, event->sbid, event->state, event->nvadu, pQos);
        // Unlock
        pthread_mutex_unlock(&pSef->mutex);
        return;
    }

    // Lock target Qos
    pthread_mutex_lock(&pQos->mutex);
    /*
     * Do not notify if:
     * - QoSD handle with target QoSD-ID is being Closed
     */
    if (atomic_load(&pQos->bClosingFlag))
    {
        bool flag = atomic_load(&pQos->bClosingFlag);
        ULOG_NOTICE(
            "sb state change log not notified. qosd id=%u, sb id=%u, transition=%u validADU=%u "
            "(because : closing=%d)\n",
            event->qosd, event->sbid, event->state, event->nvadu, flag);
        // Unlock
        pthread_mutex_unlock(&pQos->mutex);
        pthread_mutex_unlock(&pSef->mutex);
        return;
    }

    // Determine notification content according to state transition type in log
    qosNotify.QoSDomainID.id = event->qosd;
    tmpFla = GetFlashAddress(pQos, event->qosd, event->sbid);
    switch (event->state)
    {
            // mutex is locked

        case kSbStateTransToClosedFromNlc:
        case kSbStateTransToClosedFromNlw:
        case kSbStateTransToClosedFromErase: {
            // To avoid QoSD handle being deleted, increment pQos processing request counter
            bool processEvent = true;

            if (processEvent)
            {
                QOSD_REQUEST_COUNT_INC(pQos);
            }

            pthread_mutex_unlock(&pQos->mutex);
            pthread_mutex_unlock(&pSef->mutex);

            if (processEvent)
            {
                qosNotify.type = kSuperBlockStateChanged;
                qosNotify.changedFlashAddress.bits = tmpFla.bits;
                qosNotify.writtenADUs = 0;    // MRO will set it for us
                qosNotify.numADUs = event->nvadu;
                ManageResponseOrder(pQos, event->sbid, 0, &qosNotify, 0);

                // Decrement pQos processing request counter
                QOSD_REQUEST_COUNT_DEC(pQos);
            }
            return;
        }
        case kSbStateTransToUnkown:
            qosNotify.type = kRequirePatrol;
            qosNotify.patrolFlashAddress.bits = tmpFla.bits;
            break;
        default:
            ULOG_ERROR(
                "transition value in sb state change log is invalid!! qosd id=%u, sb id=%u, "
                "transition=%u validADU=%u\n",
                event->qosd, event->sbid, event->state, event->nvadu);
            // Unlock
            pthread_mutex_unlock(&pQos->mutex);
            pthread_mutex_unlock(&pSef->mutex);
            return;
    }

    QOSD_REQUEST_COUNT_INC(pQos);
    // Unlock
    pthread_mutex_unlock(&pQos->mutex);
    pthread_mutex_unlock(&pSef->mutex);
    if (pQos->notifyFunc != NULL)
    {
        // Call notification function registered to QoSD handle
        pQos->notifyFunc(pQos->pContext, qosNotify);
    }
    else
    {
        ULOG_NOTICE(
            "sb state change log not notified. qosd id=%u, sb id=%u, transition=%u validADU=%u "
            "(because : notify=%p)\n",
            event->qosd, event->sbid, event->state, event->nvadu, pQos->notifyFunc);
    }

    QOSD_REQUEST_COUNT_DEC(pQos);

    return;
}

/**
 * @brief   Issue QoSD/VD Notifications from device AEN uevents
 */
STATIC void NotifyAENProc(struct SEFHandle_ *pSef, struct nvme_sef_notification *event)
{
    SEF_ASSERT(pSef != NULL);
    SEF_ASSERT(event != NULL);

    switch (event->aen_result)
    {
        case VD_AEN:
            NotifyVdCriticalWarning(pSef, event);
            break;
        case SB_AEN:
            NotifySbStateChange(pSef, event);
            break;
        default:
            ULOG_ERROR("Event is not a SB or VD AEN but %#x instead - ignoring\n", event->aen_result);
    }

    return;
}

STATIC void SbCompleteAll(void *p1, void *p2)
{
    struct SEFQoSHandle_ *pQos = p1;
    struct SbWriteInformation *sb_info = p2;
    struct sefSlistNode *pList, *pNext, *pWriteList, *pOpenList;
    struct ReqToComplete *pReq;

    pWriteList = SefSlistPopAll(&sb_info->listHeadByWriting);
    pOpenList = SefSlistPopAll(&sb_info->listHeadByOpen);

    pList = SefSlistReverse(pWriteList);
    while (pList != NULL)
    {
        pReq = (struct ReqToComplete *)pList;
        pNext = pList->next;
        (*pReq->CommandComplete)(pReq);
        pList = pNext;
    }

    union SbInfoAtomicState state;
    state.val = atomic_load(&sb_info->state.val);
    if (state.state == kSbInfoNotifying)
    {
        if (pQos->notifyFunc != NULL)
        {
            sb_info->notificationData.writtenADUs = sb_info->writtenAdus;
            pQos->notifyFunc(pQos->pContext, sb_info->notificationData);
        }
        else
        {
            ULOG_NOTICE(
                "sb state change log not notified. qosd id=%u, fla=0x%016lx, type=%u (because : "
                "notify=%p)\n",
                sb_info->notificationData.QoSDomainID.id,
                sb_info->notificationData.changedFlashAddress.bits, sb_info->notificationData.type,
                pQos->notifyFunc);
        }
        sb_info->state.state = kSbInfoNotified;
        sem_post(&sb_info->closed);
    }

    pList = SefSlistReverse(pOpenList);
    while (pList != NULL)
    {
        pReq = (struct ReqToComplete *)pList;
        pNext = pList->next;
        (*pReq->CommandComplete)(pReq);
        pList = pNext;
    }

    DeviceInfoDeleteSb(pQos, sb_info->sbId);
}

/**
 *  @brief	Resumes all suspended responses
 *  @param	[in] pQos: QoSD handle
 *  @details	Resumes all suspended responses waiting Write to complete.
 *  @details	Issues suspended SB nofitications.
 *  @details	Resumes all suspended responses waiting SB Close to complete.
 *  @pre    QoSD handle Mutex must be locked (lock is kept even if function aborts)
 */
STATIC void ResumeAllSuspendedResponse(struct SEFQoSHandle_ *pQos)
{
    atomic_fetch_add(&pQos->sbs_ht_accesses, 1);

    LHTforeach(pQos->sbs_info, SbCompleteAll, pQos);

    struct sefSlistNode *head = SefSlistGetHead(&pQos->sbs_to_delete);
    int32_t arc = atomic_fetch_sub(&pQos->sbs_ht_accesses, 1);
    SEF_ASSERT(arc > 0);
    if (arc == 1)
    {
        DeviceInfoDeletePendingIf(pQos, head);
    }
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFLibraryInit(void)
{
    static int numSefUnits = 0;    // Number of devices

    struct SEFStatus status = {-1, 0};
    char **ppSefUnitsList;
    int numCpus;
    struct SystemInfo *pSystemInfo;
    pthread_t threadId;
    int ret;
    ULOG_ENTER_SEFAPI();

    pthread_mutex_lock(&seflibMutex);

    if (seflibInitCounter > 0)
    {
        status.error = 0;
        status.info = DeviceInfoGetNumSefUnits();
        goto ExitProc;
    }

    ULOG_SYSTEM("SEF SDK version %s\n", SEFSDKVersion);
    ULOG_SYSTEM(SEFLIB_VER_MESG, SEFLIB_VER_MAJOR, SEFLIB_VER_MIDDLE);

    numCpus = GetOnlineCpuNum();
    if (numCpus < 0)
    {
        ULOG_ERROR("failed to GetOnlineCpuNum()\n");
        goto ExitProc;
    }

    ret = DeviceIoInit(numCpus);
    if (ret != 0)
    {
        ULOG_ERROR("failed to DeviceIoInit()\n");
        goto ExitProc;
    }

    // Check connected SEF Units and DeviceName
    ret = GetSefUnitList(&numSefUnits, &ppSefUnitsList);
    if (ret != 0)
    {
        ULOG_ERROR("failed to GetSefUnitList()\n");
        goto DestroyDeviceIo;
    }
    if (numSefUnits == 0)
    {
        // Number of devices = 0 is handled as a normal case, but unneeded initialization is skipped
        ULOG_NOTICE("sef unit not detected!!\n");
        status.error = 0;
        status.info = 0;
        goto DestroyDeviceIo;
    }

    ret = DeviceInfoInit(numSefUnits, ppSefUnitsList);
    if (ret != 0)
    {
        ULOG_ERROR("failed to DeviceInfoInit()\n");
        goto FreeSefUnitsList;
    }

    // Set data alignment for transfer
    pSystemInfo = DeviceInfoGetSystemInfo();
    pSystemInfo->transferDataAlign = GetSysPageSize();

    ret = DeviceIoNotifyCreate(&threadId, NotifyAENProc);
    if (ret == 0)
    {
        pSystemInfo->asyncNotifyThreadId = threadId;
    }
    else
    {
        ULOG_ERROR("failed to create notify thread()\n");
        SeflibCleanup();
        goto ExitProc;
    }

    ret = pthread_create(&threadId, NULL, PidCloseNotifyThread, &gPCNS);
    if (ret)
    {
        ULOG_ERROR("failed to create pid close notify worker\n");
        SeflibCleanup();
        goto ExitProc;
    }
    gPidFlushWorker = threadId;

    // Return number of SEF Units
    status.error = 0;
    status.info = numSefUnits;
    goto ExitProc;

    // DestroyDeviceInfo:
    DeviceInfoCleanup();
FreeSefUnitsList:
    for (int i = 0; i < numSefUnits; i++)
    {
        free(ppSefUnitsList[i]);
    }
    free(ppSefUnitsList);
DestroyDeviceIo:
    DeviceIoCleanup();
ExitProc:
    if (status.error == 0)
    {
        seflibInitCounter++;
        ULOG_NOTICE("SEFLibraryInit() called. InitCounter=%d\n", seflibInitCounter);
    }
    pthread_mutex_unlock(&seflibMutex);
    ULOG_LEAVE_SEFAPI();

    return status;
}

/*
 *  @see    SEFAPI.h
 */
SEFHandle SEFGetHandle(uint16_t index)
{
    struct SEFHandle_ *pSef = NULL;
    int numSefUnits;
    int ret;
    uint8_t numCh;
    uint8_t numBnk;
    uint16_t maxQosd;
    uint16_t numVd;
    uint64_t maxRP;
    char *pDeviceName;
    char nvmeClassName[32];
    int fd;
    int devidx;

    ULOG_ENTER_SEFAPI();

    do
    {
        // Check parameter
        numSefUnits = DeviceInfoGetNumSefUnits();
        if ((index < 0) || (index >= numSefUnits))
        {
            ULOG_ERROR("argument error!! index=%d\n", index);
            goto ExitProc;
        }

        // Check whether already created
        pSef = DeviceInfoGetSefHandle(index);
        if (pSef != NULL)
        {
            ULOG_NOTICE("has already been created sef handle. index=%d\n", index);
            goto ExitProc;
        }

        // Retrieve device file fd because DeviceIo function is called via Request before creating handle
        pDeviceName = DeviceInfoGetDevicePath(index);
        sscanf(pDeviceName, "/dev/sef%d", &devidx);
        sprintf(nvmeClassName, "/sys/class/nvme/nvme%d", devidx);

        // Create SEFHandle
        numCh = SysReadFileDeviceInt(pDeviceName, "num_ch");       // pSefUnitInfo->NCH + 1;
        numBnk = SysReadFileDeviceInt(pDeviceName, "num_bank");    // pSefUnitInfo->NBNK + 1;
        maxQosd = SysReadFileDeviceInt(
            pDeviceName,
            "max_qosd");    // pSefUnitInfo->MAXQOSD + 1; // Add 1 to maximum ID to include ID=0,1 at creating table
        maxRP = SysReadFileDeviceInt(pDeviceName, "max_rootp");
        if (numCh < 0 || numBnk < 0 || maxQosd < 0)
        {
            ULOG_ERROR("unable to retrieve device information: %d %d %d\n", numCh, numBnk, maxQosd);
            goto ExitProc;
        }

        // Check inconsistency between SEFAPI and SEF Unit
        if (maxRP > SEFMaxRootPointer)
        {
            ULOG_ERROR("inconsistency between api and unit!! unit given max root pointer num=%u\n",
                       SEFMaxRootPointer);
            goto ExitProc;
        }

        // Set device file fd to handle
        fd = DeviceIoOpenFile(pDeviceName);
        if (fd < 0)
        {
            ULOG_ERROR("unable to open device %s: %d\n", pDeviceName, fd);
            goto ExitProc;
        }

        numVd = numCh * numBnk;
        ret = DeviceInfoCreateSefHandle(index, numVd, maxQosd, SEFINFO_ADU_NUM, &pSef);
        if (ret != 0)
        {
            ULOG_ERROR("failed to DeviceInfoCreateSefHandle()\n");
            close(fd);
            goto ExitProc;
        }

        pSef->deviceFd = fd;
        const char *unitName = strncmp(pDeviceName, "/dev/", 5) == 0 ? pDeviceName + 5 : pDeviceName;
        pSef->nvmeUnit = strdup(unitName);

        // Set SEF Unit Information in SEF handle
        int vid = SysReadFileDeviceInt(nvmeClassName, "device/vendor");
        pSef->pSefInfo->vendor[0] = vid % 0x100;
        pSef->pSefInfo->vendor[1] = (vid / 0x100) % 0x100;
        SysReadFileDeviceFixed(nvmeClassName, "serial", pSef->pSefInfo->serialNumber,
                               sizeof(pSef->pSefInfo->serialNumber));
        SysReadFileDeviceFixed(nvmeClassName, "firmware_rev", pSef->pSefInfo->FWVersion,
                               sizeof(pSef->pSefInfo->FWVersion));
        ULOG_SYSTEM("Identify - Firmware Revision (FR) %8.8s\n", pSef->pSefInfo->FWVersion);
        pSef->pSefInfo->name = pDeviceName;
        pSef->pSefInfo->unitNumber = pSef->deviceId;
        atomic_store(&pSef->numOpenVds, 0);
        pSef->numOpenAllQosds = 0;

        if (SetSefUnitInfo(pSef, true))
        {
            DeviceInfoDeleteSefHandle(pSef);
            pSef = NULL;
        }

    } while (!pSef);

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return pSef;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFLibraryCleanup(void)
{
    struct SEFStatus status = {-1, 0};
    int ret;
    ULOG_ENTER_SEFAPI();

    pthread_mutex_lock(&seflibMutex);
    // Check whether called from a wait thread
    if (DeviceIoIsIoThread())
    {
        ULOG_ERROR("calling sync functions on wait thread\n");
        status.error = -EWOULDBLOCK;
        goto ExitProc;
    }

    if (seflibInitCounter == 0)
    {
        ULOG_ERROR("SEFLibraryInit() has not been called\n");
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (seflibInitCounter == 1)
    {
        ret = SeflibCleanup();
        if (ret != 0)
        {
            status.error = ret;
            goto ExitProc;
        }
    }

    seflibInitCounter--;
    status.error = 0;
    status.info = seflibInitCounter;
    ULOG_NOTICE("SEFLibraryCleanup() called. InitCounter=%d\n", seflibInitCounter);

ExitProc:
    pthread_mutex_unlock(&seflibMutex);
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
const struct SEFInfo *SEFGetInformation(SEFHandle sefHandle)
{
    struct SEFInfo *info = NULL;
    struct SEFHandle_ *pSEFHandle_ = sefHandle;

    ULOG_ENTER_SEFAPI();

    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        goto ExitProc;
    }

    // Update SEF Unit Information retained in Library
    SetSefUnitInfo(pSEFHandle_, false);
    info = pSEFHandle_->pSefInfo;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return info;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFListVirtualDevices(SEFHandle sefHandle,
                                       struct SEFVirtualDeviceList *list,
                                       size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SEFInfo *pInfo;
    char *vd_list;
    size_t numMaxVd;
    uint16_t numVd;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (list != NULL && bufferSize < sizeof(*list) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! list=%p, bufferSize=%zu\n", list, bufferSize);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pInfo = pSef->pSefInfo;
    numVd = SEFGetInformation(pSef)->numVirtualDevices;

    // return buffer sizing info
    if (!list || !bufferSize)
    {
        status.error = 0;
        status.info = sizeof(struct SEFVirtualDeviceList) + numVd * sizeof(struct SEFVirtualDeviceID);
        goto ExitProc;
    }

    numMaxVd = (bufferSize - sizeof(struct SEFVirtualDeviceList)) / sizeof(list->virtualDeviceID[0]);
    if (numMaxVd > UINT32_MAX)
    {
        numMaxVd = UINT32_MAX;
    }

    vd_list = SysReadFileDeviceStr(pInfo->name, "vd_list");
    StrToVDList(vd_list, list, numMaxVd);
    free(vd_list);

    // if buffer didn't fit all info, return correct size for buffer
    if (numVd > numMaxVd)
    {
        status.info = sizeof(struct SEFVirtualDeviceList) + numVd * sizeof(struct SEFVirtualDeviceID);
    }
    status.error = 0;

    // Output log
    {
        ULOG_GETINFO_DATA("-------- VD LIST --------\n");
        for (uint32_t i = 0; i < numMaxVd; i++)
        {
            ULOG_GETINFO_DATA("virtualDeviceID[%u]=%u\n", i, list->virtualDeviceID[i].id);
        }
        ULOG_GETINFO_DATA("------------------------\n");
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFListQoSDomains(SEFHandle sefHandle, struct SEFQoSDomainList *list, size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SEFInfo *pInfo;
    char *qosd_list;
    size_t numMaxQd;
    uint16_t numQd;

    ULOG_ENTER_SEFAPI();
    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (list != NULL && bufferSize < sizeof(*list) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! list=%p, bufferSize=%zu\n", list, bufferSize);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pInfo = pSef->pSefInfo;
    numQd = SEFGetInformation(pSef)->numQoSDomains;

    // return buffer sizing info
    if (!list || !bufferSize)
    {
        status.error = 0;
        status.info = sizeof(struct SEFQoSDomainList) + numQd * sizeof(struct SEFQoSDomainID);
        goto ExitProc;
    }

    numMaxQd = (bufferSize - sizeof(struct SEFQoSDomainList)) / sizeof(list->QoSDomainID[0]);
    if (numMaxQd > UINT32_MAX)
    {
        numMaxQd = UINT32_MAX;
    }

    qosd_list = SysReadFileDeviceStr(pInfo->name, "qosd_list");
    StrToDomainList(qosd_list, list, numMaxQd);
    free(qosd_list);

    // if buffer didn't fit all info, return correct size for buffer
    if (numQd > numMaxQd)
    {
        status.info = sizeof(struct SEFQoSDomainList) + numQd * sizeof(struct SEFQoSDomainID);
    }
    status.error = 0;

    // Output log
    {
        ULOG_GETINFO_DATA("------ QOSD LIST -------\n");
        for (uint32_t i = 0; i < numMaxQd; i++)
        {
            ULOG_GETINFO_DATA("QoSDomainID[%u]=%u\n", i, list->QoSDomainID[i].id);
        }
        ULOG_GETINFO_DATA("------------------------\n");
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

static int CompareVdIdVdConfig(const void *pa, const void *pb)
{
    struct SEFVirtualDeviceConfig *a = *(struct SEFVirtualDeviceConfig **)pa;
    struct SEFVirtualDeviceConfig *b = *(struct SEFVirtualDeviceConfig **)pb;
    return a->virtualDeviceID.id - b->virtualDeviceID.id;
}

static int CompareMediaUnitDesc(const void *_a, const void *_b)
{
    const struct MediaUnitDescriptor *a = _a;
    const struct MediaUnitDescriptor *b = _b;

    return a->mu_id - b->mu_id;
}

static uint32_t GetDefaultReadFifo(struct SEFHandle_ *pSef, struct SEFVirtualDeviceID vdID)
{
    struct SefLogVdInfo vdInfo;
    uint32_t rfID = 0;
    int ret;

    ret = GetVDInfoLogPage(pSef, vdID.id, &vdInfo);
    if (ret)
    {
        ULOG_ERROR("Failed to get VD Info (%u)\n", ret);
        goto ExitProc;
    }
    rfID = vdInfo.drfid;
ExitProc:
    return rfID;
}

struct SEFStatus SetVDCritWarningMask(struct SEFHandle_ *pSef, uint16_t vdID, uint8_t mask)
{
    struct nvme_passthru_cmd64 cmd = {};
    struct SEFStatus status = {};
    uint32_t cmdStat;
    int ret;

    SEFAdmSetFeatureVDControlAEN(&cmd, vdID, mask);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_VD_CONTROL_AEN, &cmd, &cmdStat, NULL,
                           NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
    }
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFCreateVirtualDevices(SEFHandle sefHandle,
                                         uint16_t numVirtualDevices,
                                         struct SEFVirtualDeviceConfig *const virtualDeviceConfigs[])
{
    struct SEFHandle_ *pSef = sefHandle;
    struct CapCfgDescriptor *capCfgDesc = NULL;
    struct nvme_passthru_cmd64 cmd;
    uint32_t totalDies = 0;
    // uint32_t    lastEndGid = 0;
    uint32_t cmdStat;
    int ret;
    int totalReadQueues = 0;
    struct SEFVirtualDeviceConfig **sortedVdConfigs;
    struct SEFStatus status = {-EINVAL, 0};

    ULOG_ENTER_SEFAPI();

    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    for (int i = 0; i < numVirtualDevices; ++i)
    {
        const struct SEFVirtualDeviceConfig *vdConf = virtualDeviceConfigs[i];
        if (vdConf->numReadQueues == 0)
        {
            ULOG_ERROR("vd %u needs at least 1 read queue\n", vdConf->virtualDeviceID.id);
            status.info = 3;
            goto ExitProc;
        }
        if (vdConf->numReadQueues > SEFMaxReadQueues)
        {
            ULOG_ERROR("vd %u has more than %u read queues\n", vdConf->virtualDeviceID.id,
                       SEFMaxReadQueues);
            status.info = 3;
            goto ExitProc;
        }

        if (vdConf->superBlockDies != 0 && vdConf->dieList.numDies % vdConf->superBlockDies)
        {
            ULOG_ERROR("vd %u superBlockDies of %u doesn't divide into numDies of %u\n",
                       vdConf->virtualDeviceID.id, vdConf->superBlockDies, vdConf->dieList.numDies);
            status.info = 3;
            goto ExitProc;
        }
        totalDies += vdConf->dieList.numDies;
        totalReadQueues += vdConf->numReadQueues;
    }

    if (totalDies == 0 || numVirtualDevices == 0)
    {
        status.info = (numVirtualDevices == 0) ? 1 : 2;
        goto ExitProc;
    }
    if (totalReadQueues > pSef->pSefInfo->numReadQueues)
    {
        ULOG_ERROR("Total read queues of %u exceeds device's number of %u\n", totalReadQueues,
                   pSef->pSefInfo->numReadQueues);
        status.info = 3;
        goto ExitProc;
    }

    uint32_t descSize = sizeof(*capCfgDesc) + (totalDies * sizeof(struct MediaUnitDescriptor)) +
                        (sizeof(struct EndGrpDescriptor) * numVirtualDevices);
    descSize = TO_UPPER_DWORD_SIZE(descSize);
    capCfgDesc = (struct CapCfgDescriptor *)AllocateDeviceBuffer(descSize, true);
    if (capCfgDesc == NULL)
    {
        status.error = -ENOMEM;
        goto ExitProc;
    }

    sortedVdConfigs = calloc(sizeof(*sortedVdConfigs), numVirtualDevices);
    memcpy(sortedVdConfigs, virtualDeviceConfigs, numVirtualDevices * sizeof(*sortedVdConfigs));
    qsort(sortedVdConfigs, numVirtualDevices, sizeof(*sortedVdConfigs), CompareVdIdVdConfig);

    capCfgDesc->cci = 2;
    capCfgDesc->egcn = numVirtualDevices;

    struct EndGrpDescriptor *endGrpDesc = &capCfgDesc->egc_descs[0];
    uint16_t tchmus = 0;
    for (int i = 0; i < numVirtualDevices; ++i)
    {
        endGrpDesc->endgid = sortedVdConfigs[i]->virtualDeviceID.id;
        endGrpDesc->egchans = 1;

        struct ChnlCfgDescriptor *chnlCfgDesc = &endGrpDesc->cc_descs[0];
        chnlCfgDesc->ch_id = 0xffff;
        chnlCfgDesc->chmus = sortedVdConfigs[i]->dieList.numDies;
        tchmus += chnlCfgDesc->chmus;

        struct MediaUnitDescriptor *mediaUnitDesc = &chnlCfgDesc->mu_descs[0];
        for (int j = 0; j < chnlCfgDesc->chmus; ++j, ++mediaUnitDesc)
        {
            mediaUnitDesc->mu_id = sortedVdConfigs[i]->dieList.dieIDs[j];
        }
        qsort(&chnlCfgDesc->mu_descs[0], chnlCfgDesc->chmus, sizeof(struct MediaUnitDescriptor),
              CompareMediaUnitDesc);

        endGrpDesc = (struct EndGrpDescriptor *)mediaUnitDesc;
    }
    free(sortedVdConfigs);

    // Delete old config
    SEFAdmCapacityManagement(&cmd, 0);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_CAP_MGMT, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    // Register new config

    SEFAdmSetFeatureCapCfgReg(&cmd, capCfgDesc, numVirtualDevices, tchmus, descSize);
    DEBUG_SAVE_INFO_FILE(capCfgDesc, descSize, kDebugSaveFileCreateVd);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_CAP_CFG_REG, &cmd, &cmdStat, NULL,
                           NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    // Select new config
    SEFAdmCapacityManagement(&cmd, 2);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_CAP_MGMT, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    // Post vd creation configuration - need unique rfid's so start allocating
    // with the max rfID seen while setting the default fifo weight
    int rfID = 0;

    for (int i = 0; i < numVirtualDevices; ++i)
    {
        const struct SEFVirtualDeviceConfig *vdConf = virtualDeviceConfigs[i];
        uint16_t vdID = vdConf->virtualDeviceID.id;
        uint32_t drfID;

        // vd created with a single default read fifo, set its weight
        drfID = GetDefaultReadFifo(pSef, vdConf->virtualDeviceID);
        if (drfID)
        {
            rfID = MAX(drfID, rfID);
            SEFAdmSetFeatureVDChangeReadFifo(&cmd, vdID, drfID, vdConf->readWeights[0], false);
            ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_VD_CHANGE_RFIFO, &cmd,
                                   &cmdStat, NULL, NULL, NULL);
            if ((ret != 0) || (cmdStat != 0))
            {
                status.error = ret ?: cmdStat;
                goto ExitProc;
            }
        }
    }

    for (int i = 0; i < numVirtualDevices; ++i)
    {
        const struct SEFVirtualDeviceConfig *vdConf = virtualDeviceConfigs[i];
        uint16_t numDie = vdConf->superBlockDies ?: vdConf->dieList.numDies;
        uint16_t vdID = vdConf->virtualDeviceID.id;

        // attach the rest of the read fifos
        for (int j = 1; j < vdConf->numReadQueues; j++)
        {
            uint16_t weight = vdConf->readWeights[j];

            SEFAdmSetFeatureVDAttachReadFifo(&cmd, vdID, ++rfID, weight, false);
            ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_VD_ATTACH_RFIFO, &cmd,
                                   &cmdStat, NULL, NULL, NULL);
            if ((ret != 0) || (cmdStat != 0))
            {
                status.error = ret ?: cmdStat;
                goto ExitProc;
            }
        }

        // set number of die
        SEFAdmSetFeatureVDSetNumDies(&cmd, vdID, numDie);
        ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_VD_SET_NUM_DIES, &cmd, &cmdStat,
                               NULL, NULL, NULL);
        if ((ret != 0) || (cmdStat != 0))
        {
            status.error = ret ?: cmdStat;
            goto ExitProc;
        }

        // enable VD AEN
        status = SetVDCritWarningMask(pSef, vdID, SEF_CAE_WARN_MASK);
        if (status.error)
        {
            goto ExitProc;
        }
    }

    status.error = 0;

ExitProc:
    if (capCfgDesc)
    {
        free(capCfgDesc);
    }
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFSetNumberOfPSLCSuperBlocks(SEFVDHandle vdHandle, uint32_t numPSLCSuperBlocks)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if ((vdHandle == NULL))
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    pSef = vdHandle->pSefHandle;
    if ((pSef->pSefInfo->supportedOptions & kPSLCSupported) == 0)
    {
        status.error = -ENOTSUP;
        goto ExitProc;
    }

    SEFAdmSetFeatureVDSetNumPSLC(&cmd, vdHandle->vdId, numPSLCSuperBlocks);
    ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GLOGP_VDINFO, &cmd, &cmdStat, NULL, NULL,
                           NULL, NULL, 0);
    if (!ret && cmdStat)
    {
        ret = cmdStat;
    }

    if ((ret != 0))
    {
        status.error = ret;
        goto ExitProc;
    }
    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

struct SEFStatus SEFGetVirtualDeviceUsage(SEFVDHandle vdHandle, struct SEFVirtualDeviceUsage *usage)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef;
    struct SefLogVdInfo vdInfo;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (vdHandle == NULL)
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (usage == NULL)
    {
        ULOG_ERROR("argument error!! virtualDeviceID=%u, usage=%p\n", vdHandle->vdId, usage);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pSef = vdHandle->pSefHandle;

    ret = GetVDInfoLogPage(pSef, vdHandle->vdId, &vdInfo);
    if ((ret != 0))
    {
        status.error = ret;
        goto ExitProc;
    }
    DEBUG_SAVE_INFO_FILE(&vdInfo, sizeof(vdInfo), kDebugSaveFileGetVdDynamicInfo);

    usage->numUnallocatedSuperBlocks = vdInfo.sbsta.nfrsb;
    usage->numSuperBlocks =
        vdInfo.sbsta.nclsb + vdInfo.sbsta.neosb + vdInfo.sbsta.neosb + vdInfo.sbsta.nwosb;
    usage->numUnallocatedPSLCSuperBlocks = vdInfo.psbsta.nfrsb;
    usage->numPSLCSuperBlocks =
        vdInfo.psbsta.nclsb + vdInfo.psbsta.neosb + vdInfo.psbsta.neosb + vdInfo.psbsta.nwosb;
    usage->averagePEcount = vdInfo.avg_pe;
    usage->maxPEcount = vdInfo.max_pe;
    usage->eraseCount = vdInfo.cesn;
    usage->vdID.id = vdHandle->vdId;

    // Output log
    {
        ULOG_GETINFO_DATA("  VDID=%u\n", vdInfo.vdid);
        ULOG_GETINFO_DATA("  MAX_PEI=%u\n", vdInfo.max_pe);
        ULOG_GETINFO_DATA("  AVE_PEI=%u\n", vdInfo.avg_pe);
        ULOG_GETINFO_DATA("  NFRSB=%u\n", vdInfo.sbsta.nfrsb);
        ULOG_GETINFO_DATA("  NCLSB=%u\n", vdInfo.sbsta.nclsb);
        ULOG_GETINFO_DATA("  NOWSB=%u\n", vdInfo.sbsta.nwosb);
        ULOG_GETINFO_DATA("  NOESB=%u\n", vdInfo.sbsta.neosb);
        ULOG_GETINFO_DATA("  TCAP=0x%016lx\n", vdInfo.sbsta.tcap);
        ULOG_GETINFO_DATA("  TGCAP=0x%016lx\n", vdInfo.sbsta.tgcap);
        ULOG_GETINFO_DATA("  UCAP=0x%016lx\n", vdInfo.sbsta.ucap);
        ULOG_GETINFO_DATA("  PNFRSB=%u\n", vdInfo.psbsta.nfrsb);
        ULOG_GETINFO_DATA("  PNCLSB=%u\n", vdInfo.psbsta.nclsb);
        ULOG_GETINFO_DATA("  PNOWSB=%u\n", vdInfo.psbsta.nwosb);
        ULOG_GETINFO_DATA("  PNOESB=%u\n", vdInfo.psbsta.neosb);
        ULOG_GETINFO_DATA("  PTCAP=0x%016lx\n", vdInfo.psbsta.tcap);
        ULOG_GETINFO_DATA("  PTGCAP=0x%016lx\n", vdInfo.psbsta.tgcap);
        ULOG_GETINFO_DATA("  PUCAP=0x%016lx\n", vdInfo.psbsta.ucap);
        ULOG_GETINFO_DATA("  ASS_QOSD=%u\n", vdInfo.ass_qosd);

        ULOG_GETINFO_DATA("--VD Usage (id=%u)--\n", vdHandle->vdId);
        ULOG_GETINFO_DATA("  numUnallocatedSuperBlocks=%u\n", usage->numUnallocatedSuperBlocks);
        ULOG_GETINFO_DATA("  numSuperBlocks=%u\n", usage->numSuperBlocks);
        ULOG_GETINFO_DATA("  averagePEcount=%u\n", usage->averagePEcount);
        ULOG_GETINFO_DATA("  maxPEcount=%u\n", usage->maxPEcount);
        ULOG_GETINFO_DATA("  eraseCount=0x%08x\n", usage->eraseCount);
        ULOG_GETINFO_DATA("------------------------\n");
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

struct SEFStatus SEFGetDieList(SEFHandle sefHandle,
                               struct SEFVirtualDeviceID virtualDeviceID,
                               struct SEFDieList *list,
                               size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SefLogVdInfo vdInfo;
    size_t numMaxDie;
    uint32_t numDie;
    char devName[64];
    char *die_list;
    int vid, ret;

    ULOG_ENTER_SEFAPI();
    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if ((virtualDeviceID.id > (pSef->pSefInfo->numChannels * pSef->pSefInfo->numBanks)) ||
        (list != NULL && bufferSize < sizeof(*list) && bufferSize != 0))
    {
        ULOG_ERROR("argument error!! virtualDeviceID=%u, list=%p, bufferSize=%zu\n",
                   virtualDeviceID.id, list, bufferSize);
        status.error = -EINVAL;
        status.info =
            (virtualDeviceID.id > (pSef->pSefInfo->numChannels * pSef->pSefInfo->numBanks)) ? 2 : 3;
        goto ExitProc;
    }

    // validate virtual device id exists
    snprintf(devName, sizeof(devName), "%s/vd%u", pSef->nvmeUnit, virtualDeviceID.id);
    vid = SysReadFileDeviceInt(devName, "/vid");
    if (vid != virtualDeviceID.id)
    {
        status = (struct SEFStatus){-EINVAL, 2};
        goto ExitProc;
    }

    // get number of dies from the virtual device
    ret = GetVDInfoLogPage(pSef, virtualDeviceID.id, &vdInfo);
    if (ret != 0)
    {
        status.error = ret;
        goto ExitProc;
    }
    numDie = vdInfo.ndie + 1U;    // ndie is zero-based

    if (list)
    {
        list->numDies = numDie;
    }

    // return buffer sizing info
    if (!list || !bufferSize)
    {
        status.error = 0;
        status.info = CALC_SAVE_DIE_LIST_BUFFER_SIZE(list, numDie);
        goto ExitProc;
    }

    // calculate caller buffer capacity
    numMaxDie = CALC_SAVE_DIE_LIST_MAX(list, bufferSize);
    if (numMaxDie > UINT32_MAX)
    {
        numMaxDie = UINT32_MAX;
    }

    // get die ID list
    die_list = SysReadFileDeviceStr(devName, "die_list");
    StrToDieList(die_list, list, numMaxDie);
    free(die_list);

    // check that caller buffer fit complete die ID list
    status.info = (numMaxDie < numDie) ? CALC_SAVE_DIE_LIST_BUFFER_SIZE(list, numDie) : 0;
    status.error = 0;

    // Output log
    {
        ULOG_GETINFO_DATA("------- DIE LIST -------\n");
        for (uint32_t i = 0; i < numMaxDie; i++)
        {
            ULOG_GETINFO_DATA("DieList.DieIDs[%u]=%u\n", i, list->dieIDs[i]);
        }
        ULOG_GETINFO_DATA("------------------------\n");
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

uint32_t getSBCapacity(const char *vdSysPath, struct SEFInfo *sefInfo)
{
    uint32_t sbCap = SysReadFileDeviceInt(vdSysPath, "sb_size") * sefInfo->numPages *
                     sefInfo->numPlanes * sefInfo->pageSize / 4096;
    return sbCap;
    // return vd->sbc = vd->sbs * sef_device->id_info->npag
    // * sef_device->id_info->npl * sef_device->id_info->nadu;
}

// Using the sysfs entries for a vd, calculates how much space is available for
// creating a qos domain.  This is the vd size less the larger of used space and
// sum of other domains reserved capacities.
uint64_t CalcVdAvail(struct SEFHandle_ *pSef, const char *devName, bool pslc)
{
    const char *cap[][3] = {{"cap", "gcap", "ucap"}, {"pcap", "pgcap", "pucap"}};
    uint64_t tcap = SysReadFileDeviceInt(devName, cap[pslc][0]);
    uint64_t gcap = SysReadFileDeviceInt(devName, cap[pslc][1]);
    uint64_t ucap = SysReadFileDeviceInt(devName, cap[pslc][2]);
    uint32_t padADUs =
        getSBCapacity(devName, pSef->pSefInfo) - pSef->numADUsPerPlane * pSef->numPlanesPerDie;
    uint64_t available = tcap;

    available -= (gcap > ucap) ? gcap : ucap;
    available *= pSef->numADUsPerPlane;

    // request domain size is increased up to SBCAP - 1 plane by the device so
    // take it out of what's available from the virtual device.
    if (available > padADUs)
    {
        available -= padADUs;
    }
    else
    {
        available = 0;
    }
    return available;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetVirtualDeviceInformation(SEFHandle sefHandle,
                                                struct SEFVirtualDeviceID virtualDeviceID,
                                                struct SEFVirtualDeviceInfo *info,
                                                size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SefLogVdInfo vdInfo;
    int ret;
    char devName[32];
    char *str_list;
    int vid;
    size_t numMaxQd;
    uint32_t numQd;

    ULOG_ENTER_SEFAPI();
    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if ((virtualDeviceID.id > (pSef->pSefInfo->numChannels * pSef->pSefInfo->numBanks)) ||
        (info != NULL && bufferSize < sizeof(*info) && bufferSize != 0))
    {
        ULOG_ERROR("argument error!! virtualDeviceID=%u, info=%p, bufferSize=%zu\n",
                   virtualDeviceID.id, info, bufferSize);
        status.error = -EINVAL;
        status.info =
            (virtualDeviceID.id > (pSef->pSefInfo->numChannels * pSef->pSefInfo->numBanks)) ? 2 : 3;
        goto ExitProc;
    }

    // validate virtual device id exists
    snprintf(devName, sizeof(devName), "%s/vd%u", pSef->nvmeUnit, virtualDeviceID.id);
    vid = SysReadFileDeviceInt(devName, "vid");
    if (vid != virtualDeviceID.id)
    {
        status = (struct SEFStatus){-EINVAL, 2};
        goto ExitProc;
    }

    // get number of QoS Domains assigned to this virtual device
    ret = GetVDInfoLogPage(pSef, virtualDeviceID.id, &vdInfo);
    if (ret != 0)
    {
        status.error = ret;
        goto ExitProc;
    }
    numQd = vdInfo.ass_qosd;

    // return buffer sizing info
    if (!info || !bufferSize)
    {
        status.error = 0;
        status.info = sizeof(struct SEFVirtualDeviceInfo) + numQd * sizeof(struct SEFQoSDomainID);
        goto ExitProc;
    }

    info->QoSDomains.numQoSDomains = numQd;

    // calculate caller buffer capacity
    numMaxQd = (bufferSize - offsetof(struct SEFVirtualDeviceInfo, QoSDomains.QoSDomainID)) /
               sizeof(info->QoSDomains.QoSDomainID[0]);
    if (numMaxQd > UINT32_MAX)
    {
        numMaxQd = UINT32_MAX;
    }

    info->superBlockCapacity = getSBCapacity(devName, pSef->pSefInfo);
    info->pSLCSuperBlockCapacity = info->superBlockCapacity / 4;    // hard coded 4:1
    info->flashCapacity = SysReadFileDeviceInt(devName, "cap") * pSef->numADUsPerPlane;
    info->flashAvailable = CalcVdAvail(pSef, devName, false);
    info->pSLCFlashCapacity = SysReadFileDeviceInt(devName, "pcap") * pSef->numADUsPerPlane;
    info->pSLCFlashAvailable = CalcVdAvail(pSef, devName, true);
    info->superBlockDies = SysReadFileDeviceInt(devName, "sb_size");
    info->maxOpenSuperBlocks = SysReadFileDeviceInt(devName, "mxosb");
    info->numPSLCSuperBLocks = SysReadFileDeviceInt(devName, "npblk");
    info->aduOffsetBitWidth = SysReadFileDeviceInt(devName, "adu_bits");
    info->superBlockIdBitWidth = SysReadFileDeviceInt(devName, "sb_bits");
    // Set info->QoSDomains
    str_list = SysReadFileDeviceStr(devName, "qosd_list");
    StrToDomainList(str_list, &info->QoSDomains, numMaxQd);
    free(str_list);

    info->numReadQueues = SysReadFileDeviceInt(devName, "nrf");
    str_list = SysReadFileDeviceStr(devName, "rfws");
    memset(info->readWeights, 0xff, sizeof(info->readWeights));
    StrToRWList(str_list, info->readWeights);
    free(str_list);

    info->suspendConfig.maxTimePerSuspend = SysReadFileDeviceInt(devName, "mtps");
    info->suspendConfig.minTimeUntilSuspend = SysReadFileDeviceInt(devName, "mtus");
    info->suspendConfig.maxSuspendInterval = SysReadFileDeviceInt(devName, "msi");

    // check that caller buffer fit full QoS Domain ID list
    status.info = (numMaxQd < numQd)
                      ? sizeof(struct SEFVirtualDeviceInfo) + numQd * sizeof(struct SEFQoSDomainID)
                      : 0;
    status.error = 0;

    // Output log
    {
        size_t numMaxRq = info->numReadQueues > SEFMaxReadQueues ? SEFMaxReadQueues
                                                                 : info->numReadQueues;

        ULOG_GETINFO_DATA("-------- VD INFO -------\n");
        ULOG_GETINFO_DATA("  flashCapacity=0x%016lx\n", info->flashCapacity);
        ULOG_GETINFO_DATA("  flashAvailable=0x%016lx\n", info->flashAvailable);
        ULOG_GETINFO_DATA("  pSLCFlashCapacity=0x%016lx\n", info->pSLCFlashCapacity);
        ULOG_GETINFO_DATA("  pSLCFlashAvailable=0x%016lx\n", info->pSLCFlashAvailable);
        ULOG_GETINFO_DATA("  superBlockCapacity=0x%08x\n", info->superBlockCapacity);
        ULOG_GETINFO_DATA("  pSLCSuperBlockCapacity=0x%08x\n", info->pSLCSuperBlockCapacity);
        ULOG_GETINFO_DATA("  maxOpenSuperBlocks=0x%08x\n", info->maxOpenSuperBlocks);
        ULOG_GETINFO_DATA("  numPSLCSuperBLocks=0x%08x\n", info->numPSLCSuperBLocks);
        ULOG_GETINFO_DATA("  suspendConfig.maxTimePerSuspend=0x%08x\n",
                          info->suspendConfig.maxTimePerSuspend);
        ULOG_GETINFO_DATA("  suspendConfig.minTimeUntilSuspend=0x%08x\n",
                          info->suspendConfig.minTimeUntilSuspend);
        ULOG_GETINFO_DATA("  suspendConfig.maxSuspendInterval=0x%08x\n",
                          info->suspendConfig.maxSuspendInterval);
        ULOG_GETINFO_DATA("  superBlockDies=%u\n", info->superBlockDies);
        ULOG_GETINFO_DATA("  aduOffsetBitWidth=0x%08x\n", info->aduOffsetBitWidth);
        ULOG_GETINFO_DATA("  superBlockIdBitWidth=0x%08x\n", info->superBlockIdBitWidth);
        ULOG_GETINFO_DATA("  numReadQueues=0x%08x\n", info->numReadQueues);
        for (uint32_t i = 0; i < numMaxRq; i++)
        {
            ULOG_GETINFO_DATA("  readWeight[%u]=%u\n", i, info->readWeights[i]);
        }
        ULOG_GETINFO_DATA("  QoSDomains.numQoSDomains=%u\n", info->QoSDomains.numQoSDomains);
        for (uint32_t i = 0; i < numMaxQd; i++)
        {
            ULOG_GETINFO_DATA("  QoSDomains.QoSDomainID[%u]=%u\n", i,
                              info->QoSDomains.QoSDomainID[i].id);
        }
        ULOG_GETINFO_DATA("------------------------\n");
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFSetVirtualDeviceSuspendConfig(SEFVDHandle vdHandle,
                                                  const struct SEFVirtualDeviceSuspendConfig *config)
{
    // todo: remove the return line below once support has been added in the device
    return (struct SEFStatus){-ENOTSUP, 0};

    struct SEFStatus status = {-1, 0};
    struct nvme_passthru_cmd64 cmd;
    struct SEFHandle_ *pSef;
    uint32_t cmdStat;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (vdHandle == NULL)
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (config == NULL)
    {
        ULOG_ERROR("argument error!! config=%p\n", config);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pSef = vdHandle->pSefHandle;

    // set suspend resume config
    SEFAdmSetFeatureVDSetSuspendConfig(&cmd, vdHandle->vdId, config->maxTimePerSuspend,
                                       config->minTimeUntilSuspend, config->maxSuspendInterval);
    ret = IssueNoDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_VD_SET_SUSPEND_CONF, &cmd, &cmdStat,
                         NULL, NULL, NULL);
    if (!ret && cmdStat)
    {
        ret = cmdStat;
    }

    if ((ret != 0))
    {
        status.error = ret;
        goto ExitProc;
    }
    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/**
 *  @brief	Responds to Patrol Super Block
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
STATIC void CheckResultCreateQoSDomain(struct ReqToComplete *pReq2Cmp)
{
    uint32_t *nsid = (uint32_t *)pReq2Cmp->param;

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        goto ExitProc;
    }
    *nsid = pReq2Cmp->result & UINT32_MAX;

ExitProc:
    CompleteIocbProc(pReq2Cmp);

    SEF_ASSERT(pReq2Cmp->bSync);    // only sync calls. if async we'd need to
                                    // dec pQoS refcnt but we don't have one.
}

static uint32_t GetVdReadFifos(const char *vdName, uint32_t read_queues[SEFMaxReadQueues]);

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFCreateQoSDomain(SEFVDHandle vdHandle,
                                    struct SEFQoSDomainID *QoSDomainID,
                                    struct SEFQoSDomainCapacity *flashCapacity,
                                    struct SEFQoSDomainCapacity *pSLCFlashCapacity,
                                    int ADUindex,
                                    enum SEFAPIIdentifier api,
                                    enum SEFDefectManagementMethod defectStrategy,
                                    enum SEFErrorRecoveryMode recovery,
                                    const char *encryptionKey,
                                    uint16_t numPlacementIDs,
                                    uint16_t maxOpenSuperBlocks,
                                    uint8_t defaultReadQueue,
                                    struct SEFWeights weights)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef;
    struct SEFVDHandle_ *pVd = vdHandle;
    uint64_t maxVdCapacity;
    struct nvme_passthru_cmd64 cmd;
    uint32_t size;
    uint32_t cmdStat = 0;
    uint32_t numFifos;
    uint32_t readFifos[SEFMaxReadQueues];
    uint32_t qosds;
    char vdName[64];
    struct NamespaceManagement *pNsCreate = NULL;
    struct SEFQoSDomainCapacity defaultCap = {0};
    int ret = 0;
    ULOG_ENTER_SEFAPI();

    if (!flashCapacity)
    {
        flashCapacity = &defaultCap;
    }

    if (!pSLCFlashCapacity)
    {
        pSLCFlashCapacity = &defaultCap;
    }

    // Check parameter
    if (vdHandle == NULL)
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    pSef = pVd->pSefHandle;
    maxVdCapacity = pVd->pDieList->numDies;
    maxVdCapacity *= pSef->pSefInfo->numPlanes;
    maxVdCapacity *= pSef->pSefInfo->numBlocks;
    maxVdCapacity *= pSef->pSefInfo->numPages;
    maxVdCapacity *= pSef->numADUsPerPlane;

    snprintf(vdName, sizeof(vdName), "%s/vd%d", pSef->nvmeUnit, pVd->vdId);
    numFifos = GetVdReadFifos(vdName, readFifos);

    /*
     * Check parameters
     */
    if ((flashCapacity->flashCapacity > maxVdCapacity) || (numPlacementIDs == 0) ||
        (numPlacementIDs > pSef->pSefInfo->maxPlacementIDs) || defaultReadQueue > numFifos)
    {
        ULOG_DEBUG("maxVdCapacity=0x%lx\n", maxVdCapacity);
        ULOG_ERROR(
            "argument error!! flashCapacity=0x%lx, ADUindex=%u, api=%d, recovery=%d, "
            "encryption=%d, "
            "numPlacementIDs=%u numReadQueues=%u\n",
            flashCapacity->flashCapacity, ADUindex, api, recovery, encryptionKey ? 1 : 0,
            numPlacementIDs, numFifos);
        status.error = -EINVAL;
        status.info = flashCapacity->flashCapacity > maxVdCapacity ? 3 : 10;
        goto ExitProc;
    }

    // Check parameter
    if (flashCapacity->flashCapacity && flashCapacity->flashCapacity < pSef->numADUsPerPlane)
    {
        ULOG_ERROR("argument error!! flashCapacity=0x%lx, numADUsPerPlane=%x\n",
                   flashCapacity->flashCapacity, pSef->numADUsPerPlane);
        status.error = -EINVAL;
        status.info = 3;
        goto ExitProc;
    }
    if (pSLCFlashCapacity->flashCapacity && pSLCFlashCapacity->flashCapacity < pSef->numADUsPerPlane)
    {
        ULOG_ERROR("argument error!! pSLCFlashCapacity=0x%lx, numADUsPerPlane=%x\n",
                   pSLCFlashCapacity->flashCapacity, pSef->numADUsPerPlane);
        status.error = -EINVAL;
        status.info = 4;
        goto ExitProc;
    }
    // Set command information
    size = sizeof(struct NamespaceManagement);
    pNsCreate = (struct NamespaceManagement *)AllocateDeviceBuffer(size, true);
    if (pNsCreate == NULL)
    {
        status.error = -ENOMEM;
        goto ExitProc;
    }

    pNsCreate->sbsta.gcap = htole64(flashCapacity->flashCapacity / pSef->numADUsPerPlane);
    pNsCreate->sbsta.quota = htole64(flashCapacity->flashQuota / pSef->numADUsPerPlane);
    pNsCreate->psbsta.gcap = htole64(pSLCFlashCapacity->flashCapacity / pSef->numADUsPerPlane);
    pNsCreate->psbsta.quota = htole64(pSLCFlashCapacity->flashQuota / pSef->numADUsPerPlane);
    pNsCreate->endgid = htole16(pVd->vdId);
    pNsCreate->nplid = htole16(numPlacementIDs - 1);
    pNsCreate->mxosb = htole16(maxOpenSuperBlocks);
    pNsCreate->ewt = htole16(weights.eraseWeight);
    pNsCreate->wwt = htole16(weights.programWeight);
    pNsCreate->drfid = htole32(readFifos[defaultReadQueue]);
    qosds = ConvReadDeadlineForDev(kTypical);
    qosds |= ConvDefectStrategyForDev(defectStrategy) << 2;
    pNsCreate->qosds = htole32(qosds);

    // Necessary for linux to create ngXnY char device
    if (!pNsCreate->sbsta.gcap && !pNsCreate->psbsta.gcap)
    {
        pNsCreate->sbsta.gcap = htole64(1);
    }

    // Create QoSD
    size = sizeof(*pNsCreate);
    uint32_t nsid = 0;
    SEFAdmNamespaceManagement(&cmd, pNsCreate, size, 0, true);
    DEBUG_SAVE_INFO_FILE(pNsCreate, size, kDebugSaveFileCreateQosd);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_QOSD_CREATE, &cmd, &cmdStat, NULL, &nsid,
                           CheckResultCreateQoSDomain);
    free(pNsCreate);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    uint16_t ctrlId[4096 / sizeof(uint16_t)] = {1, 0};
    SEFAdmNameSpaceAttachment(&cmd, &ctrlId, sizeof(ctrlId), nsid, true);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_NS_ATTACH, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        ULOG_ERROR("Failed to attach QoSD %u\n", nsid);
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }
    ULOG_DEBUG("QoSD created as NSID %u\n", nsid);
    QoSDomainID->id = nsid;
    pSef->pSefInfo->numQoSDomains++;
    status.error = 0;
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFSetQoSDomainCapacity(SEFVDHandle vdHandle,
                                         struct SEFQoSDomainID QoSDomainID,
                                         enum SEFSuperBlockType type,
                                         struct SEFQoSDomainCapacity *capacity)
{
    struct SEFStatus status = {-1, 0};
    struct SEFVDHandle_ *pVd = vdHandle;
    struct SEFHandle_ *pSef;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    uint64_t maxVdCapacity;
    uint64_t flashCapacity;
    uint64_t flashQuota;
    struct SEFQoSDomainCapacity defaultCap = {0};
    int ret;

    ULOG_ENTER_SEFAPI();

    if (!capacity)
    {
        capacity = &defaultCap;
    }

    flashCapacity = capacity->flashCapacity;
    flashQuota = capacity->flashQuota;

    // Check parameter
    if (vdHandle == NULL)
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    // Check parameter
    pSef = pVd->pSefHandle;
    if (flashCapacity < pSef->numADUsPerPlane)
    {
        ULOG_ERROR("argument error!! flashCapacity=0x%lx, numADUsPerPlane=0x%x\n", flashCapacity,
                   pSef->numADUsPerPlane);
        status.error = -EINVAL;
        status.info = 4;
        goto ExitProc;
    }
    else if (flashQuota < flashCapacity)
    {
        flashQuota = flashCapacity;
    }

    // Check specified capacity
    maxVdCapacity = pVd->pDieList->numDies;
    maxVdCapacity *= pSef->pSefInfo->numPlanes;
    maxVdCapacity *= pSef->pSefInfo->numBlocks;
    maxVdCapacity *= pSef->pSefInfo->numPages;
    maxVdCapacity *= pSef->numADUsPerPlane;
    if (flashCapacity > maxVdCapacity)
    {
        ULOG_ERROR("argument error!! flashCapacity=0x%lx, maxVdCapacity=0x%lx\n", flashCapacity,
                   maxVdCapacity);
        status.error = -EINVAL;
        status.info = 4;
        goto ExitProc;
    }

    // Change QoSD capacity
    SEFAdmSetFeatureQoSDChangeCapacity(&cmd, QoSDomainID.id, type == kForPSLCWrite,
                                       (flashCapacity / pSef->numADUsPerPlane - 1),
                                       (flashQuota / pSef->numADUsPerPlane - 1));
    ret = IssueNoDataCmd(pSef->deviceFd, true, CMD_A_SFEAT_QOSD_CHANGECAP, &cmd, &cmdStat, NULL,
                         NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }
    status.error = 0;
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFSetRootPointer(SEFQoSHandle qosHandle, int index, struct SEFFlashAddress value)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos = qosHandle;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if ((index < 0) || (index >= pQos->pSefHandle->pSefInfo->maxRootPointers))
    {
        ULOG_ERROR("argument error!! index=%d\n", index);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Set root pointer
    SEFAdmSetFeatureQoSDSetRootPointer(&cmd, pQos->qosId, index, value.bits);
    ret = IssueNoDataCmd(pQos->deviceFd, true, CMD_A_SFEAT_QOSD_SETROOTPTR, &cmd, &cmdStat, NULL,
                         NULL, NULL);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFSetReadDeadline(SEFQoSHandle qosHandle, enum SEFDeadlineType deadline)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos = qosHandle;
    enum ReadDeadlineForDev deadlineForDev = ConvReadDeadlineForDev(deadline);
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Set Read Deadline
    SEFAdmSetFeatureQoSDChangeReadDeadline(&cmd, pQos->qosId, deadlineForDev);
    ret = IssueNoDataCmd(pQos->deviceFd, true, CMD_A_SFEAT_QOSD_CHANGERDDEAD, &cmd, &cmdStat, NULL,
                         NULL, NULL);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFSetWeights(SEFQoSHandle qosHandle, struct SEFWeights weights)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos = qosHandle;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Set Read Deadline
    SEFAdmSetFeatureQoSDChangeWeights(&cmd, pQos->qosId, weights.programWeight, weights.eraseWeight);
    ret = IssueNoDataCmd(pQos->deviceFd, true, CMD_A_SFEAT_QOSD_CHANGERDDEAD, &cmd, &cmdStat, NULL,
                         NULL, NULL);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

int GetSbListSegment(SEFQoSHandle qosHandle,
                     uint32_t sbRecordMax,
                     struct SEFSuperBlockList *list,
                     bool *entireList)
{
    struct GetLogPageSuperBlockList *pSbLogPage = NULL;
    struct SuperBlockDescriptor *sbd;
    struct SEFSuperBlockRecord *sbr;
    struct nvme_passthru_cmd64 cmd;
    uint32_t sbRecCount;
    uint32_t reqSize;
    uint32_t cmdStat;
    uint32_t maxRec;
    uint32_t offset;
    uint32_t size;
    uint32_t rec;
    int ret;

    sbRecCount = list->numSuperBlocks;
    offset = sbRecCount * sizeof(pSbLogPage->SuperBlockDescriptor[0]);
    /* command to get SB log page always sets SBID of last entry of list to
       0xFFFFFFFF, so request one more */
    size = (sbRecordMax + 1) * sizeof(pSbLogPage->SuperBlockDescriptor[0]);
    size = TO_UPPER_DWORD_SIZE(size);
    reqSize = MIN(qosHandle->maxBufSize, size - offset);
    maxRec = reqSize / sizeof(pSbLogPage->SuperBlockDescriptor[0]);
    SEFAdmGetLogPageSBList(&cmd, NULL, reqSize, qosHandle->qosId, SEF_LODR_OPEN_CLOSED, offset);
    ret = IssueRecvDataCmd(qosHandle->deviceFd, true, CMD_A_GLOGP_SBLIST_OPENCLOSED, &cmd, &cmdStat,
                           NULL, NULL, NULL, (void **)&pSbLogPage, reqSize);

    if ((ret != 0) || (cmdStat != 0))
    {
        ret = ret ?: cmdStat;
        goto ExitProc;
    }
    // save valid list contents to caller buffer and check if complete list
    for (rec = 0; rec < maxRec && sbRecCount < sbRecordMax; rec++, sbRecCount++)
    {
        sbd = &pSbLogPage->SuperBlockDescriptor[rec];
        sbr = &list->superBlockRecords[sbRecCount];
        ULOG_DEBUG("pSbLogPage->SuperBlockDescriptor[%d].SBID=%u\n", sbRecCount, sbd->SBID);
        if (sbd->SBID == SBLIST_LOG_CMD_LIST_END)
        {
            break;
        }
        sbr->flashAddress = GetFlashAddress(qosHandle, qosHandle->qosId, sbd->SBID);
        sbr->PEIndex = sbd->PECI;
        sbr->state = DiiSbs2SbState(sbd->DIISBS);
        memset(sbr->reserved, 0, sizeof(sbr->reserved));
    }
    *entireList =
        (rec < maxRec && pSbLogPage->SuperBlockDescriptor[rec].SBID == SBLIST_LOG_CMD_LIST_END);
    list->numSuperBlocks = sbRecCount;
ExitProc:
    free(pSbLogPage);
    return ret;
}

struct SEFStatus SEFGetSuperBlockList(SEFQoSHandle qosHandle, struct SEFSuperBlockList *list, size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    bool entireList = false;
    size_t saveSbRecordMax;    // Number of superBlockRecords that can be set to info
    uint32_t numSb;
    int ret;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (list != NULL && bufferSize < sizeof(*list) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! QoSDomainID=%u, list=%p, bufferSize=%zu\n", qosHandle->qosId,
                   list, bufferSize);
        status.error = -EINVAL;
        goto ExitProc;
    }

    if (list)
    {
        list->numSuperBlocks = 0;
        list->reserved = 0;
    }
    else
    {
        bufferSize = 0;
    }

    QOSD_REQUEST_COUNT_INC(qosHandle);

    // calculate caller buffer capacity
    saveSbRecordMax = CALC_SAVE_SB_RECORD_MAX(list, bufferSize);
    if (saveSbRecordMax > UINT32_MAX)
    {
        saveSbRecordMax = UINT32_MAX;
    }

    if (list)
    {
        do
        {
            ret = GetSbListSegment(qosHandle, saveSbRecordMax, list, &entireList);
            if (ret)
            {
                status.error = ret;
                goto DecRef;
            }
        } while (list->numSuperBlocks < saveSbRecordMax && !entireList);

        ULOG_DEBUG("sbRecordCount=%d\n", list->numSuperBlocks);

        // Output log
        ULOG_GETINFO_DATA("--QoSD SB List (id=%u)--\n", qosHandle->qosId);
        ULOG_GETINFO_DATA("  numSuperBlocks=%u\n", list->numSuperBlocks);
        for (int i = 0; i < list->numSuperBlocks; i++)
        {
            ULOG_GETINFO_DATA("  superBlockRecords[%04d].flashAddress=0x%016lx\n", i,
                              list->superBlockRecords[i].flashAddress.bits);
            ULOG_GETINFO_DATA("  superBlockRecords[%04d].PEIndex=0x%x\n", i,
                              list->superBlockRecords[i].PEIndex);
            ULOG_GETINFO_DATA("  superBlockRecords[%04d].state=0x%x\n", i,
                              list->superBlockRecords[i].state);
        }
        ULOG_GETINFO_DATA("------------------------\n");
    }

    // if caller buffer too small for entire list, get size from domain
    if (!entireList)
    {
        status = GetDomainNumSb(qosHandle, &numSb);
        if (status.error)
        {
            goto DecRef;
        }
        // numSb can be smaller than saveSbRecordMax when enough SBs are released
        // between reading the sb log page and ns ident page.  No matter what
        // we do, it'll be confusing
        status.info = CALC_SAVE_SB_BUFFER_SIZE(list, numSb);
        if (list)
        {
            list->numSuperBlocks = numSb;
        }
        ULOG_DEBUG("numSb=%d\n", numSb);
    }

    status.error = 0;

DecRef:
    QOSD_REQUEST_COUNT_DEC(qosHandle);
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

static int ReadNvmeNSID(int unit, int ns)
{
    char path[256];
    FILE *fh;
    int nsid = 0;

    sprintf(path, "/sys/class/sef/sef%d/sef%dn%d/id", unit, unit, ns);
    fh = fopen(path, "r");
    if (fh)
    {
        if (fscanf(fh, "%d", &nsid) != 1)
        {
            nsid = 0;
        }
        fclose(fh);
    }
    return nsid;
}

static int GetNvmeNS(const char *fname, int *unit, int *ns)
{
    return (sscanf(fname, "sef%dn%d", unit, ns) == 2);
}

static int ScanNvmeNS(const struct dirent *pDirent)
{
    int unit, ns;

    if (GetNvmeNS(pDirent->d_name, &unit, &ns))
    {
        return ReadNvmeNSID(unit, ns);
    }
    return 0;
}

/*
 * @brief Returns if nsid is valid
 */
static char *GetBlkNameFromNSID(const char *devName, int nsid)
{
    struct dirent **ppCtrls = NULL;
    int num_nsid;
    int i;
    int found = 0;
    char *ret = NULL;
    char unitPath[256];

    if (strncmp("/dev/", devName, 5) == 0)
    {
        devName += 5;
    }

    sprintf(unitPath, "/sys/class/sef/%s", devName);
    num_nsid = scandir(unitPath, &ppCtrls, ScanNvmeNS, alphasort);
    if (num_nsid < 0)
    {
        return ret;
    }
    for (i = 0; !found && i < num_nsid; i++)
    {
        int tunit, tns;

        if (GetNvmeNS(ppCtrls[i]->d_name, &tunit, &tns))
        {
            found = (ReadNvmeNSID(tunit, tns) == nsid);
        }
        if (found)
        {
            ret = strdup(ppCtrls[i]->d_name);
            break;
        }
    }

    for (i = 0; i < num_nsid; ++i)
    {
        free(ppCtrls[i]);
    }
    free(ppCtrls);
    return ret;
}

static uint32_t GetVdReadFifos(const char *vdName, uint32_t read_queues[SEFMaxReadQueues])
{
    uint32_t nrf = SysReadFileDeviceInt(vdName, "nrf");
    char *rf_list = SysReadFileDeviceStr(vdName, "rfids");
    StrToRFList(rf_list, read_queues);
    free(rf_list);
    return nrf;
}

static uint32_t GetVdDMSize(SEFHandle sefHandle, const char *vdName)
{
    uint32_t dmSizeBits;

    dmSizeBits = sefHandle->numPlanesPerDie * SysReadFileDeviceInt(vdName, "sb_size");
    return (7 + dmSizeBits) / 8;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetQoSDomainInformation(SEFHandle sefHandle,
                                            struct SEFQoSDomainID QoSDomainID,
                                            struct SEFQoSDomainInfo *info)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    char devName[128];
    char nvmeDevName[128];
    char vdName[148];
    char *qdName;
    int devidx, nsidx;

    ULOG_ENTER_SEFAPI();
    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (IS_INVALID_QOSD_ID(QoSDomainID.id, pSef->pSefInfo->maxQoSDomains) || (info == NULL))
    {
        ULOG_ERROR("argument error!! QoSDomainID=%u, info=%p\n", QoSDomainID.id, info);
        status.error = -EINVAL;
        status.info = info == NULL ? 3 : 2;
        goto ExitProc;
    }

    qdName = GetBlkNameFromNSID(pSef->nvmeUnit, QoSDomainID.id);
    if (!qdName)
    {
        status = (struct SEFStatus){-EINVAL, 2};
        goto ExitProc;
    }

    sscanf(qdName, "sef%dn%d", &devidx, &nsidx);
    snprintf(nvmeDevName, sizeof(nvmeDevName), "/sys/class/nvme/nvme%d/nvme%dn%d", devidx, devidx,
             nsidx);
    snprintf(devName, sizeof(devName), "%s/%s", pSef->nvmeUnit, qdName);
    free(qdName);

    // Set output parameters
    uint16_t vdid = SysReadFileDeviceInt(devName, "vdid");    // pQosdInfo->VDID;
    strcpy(vdName, devName);
    char *vddir = strrchr(vdName, '/');

    vddir++;
    snprintf(vddir, sizeof(vdName), "vd%d", vdid);
    info->virtualDeviceID.id = vdid;
    info->numPlacementIDs = SysReadFileDeviceInt(devName, "nplid");    // pQosdInfo->NPLID + 1;
    info->encryption = 0;                                              // pQosdInfo->ENC;
    info->recoveryMode = kAutomatic;
    uint32_t features = SysReadFileDeviceInt(devName, "features");
    info->defectStrategy = ConvDefectStrategyForApi((enum DefectStrategyForDev)((features >> 2) & 0x3));
    info->api = kSuperBlock;
    info->flashCapacity = SysReadFileDeviceInt(devName, "gcap") * pSef->numADUsPerPlane;
    info->flashQuota = SysReadFileDeviceInt(devName, "quota") * pSef->numADUsPerPlane;
    info->flashUsage = SysReadFileDeviceInt(devName, "ucap") * pSef->numADUsPerPlane;
    info->pSLCFlashCapacity = SysReadFileDeviceInt(devName, "pgcap") * pSef->numADUsPerPlane;
    info->pSLCFlashQuota = SysReadFileDeviceInt(devName, "pquota") * pSef->numADUsPerPlane;
    info->pSLCFlashUsage = SysReadFileDeviceInt(devName, "pucap") * pSef->numADUsPerPlane;
    uint32_t aduSize = GET_ADU_SIZE(pSef);
    info->ADUsize.data = aduSize;
    info->ADUsize.meta = SysReadFileDeviceInt(devName, "msize");
    info->superBlockCapacity = getSBCapacity(vdName, pSef->pSefInfo) * (aduSize / 4096);
    info->pSLCSuperBlockCapacity = info->superBlockCapacity / 4;    // hard coded 4:1
    info->maxOpenSuperBlocks = SysReadFileDeviceInt(devName, "mxosb");
    info->defectMapSize = info->defectStrategy == kPerfect ? 0 : GetVdDMSize(sefHandle, vdName);
    info->weights.eraseWeight = SysReadFileDeviceInt(devName, "ewt");
    info->weights.programWeight = SysReadFileDeviceInt(devName, "wwt");
    info->deadline = ConvReadDeadlineForApi((enum ReadDeadlineForDev)(features & 0x3));    // pQosdInfo->RDL);
    info->defaultReadQueue = 0;

    uint32_t drfid = SysReadFileDeviceInt(devName, "drfid");
    uint32_t read_fifos[SEFMaxReadQueues];

    info->numReadQueues = GetVdReadFifos(vdName, read_fifos);
    for (int i = 0; i < info->numReadQueues; i++)
    {
        if (read_fifos[i] == drfid)
        {
            info->defaultReadQueue = i;
            break;
        }
    }

    //////////////////////////////
    char *rp_list = SysReadFileDeviceStr(devName, "rootp");
    StrToRootList(rp_list, info->rootPointers);
    free(rp_list);
    memset(info->reserved, 0, sizeof(info->reserved));
    status.error = 0;

    // Output log
    {
        ULOG_GETINFO_DATA("--QoSD Information (id=%u)--\n", QoSDomainID.id);
        ULOG_GETINFO_DATA("  ADUsize=%u\n", info->ADUsize.data);
        ULOG_GETINFO_DATA("  virtualDeviceID=%u\n", info->virtualDeviceID.id);
        ULOG_GETINFO_DATA("  numPlacementIDs=%u\n", info->numPlacementIDs);
        ULOG_GETINFO_DATA("  encryption=%u\n", info->encryption);
        ULOG_GETINFO_DATA("  api=%d\n", info->api);
        ULOG_GETINFO_DATA("  capacity=0x%016lx\n", info->flashCapacity);
        ULOG_GETINFO_DATA("  quota=0x%016lx\n", info->flashQuota);
        ULOG_GETINFO_DATA("  recoveryMode=%d\n", info->recoveryMode);
        ULOG_GETINFO_DATA("  deadline=%d\n", info->deadline);
        ULOG_GETINFO_DATA("  defaultReadQueue=%u\n", info->defaultReadQueue);
        ULOG_GETINFO_DATA("  numReadQueue=%u\n", info->numReadQueues);
        ULOG_GETINFO_DATA("  programWeight=%u\n", info->weights.programWeight);
        ULOG_GETINFO_DATA("  eraseWeight=%u\n", info->weights.eraseWeight);
        for (int i = 0; i < SEFMaxRootPointer; i++)
        {
            ULOG_GETINFO_DATA("  rootPointers[%d]=%lu\n", i, info->rootPointers[i].bits);
        }
        ULOG_GETINFO_DATA("------------------------\n");
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetReuseList(SEFQoSHandle qosHandle, struct SEFWearInfo *info, size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos;
    uint32_t numSb;
    size_t saveSbRecordMax;
    int ret;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (info != NULL && bufferSize < sizeof(*info) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! info=%p, bufferSize=0x%lx\n", info, bufferSize);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pQos = qosHandle;
    saveSbRecordMax = info ? CALC_SAVE_SB_RECORD_MAX(info, bufferSize) : 0U;
    if (saveSbRecordMax > UINT32_MAX)
    {
        saveSbRecordMax = UINT32_MAX;
    }

    // if buffer is null or holds no entries, return info on buffer sizing
    if (!saveSbRecordMax || !bufferSize)
    {
        status = GetDomainNumSb(pQos, &numSb);
        if (status.error)
        {
            goto ExitProc;
        }
        if (info)
        {
            info->numSuperBlocks = numSb;
        }
        status.info = CALC_SAVE_SB_BUFFER_SIZE(info, numSb);
        status.error = 0;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Retrieve SB list
    ret = GetSuperBlockList(pQos, SEF_LODR_CLOSE_WEARLEVEL, &info->numSuperBlocks,
                            &info->superBlockRecords[0], saveSbRecordMax);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if (ret != 0)
    {
        status.error = ret;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetRefreshList(SEFQoSHandle qosHandle, struct SEFRefreshInfo *info, size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos;
    uint32_t numSb;
    size_t saveSbRecordMax;
    int ret;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (info != NULL && bufferSize < sizeof(*info) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! info=%p, bufferSize=%zu\n", info, bufferSize);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pQos = qosHandle;
    saveSbRecordMax = info ? CALC_SAVE_SB_RECORD_MAX(info, bufferSize) : 0U;
    if (saveSbRecordMax > UINT32_MAX)
    {
        saveSbRecordMax = UINT32_MAX;
    }

    // if buffer is null or holds no entries, return info on buffer sizing
    if (!saveSbRecordMax || !bufferSize)
    {
        status = GetDomainNumSb(pQos, &numSb);
        if (status.error)
        {
            goto ExitProc;
        }
        if (info)
        {
            info->numSuperBlocks = numSb;
        }
        status.info = CALC_SAVE_SB_BUFFER_SIZE(info, numSb);
        status.error = 0;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Retrieve list of Most Urgent to Refresh SBs
    ret = GetSuperBlockList(pQos, SEF_LODR_CLOSE_REFRESH, &info->numSuperBlocks,
                            &info->superBlockRecords[0], saveSbRecordMax);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if (ret != 0)
    {
        status.error = ret;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetCheckList(SEFQoSHandle qosHandle, struct SEFCheckInfo *info, size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos;
    uint32_t numSb;
    size_t saveSbRecordMax;
    int ret;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (info != NULL && bufferSize < sizeof(*info) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! info=%p, bufferSize=%zu\n", info, bufferSize);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    pQos = qosHandle;
    saveSbRecordMax = info ? CALC_SAVE_SB_RECORD_MAX(info, bufferSize) : 0U;
    if (saveSbRecordMax > UINT32_MAX)
    {
        saveSbRecordMax = UINT32_MAX;
    }

    // if buffer is null or holds no entries, return info on buffer sizing
    if (!saveSbRecordMax || !bufferSize)
    {
        status = GetDomainNumSb(pQos, &numSb);
        if (status.error)
        {
            goto ExitProc;
        }
        if (info)
        {
            info->numSuperBlocks = numSb;
        }
        status.info = CALC_SAVE_SB_BUFFER_SIZE(info, numSb);
        status.error = 0;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Retrieve list of Most Urgent to Check SBs
    ret = GetSuperBlockList(pQos, SEF_LODR_CLOSE_CHECK, &info->numSuperBlocks,
                            &info->superBlockRecords[0], saveSbRecordMax);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if (ret != 0)
    {
        status.error = ret;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetUserAddressList(SEFQoSHandle qosHandle,
                                       struct SEFFlashAddress flashAddress,
                                       struct SEFUserAddressList *list,
                                       size_t bufferSize)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef;
    struct SEFQoSHandle_ *pQos;
    uint32_t sbId;
    uint32_t sbAduCap;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    struct GetLogPageUserAddressList *pUserAddress;
    int ret;
    int offset = 0;
    bool incompleteList = false;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (list != NULL && bufferSize < sizeof(*list) && bufferSize != 0)
    {
        ULOG_ERROR("argument error!! list=%p, bufferSize=%zu\n", list, bufferSize);
        status.error = -EINVAL;
        status.info = 3;
        goto ExitProc;
    }

    pQos = qosHandle;
    pSef = pQos->pSefHandle;
    sbId = GetSbId(pQos, flashAddress);
    sbAduCap = pQos->sbCapacity;

    if (list)
    {
        list->numADUs = 0U;
    }

    // calculate buffer capacity
    size_t saveUserAddressNum = list ? CALC_SAVE_UA_RECOVERY_MAX(list, bufferSize) : 0U;
    if (saveUserAddressNum > UINT32_MAX)
    {
        saveUserAddressNum = UINT32_MAX;
    }

    // if buffer holds no entries or no buffer provided, return buffer sizing info
    if (!saveUserAddressNum || !bufferSize)
    {
        status.info = CALC_SAVE_UA_BUFFER_SIZE(list, sbAduCap);
        status.error = 0;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Retrieve UserAddressList
    ULOG_DEBUG("sbId=%u, qosId=%u, sbAduCap=0x%x, sbSize=0x%xnumPlanesPerDie=0x%x\n", sbId,
               pQos->qosId, sbAduCap, pQos->sbSize, pSef->numPlanesPerDie);

    // do retrieval in segments if transfer buffer size would be exceeded
    uint32_t totalSize = sizeof(pUserAddress->userAddresses[0]) * sbAduCap;
    while (offset < totalSize)
    {
        uint32_t size = totalSize > pQos->maxBufSize ? pQos->maxBufSize : totalSize;
        uint32_t numADUs = size / sizeof(list->userAddressesRecovery[0]);
        size = numADUs * sizeof(list->userAddressesRecovery[0]);

        SEFAdmGetLogPageUserAddrList(&cmd, NULL, size, pQos->qosId, sbId, offset);
        ret = IssueRecvDataCmd(pQos->deviceFd, true, CMD_A_GLOGP_USERADDRLIST, &cmd, &cmdStat, NULL,
                               NULL, NULL, (void **)&pUserAddress, size);
        if (ret || cmdStat)
        {
            break;
        }

        for (int i = 0; i < numADUs; i++)
        {
            // Retrieve UserAddress information
            list->userAddressesRecovery[list->numADUs] = pUserAddress->userAddresses[i];
            list->numADUs++;
            if (saveUserAddressNum <= list->numADUs)
            {
                incompleteList = true;
                break;
            }
        }

        offset += size;

        free(pUserAddress);
    }

    QOSD_REQUEST_COUNT_DEC(pQos);

    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    ULOG_INFORMATION("saveUserAddressNum=%ld, list->numADUs=%d\n", saveUserAddressNum, list->numADUs);
    DEBUG_SAVE_INFO_FILE(pUserAddress, size, kDebugSaveFileGetLogUserAddrList);

    // check if caller buffer fit complete user address list
    status.info = incompleteList ? CALC_SAVE_UA_BUFFER_SIZE(list, sbAduCap) : 0;
    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFGetSuperBlockInfo(SEFQoSHandle qosHandle,
                                      struct SEFFlashAddress flashAddress,
                                      int getDefectMap,
                                      struct SEFSuperBlockInfo *info)
{
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos;
    uint32_t sbId, sbQosId;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (info == NULL)
    {
        ULOG_ERROR("argument error!! info=%p\n", info);
        status.error = -EINVAL;
        status.info = 3;
        goto ExitProc;
    }

    pQos = qosHandle;
    sbId = GetSbId(pQos, flashAddress);
    sbQosId = GetQosdId(flashAddress);

    if (pQos->qosId != sbQosId)
    {
        ULOG_ERROR("argument error!! qos domain Id=%d, flashAddress Qos Domain Id=%d\n",
                   pQos->qosId, sbQosId);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    QOSD_REQUEST_COUNT_INC(pQos);

    // Retrieve SB information
    ret = GetSuperBlockInfo(pQos, sbId, getDefectMap, info);

    QOSD_REQUEST_COUNT_DEC(pQos);

    if (ret != 0)
    {
        status.error = ret;
        status.info = ret == -EINVAL ? 2 : 0;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/**
 *  @brief	Structure of parameters retained for responding to Patrol Super Block
 *  @note    Set to param of Req2Cmp at creating request
 */
struct PatrolSbParam
{
    struct SEFQoSHandle_ *pQos;    //!< QoSD handle
    uint32_t sbId;                 //!< Super Block ID specified at request
};

/**
 *  @brief	Responds to Patrol Super Block
 *  @param	[in] pReq2Cmp: request information and completion information
 *  @return	None
 *  @note    User completion callback is executed here.
 *  @note    Memory for related parameters in Req2Cmp is released here.
 */
STATIC void CheckResultPatrolSb(struct ReqToComplete *pReq2Cmp)
{
    struct PatrolSbParam *pParam = (struct PatrolSbParam *)pReq2Cmp->param;
    struct PatrolSuperBlockIOCB *pPatrolSbIocb = &pReq2Cmp->pIocb->patrolSb;
    uint8_t dataIntegrityIdx = pReq2Cmp->result & UINT8_MAX;

    // In case of command error, only postprocessing is done
    if (pReq2Cmp->status != 0)
    {
        goto ExitProc;
    }

    ULOG_SBCMD_DBG("sbid=0x%04x, dataIntegrityIdx=%u\n", pParam->sbId, dataIntegrityIdx);

    pPatrolSbIocb->common.status.info = dataIntegrityIdx;

ExitProc:
    CompleteIocbProc(pReq2Cmp);

    QOSD_REQUEST_COUNT_DEC(pParam->pQos);

    free(pParam);
}

/**
 *  @brief	Internal processing for SEFCheckPage
 *  @param	[in] bSync: Synchronous flag (async if false)
 *  @param	[in] pQos: QoSD handle
 *  @param	[in] iocb: IOCB for Patroll Super Block
 *  @details	Area of arbitrary parameter for responding to Patrol Super Block is allocated.
 *  @details	This function releases arbitrary parameter area if command issuing failed.
 *  @note    CheckResultExt must release arbitrary parameter area.
 *  @return	struct SEFStatus.error: 0: ended normally, otherwise: error; struct SEFStatus.info: fixed to 0
 */
STATIC struct SEFStatus PatrolSbProc(bool bSync, struct SEFQoSHandle_ *pQos, struct SEFCommonIOCB *common)
{
    struct PatrolSuperBlockIOCB *iocb = (void *)common;
    struct SEFStatus status = {-1, 0};
    struct nvme_passthru_cmd64 cmd;
    struct PatrolSbParam *pParam;
    int ret;

    QOSD_REQUEST_COUNT_INC(pQos);

    // Check parameter
    if (pQos->qosId != GetQosdId(iocb->flashAddress))
    {
        ULOG_ERROR("argument error!! flashAddress=0x%016lx\n", iocb->flashAddress.bits);
        status.error = -EINVAL;
        goto DecrementProcReq;
    }

    // Prepare parameters
    pParam = (struct PatrolSbParam *)malloc(sizeof(*pParam));
    if (pParam == NULL)
    {
        goto DecrementProcReq;
    }

    pParam->pQos = pQos;
    pParam->sbId = GetSbId(pQos, iocb->flashAddress);

    iocb->common.reserved = 1;

    // Retrieve SB Patrol information
    SEFNvmSuperBlockManagement(&cmd, NULL, 0, SEF_NVMOPE_SBMANAGEMENT_PATROL, pQos->qosId,
                               pParam->sbId, NULL, 0);
    ret = IssueNoDataCmd(pQos->deviceFd, bSync, CMD_I_SBMNG_PATROL, &cmd, NULL,
                         (union RequestIocb *)iocb, pParam, CheckResultPatrolSb);
    if (ret != 0)
    {
        status.error = ret;
        goto FreeParam;
    }

    if (bSync)
    {
        // For synchronous operation, command result is set
        status = iocb->common.status;
    }
    else
    {
        // For async, SUCCESS is returned if command is issued
        status.error = 0;
    }
    goto ExitProc;

FreeParam:
    free(pParam);    // Released only in case of an error; otherwise, CheckResult releases it
DecrementProcReq:
    QOSD_REQUEST_COUNT_DEC(
        pQos);    // Decremented only in case of an error; otherwise, CheckResult decrements it
ExitProc:
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFCheckSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress)
{
    struct SEFStatus status = {-1, 0};
    struct PatrolSuperBlockIOCB iocb = {
        .common = {.status = {0, 0}, .opcode = 0, .flags = 0, .param1 = NULL, .complete_func = NULL},
        .flashAddress = flashAddress};
    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    status = PatrolSbProc(true, qosHandle, &iocb.common);
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFDeleteVirtualDevices(SEFHandle sefHandle)
{
    struct SEFHandle_ *pSef = sefHandle;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;
    struct SEFStatus status = {-EINVAL, 0};

    ULOG_ENTER_SEFAPI();

    if (!pSef)
    {
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (atomic_load(&pSef->numOpenVds))
    {
        status.error = -EBUSY;
        goto ExitProc;
    }
    if (pSef->pSefInfo->numQoSDomains)
    {
        status.error = -ENOTEMPTY;
        goto ExitProc;
    }
    // Delete old config
    SEFAdmCapacityManagement(&cmd, 0);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_CAP_MGMT, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFDeleteQoSDomain(SEFHandle sefHandle, struct SEFQoSDomainID QoSDomainID)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SEFQoSHandle_ *pQos;
    struct nvme_passthru_cmd64 cmd;
    uint32_t cmdStat;
    int ret;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (IS_INVALID_QOSD_ID(QoSDomainID.id, pSef->pSefInfo->maxQoSDomains))
    {
        ULOG_ERROR("argument error!! QoSDomainID=%u\n", QoSDomainID.id);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    // Check whether QoS Domain is opened (error if opened)
    pQos = DeviceInfoGetQosHandle(pSef, QoSDomainID.id);
    if (pQos != NULL)
    {
        ULOG_ERROR("still open the qosd handle\n");
        status.error = -EBUSY;
        goto ExitProc;
    }

    // Detach QoSD
    uint16_t ctrlId[4096 / sizeof(uint16_t)] = {1, 0};
    SEFAdmNameSpaceAttachment(&cmd, &ctrlId, sizeof(ctrlId), QoSDomainID.id, false);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_NS_ATTACH, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        ULOG_ERROR("Failed to detach QoSD %u\n", QoSDomainID.id);
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    // Delete QoSD information
    SEFAdmNamespaceManagement(&cmd, NULL, 0, QoSDomainID.id, false);
    ret = IssueSendDataCmd(pSef->deviceFd, true, CMD_A_QOSD_DELETE, &cmd, &cmdStat, NULL, NULL, NULL);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        // Try to reattach QoSD
        SEFAdmNameSpaceAttachment(&cmd, &ctrlId, sizeof(ctrlId), QoSDomainID.id, true);
        IssueSendDataCmd(pSef->deviceFd, true, CMD_A_NS_ATTACH, &cmd, &cmdStat, NULL, NULL, NULL);
        goto ExitProc;
    }
    pSef->pSefInfo->numQoSDomains--;
    status.error = 0;
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFResetEncryptionKey(SEFVDHandle vdHandle, struct SEFQoSDomainID QoSDomainID)
{    // Not support by cmd spec 1.13, fake it worked.
    struct SEFStatus status = {-1, 0};
    struct SEFQoSHandle_ *pQos;
    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (vdHandle == NULL)
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    pQos = DeviceInfoGetQosHandle(vdHandle->pSefHandle, QoSDomainID.id);
    // Check QoSDomainID parameter
    if (pQos == NULL)
    {
        ULOG_ERROR("argument error!! QoSDomainID=%u pQos=%p\n", QoSDomainID.id, pQos);
        status.error = -EINVAL;
        goto ExitProc;
    }

    status.error = 0;
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFOpenVirtualDevice(SEFHandle sefHandle,
                                      struct SEFVirtualDeviceID virtualDeviceID,
                                      void (*notifyFunc)(void *, struct SEFVDNotification),
                                      void *context,
                                      SEFVDHandle *vdHandle)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SEFVDHandle_ *pVd;
    struct nvme_passthru_cmd64 cmd;
    uint32_t size;
    uint32_t cmdStat;
    struct SefLogVdInfo *pVdInfo = NULL;
    struct SEFDieList *pDieList = NULL;
    int ret = 0;
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if ((virtualDeviceID.id > (pSef->pSefInfo->numChannels * pSef->pSefInfo->numBanks)) ||
        (vdHandle == NULL))
    {
        ULOG_ERROR("argument error!! virtualDeviceID=%u, vdHandle=%p\n", virtualDeviceID.id, vdHandle);
        status.error = -EINVAL;
        status.info = vdHandle == NULL ? 5 : 2;
        goto ExitProc;
    }

    // Check whether Virtual Device is opened
    pVd = DeviceInfoGetVdHandle(pSef, virtualDeviceID.id);
    if (pVd != NULL)
    {
        ULOG_ERROR("already open the vd handle\n");
        status.error = -EALREADY;
        goto ExitProc;
    }

    // Retrieve VD Static information
    uint32_t numDies = pSef->pSefInfo->numBanks * pSef->pSefInfo->numChannels;
    size = sizeof(*pVdInfo) + sizeof(uint16_t) * numDies;
    size = TO_UPPER_DWORD_SIZE(size);
    SEFAdmGetLogPageVDInfo(&cmd, pVdInfo, size, virtualDeviceID.id);
    ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GLOGP_VDINFO, &cmd, &cmdStat, NULL, NULL,
                           NULL, (void **)&pVdInfo, size);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }
    DEBUG_SAVE_INFO_FILE(pVdInfo, size, kDebugSaveFileGetVdDynamicInfo);

    if (pVdInfo->ndie >= numDies)
    {
        ULOG_ERROR("Only partial die list obtained %s()\n", __FUNCTION__);
    }

    pDieList = malloc(sizeof(struct SEFDieList) + (pVdInfo->ndie + 1) * sizeof(uint16_t));
    if (!pDieList)
    {
        status.error = -ENOMEM;
        goto FreeVdInfo;
    }
    pDieList->numDies = pVdInfo->ndie + 1;
    for (int i = 0; i < pDieList->numDies && i < numDies; ++i)
    {
        pDieList->dieIDs[i] = pVdInfo->die_ids[i];
    }

    // held until vd handle is fully set up so vd aen sees a complete handle
    // Note: notify can't open any vd handles w/o deadlock.
    pthread_mutex_lock(&pSef->vd_mutex);

    /* Request to create SEFVDHandle_ */
    ret = DeviceInfoCreateVdHandle(pSef, virtualDeviceID.id, &pVd);
    if (ret != 0)
    {
        pthread_mutex_unlock(&pSef->vd_mutex);
        ULOG_ERROR("failed to DeviceInfoCreateVdHandle()\n");
        free(pDieList);
        status.error = ret;
        goto FreeVdInfo;
    }

    // Set parameters and VD information retrieved from device to SEFVDHandle_
    pVd->pDieList = pDieList;
    pVd->notifyFunc = notifyFunc;
    pVd->pContext = context;
    pthread_mutex_unlock(&pSef->vd_mutex);

    status = SetVDCritWarningMask(pSef, pVd->vdId, SEF_CAE_WARN_MASK);
    if (status.error != 0)
    {
        ULOG_ERROR("Failed to enable VD AEN (%d)\n", status.error);
        // VD is still open, just report failure.
        status.error = 0;
        status.info = 0;
    }

    // Count up number of VDs if created successfully
    atomic_fetch_add(&pSef->numOpenVds, 1);

    // Set output parameters
    *vdHandle = pVd;

FreeVdInfo:
    free(pVdInfo);
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFCloseVirtualDevice(SEFVDHandle vdHandle)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef;
    struct SEFVDHandle_ *pVd = vdHandle;
    ULOG_ENTER_SEFAPI();

    // Check whether called from a wait thread
    if (DeviceIoIsIoThread())
    {
        ULOG_ERROR("calling sync functions on wait thread\n");
        status.error = -EWOULDBLOCK;
        goto ExitProc;
    }

    // Check parameter
    if (pVd == NULL)
    {
        ULOG_ERROR("argument error!! vdHandle=%p\n", vdHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    pSef = pVd->pSefHandle;

    // Lock in case we're closing what VD AEN is getting - we'll wait or it'll
    // see NULL.  Note: notify can't close any vd handle w/o deadlock.
    pthread_mutex_lock(&pSef->vd_mutex);
    // Request to delete SEFVDHandle_
    DeviceInfoDeleteVdHandle(pVd);
    pthread_mutex_unlock(&pSef->vd_mutex);
    // Count down number of open VDs if closed successfully
    atomic_fetch_sub(&pSef->numOpenVds, 1);

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFOpenQoSDomain(SEFHandle sefHandle,
                                  struct SEFQoSDomainID QoSDomainID,
                                  void (*notifyFunc)(void *, struct SEFQoSNotification),
                                  void *context,
                                  const void *encryptionKey,
                                  SEFQoSHandle *qosHandle)
{
    struct SEFStatus status = {-1, 0};
    struct SEFHandle_ *pSef = sefHandle;
    struct SEFQoSHandle_ *pQos;
    struct nvme_passthru_cmd64 cmd;
    uint32_t size;
    uint32_t cmdStat;
    uint16_t features;
    uint16_t nPID;
    int ret;
    char devName[64];
    char nvmeDevName[64];
    char vdName[64];
    char *qdName;
    int devidx, nsidx;

    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (sefHandle == NULL)
    {
        ULOG_ERROR("argument error!! sefHandle=%p\n", sefHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (IS_INVALID_QOSD_ID(QoSDomainID.id, pSef->pSefInfo->maxQoSDomains) || (qosHandle == NULL))
    {
        ULOG_ERROR("argument error!! QoSDomainID=%u, qosHandle=%p\n", QoSDomainID.id, qosHandle);
        status.error = -EINVAL;
        status.info = qosHandle == NULL ? 6 : 2;
        goto ExitProc;
    }

    // Check whether QoS Domain is opened
    pQos = DeviceInfoGetQosHandle(pSef, QoSDomainID.id);
    if (pQos != NULL)
    {
        ULOG_ERROR("already open the qosd handle\n");
        status.error = -EALREADY;
        goto ExitProc;
    }

    // Retrieve QoSD information
    struct SefNsIdentify *pSefNsIdentify;
    size = sizeof(*pSefNsIdentify);
    size = TO_UPPER_DWORD_SIZE(size);
    SEFAdmSefNsIdentify(&cmd, QoSDomainID.id, NULL, size);
    ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GFEAT_QOSD_INFO, &cmd, &cmdStat, NULL, NULL,
                           NULL, (void **)&pSefNsIdentify, size);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    qdName = GetBlkNameFromNSID(pSef->nvmeUnit, QoSDomainID.id);
    if (!qdName)
    {
        status = (struct SEFStatus){-EINVAL, 2};
        goto ExitProc;
    }

    sscanf(qdName, "sef%dn%d", &devidx, &nsidx);
    snprintf(nvmeDevName, sizeof(nvmeDevName), "/sys/class/nvme/nvme%d/nvme%dn%d", devidx, devidx,
             nsidx);
    snprintf(devName, sizeof(devName), "%s/%s", pSef->nvmeUnit, qdName);
    free(qdName);
    features = SysReadFileDeviceInt(devName, "features");
    snprintf(vdName, sizeof(vdName), "%s/vd%d", pSef->nvmeUnit, pSefNsIdentify->endgid);
    // Lock sef
    pthread_mutex_lock(&pSef->mutex);

    nPID = SysReadFileDeviceInt(devName, "nplid");
    // Request to create SEFQoSHandle_
    ret = DeviceInfoCreateQosHandle(pSef, pSefNsIdentify->endgid, QoSDomainID.id, nPID, &pQos);
    if (ret != 0)
    {
        ULOG_ERROR("failed to DeviceInfoCreateQosHandle()\n");
        pthread_mutex_unlock(&pSef->mutex);
        status.error = ret;
        goto FreeNsInfo;
    }

    // Set parameters and QoSD information retrieved from device to SEFQoSHandle_
    pthread_mutex_lock(&pQos->mutex);
    pQos->aduOffsetBitLen = SysReadFileDeviceInt(vdName, "adu_bits");
    pQos->sbIdBitLen = SysReadFileDeviceInt(vdName, "sb_bits");
    pQos->sbCapacity = getSBCapacity(vdName, pSef->pSefInfo);
    pQos->sbSize = SysReadFileDeviceInt(vdName, "sb_size");
    pQos->ndie = SysReadFileDeviceInt(vdName, "ndie");
    pQos->maxBufSize =
        (size_t)GetSysPageSize() * (SysReadFileDeviceInt(nvmeDevName, "queue/max_segments") - 1);
    pQos->notifyFunc = notifyFunc;
    pQos->pContext = context;
    atomic_store(&pQos->numProcessingRequests, 0);
    atomic_store(&pQos->numProcessingNlw, 0);
    pQos->privateData.type = kSefPropertyTypeNull;
    pQos->defectStrategy = ConvDefectStrategyForApi((enum DefectStrategyForDev)((features >> 2) & 0x3));
    atomic_store(&pQos->bClosingFlag, false);
    utl_DListInit(&pQos->nlcQueue);
    pQos->numPlacementIds = nPID;
    pQos->nrq = GetVdReadFifos(vdName, pQos->read_fifos);

    SEF_ASSERT(!ret);
    pthread_mutex_unlock(&pQos->mutex);

    // Count up number of QosDs if created successfully
    pSef->numOpenAllQosds++;
    pthread_mutex_unlock(&pSef->mutex);

    // Set output parameters
    *qosHandle = pQos;

    status.error = 0;

FreeNsInfo:
    free(pSefNsIdentify);
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

struct SbOpenList
{
    uint32_t size;
    uint32_t index;
    uint32_t *sbs;
};

STATIC void GeSbid(void *p1, void *p2)
{
    struct SbWriteInformation *sb_info = p2;
    struct SbOpenList *open_list = p1;
    uint32_t index = open_list->index++;
    if (open_list->index == open_list->size)
    {
        uint32_t size = open_list->size * 2;
        open_list->sbs = realloc(open_list->sbs, size);
        open_list->size = size;
    }
    open_list->sbs[index] = sb_info->sbId;
}

/*
 * @brief Closes all the open super blocks
 *
 * Closes the super blocks the library thinks are open.  It's possible there are
 * closes inflight that have yet to be processed.  This function won't return
 * until the all close notifications have been processed by the client.
 *
 * @param pQos  Domain handle to close the open super blocks for
 */
STATIC void CloseOpenBlocks(struct SEFQoSHandle_ *pQos)
{
    int i = 0;
    struct SbOpenList open_list = {.index = 0, .size = 16};
    open_list.sbs = malloc(sizeof(uint32_t) * open_list.size);

    LHTforeach(pQos->sbs_info, GeSbid, &open_list);

    if (open_list.index > 3 * pQos->numPlacementIds)
    {
        ULOG_ERROR("There are only %d placement id's yet %d super blocks are still open",
                   pQos->numPlacementIds, open_list.index);
    }
    // Close all open Super Blocks we think are open
    for (i = 0; i < open_list.index; i++)
    {
        SEFCloseSuperBlock(pQos, GetFlashAddress(pQos, pQos->qosId, open_list.sbs[i]));
    }
    free(open_list.sbs);
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFCloseQoSDomain(SEFQoSHandle qosHandle)
{
    struct SEFStatus status = {-1, 0};
    uint16_t qosId;
    struct SEFHandle_ *pSef;
    struct SEFQoSHandle_ *pQos = qosHandle;
    bool bLocking = false;
    uint16_t openSbNum;
    uint16_t closeSbCount;
    uint16_t maxGetListNum;
    struct nvme_passthru_cmd64 cmd;
    uint32_t size;
    uint32_t cmdStat;
    struct SefNsIdentify *pSefNsIdentify;
    struct GetLogPageSuperBlockList *pOpenSbList = NULL;
    int ret;
    uint32_t sbListDescriptorMax;
    ULOG_ENTER_SEFAPI();

    // Check whether called from a wait thread
    if (DeviceIoIsIoThread())
    {
        ULOG_ERROR("calling sync functions on wait thread\n");
        status.error = -EWOULDBLOCK;
        goto ExitProc;
    }

    // Check parameter
    if ((qosHandle == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    qosId = pQos->qosId;
    pSef = pQos->pSefHandle;

    CloseOpenBlocks(pQos);

    // Wait uncompleted requests to complete
    bLocking = true;
    atomic_store(&pQos->bClosingFlag,
                 true);    // Enable being closed flag (so as not to call NotifyFunc at SB State Change)
    ResumeAllSuspendedResponse(pQos);    // Execute all suspended responses and SB notifications
    pthread_mutex_lock(&pQos->mutex);
    WaitAllRequestsComplete(pQos);    // Wait until there is no ongoing request
    DeviceInfoSbCleanup(pQos);

    // Retrieve QoSD information
    size = sizeof(*pSefNsIdentify);
    size = TO_UPPER_DWORD_SIZE(size);
    SEFAdmSefNsIdentify(&cmd, qosId, NULL, size);
    ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GFEAT_QOSD_INFO, &cmd, &cmdStat, NULL, NULL,
                           NULL, (void **)&pSefNsIdentify, size);
    if ((ret != 0) || (cmdStat != 0))
    {
        status.error = ret ?: cmdStat;
        goto ExitProc;
    }

    openSbNum = pSefNsIdentify->sbsta.nwosb + pSefNsIdentify->sbsta.neosb +
                pSefNsIdentify->psbsta.nwosb + pSefNsIdentify->psbsta.neosb;
    closeSbCount = 0;
    sbListDescriptorMax = SBLIST_MAX_LEN(pQos);
    maxGetListNum = (openSbNum + (sbListDescriptorMax - 1)) / sbListDescriptorMax;
    ULOG_DEBUG("open sb=%u, NSB=%u, NWOSB=%u, NOESB=%u\n", openSbNum, pSefNsIdentify->sbsta.nsb,
               pSefNsIdentify->sbsta.nwosb, pSefNsIdentify->sbsta.neosb);
    ULOG_DEBUG("open psb=%u, PNSB=%u, PNWOSB=%u, PNOESB=%u\n", openSbNum, pSefNsIdentify->psbsta.nsb,
               pSefNsIdentify->psbsta.nwosb, pSefNsIdentify->psbsta.neosb);
    if (maxGetListNum)    // nothing should be open
    {
        ULOG_ERROR("There are %d super blocks still open at shutdown\n", maxGetListNum);
    }
    for (int i = 0; i < maxGetListNum; i++)
    {
        // Retrieve SB list
        size = sizeof(pOpenSbList->SuperBlockDescriptor[0]) * sbListDescriptorMax;
        size = TO_UPPER_DWORD_SIZE(size);
        SEFAdmGetLogPageSBList(&cmd, NULL, size, qosId, SEF_LODR_OPEN_CLOSED, 0);
        ret = IssueRecvDataCmd(pSef->deviceFd, true, CMD_A_GLOGP_SBLIST_OPENCLOSED, &cmd, &cmdStat,
                               NULL, NULL, NULL, (void **)&pOpenSbList, size);
        if ((ret != 0) || (cmdStat != 0))
        {
            ULOG_ERROR("can't retrieve QOSD info %d\n", ret);
            status.error = ret ?: cmdStat;
            goto FreeQosdInfo;
        }

        // Close open Super Blocks
        for (int j = 0; (j < sbListDescriptorMax) && (closeSbCount < openSbNum); j++)
        {
            uint32_t sbId = pOpenSbList->SuperBlockDescriptor[j].SBID;
            uint8_t diisbs = pOpenSbList->SuperBlockDescriptor[j].DIISBS;
            if (sbId == SBLIST_LOG_CMD_LIST_END)
            {
                ULOG_ERROR("inconsistensy open sb list!! open=%u, close=%u\n", openSbNum, closeSbCount);
                goto FreeOpenSbList;
            }
            if (diisbs & kSbClosed)
            {    // skip if not open
                continue;
            }
            ULOG_INFORMATION("Forcing SB 0x%x closed\n", sbId);
            SEFNvmSuperBlockManagement(&cmd, NULL, 0, SEF_NVMOPE_SBMANAGEMENT_CLOSE, qosId, sbId,
                                       NULL, 0);
            ret = IssueNoDataCmd(pQos->deviceFd, true, CMD_I_SBMNG_CLOSE, &cmd, &cmdStat, NULL,
                                 NULL, NULL);
            if ((ret != 0) || (cmdStat != 0))
            {
                status.error = ret ?: cmdStat;
                goto FreeOpenSbList;
            }
            closeSbCount++;
        }

        free(pOpenSbList);
        pOpenSbList = NULL;    // Assign NULL as released
    }

    pthread_mutex_unlock(&pQos->mutex);
    bLocking = false;

    // Lock sef
    pthread_mutex_lock(&pSef->mutex);
    // Request to delete SEFQoSHandle_
    DeviceInfoDeleteQosHandle(pQos);

    // Count down number of open QosDs if closed successfully
    pSef->numOpenAllQosds--;
    pthread_mutex_unlock(&pSef->mutex);

    status.error = 0;

FreeOpenSbList:
    if (pOpenSbList != NULL)
    {
        free(pOpenSbList);
    }
FreeQosdInfo:
    free(pSefNsIdentify);
ExitProc:
    if (bLocking)
    {
        atomic_store(&pQos->bClosingFlag, false);    // Restore being closed flag because deletion aborts
        pthread_mutex_unlock(&pQos->mutex);
    }
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see	SEFAPI.h
 */
struct SEFProperty SEFGetQoSHandleProperty(SEFQoSHandle qosHandle, enum SEFPropertyID propID)
{
    struct SEFProperty prop = {};

    switch (propID)
    {
        case kSefPropertyNumActiveRequests:
            prop.type = kSefPropertyTypeInt;
            pthread_mutex_lock(&qosHandle->mutex);
            prop.intVal = atomic_load(&qosHandle->numProcessingRequests);
            prop.intVal += atomic_load(&qosHandle->numProcessingNlw);
            pthread_mutex_unlock(&qosHandle->mutex);
            break;
        case kSefPropertyPrivateData:
            prop = qosHandle->privateData;
            break;
        case kSefPropertyQoSDomainID:
            prop.type = kSefPropertyTypeQoSDomainID;
            prop.qosID.id = qosHandle->qosId;
            break;
        case kSefPropertyQoSNotify:
            prop.type = kSefPropertyTypeQoSNotify;
            prop.qosNotify = qosHandle->notifyFunc;
            break;
        case kSefPropertyUnitNumber:
            prop.type = kSefPropertyTypeInt;
            prop.intVal = qosHandle->pSefHandle->deviceId;
            break;
        case kSefPropertyVirtualDeviceID:
            prop.type = kSefPropertyTypeVirtualDeviceID;
            prop.vdID.id = qosHandle->vdId;
            break;
        default:
            break;
    }
    return prop;
}

/*
 *  @see	SEFAPI.h
 */
struct SEFStatus SEFSetQoSHandleProperty(SEFQoSHandle qosHandle,
                                         enum SEFPropertyID propID,
                                         struct SEFProperty value)
{
    struct SEFStatus status = {};

    if (qosHandle == NULL)
    {
        status = (struct SEFStatus){-ENODEV, 0};
    }
    else if (propID != kSefPropertyPrivateData)
    {
        status = (struct SEFStatus){-EINVAL, 2};
    }
    else if (value.type == kSefPropertyTypeInvalid)
    {
        status = (struct SEFStatus){-EINVAL, 3};
    }
    else
    {
        qosHandle->privateData = value;
    }

    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFParseFlashAddress(SEFQoSHandle qosHandle,
                                      struct SEFFlashAddress flashAddress,
                                      struct SEFQoSDomainID *QoSDomainID,
                                      uint32_t *blockNumber,
                                      uint32_t *ADUOffset)
{
    struct SEFStatus status = {-1, 0};
    ULOG_ENTER_SEFAPI();

    if (qosHandle == NULL && (blockNumber || ADUOffset))
    {
        ULOG_ERROR("argument error!! qosHandle=NULL with non-NULL domain specific fields\n");
        status.error = -EINVAL;
        goto ExitProc;
    }

    if (QoSDomainID != NULL)
    {
        QoSDomainID->id = GetQosdId(flashAddress);
    }

    if (blockNumber != NULL)
    {
        *blockNumber = GetSbId(qosHandle, flashAddress);
    }

    if (ADUOffset != NULL)
    {
        *ADUOffset = GetAduOffset(qosHandle, flashAddress);
    }

    status.error = 0;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFFlashAddress SEFCreateFlashAddress(SEFQoSHandle qosHandle,
                                             struct SEFQoSDomainID QoSDomainID,
                                             uint32_t blockNumber,
                                             uint32_t ADUOffset)
{
    struct SEFFlashAddress flashAddress = {0};
    ULOG_ENTER_SEFAPI();

    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        goto ExitProc;
    }

    flashAddress = GetFlashAddress(qosHandle, QoSDomainID.id, blockNumber);
    if (SEFIsNullFlashAddress(flashAddress))
    {
        goto ExitProc;
    }
    flashAddress.bits |= ADUOffset;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return flashAddress;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFWriteWithoutPhysicalAddress(SEFQoSHandle qosHandle,
                                                struct SEFFlashAddress flashAddress,
                                                struct SEFPlacementID placementID,
                                                struct SEFUserAddress userAddress,
                                                uint32_t numADU,
                                                const struct iovec *iov,
                                                uint16_t iovcnt,
                                                const void *metadata,
                                                struct SEFFlashAddress *permanentAddresses,
                                                uint32_t *distanceToEndOfSuperBlock,
                                                const struct SEFWriteOverrides *overrides)
{
    struct SEFStatus status = {-1, 0};
    struct SEFWriteWithoutPhysicalAddressIOCB iocb = {.common = {},
                                                      .tentativeAddresses = permanentAddresses,
                                                      .flashAddress = flashAddress,
                                                      .userAddress = userAddress,
                                                      .iov = iov,
                                                      .iovcnt = iovcnt,
                                                      .placementID = placementID,
                                                      .numADU = numADU,
                                                      .metadata = metadata,
                                                      .distanceToEndOfSuperBlock = 0};
    ULOG_ENTER_SEFAPI();

    // Check parameter (except for qosHandle, parameters common to sync/async are checked in WriteProc)
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (overrides)
    {
        iocb.common.flags |= kSefIoFlagOverride;
        iocb.overrides = *overrides;
    }
    status = WriteProc(true, qosHandle, &iocb.common);
    if ((status.error == 0) && (distanceToEndOfSuperBlock != NULL))
    {
        *distanceToEndOfSuperBlock = iocb.distanceToEndOfSuperBlock;
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

static void SEFReadComplete(struct SEFCommonIOCB *iocb)
{
    sem_t *sem = (sem_t *)iocb->param1;
    sem_post(sem);
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFReadWithPhysicalAddress(SEFQoSHandle qosHandle,
                                            struct SEFFlashAddress flashAddress,
                                            uint32_t numADU,
                                            const struct iovec *iov,
                                            uint16_t iovcnt,
                                            size_t iovOffset,
                                            struct SEFUserAddress userAddress,
                                            void *metadata,
                                            const struct SEFReadOverrides *overrides)
{
    sem_t complete_sem;
    struct SEFStatus status = {-1, 0};
    struct SEFReadWithPhysicalAddressIOCB iocb = {
        {.status = {0, 0}, .opcode = 0, .flags = 0, .param1 = &complete_sem, .complete_func = SEFReadComplete},
        .flashAddress = flashAddress,
        .userAddress = userAddress,
        .iov = iov,
        .iovOffset = iovOffset,
        .numADU = numADU,
        .metadata = metadata,
        .iovcnt = iovcnt};
    ULOG_ENTER_SEFAPI();

    sem_init(&complete_sem, 0, 0);
    // Check parameter (except for qosHandle, parameters common to sync/async are checked in ReadProc)
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (overrides)
    {
        iocb.common.flags |= kSefIoFlagOverride;
        iocb.overrides = *overrides;
    }

    status = ReadProc(true, qosHandle, &iocb.common);
    if (status.error)
    {
        goto ExitProc;
    }
    sem_wait(&complete_sem);
    status = iocb.common.status;
ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFReleaseSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress)
{
    struct SEFStatus status = {-1, 0};
    struct SEFReleaseSuperBlockIOCB iocb = {
        {.status = {0, 0}, .opcode = 0, .flags = 0, .param1 = NULL, .complete_func = NULL},
        .flashAddress = flashAddress};
    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    status = ReleaseSbProc(true, qosHandle, &iocb.common);

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFAllocateSuperBlock(SEFQoSHandle qosHandle,
                                       struct SEFFlashAddress *flashAddress,
                                       enum SEFSuperBlockType type,
                                       uint8_t *defectMap,
                                       const struct SEFAllocateOverrides *overrides)
{
    struct SEFStatus status = {-1, 0};
    struct SEFAllocateSuperBlockIOCB iocb = {
        {.status = {0, 0}, .opcode = 0, .flags = 0, .param1 = NULL, .complete_func = NULL},
        .flashAddress = {0},
        .defectMap = defectMap,
        .type = type};
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (flashAddress == NULL)
    {
        ULOG_ERROR("argument error!! flashAddress=%p\n", flashAddress);
        status.error = -EINVAL;
        status.info = 2;
        goto ExitProc;
    }

    if (overrides)
    {
        iocb.common.flags |= kSefIoFlagOverride;
        iocb.overrides = *overrides;
    }
    // Allocate SB
    status = AllocateSbProc(true, qosHandle, &iocb.common);
    if (status.error == 0)
    {
        *flashAddress = iocb.flashAddress;
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFFlushSuperBlock(SEFQoSHandle qosHandle,
                                    struct SEFFlashAddress flashAddress,
                                    uint32_t *distanceToEndOfSuperBlock)
{
    struct SEFStatus status = {-1, 0};
    struct FlushSuperBlockIOCB iocb = {
        {.status = {0, 0}, .opcode = 0, .flags = 0, .param1 = NULL, .complete_func = NULL},
        .flashAddress = flashAddress,
        .distanceToEndOfSuperBlock = 0};
    sem_t flushDone;
    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    // Flush SB
    sem_init(&flushDone, 0, 0);
    iocb.common.reserved = 1;
    iocb.common.param1 = &flushDone;
    iocb.common.complete_func = NLCQIocbComplete;
    if (QueueNLCIocb(qosHandle, flashAddress, &iocb.common, kQFlush))
    {
        sem_wait(&flushDone);
        StartNextCopy(qosHandle);
        status = iocb.common.status;
    }
    else
    {
        status = FlushSbProc(true, qosHandle, &iocb.common, false, false);
    }
    if (status.error != 0)
    {
        goto ExitProc;
    }
    // May be NULL.
    if (distanceToEndOfSuperBlock == NULL)
    {
        goto ExitProc;
    }

    *distanceToEndOfSuperBlock = iocb.distanceToEndOfSuperBlock;

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFCloseSuperBlock(SEFQoSHandle qosHandle, struct SEFFlashAddress flashAddress)
{
    struct SEFStatus status = {-1, 0};
    struct SEFCloseSuperBlockIOCB iocb = {
        {.status = {0, 0}, .opcode = 0, .flags = 0, .param1 = NULL, .complete_func = NULL},
        .flashAddress = flashAddress};
    sem_t closeDone;

    ULOG_ENTER_SEFAPI();

    // Check parameter
    if (qosHandle == NULL)
    {
        ULOG_ERROR("argument error!! qosHandle=%p\n", qosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    sem_init(&closeDone, 0, 0);
    iocb.common.reserved = 1;
    iocb.common.param1 = &closeDone;
    iocb.common.complete_func = NLCQIocbComplete;
    if (QueueNLCIocb(qosHandle, flashAddress, &iocb.common, kQClose))
    {
        sem_wait(&closeDone);
        StartNextCopy(qosHandle);
        status = iocb.common.status;
    }
    else
    {
        status = CloseSbProc(true, qosHandle, &iocb.common, true);
    }

    if (status.error)
    {
        ULOG_ERROR("Block %" PRIx64 "failed to close\n", flashAddress.bits);
        goto ExitProc;
    }
    FlushCloseNotification(qosHandle, GetSbId(qosHandle, flashAddress));
    SEF_ASSERT(DeviceInfoIsClosingSb(qosHandle, GetSbId(qosHandle, flashAddress)) == 0);
    SEF_ASSERT(DeviceInfoIsOpenedSb(qosHandle, GetSbId(qosHandle, flashAddress)) == 0);

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
void SEFFreeBufferForNamelessCopy(void *copyContext)
{
    ULOG_ENTER_SEFAPI();
    if (copyContext != NULL)
    {
        struct CopyContext *pContext = copyContext;
        free(pContext->acr);
        free(pContext);
    }
    ULOG_LEAVE_SEFAPI();
}

/*
 *  @see    SEFAPI.h
 */
struct SEFStatus SEFNamelessCopy(SEFQoSHandle srcQosHandle,
                                 struct SEFCopySource copySource,
                                 SEFQoSHandle dstQosHandle,
                                 struct SEFFlashAddress copyDestination,
                                 const struct SEFUserAddressFilter *filter,
                                 const struct SEFCopyOverrides *overrides,
                                 uint32_t numAddressChangeRecords,
                                 struct SEFAddressChangeRequest *addressChangeInfo)
{
    struct SEFStatus status = {-1, 0};
    struct SEFNamelessCopyIOCB iocb = {
        {.status = {0, 0}, .opcode = 0, .flags = kSefIoFlagCommit, .param1 = NULL, .complete_func = NULL},
        .dstQosHandle = dstQosHandle,
        .copyDestination = copyDestination,
        .addressChangeInfo = addressChangeInfo,
        .numAddressChangeRecords = numAddressChangeRecords,
        .copySource = copySource,
        .filter = filter};
    ULOG_ENTER_SEFAPI();

    // Check parameter (except for qosHandle, parameters common to sync/async are checked in CopyProc)
    if (srcQosHandle == NULL)
    {
        ULOG_ERROR("argument error!! srcQosHandle=%p\n", srcQosHandle);
        status.error = -ENODEV;
        goto ExitProc;
    }

    if (overrides)
    {
        iocb.common.flags |= kSefIoFlagOverride;
        iocb.overrides = *overrides;
    }
    status = CopyProc(true, srcQosHandle, &iocb.common);

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return status;
}

/*
 *  @see    SEFAPI.h
 */
void SEFWriteWithoutPhysicalAddressAsync(SEFQoSHandle qosHandle,
                                         struct SEFWriteWithoutPhysicalAddressIOCB *iocb)
{
    struct SEFStatus status = {-1, 0};
    ULOG_ENTER_SEFAPI();

    // Check parameters (except for qosHandle, parameters common to sync/async are checked in WriteProc)
    if ((qosHandle == NULL) || (iocb == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p iocb=%p\n", qosHandle, iocb);
        if (iocb != NULL)
        {
            status.error = qosHandle == NULL ? -ENODEV : -EINVAL;
            goto SetStatus;
        }
        goto ExitProc;
    }

    iocb->common.status.info = 0;
    iocb->common.status.error = 0;
    iocb->common.flags &= ~kSefIoFlagDone;
    status = WriteProc(false, qosHandle, &iocb->common);
    if (status.error != 0)
    {
    SetStatus:
        iocb->common.status = status;
        const struct iovec *iov = iocb->iov;
        uint16_t iovcnt = iocb->iovcnt;
        int nrel = iocb->common.flags & kSefIoFlagNotifyBufferRelease;
        iocb->common.flags |= kSefIoFlagDone;
        if (iocb->common.complete_func != NULL)
        {
            iocb->common.complete_func(&iocb->common);
        }
        if (nrel && qosHandle->notifyFunc)
        {
            struct SEFQoSNotification notify;

            notify.type = kBufferRelease;
            notify.iov = iov;
            notify.iovcnt = iovcnt;
            qosHandle->notifyFunc(qosHandle->pContext, notify);
        }
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return;
}

/*
 *  @see    SEFAPI.h
 */
void SEFReadWithPhysicalAddressAsync(SEFQoSHandle qosHandle, struct SEFReadWithPhysicalAddressIOCB *iocb)
{
    struct SEFStatus status = {-1, 0};
    ULOG_ENTER_SEFAPI();

    // Check parameters (except for qosHandle, parameters common to sync/async are checked in ReadProc)
    if ((qosHandle == NULL) || (iocb == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p, iocb=%p\n", qosHandle, iocb);
        if (iocb != NULL)
        {
            void (*complete_func)(struct SEFCommonIOCB *) = iocb->common.complete_func;
            status.error = qosHandle == NULL ? -ENODEV : -EINVAL;
            iocb->common.status = status;
            iocb->common.flags |= kSefIoFlagDone;
            if (complete_func != NULL)
            {
                complete_func(&iocb->common);
            }
        }
        goto ExitProc;
    }

    iocb->common.status.info = 0;
    iocb->common.status.error = 0;
    iocb->common.flags &= ~kSefIoFlagDone;
    ReadProc(false, qosHandle, &iocb->common);

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return;
}

/*
 *  @see    SEFAPI.h
 */
void SEFReleaseSuperBlockAsync(SEFQoSHandle qosHandle, struct SEFReleaseSuperBlockIOCB *iocb)
{
    struct SEFStatus status = {-1, 0};
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if ((qosHandle == NULL) || (iocb == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p, iocb=%p\n", qosHandle, iocb);
        if (iocb != NULL)
        {
            status.error = qosHandle == NULL ? -ENODEV : -EINVAL;
            goto SetStatus;
        }
        goto ExitProc;
    }

    iocb->common.status.info = 0;
    iocb->common.status.error = 0;
    iocb->common.flags &= ~kSefIoFlagDone;
    status = ReleaseSbProc(false, qosHandle, &iocb->common);
    if (status.error != 0)
    {
    SetStatus:
        iocb->common.status = status;
        iocb->common.flags |= kSefIoFlagDone;
        if (iocb->common.complete_func != NULL)
        {
            iocb->common.complete_func(&iocb->common);
        }
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return;
}

/*
 *  @see    SEFAPI.h
 */
void SEFAllocateSuperBlockAsync(SEFQoSHandle qosHandle, struct SEFAllocateSuperBlockIOCB *iocb)
{
    struct SEFStatus status = {-1, 0};
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if ((qosHandle == NULL) || (iocb == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p, iocb=%p\n", qosHandle, iocb);
        if (iocb != NULL)
        {
            status.error = qosHandle == NULL ? -ENODEV : -EINVAL;
            goto SetStatus;
        }
        goto ExitProc;
    }

    iocb->common.status.info = 0;
    iocb->common.status.error = 0;
    iocb->common.flags &= ~kSefIoFlagDone;
    status = AllocateSbProc(false, qosHandle, &iocb->common);
    if (status.error != 0)
    {
    SetStatus:
        iocb->common.status = status;
        iocb->common.flags |= kSefIoFlagDone;
        if (iocb->common.complete_func != NULL)
        {
            iocb->common.complete_func(&iocb->common);
        }
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return;
}

/*
 *  @see    SEFAPI.h
 */
void SEFCloseSuperBlockAsync(SEFQoSHandle qosHandle, struct SEFCloseSuperBlockIOCB *iocb)
{
    struct SEFStatus status = {-1, 0};
    ULOG_ENTER_SEFAPI();

    // Check parameters
    if ((qosHandle == NULL) || (iocb == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p, iocb=%p\n", qosHandle, iocb);
        if (iocb != NULL)
        {
            status.error = qosHandle == NULL ? -ENODEV : -EINVAL;
            goto SetStatus;
        }
        goto ExitProc;
    }

    iocb->common.status.info = 0;
    iocb->common.status.error = 0;
    iocb->common.flags &= ~kSefIoFlagDone;
    status = CloseSbProc(false, qosHandle, &iocb->common, true);
    if (status.error != 0)
    {
    SetStatus:
        iocb->common.status = status;
        iocb->common.flags |= kSefIoFlagDone;
        if (iocb->common.complete_func != NULL)
        {
            iocb->common.complete_func(&iocb->common);
        }
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return;
}

/*
 *  @see    SEFAPI.h
 */
void SEFNamelessCopyAsync(SEFQoSHandle qosHandle, struct SEFNamelessCopyIOCB *iocb)
{
    struct SEFStatus status = {-1, 0};
    struct NlcQEntry *nlcEntry = NULL;
    bool kickstart = false;
    ULOG_ENTER_SEFAPI();

    // Check parameters (except for qosHandle, parameters common to sync/async are checked in CopyProc)
    if ((qosHandle == NULL) || (iocb == NULL))
    {
        ULOG_ERROR("argument error!! qosHandle=%p, iocb=%p\n", qosHandle, iocb);
        if (iocb != NULL)
        {
            status.error = qosHandle == NULL ? -ENODEV : -EINVAL;
            goto SetStatus;
        }
        goto ExitProc;
    }

    iocb->common.status.info = 0;
    iocb->common.status.error = 0;
    nlcEntry = malloc(sizeof(*nlcEntry));
    utl_DListInitEntry(&nlcEntry->link);
    nlcEntry->iocb = &iocb->common;
    nlcEntry->type = kQCopy;
    nlcEntry->sbid = GetSbId(qosHandle, iocb->copyDestination);
    pthread_mutex_lock(&qosHandle->mutex);
    kickstart = utl_DListIsEmpty(&qosHandle->nlcQueue);
    utl_DListPushTail(&qosHandle->nlcQueue, &nlcEntry->link);
    pthread_mutex_unlock(&qosHandle->mutex);
    status = (struct SEFStatus){0, 0};
    if (kickstart)
    {
        status = CopyProc(false, qosHandle, &iocb->common);
    }
    if (status.error != 0)
    {
        StartNextCopy(qosHandle);
    SetStatus:
        iocb->common.status = status;
        iocb->common.flags |= kSefIoFlagDone;
        if (iocb->common.complete_func != NULL)
        {
            iocb->common.complete_func(&iocb->common);
        }
    }

ExitProc:
    ULOG_LEAVE_SEFAPI();
    return;
}
