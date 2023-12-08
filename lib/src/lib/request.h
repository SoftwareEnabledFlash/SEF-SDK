/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * request.h
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

#ifndef __REQUEST_H__
#define __REQUEST_H__

#ifdef __cplusplus
#include <atomic>
using std::atomic_int;
#else /* not __cplusplus */
#ifdef __STDC_NO_ATOMICS__
#error C11 atomics are required
#endif /* __STDC_NO_ATOMICS__ */

#include <stdatomic.h>
#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include "SEFAPI.h"
#include "device_info.h"
#include "device_io.h"
#include "io_thread.h"    // for io_request
#include "nvme_ioctl.h"
#include "sef_api.h"
#include "ulog.h"

#define REQUEST_STATUS_TIMEOUT \
    (0xFFFFFFFF)    //!< Status in case of timeouts (more than 16bit to avoid overlap with StatusField)
#define REQUEST_MISMATCH_SBID \
    (0xFFFFFFFE)    //!< Status where SBID of result does not match with SBID of request
#define REQUEST_MEMORY_ERROR  (0xFFFFFFFD)    //!< Memory allocation failed
#define FLA_QOSD_ID_BIT_SHIFT (48)            //!< Bit location of QoS Domain ID in Flash Address
#define MAX_REQ_RETRY         (30000)         //!< Uring retries max count ~30 seconds
/**
 *  @brief	Retrieves ADU data size from SEF Unit information in SEF handle
 *  @param	[in] pSefHandle: address of SEF handle (struct SEFHandle_ *)
 *  @return	ADU size
 *  @note    ADUsize index for referencing SEF Unit information is fixed to 0
 */
#define GET_ADU_SIZE(pSefHandle) ((pSefHandle)->pSefInfo->ADUsize[0].data)

/**
 *  @brief	Retrieves ADU meta size from SEF Unit information in SEF handle
 *  @param	[in] pSefHandle: address of SEF handle (struct SEFHandle_ *)
 *  @return	ADU size
 *  @note    ADUsize index for referencing SEF Unit information is fixed to 0
 */
#define GET_META_SIZE(pSefHandle) ((pSefHandle)->pSefInfo->ADUsize[0].meta)

/**
 *  @brief	Structure of request information and completion information
 */
struct ReqToComplete
{
    struct io_request io_request;
    union RequestIocb *pIocb;             //!< [Req] IOCB defined in SEFAPI (can be NULL)
    enum IoctlCommandType type;           //!< [Req] SUBMIT type specified in ioctl()
    struct nvme_passthru_cmd64 submit;    //!< [Req] SUBMIT information specified in ioctl()
    bool bSync;                           //!< [Req] Synchronous flag (async if false)
    bool syncOverride;
    sem_t *pSyncSemaphore;    //!< [Req] Information for synchronous execution (NULL if async)
    void *param;              //!< [Req] For holding arbitrary parameter (NULL if not needed)
    void (*CommandComplete)(struct ReqToComplete *pReq2Cmp);    //!< [Req] Command completion callback
    void (*CheckResultExt)(struct ReqToComplete *pReq2Cmp);    //!< [Req] Check function for additional
                                                               //   check of other fields in case of Status=0
    bool (*CheckSuspendFunc)(struct ReqToComplete *pReq2Cmp);    //!< [Req] Check function for checking
                                                                 //   suspend of CommandComplete function
    struct ReqToComplete *fused;    //!< [Req] 2nd fused request if this is the 1st
    int retryCnt;                   //!< [Req] number of times retried
    int fd;                         //!< [Req] fd to used when resubmitting
    uint64_t result;                //!< [Cmp] Completion Queue Entry [DW1:DW0]
    uint32_t status;                //!< [Cmp] Execution result (StatusField of CQ
                                    //         except for internal errors)
};

/**
 *  @brief	Union of IOCB defined in SEFAPI
 *  @note    Top of IOCB is defined as common because it is common across all IOCBs
 */
union RequestIocb
{
    struct SEFCommonIOCB common;                        //!< Common parameters across all IOCBs
    struct SEFWriteWithoutPhysicalAddressIOCB write;    //!< IOCB for Write
    struct SEFReadWithPhysicalAddressIOCB read;         //!< IOCB for Read
    struct SEFReleaseSuperBlockIOCB releaseSb;          //!< IOCB for ReleaseSB
    struct SEFAllocateSuperBlockIOCB allocateSb;        //!< IOCB for AllocateSB
    struct SEFCloseSuperBlockIOCB closeSb;              //!< IOCB for CloseSB
    struct SEFNamelessCopyIOCB namelessCopy;            //!< IOCB for Copy
    struct FlushSuperBlockIOCB flushSb;                 //!< IOCB for FlushSB
    struct PatrolSuperBlockIOCB patrolSb;               //!< IOCB for PatrolSB
};

/**
 *  @brief	Structure for information used for issuing requests (without data)
 */
struct RequestInfo
{
    struct ReqToComplete *pReq2Cmp;    //!< Structure for request information and completion information
    union RequestIocb *pIocb;          //!< IOCB defined in SEFAPI
    void *pParam;                      //!< For holding arbitrary parameter
    sem_t *pSem;                       //!< Semaphore
};

/**
 *  @brief	Structure for information used for issuing requests (with data)
 */
struct RequestInfoExt
{
    struct ReqToComplete *pReq2Cmp;    //!< Structure for request information and completion information
    union RequestIocb *pIocb;          //!< IOCB defined in SEFAPI
    void *pParam;                      //!< For holding arbitrary parameter
    sem_t *pSem;                       //!< Semaphore
    void **ppDevBuff;                  //!< Buffer for device
    int buffSize;                      //!< Buffer size
};

/**
 *  @brief	Returns number of currently on-line (available) processors
 *  @return	Number of processors
 */
static inline int GetOnlineCpuNum(void)
{
    int num;
    cpu_set_t cs;
    if (sched_getaffinity(0, sizeof(cs), &cs))
    {
        ULOG_ERROR("Failed to get affinity, setting cpu num to 1");
        return 1;
    }

    num = CPU_COUNT_S(sizeof(cs), &cs);
    ULOG_SYSTEM("online cpu num %d\n", num);
    return num;
}

/**
 *  @brief	Returns page size of the system
 *  @return	Page size
 */
static inline uint32_t GetSysPageSize(void)
{
    uint32_t pageSize;
    pageSize = sysconf(_SC_PAGESIZE);
    ULOG_SYSTEM("system page size %d\n", pageSize);
    return pageSize;
}

/**
 *  @brief	Retrives ADU Offset in Flash Address
 *  @param	[in] pSefHandle: SEF handle
 *  @param	[in] fla: Flash Address
 *  @return	ADU Offset
 */
static inline uint32_t GetAduOffset(struct SEFQoSHandle_ *pQoSHandle, struct SEFFlashAddress fla)
{
    uint64_t mask = (((uint64_t)1) << pQoSHandle->aduOffsetBitLen) - 1;
    return (uint32_t)(fla.bits & mask);
}

/**
 *  @brief	Converts a flash address to a superblock address
 *  @param	[in] pSefHandle: address of SEF handle (struct SEFHandle_ *)
 *  @param	[in] fla: Flash address of any adu in a super block
 *  @return	Flash Address that references a super block
 */
static inline struct SEFFlashAddress GetSBAddress(struct SEFQoSHandle_ *pQoSHandle,
                                                  struct SEFFlashAddress fla)
{
    uint64_t aduMask = (UINT64_C(1) << pQoSHandle->aduOffsetBitLen) - 1;
    return (struct SEFFlashAddress){htole64(le64toh(fla.bits) & ~aduMask)};
}

/**
 *  @brief	Retrieves Flash Address using QoS Domain ID and Super Block ID
 *  @param	[in] pSefHandle: address of SEF handle (struct SEFHandle_ *)
 *  @param	[in] qosid: QoS Domain ID
 *  @param	[in] sbid : Super Block ID
 *  @return	Flash Address
 */
static inline struct SEFFlashAddress GetFlashAddress(struct SEFQoSHandle_ *pQoSHandle,
                                                     uint32_t qosid,
                                                     uint32_t sbid)
{
    struct SEFFlashAddress fla;
    if (sbid >= ((uint64_t)1 << (48 - pQoSHandle->aduOffsetBitLen)))
    {
        ULOG_ERROR("sbid too large for qosd %d: %d, max %ld\n", pQoSHandle->qosId, sbid,
                   (1UL << (48 - pQoSHandle->aduOffsetBitLen)) - 1);
        return SEFNullFlashAddress;
    }
    fla.bits = (((uint64_t)qosid << FLA_QOSD_ID_BIT_SHIFT) |
                ((uint64_t)sbid << pQoSHandle->aduOffsetBitLen));
    return fla;
}

/**
 *  @brief	Retrieves Super Block ID in Flash Address
 *  @param	[in] pSefHandle: address of SEF handle (struct SEFHandle_ *)
 *  @param	[in] fla: Flash Address
 *  @return	Super Block ID
 */
static inline uint32_t GetSbId(struct SEFQoSHandle_ *pQoSHandle, struct SEFFlashAddress fla)
{
    uint64_t mask = (((uint64_t)1) << pQoSHandle->sbIdBitLen) - 1;
    return (uint32_t)((fla.bits >> pQoSHandle->aduOffsetBitLen) & mask);
}

/**
 *  @brief	Retrieves QoS Domain ID in Flash Address
 *  @param	[in] fla: Flash Address
 *  @return	QoS Domain ID
 */
static inline uint16_t GetQosdId(struct SEFFlashAddress fla)
{
    return (uint16_t)(fla.bits >> FLA_QOSD_ID_BIT_SHIFT);
}

/*
 * function prototype
 */
void CompleteIocbProc(struct ReqToComplete *pReq2Cmp);
void Complete(struct ReqToComplete *pReq2Cmp);
int Request(int fd, struct ReqToComplete *pReq2Cmp);
int RequestFusedWrite(int fd, struct ReqToComplete *pReq2Cmp1st, struct ReqToComplete *pReq2Cmp2nd);
int RequestResubmit(struct ReqToComplete *pReq2Cmp);

size_t GetIovBuffLen(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs);
bool IsShortageIovLen(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, size_t size);
void GetIovOffsPosition(
    const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, uint16_t *pIndex, size_t *pOffs);
void CopyIovToBuff(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, size_t copyLen, void *pBuff);
void CopyIovFromBuff(const struct iovec *pIov, uint16_t iovCnt, size_t iovOffs, size_t copyLen, void *pBuff);
int CloneIov(const struct iovec *src, int32_t iovcnt, size_t offset, size_t dataSize, struct iovec **dest);
int GetSefUnitList(int *pNum, char ***pppList);

void *AllocateDeviceBuffer(uint32_t size, bool bZero);
int AllocateReqInfo(struct ReqToComplete **ppReq2Cmp,
                    union RequestIocb **ppIocb,
                    void **ppParam,
                    int paramSize,
                    sem_t **ppSem);
int AllocateReqInfoExt(struct ReqToComplete **ppReq2Cmp,
                       union RequestIocb **ppIocb,
                       void **ppParam,
                       int paramSize,
                       sem_t **ppSem,
                       void **ppDevBuff,
                       int buffSize);

void WaitAllRequestsComplete(struct SEFQoSHandle_ *pQosd);
void WaitAllRequestsCompleteForSef(struct SEFHandle_ *pSef);

#ifdef __cplusplus
}
#endif

#endif /* __REQUEST_H__ */
