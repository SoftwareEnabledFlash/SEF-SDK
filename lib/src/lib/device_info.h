/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * device_info.h
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

#ifndef __DEVICE_INFO_H__
#define __DEVICE_INFO_H__

#ifdef __cplusplus
#include <atomic>
using std::atomic_bool;
using std::atomic_int_least32_t;
using std::atomic_uint_least32_t;
#define _Atomic(t) std::atomic<t>
#else /* not __cplusplus */
#ifdef __STDC_NO_ATOMICS__
#error C11 atomics are required
#endif /* __STDC_NO_ATOMICS__ */

#include <stdatomic.h>
#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>

#include "SEFAPI.h"
#include "dlist.h"
#include "fpw-counter.h"
#include "locklessHash.h"
#include "sef-slist.h"

// #define USE_READ_CACHE
#ifdef USE_READ_CACHE
#include "locklessHash.h"
#endif

struct ReqToComplete;

/**
 *  @brief     Write status of Super Block
 */
enum SbWriteState {
    kSbNoWriting,      //!< No i/o in flight, no pending close
    kSbWriting,        //!< i/o is in flight
    kSbAllWritten,     //!< No i/o in flight, close notify should be sent
    kSbWriteClosed,    //!< No i/o sb is closed/closing
};

enum SbInfoState {
    kSbInfoOpen,            //!< SB is open
    kSbInfoClosePending,    //!< SB is closed (notification data is valid),
                            //   awaiting numIO to go to zero
    kSbInfoNotifying,       //!< Close notify is being sent
    kSbInfoNotified         //!< Close notify has been sent
};

union SbInfoAtomicState
{
    struct
    {
        uint32_t state : 2;     // See enum SbInfoState
        uint32_t numIO : 30;    // Number of inflight i/o
    };
    atomic_uint_least32_t val;
};

/**
 *  @brief Management area for flow control according to SB Write status
 */
struct SbWriteInformation
{
    struct sefSlistNode link;
    atomic_uint_least32_t refcnt;                  //!< Refcount for this structure
    atomic_uint_least32_t writtenAdus;             //!< Total ADUs written by user i/o
    uint32_t sbId;                                 //!< Super Block ID
    union SbInfoAtomicState state;                 //!< Super block state & i/o count
    sem_t closed;                                  //!< posted when closed
    struct SEFQoSNotification notificationData;    //!< For storing notification data
    struct sefSlist listHeadByOpen;       //!< List of responses suspended because SB is open
    struct sefSlist listHeadByWriting;    //!< List of responses suspended because SB is being written
};

/**
 * @brief State tracked per PID in SEFQoSHandle
 */
struct PidState
{
    struct FencedPendingWorkCounter numProcessingFAR;    //!< Number of FAR requests being processed
};

/**
 *  @brief	QoSD (QoS Domain) handle
 */
struct SEFQoSHandle_
{
    pthread_mutex_t mutex;       //!< Mutex for this handle
    uint16_t qosId;              //!< ID of this QoS Domain
    uint16_t numPlacementIds;    //!< Number of placement IDs
    int deviceFd;
    uint32_t aduOffsetBitLen;         //!< Bit length of ADU offset part in Flash Address
    uint32_t sbIdBitLen;              //!< Bit length of super block ID part in Flash Address
    struct SEFHandle_ *pSefHandle;    //!< Information of SEFUnit this QD belongs to
    uint32_t sbCapacity;              //!< Super block capacity in ADUs
    size_t maxBufSize;    //!< pageSize*(/sys/class/nvme/nvmeX/ngXnY/queue/max_segments-1)
    uint16_t vdId;        //!< ID of Virtual Device this QD belongs to
    int16_t sbSize;       //!< Super block size in dies
    int16_t ndie;         //!< Number of die in vd
    enum SEFDefectManagementMethod defectStrategy;            //!< Defect management method
    struct SEFProperty privateData;                           //!< Private data property
    void (*notifyFunc)(void *, struct SEFQoSNotification);    //!< Callback to execute in case of events
    void *pContext;
    atomic_uint_least32_t nlcid;    //!< Next nlcid to use for NLC commands
    atomic_uint_least32_t numProcessingRequests;    //!< Number of requests being processed (except those for NamelessWrite)
    atomic_uint_least32_t numProcessingNlw;    //!< Number of requests being processed (for NamelessWrite)
    atomic_bool bClosingFlag;    //!< true: QosD is being closed; false: not being closed
    LHThandle sbs_info;          //!< Head of list for flow control according to SB Write status
    struct sefSlist sbs_to_delete;
    atomic_int_least32_t sbs_ht_accesses;
    TmaDList nlcQueue;    //!< Queue of NLCs so they don't all submit at once (can't interleave segmented copies)
    atomic_uint_least32_t nlwid;
    struct sefSlist sbToDelete;
#ifdef USE_READ_CACHE
    LHThandle cache;
#endif
    uint8_t nrq;                              //!< Number of domain read queues
    uint32_t read_fifos[SEFMaxReadQueues];    //!< Map sef queue id to device fifo id
    struct PidState pidState[];
};

/**
 *  @brief	VD (Virtual Device) handle
 */
struct SEFVDHandle_
{
    uint16_t vdId;                    //!< ID of this Virtual Device
    struct SEFHandle_ *pSefHandle;    //!< Information of SEFUnit this VD belongs to
    void (*notifyFunc)(void *, struct SEFVDNotification);    //!< Callback to execute in case of events
    void *pContext;                                          //!< User context to pass to notifyFunc
    enum SEFDefectManagementMethod defectStrategy;           //!< Defect management information
    struct SEFDieList *pDieList;
};

/**
 *  @brief	SEF handle
 */
struct SEFHandle_
{
    pthread_mutex_t mutex;               //!< Mutex for this handle
    pthread_mutex_t vd_mutex;            //!< Mutex for ppVdHandle[]
    char *nvmeUnit;                      //!< Nvme unit name (e.g. nvme0)
    uint32_t deviceId;                   //!< ID of the device (for reference by ppDeviceName)
    int deviceFd;                        //!< File descriptor of the device
    uint32_t numPlanesPerDie;            //!< Planes per die
    uint32_t numADUsPerPlane;            //!< ADUs per plane
    struct SEFInfo *pSefInfo;            //!< SEF Unit information (retrieved from device)
    atomic_uint_least32_t numOpenVds;    //!< Open Virtual Devices
    uint32_t numOpenAllQosds;            //!< Open QoS Domains
    uint32_t numQosds;                   //!< Size of ppQosHandle
    uint32_t writeTimeout;               //!< Timeout in ms used by NLW and GCR
    atomic_uint_least32_t numProcessingRequests;    //!< Requests being processed
    struct SEFVDHandle_ **ppVdHandle;    //!< VD handle table (with number-of-channels x number-of-banks entries)
    struct SEFQoSHandle_ **ppQosHandle;    //!< QD handle table (with maximum-number-of-QoS-Domains entries)
    struct ReqToComplete *pReq2CmpSbStateChange;    //!< For SBStateChange notification
};

/**
 *  @brief	SEF Library management information
 */
struct SeflibInfo
{
    int numSefUnits;        //!< Number of SEF Units
    char **ppDeviceName;    //!< Device filename table (with numSefUnits entries)
    _Atomic(struct SEFHandle_ *) *ppSefHandle;    //!< SEF handle table (with numSefUnits entries)
};

/**
 *  @brief	System information used in SEF Library
 */
struct SystemInfo
{
    uint32_t transferDataAlign;    //!< Data alignment of buffer for sending/receiving data to/from device
    pthread_t asyncNotifyThreadId;    //!< Thread id of async notify thread
};

/*
 * function prototype
 */
int DeviceInfoInit(int numSefUnits, char **ppDeviceName);
void DeviceInfoCleanup(void);
int DeviceInfoCreateSefHandle(uint32_t index,
                              uint16_t numVds,
                              uint16_t numQosds,
                              uint16_t numAduSizes,
                              struct SEFHandle_ **ppSefHandle);
int DeviceInfoSetSefHandle(struct SEFHandle_ *pSefHandle);
void DeviceInfoDeleteSefHandle(struct SEFHandle_ *pSefHandle);
struct SEFHandle_ *DeviceInfoGetSefHandle(uint32_t index);
int DeviceInfoCreateVdHandle(struct SEFHandle_ *pSefHandle, uint16_t vdId, struct SEFVDHandle_ **ppVdHandle);
void DeviceInfoDeleteVdHandle(struct SEFVDHandle_ *pVdHandle);
struct SEFVDHandle_ *DeviceInfoGetVdHandle(struct SEFHandle_ *pSefHandle, uint16_t vdId);
int DeviceInfoCreateQosHandle(struct SEFHandle_ *pSefHandle,
                              uint16_t vdId,
                              uint16_t qosId,
                              uint16_t nPID,
                              struct SEFQoSHandle_ **ppQosHandle);
void DeviceInfoDeleteQosHandle(struct SEFQoSHandle_ *pQosHandle);
struct SEFQoSHandle_ *DeviceInfoGetQosHandle(struct SEFHandle_ *pSefHandle, uint16_t qosId);
struct SystemInfo *DeviceInfoGetSystemInfo(void);
char *DeviceInfoGetDeviceName(uint32_t index);
char *DeviceInfoGetDevicePath(uint32_t index);
int DeviceInfoGetNumSefUnits(void);

int DeviceInfoAddOpenInfoForSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);
void DeviceInfoUpdateWriteInfoForSb(struct SbWriteInformation *sb_info,
                                    int refAdj,
                                    int numADUs,
                                    enum SbWriteState *pState);
int DeviceInfoUpdateSbClosed(struct SbWriteInformation *sb_info,
                             struct SEFQoSNotification *pNotify,
                             enum SbWriteState *pState);
struct SbWriteInformation *DeviceInfoGetSbWriteInfoList(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);
struct SbWriteInformation *DeviceInfoPopSbWriteInfoList(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);

void DeviceInfoSbCleanup(struct SEFQoSHandle_ *pQosHandle);

struct SbWriteInformation *DeviceInfoGetSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId, bool create);
void DeviceInfoReleaseSb(struct SEFQoSHandle_ *pQosHandle, struct SbWriteInformation *sbInfo);
void DeviceInfoDeleteSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);
void DeviceInfoDeletePendingIf(struct SEFQoSHandle_ *pQosHandle, struct sefSlistNode *head);

void DeviceInfoAddSuspendListByOpen(struct SEFQoSHandle_ *pQosHandle,
                                    uint32_t sbId,
                                    struct ReqToComplete *pReq2Cmp);
void DeviceInfoAddSuspendListByWriting(struct SEFQoSHandle_ *pQosHandle,
                                       uint32_t sbId,
                                       struct ReqToComplete *pReq2Cmp);
bool DeviceInfoIsOpenedSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);
bool DeviceInfoIsClosingSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);
bool DeviceInfoIsWritingSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);
bool DeviceInfoIsClosedSb(struct SEFQoSHandle_ *pQosHandle, uint32_t sbId);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_INFO_H__ */
