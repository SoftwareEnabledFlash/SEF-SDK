/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_cmd.h
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

#ifndef __SEF_CMD_H__
#define __SEF_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "SEFAPI.h"
#include "device_io.h"
#include "nvme_ioctl.h"

#define SEF_CMD_DWORD     (4)    //!< Number of bytes in DWORD
#define SEF_CMD_ZERO_BASE (1)    //!< For converting zero-base

// FORMAT : "X:COMMAND"
// X meaning is command type (A:kIoctlAdminCommand, I:kIoctlIoCommand, F:kIoctlIoFusedCommand)
#define CMD_A_IDENTIFY                  "A:Identify"
#define CMD_A_CAP_MGMT                  "A:CapMgmt(Select)"
#define CMD_A_QOSD_DELETE               "A:NsMgmt(QoSDMng,Delete)"
#define CMD_A_QOSD_CREATE               "A:NsMgmt(QoSDMng,Create)"
#define CMD_A_NS_ATTACH                 "A:NsAttach(NsAttach,Attach)"
#define CMD_A_NS_DETACH                 "A:NsAttach(NSAttach,Detach)"
#define CMD_A_GLOGP_VDINFO              "A:GetLogPage(VDInfo)"
#define CMD_A_GLOGP_QOSDLIST            "A:GetLogPage(QoSDList)"
#define CMD_A_GLOGP_SBLIST_WEARLVCLOSE  "A:GetLogPage(SBList,WearLevelingClosed)"
#define CMD_A_GLOGP_SBLIST_REFRESHCLOSE "A:GetLogPage(SBList,ToRefreshClosed)"
#define CMD_A_GLOGP_SBLIST_CHECKCLOSE   "A:GetLogPage(SBList,ToCheckClosed)"
#define CMD_A_GLOGP_SBLIST_OPENCLOSED   "A:GetLogPage(SBList,OpenClosed)"
#define CMD_A_GLOGP_SBINFO              "A:GetLogPage(SBInfo)"
#define CMD_A_GLOGP_USERADDRLIST        "A:GetLogPage(UserAddressList)"
#define CMD_A_GLOGP_ACO                 "A:GetLogPage(AdressChangeOrder)"
#define CMD_A_GFEAT_ERRRECOVERY         "A:GetFeat(ErrorRecovery)"
#define CMD_A_GFEAT_QOSD_INFO           "A:GetFeat(QoSDMng,GetInformation)"
#define CMD_A_SFEAT_ERRRECOVERY         "A:SetFeat(ErrorRecovery)"
#define CMD_A_SFEAT_VD_SET_NUM_DIES     "A:SetFeat(VDManag,SetNumDies"
#define CMD_A_SFEAT_VD_ATTACH_RFIFO     "A:SetFeat(VDManag,AttachReadFifo"
#define CMD_A_SFEAT_VD_CHANGE_RFIFO     "A:SetFeat(VDManag,ChangeReadFifo"
#define CMD_A_SFEAT_VD_DETACH_RFIFO     "A:SetFeat(VDManag,DetachReadFifo"
#define CMD_A_SFEAT_VD_SET_NUM_PSLC     "A:SetFeat(VDManag,SetNumPSLC"
#define CMD_A_SFEAT_VD_CONTROL_AEN      "A:SetFeat(VDManag,ControlAEN"
#define CMD_A_SFEAT_VD_RESET_SCHED      "A:SetFeat(VDManag,ResetScheduler"
#define CMD_A_SFEAT_VD_SET_SUSPEND_CONF "A:SetFeat(VDManag,SetSuspendConf)"
#define CMD_A_SFEAT_QOSD_SETROOTPTR     "A:SetFeat(QoSDMng,SetRootPointer)"
#define CMD_A_SFEAT_QOSD_CHANGECAP      "A:SetFeat(QoSDMng,ChangeCapacity)"
#define CMD_A_SFEAT_QOSD_CHANGEWEIGHTS  "A:SetFeat(QoSDMng,ChangeWeights)"
#define CMD_A_SFEAT_QOSD_CHANGERDDEAD   "A:SetFeat(QoSDMng,ChangeReadDeadline)"
#define CMD_A_SFEAT_QOSD_CHANGEMAXSBS   "A:SetFeat(QoSDMng,ChangeMaxSbs)"
#define CMD_A_SFEAT_QOSD_DEFREADFIFO    "A:SetFeat(QoSDMng,ChangeDefReadFifo)"
#define CMD_A_SFEAT_CAP_CFG_REG         "A:SetFeat(CapCfg,Register)"
#define CMD_I_SBMNG_ERASE               "I:SBMng(Erase)"
#define CMD_I_SBMNG_ERASEFORCOPY        "I:SBMng(EraseForNamelessCopy)"
#define CMD_I_SBMNG_CLOSE               "I:SBMng(Close)"
#define CMD_I_SBMNG_FREE                "I:SBMng(Free)"
#define CMD_I_SBMNG_PATROL              "I:SBMng(Patrol)"
#define CMD_I_SBMNG_FLUSH               "I:SBMng(Flush)"
#define CMD_F_FLASHADDRREQ              "F:FlashAddressRequest"
#define CMD_F_NAMELESSWRITE             "F:NamelessWrite"
#define CMD_I_READ                      "I:Read"
#define CMD_F_NAMELESSCOPY              "F:NamelessCopy"
#define CMD_F_GETCOPYRESULT             "F:GetCopyResult"

// Admin Command Opecode
#define SEF_ADMOPC_GETLOGPAGE       0x02    //!< Get Log Page
#define SEF_ADMOPC_CMDSPEC_IDENTIFY 0x05    //!< Identify
#define SEF_ADMOPC_IDENTIFY         0x06    //!< Identify
#define SEF_ADMOPC_SETFEATURE       0x09    //!< Set Features
#define SEF_ADMOPC_GETFEATURE       0x0A    //!< Get Features
#define SEF_ADMOPC_NS_MGMT          0x0D    //!< Namespace Management
#define SEF_ADMOPC_NS_ATTACH        0x15    //!< Namespace Attachent

// Get Log Page : Log Identifire
#define SEF_LPID_VD_INFORMATION        0xD0    //!< SEF Unit Information
#define SEF_LPID_QOS_DOMAINLIST        0xD1    //!< QoS Domain List
#define SEF_LPID_SUPERBLOCKLIST        0xD2    //!< Super Block List
#define SEF_LPID_SUPERBLOCKINFORMATION 0xD3    //!< Super Block Information
#define SEF_LPID_USER_ADDRESSLIST      0xD4    //!< User Address List
#define SEF_LPID_ADDRESSCHANGEORDER    0xD5    //!< Super Block State Change

// Get Log Page Super Block List : List Order
#define SEF_LODR_CLOSE_WEARLEVEL 0x0    //!< Get Close SB List for Wear Leveling
#define SEF_LODR_CLOSE_REFRESH   0x1    //!< Get Close SB List for Garbage Collection
#define SEF_LODR_CLOSE_CHECK     0x2    //!< Get Close SB List for Check Data Retention
#define SEF_LODR_OPEN_CLOSED     0x3    //!< Get Open SB and Closed List
#define SEF_LODR_CLOSED          0x4    //!< Get Closed SB List by SBID

// Set/Get Features : Feature Identifier
#define SEF_FID_ERROR_RECOVERY          0x05    //!< Error Recovery
#define SEF_FID_VIRTUALDEVOCEMANAGEMENT 0xD0    //!< Virtual DeviceManagement
#define SEF_FID_QOSDOMAINMANAGEMENT     0xD1    //!< QoS DomainManagement
#define SEF_FID_CAP_CFG_REGISTER        0xD2    //!< Register Capacity Configuration

// Set Features QoSD Management : Select
#define SEF_FEATURE_ROOTPOINTER      0x0    //!< Set a Root Pointer
#define SEF_FEATURE_CHG_CAPACITY     0x1    //!< Change Capacity
#define SEF_FEATURE_CHG_WEIGHTS      0x2    //!< Change Capacity
#define SEF_FEATURE_CHG_READDEADLINE 0x3    //!< Change Read Dead-line
#define SEF_FEATURE_CHG_MAXSBS       0x4    //!< Change Capacity
#define SEF_FEATURE_DEFAULT_RFIFO    0x5    //!< Change Default Read Fifo

// Get Features VD Management : Select
#define SEF_FEATURE_SEL_SET_NUM_DIES     0x0    //!< Set Number of Dies
#define SEF_FEATURE_SEL_ATTACH_RFIFO     0x1    //!< Attach Read Fifo
#define SEF_FEATURE_SEL_CHANGE_RFIFO     0x2    //!< Change Read Fifo
#define SEF_FEATURE_SEL_DETACH_RFIFO     0x3    //!< Detach Read Fifo
#define SEF_FEATURE_SEL_SET_NUM_PSLC     0x4    //!< Set Number of pSLC blocks
#define SEF_FEATURE_SEL_CTRL_ASYNC_EVENT 0x5    //!< Control Asynchronous Event
#define SEF_FEATURE_SEL_SET_SUSPEND_CONF 0x6    //!< Set Suspend Config
#define SEF_FEATURE_RESET_SCHEDULER      0xf    //!< Reset Scheduler

/* Critical warning bit masks */
#define SEF_CAE_OUT_OF_BLOCKS 1
#define SEF_CAE_OUT_OF_PSLC   2
#define SEF_CAE_CAP_REDUCED   4
#define SEF_CAE_WARN_MASK     (SEF_CAE_OUT_OF_BLOCKS | SEF_CAE_OUT_OF_PSLC | SEF_CAE_CAP_REDUCED)

// Get Features QoSD Management : Select
#define SEF_FEATURE_SEL_GETREADFIFOS 0x1    //!< Get Information

// Capacity Configuration Management
#define NVME_CAPACITY_MANAGEMENT 0x20

// Nvme Error codes
#define NVME_CAP_EXCEEDED 0x081
#define SEF_PARTIAL_WRITE 0x7f0

// Set/Get Features Change Read Dead Line : Read Dead-line
enum ReadDeadlineForDev {
    kReadDeadlineForDevTypical = 0x0,    //!< Typical
    kReadDeadlineForDevLong = 0x1,       //!< Long
    kReadDeadlineForDevHeroic = 0x2,     //!< Heroic
    kReadDeadlineForDevFastest = 0x3     //!< Fastest
};

enum DefectStrategyForDev {
    kDevPerfect = 0x0,       //!< Perfect
    kDevPacked = 0x1,        //!< Packed
    kDevFragmented = 0x2,    //!< Fragmented
};

// Nvm Command Opecode
#define SEF_NVMOPC_SUPERBLOCKMANAGEMENT 0xDA    //!< Super Block Management
#define SEF_NVMOPC_NAMELESSWRITE        0xD1    //!< Nameless Write
#define SEF_NVMOPC_FLASHADDRESSREQUEST  0xD2    //!< Flash Address Request
#define SEF_NVMOPC_READ                 0xD6    //!< Read
#define SEF_NVMOPC_NAMELESSCOPY         0xDD    //!< Nameless Copy
#define SEF_NVMOPC_GETCOPYRESULT        0xDE    //!< Nameless Copy Result

// Super Block Management : Operation
#define SEF_NVMOPE_SBMANAGEMENT_ERASE  0x0    //!< Erase
#define SEF_NVMOPE_SBMANAGEMENT_CLOSE  0x1    //!< Close
#define SEF_NVMOPE_SBMANAGEMENT_FREE   0x2    //!< Free
#define SEF_NVMOPE_SBMANAGEMENT_PATROL 0x3    //!< Patrol
#define SEF_NVMOPE_SBMANAGEMENT_FLUSH  0x4    //!< Flush
#define SEF_NVMOPE_SBMANAGEMENT_PERASE 0x8    //!< pSLC Erase

#define SEF_ID_NS_CSI 0x30

/*
 * function prototype
 */
const char *GetCmdName(enum IoctlCommandType type, struct nvme_passthru_cmd64 *pSubmitInfo);

void SEFAdmGetLogPageVDInfo(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t vdid);
void SEFAdmGetLogPageQoSDList(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize);
void SEFAdmGetLogPageSBList(struct nvme_passthru_cmd64 *cmd,
                            void *data,
                            uint32_t dataSize,
                            uint32_t QoSDId,
                            uint8_t listOrder,
                            uint32_t offset);
void SEFAdmGetLogPageSBInfo(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t VDId, uint32_t SBId);
void SEFAdmGetLogPageUserAddrList(struct nvme_passthru_cmd64 *cmd,
                                  void *data,
                                  uint32_t dataSize,
                                  uint32_t QoSDId,
                                  uint32_t SBId,
                                  uint64_t offset);
void SEFAdmGetLogPageAco(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t QoSDId, uint16_t acoid);

void SEFAdmIdentify(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize);
void SEFAdmSefNsIdentify(struct nvme_passthru_cmd64 *cmd, uint32_t nsid, void *data, uint32_t dataSize);
void SEFAdmNamespaceManagement(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t qd_id, bool create);
void SEFAdmNameSpaceAttachment(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t qd_id, bool attach);

void SEFAdmSetFeatureCapCfgReg(
    struct nvme_passthru_cmd64 *cmd, void *data, uint16_t egcn, uint16_t tchmus, uint32_t dataSize);
void SEFAdmCapacityManagement(struct nvme_passthru_cmd64 *cmd, uint32_t cfg_id);
void SEFAdmSetFeatureVDManagement(struct nvme_passthru_cmd64 *cmd, uint8_t SEL, uint32_t VDId);
void SEFAdmSetFeatureVDSetNumDies(struct nvme_passthru_cmd64 *cmd, uint16_t vdid, uint16_t numDie);
void SEFAdmSetFeatureVDAttachReadFifo(
    struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t rfID, uint16_t weight, bool isDefault);
void SEFAdmSetFeatureVDChangeReadFifo(
    struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t rfID, uint16_t weight, bool isDefault);
void SEFAdmSetFeatureVDDetachReadFifo(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t rfID);
void SEFAdmSetFeatureVDSetNumPSLC(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t numPSLC);
void SEFAdmSetFeatureVDControlAEN(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint8_t cwc);
void SEFAdmSetFeatureVDSetSuspendConfig(struct nvme_passthru_cmd64 *cmd,
                                        uint16_t vdID,
                                        uint32_t maxTimePerSuspend,
                                        uint32_t minTimeUntilSuspend,
                                        uint32_t maxSuspendInterval);
void SEFAdmSetFeatureQoSDManagement(struct nvme_passthru_cmd64 *cmd,
                                    uint8_t SEL,
                                    uint32_t QoSDId,
                                    uint32_t dword1,
                                    uint32_t dword1shift,
                                    uint64_t qword1,
                                    uint64_t qword2);
void SEFAdmSetFeatureQoSDSetRootPointer(struct nvme_passthru_cmd64 *cmd,
                                        uint32_t QoSDId,
                                        uint32_t rootPointerIndex,
                                        uint64_t rootPointer);
void SEFAdmSetFeatureQoSDChangeCapacity(
    struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, bool pSLC, uint64_t gcap, uint64_t quota);
void SEFAdmSetFeatureQoSDChangeWeights(struct nvme_passthru_cmd64 *cmd,
                                       uint32_t QoSDId,
                                       uint16_t wwt,
                                       uint16_t ewt);
void SEFAdmSetFeatureQoSDChangeReadDeadline(struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, uint8_t rdl);
void SEFAdmSetFeatureQoSDChangeMaxSBS(struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, uint32_t maxosbs);
void SEFAdmSetFeatureQoSDChangeDefRFifo(struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, uint32_t rfid);
void SEFAdmGetFeatureVDManagement(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint8_t SEL, uint32_t VDId);
void SEFAdmGetFeatureQoSDManagement(struct nvme_passthru_cmd64 *cmd,
                                    void *data,
                                    uint32_t dataSize,
                                    uint8_t SEL,
                                    uint8_t SSEL,
                                    uint32_t QoSDId);

void SEFNvmSuperBlockManagement(struct nvme_passthru_cmd64 *cmd,
                                void *data,
                                uint32_t dataSize,
                                uint8_t operation,
                                uint32_t QoSDId,
                                uint32_t SBId,
                                const struct SEFAllocateOverrides *overrides,
                                uint32_t timeout);
void SEFNvmFlashAddressRequest(struct nvme_passthru_cmd64 *cmd,
                               void *data,
                               uint32_t dataSize,
                               uint32_t QoSDId,
                               uint64_t flashAddress,
                               struct SEFPlacementID placementId,
                               uint16_t eraseWeight,
                               uint32_t nlwid);
void SEFNvmNamelessWriteCommand(struct nvme_passthru_cmd64 *cmd,
                                const struct iovec *iovec,
                                uint32_t nvecs,
                                const void *metadata,
                                uint32_t QoSDId,
                                uint32_t sbid,
                                uint8_t aduBits,
                                struct SEFUserAddress userAddress,
                                uint32_t numAdu,
                                struct SEFPlacementID placementId,
                                uint16_t programWeight,
                                uint32_t nlwid,
                                bool forceUnitAccess,
                                uint32_t timeout,
                                bool pSLC);
void SEFNvmReadCommand(struct nvme_passthru_cmd64 *cmd,
                       const struct iovec *iovec,
                       uint32_t iovcnt,
                       void *metadata,
                       uint32_t QoSDId,
                       struct SEFUserAddress userAddress,
                       uint32_t numAdu,
                       struct SEFFlashAddress flashAddress,
                       uint32_t fifo,
                       uint16_t ioWeight);
void SEFNvmNamelessCopy(struct nvme_passthru_cmd64 *cmd,
                        void *data,
                        uint32_t dataSize,
                        uint32_t QoSDId,
                        struct SEFCopySource *copySrc,
                        const struct SEFUserAddressFilter *filter,
                        const struct SEFCopyOverrides *overrides,
                        uint32_t nlcid,
                        struct SEFFlashAddress dstFlashAddress,
                        uint32_t timeout);
void SEFNvmGetCopyResult(struct nvme_passthru_cmd64 *cmd,
                         void *data,
                         uint32_t dataSize,
                         uint32_t QoSDId,
                         int fua,
                         uint32_t numAddressChangeEntries,
                         uint32_t nlcid,
                         uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __SEF_CMD_H__ */
