/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_cmd.c
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

#define MODULE_NAME_SELECT "SEFCMD"

#include "sef_cmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

/**
 *  @brief	Retrieves information to identify command name (command-independent information)
 *  @param	[in] type: command type to issue at ioctl()
 *  @param	[in] opc: opecode to issue at ioctl()
 *  @return	Information to identify command name
 *  @note	If command-specific information is needed, use in conjunction with @see CMD_SPECIFIC
 *  @note	Bit assignment of command-independent information is as follows (does not overlap with
 * command-specific information):
 *  @note	[31:24] type
 *  @note	[23:16] opcode
 */
#define CMD_INFO(type, opc) (((type) << ((uint32_t)24)) | ((opc) << ((uint32_t)16)))

/**
 *  @brief	Retrieves information to identify command name (command-specific information)
 *  @param	[in] spe1: additional command information 1 (varies for each ope)
 *  @param	[in] spe2: additional command information 2 (varies for each ope)
 *  @return	Information to identify command name
 *  @note	Use in conjunction with @see CMD_INFO
 *  @note	Bit assignment of command-specific information is as follows (does not overlap with
 * command-independent information):
 *  @note	[15:08] specific1
 *  @note	[07:00] specific2
 */
#define CMD_SPECIFIC(spe1, spe2) (((spe1) << ((uint32_t)8)) | (spe2))

#define GLOGP_INFO(spe1, spe2) \
    (CMD_INFO(kIoctlAdminCommand, SEF_ADMOPC_GETLOGPAGE) | CMD_SPECIFIC(spe1, spe2))
#define GFEAT_INFO(spe1, spe2) \
    (CMD_INFO(kIoctlAdminCommand, SEF_ADMOPC_GETFEATURE) | CMD_SPECIFIC(spe1, spe2))
#define SFEAT_INFO(spe1, spe2) \
    (CMD_INFO(kIoctlAdminCommand, SEF_ADMOPC_SETFEATURE) | CMD_SPECIFIC(spe1, spe2))
#define SBMNG_INFO(spe1, spe2) \
    (CMD_INFO(kIoctlIoCommand, SEF_NVMOPC_SUPERBLOCKMANAGEMENT) | CMD_SPECIFIC(spe1, spe2))

/**
 *  @brief	Retrieves command name
 *  @param	[in] type: command type to issue at ioctl()
 *  @param	[in] pSubmitInfo: command information to issue at ioctl()
 *  @return	command name
 */
const char *GetCmdName(enum IoctlCommandType type, struct nvme_passthru_cmd64 *pSubmitInfo)
{
    uint8_t opcode = pSubmitInfo->opcode;
    uint32_t cmd = CMD_INFO(type, opcode);    // command info without specific
    uint32_t cmdInfo = cmd;

    // A:GetLogPage specific
    if (cmd == GLOGP_INFO(0, 0))
    {
        uint8_t logPageId = pSubmitInfo->cdw10 & 0xFF;
        if (logPageId == SEF_LPID_SUPERBLOCKLIST)
        {
            uint8_t listOrder = (pSubmitInfo->cdw10 >> 12) & 0x07;
            cmdInfo |= CMD_SPECIFIC(logPageId, listOrder);
        }
        else
        {
            cmdInfo |= CMD_SPECIFIC(logPageId, 0);
        }
        // A:Set/GetFeautre specific
    }
    else if (cmd == GFEAT_INFO(0, 0) || cmd == SFEAT_INFO(0, 0))
    {
        uint8_t featureId = pSubmitInfo->cdw10 & 0xFF;
        if (featureId == SEF_FID_ERROR_RECOVERY)
        {
            cmdInfo |= CMD_SPECIFIC(featureId, 0);
        }
        else
        {
            uint8_t select = pSubmitInfo->cdw11 & 0x0F;
            cmdInfo |= CMD_SPECIFIC(featureId, select);
        }
        // I:SbManage specific
    }
    else if (cmd == SBMNG_INFO(0, 0))
    {
        uint8_t operation = (pSubmitInfo->cdw11 >> 16) & 0x0F;
        cmdInfo |= CMD_SPECIFIC(operation, 0);
    }

    switch (cmdInfo)
    {
        case CMD_INFO(kIoctlAdminCommand, SEF_ADMOPC_IDENTIFY):
            return CMD_A_IDENTIFY;
        case CMD_INFO(kIoctlAdminCommand, SEF_ADMOPC_NS_MGMT):
            return CMD_A_QOSD_CREATE;
        case CMD_INFO(kIoctlAdminCommand, SEF_ADMOPC_NS_ATTACH):
            return CMD_A_NS_ATTACH;
        case GLOGP_INFO(SEF_LPID_QOS_DOMAINLIST, 0):
            return CMD_A_GLOGP_QOSDLIST;
        case GLOGP_INFO(SEF_LPID_VD_INFORMATION, 0):
            return CMD_A_GLOGP_VDINFO;
        case GLOGP_INFO(SEF_LPID_SUPERBLOCKLIST, SEF_LODR_CLOSE_WEARLEVEL):
            return CMD_A_GLOGP_SBLIST_WEARLVCLOSE;
        case GLOGP_INFO(SEF_LPID_SUPERBLOCKLIST, SEF_LODR_CLOSE_REFRESH):
            return CMD_A_GLOGP_SBLIST_REFRESHCLOSE;
        case GLOGP_INFO(SEF_LPID_SUPERBLOCKLIST, SEF_LODR_CLOSE_CHECK):
            return CMD_A_GLOGP_SBLIST_CHECKCLOSE;
        case GLOGP_INFO(SEF_LPID_SUPERBLOCKLIST, SEF_LODR_OPEN_CLOSED):
            return CMD_A_GLOGP_SBLIST_OPENCLOSED;
        case GLOGP_INFO(SEF_LPID_SUPERBLOCKINFORMATION, 0):
            return CMD_A_GLOGP_SBINFO;
        case GLOGP_INFO(SEF_LPID_USER_ADDRESSLIST, 0):
            return CMD_A_GLOGP_USERADDRLIST;
        case GLOGP_INFO(SEF_LPID_ADDRESSCHANGEORDER, 0):
            return CMD_A_GLOGP_ACO;
        case GFEAT_INFO(SEF_FID_ERROR_RECOVERY, 0):
            return CMD_A_GFEAT_ERRRECOVERY;
        case SFEAT_INFO(SEF_FID_ERROR_RECOVERY, 0):
            return CMD_A_SFEAT_ERRRECOVERY;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_SEL_SET_NUM_DIES):
            return CMD_A_SFEAT_VD_SET_NUM_DIES;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_SEL_ATTACH_RFIFO):
            return CMD_A_SFEAT_VD_ATTACH_RFIFO;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_SEL_CHANGE_RFIFO):
            return CMD_A_SFEAT_VD_CHANGE_RFIFO;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_SEL_DETACH_RFIFO):
            return CMD_A_SFEAT_VD_DETACH_RFIFO;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_SEL_SET_NUM_PSLC):
            return CMD_A_SFEAT_VD_SET_NUM_PSLC;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_SEL_CTRL_ASYNC_EVENT):
            return CMD_A_SFEAT_VD_CONTROL_AEN;
        case SFEAT_INFO(SEF_FID_VIRTUALDEVOCEMANAGEMENT, SEF_FEATURE_RESET_SCHEDULER):
            return CMD_A_SFEAT_VD_RESET_SCHED;
        case SFEAT_INFO(SEF_FID_QOSDOMAINMANAGEMENT, SEF_FEATURE_ROOTPOINTER):
            return CMD_A_SFEAT_QOSD_SETROOTPTR;
        case SFEAT_INFO(SEF_FID_QOSDOMAINMANAGEMENT, SEF_FEATURE_CHG_CAPACITY):
            return CMD_A_SFEAT_QOSD_CHANGECAP;
        case SFEAT_INFO(SEF_FID_QOSDOMAINMANAGEMENT, SEF_FEATURE_CHG_WEIGHTS):
            return CMD_A_SFEAT_QOSD_CHANGEWEIGHTS;
        case SFEAT_INFO(SEF_FID_QOSDOMAINMANAGEMENT, SEF_FEATURE_CHG_READDEADLINE):
            return CMD_A_SFEAT_QOSD_CHANGERDDEAD;
        case SFEAT_INFO(SEF_FID_QOSDOMAINMANAGEMENT, SEF_FEATURE_CHG_MAXSBS):
            return CMD_A_SFEAT_QOSD_CHANGEMAXSBS;
        case SFEAT_INFO(SEF_FID_QOSDOMAINMANAGEMENT, SEF_FEATURE_DEFAULT_RFIFO):
            return CMD_A_SFEAT_QOSD_DEFREADFIFO;
        case SFEAT_INFO(SEF_FID_CAP_CFG_REGISTER, 0):
            return CMD_A_SFEAT_CAP_CFG_REG;
        case SBMNG_INFO(SEF_NVMOPE_SBMANAGEMENT_ERASE, 0):
            return CMD_I_SBMNG_ERASE;
        case SBMNG_INFO(SEF_NVMOPE_SBMANAGEMENT_CLOSE, 0):
            return CMD_I_SBMNG_CLOSE;
        case SBMNG_INFO(SEF_NVMOPE_SBMANAGEMENT_FREE, 0):
            return CMD_I_SBMNG_FREE;
        case SBMNG_INFO(SEF_NVMOPE_SBMANAGEMENT_PATROL, 0):
            return CMD_I_SBMNG_PATROL;
        case SBMNG_INFO(SEF_NVMOPE_SBMANAGEMENT_FLUSH, 0):
            return CMD_I_SBMNG_FLUSH;
        case CMD_INFO(kIoctlIoCommand, SEF_NVMOPC_FLASHADDRESSREQUEST):
        case CMD_INFO(kIoctlIoFusedCommand, SEF_NVMOPC_FLASHADDRESSREQUEST):
            return CMD_F_FLASHADDRREQ;
        case CMD_INFO(kIoctlIoCommand, SEF_NVMOPC_NAMELESSWRITE):
        case CMD_INFO(kIoctlIoFusedCommand, SEF_NVMOPC_NAMELESSWRITE):
            return CMD_F_NAMELESSWRITE;
        case CMD_INFO(kIoctlIoCommand, SEF_NVMOPC_READ):
            return CMD_I_READ;
        case CMD_INFO(kIoctlIoCommand, SEF_NVMOPC_NAMELESSCOPY):
        case CMD_INFO(kIoctlIoFusedCommand, SEF_NVMOPC_NAMELESSCOPY):
            return CMD_F_NAMELESSCOPY;
        case CMD_INFO(kIoctlIoCommand, SEF_NVMOPC_GETCOPYRESULT):
        case CMD_INFO(kIoctlIoFusedCommand, SEF_NVMOPC_GETCOPYRESULT):
            return CMD_F_GETCOPYRESULT;
        default:
            return "Unknown Command";
    }
}

/**
 *  @brief	Generates command information of Admin Command Get Log Page
 *  @param	[in/out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] logPageId: Log Page Identifier
 *  @return	None
 *  @note    Set Retain Asynchronous Event to 0
 */
STATIC void AdmGetLogPage(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint8_t logPageId)
{
    uint32_t NmofDwSize = (dataSize / SEF_CMD_DWORD) - SEF_CMD_ZERO_BASE;

    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_ADMOPC_GETLOGPAGE;    // Opcode
    cmd->addr = (__u64)data;                // Data Pointer
    cmd->data_len = dataSize;               // Data Length for SEF Driver
    cmd->cdw10 |= logPageId;                // [CDW10][Bit07:00] Log Page Identifier
    cmd->cdw10 |= (0 << 15);                // [CDW10][Bit  :15] Retain Asynchronous Event
    cmd->cdw10 |= (NmofDwSize << 16);       // [CDW10][Bit31:16] Number of Dwords Lower
    cmd->cdw11 |= (NmofDwSize >> 16);       // [CDW11][Bit15:00] Number of Dwords
    cmd->cdw12 |= 0;                        // [CDW12][Bit31:00] Log Page Offset Lower
    cmd->cdw13 |= 0;                        // [CDW13][Bit31:00] Log Page Offset Upper
}

void SEFAdmGetLogPageAco(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t QoSDId, uint16_t acoid)
{
    AdmGetLogPage(cmd, data, dataSize, SEF_LPID_ADDRESSCHANGEORDER);

    // Specific
    cmd->nsid = QoSDId;
    cmd->cdw12 = acoid;
}

/**
 *  @brief	Generates command information of Admin Command Get Log Page
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] VDId: VirtualDevice ID
 *  @param	[in] SBId: Super Block ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Log Page Identifier : D3h (Super Block Information)
 *  @details	Target SEFAPI
 *  @details	- SEFGetVirtualDeviceInformation()
 *  @details	- SEFGetQoSDomainInformation()
 *  @details	- SEFGetSuperBlockInfo()
 */
void SEFAdmGetLogPageVDInfo(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t VDId)
{
    AdmGetLogPage(cmd, data, dataSize, SEF_LPID_VD_INFORMATION);

    // Specific
    cmd->nsid = 0;              // Namespace Identifier : Virtual Device ID
    cmd->cdw11 = VDId << 16;    // [CDW12][Bit31:00] Log Page Offset Lower : Super Block ID
}

/**
 *  @brief	Generates command information of Admin Command Get Log Page
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Log Page Identifier : D1h (QoS Domain List)
 *  @details	Target SEFAPI
 *  @details	- SEFListQoSDomains()
 */
void SEFAdmGetLogPageQoSDList(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize)
{
    AdmGetLogPage(cmd, data, dataSize, SEF_LPID_QOS_DOMAINLIST);
}

/**
 *  @brief	Generates command information of Admin Command Get Log Page
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] listOrder: List Order
 *  @param  [in] offset: Log Page Offse lower
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Log Page Identifier : D2h (Super Block List)
 *  @details	Target SEFAPI
 *  @details	- SEFGetQoSDomainInformation()
 */
void SEFAdmGetLogPageSBList(struct nvme_passthru_cmd64 *cmd,
                            void *data,
                            uint32_t dataSize,
                            uint32_t QoSDId,
                            uint8_t listOrder,
                            uint32_t offset)
{
    AdmGetLogPage(cmd, data, dataSize, SEF_LPID_SUPERBLOCKLIST);

    // Specific
    cmd->nsid = QoSDId;                 // Namespace Identifier : QoS Domain ID
    cmd->cdw10 |= (listOrder << 12);    // [CDW10][Bit14-12] List Order
    cmd->cdw12 = offset;                // [CDW12][Bit0-32] Log Page Offset Lower
}

/**
 *  @brief	Generates command information of Admin Command Get Log Page
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] VDId: VirtualDevice ID
 *  @param	[in] SBId: Super Block ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Log Page Identifier : D3h (Super Block Information)
 *  @details	Target SEFAPI
 *  @details	- SEFGetVirtualDeviceInformation()
 *  @details	- SEFGetQoSDomainInformation()
 *  @details	- SEFGetSuperBlockInfo()
 */
void SEFAdmGetLogPageSBInfo(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t VDId, uint32_t SBId)
{
    AdmGetLogPage(cmd, data, dataSize, SEF_LPID_SUPERBLOCKINFORMATION);

    // Specific
    // cmd->nsid = VDId;      // Namespace Identifier : Virtual Device ID
    cmd->cdw11 |= VDId << 16;    // [CDW11][Bits31:16] Endurance Gropu ID : Virtual Device ID
    cmd->cdw12 |= SBId;          // [CDW12][Bit31:00] Log Page Offset Lower : Super Block ID
    cmd->cdw13 |= 0;             // [CDW13][Bit31:00] Log Page Offset Upper : Super Block ID
}

/**
 *  @brief	Generates command information of Admin Command Get Log Page
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] SBId: Super Block ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Log Page Identifier : D4h (User Address List)
 *  @details	Target SEFAPI
 *  @details	- SEFGetUserAddressList()
 */
void SEFAdmGetLogPageUserAddrList(struct nvme_passthru_cmd64 *cmd,
                                  void *data,
                                  uint32_t dataSize,
                                  uint32_t QoSDId,
                                  uint32_t SBId,
                                  uint64_t offset)
{
    AdmGetLogPage(cmd, data, dataSize, SEF_LPID_USER_ADDRESSLIST);

    // Specific
    cmd->nsid = QoSDId;    // Namespace Identifier : QoS Domain ID
    cmd->cdw12 = offset & 0xffffffff;
    cmd->cdw13 = SBId;
}

/**
 *  @brief	Generates command information of Admin Command Identify
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Identify
 *  @details	Target SEFAPI
 *  @details	- SEFGetHandle()
 */
void SEFAdmIdentify(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_ADMOPC_IDENTIFY;    // Opcode
    cmd->addr = (__u64)data;              // Data Pointer
    cmd->data_len = dataSize;             // Data Length for SEF Driver
    cmd->cdw10 |= 0x01;                   // [CDW10][Bit07:00] Controller or Namespace Structure
    cmd->cdw10 |= (0 << 16);              // [CDW10][Bit31:16] Controller Identifier : 0
}

void SEFAdmSefNsIdentify(struct nvme_passthru_cmd64 *cmd, uint32_t nsid, void *data, uint32_t dataSize)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_ADMOPC_IDENTIFY;    // Opcode
    cmd->nsid = nsid;
    cmd->addr = (__u64)data;              // Data Pointer
    cmd->data_len = dataSize;             // Data Length for SEF Driver
    cmd->cdw10 |= 0x05;                   // Cmd set specific structure
    cmd->cdw11 |= SEF_ID_NS_CSI << 24;    // Sef id
}

/**
 *  @brief	Generates command information of Admin Command Capacity Management
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Identify
 *  @details	Target SEFAPI
 *  @details	- SEFGetHandle()
 */
void SEFAdmCapacityManagement(struct nvme_passthru_cmd64 *cmd, uint32_t cfg_id)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = NVME_CAPACITY_MANAGEMENT;    // Opcode
    cmd->cdw10 = cfg_id << 16;
}

/**
 *  @brief	Generates command information of Admin Command Namespace Management
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data (valid only for Create)
 *  @param	[in] dataSize: size of area that data pointer points to (valid only for Create)
 *  @param	[in] qd_id: Namespace
 *  @param	[in] create: true to create, false to delete.
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand : 0Dh (Namespace Management)
 *  @details	Target SEFAPI
 *  @details	- SEFCreateQoSDomain() : SEL (1)Create
 *  @details	- SEFDeleteQoSDomain() : SEL (0)Delete
 */
void SEFAdmNamespaceManagement(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t qd_id, bool create)
{
    memset(cmd, 0, sizeof(*cmd));

    // Specific
    cmd->opcode = SEF_ADMOPC_NS_MGMT;
    cmd->nsid = qd_id;    // Namespace Identifier
    cmd->cdw10 = create ? 0 : 1;
    if (create)
    {
        cmd->cdw11 = (0x30 << 24);    // CSI is 30h for SEF cmd set
    }

    cmd->addr = (__u64)data;     // Data Pointer
    cmd->data_len = dataSize;    // Data Length for SEF Driver
}

/**
 *  @brief	Generates command information of Admin Command Namespace Attachment
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data (controller list)
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] qd_id: Namespace
 *  @param	[in] attach: true to attach, false to detach.
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand : 15h (Namespace Attachment)
 *  @details	Target SEFAPI
 *  @details	- SEFCreateQoSDomain() : SEL (1)Create
 *  @details	- SEFDeleteQoSDomain() : SEL (0)Delete
 */
void SEFAdmNameSpaceAttachment(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint32_t qd_id, bool attach)
{
    memset(cmd, 0, sizeof(*cmd));

    // Specific
    cmd->opcode = SEF_ADMOPC_NS_ATTACH;
    cmd->nsid = qd_id;    // Namespace Identifier
    cmd->cdw10 = attach ? 0 : 1;

    cmd->addr = (__u64)data;     // Data Pointer
    cmd->data_len = dataSize;    // Data Length for SEF Driver
}

/**
 *  @brief	Generates command information of Admin Command Set Features
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] featureId: feature identifier
 *  @return	None
 *  @note    Set Save to 1
 */
STATIC void AdmSetFeature(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint8_t featureId)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_ADMOPC_SETFEATURE;    // Opcode
    cmd->addr = (__u64)data;                // Data Pointer
    cmd->data_len = dataSize;               // Data Length for SEF Driver
    cmd->cdw10 |= featureId;                // [CDW10][Bit07:00] Feature Identifier
    cmd->cdw10 |= (1U << 31);               // [CDW10][Bit  :31] Save : 1
}

/**
 *  @brief	Generates command information of Admin Command VD Set Features
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] SEL: Select
 *  @param	[in] VDId: Virtual Device ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D0h (VirtualDevice Management)
 *  @details	- SEL (0) Set Number of dies
 *  @details	- SEL (1) Attach Read Fifo
 *  @details	- SEL (2) Change Read Fifo
 *  @details	- SEL (3) Detach Read Fifo
 *  @details	- SEL (4) Set Number of pSLC blocks
 */
void SEFAdmSetFeatureVDManagement(struct nvme_passthru_cmd64 *cmd, uint8_t SEL, uint32_t VDId)
{
    AdmSetFeature(cmd, NULL, 0, SEF_FID_VIRTUALDEVOCEMANAGEMENT);

    // Specific
    cmd->nsid = VDId;             // Namespace Identifier : Virtual Device ID
    cmd->cdw11 |= (SEL & 0xF);    // [CDW11][Bit03:00] Select
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Set Num Dies
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] numDie: Number of Dies
 *  @return	None
 */
void SEFAdmSetFeatureVDSetNumDies(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint16_t numDie)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_SET_NUM_DIES, vdID);

    // Specific
    cmd->cdw11 |= (((uint32_t)(numDie - 1) & 0xff) << 8);    // note: truncated to 8 bits per v1.11 CS
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Attach Read FIFO
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] rfID: Read Fifo ID
 *  @param	[in] weight: Default Read Weight
 *  @param	[in] isDefault: True if read fifo is the VD default
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D1h (QoS Domain Management)
 */
void SEFAdmSetFeatureVDAttachReadFifo(
    struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t rfID, uint16_t weight, bool isDefault)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_ATTACH_RFIFO, vdID);

    // Specific
    cmd->cdw11 |= ((uint32_t)weight) << 16;
    cmd->cdw12 |= isDefault ? 1 << 31 : 0;
    cmd->cdw12 |= rfID & 0x7fffFFFF;
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Change Read FIFO
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] rfID: Read Fifo ID
 *  @param	[in] weight: Default Read Weight
 *  @param	[in] isDefault: True if read fifo is the VD default
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D1h (QoS Domain Management)
 */
void SEFAdmSetFeatureVDChangeReadFifo(
    struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t rfID, uint16_t weight, bool isDefault)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_CHANGE_RFIFO, vdID);

    // Specific
    cmd->cdw11 |= ((uint32_t)weight) << 16;
    cmd->cdw12 |= isDefault ? 1 << 31 : 0;
    cmd->cdw12 |= rfID & 0x7fffFFFF;
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Detach Read FIFO
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] rfID: Read Fifo ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D1h (QoS Domain Management)
 */
void SEFAdmSetFeatureVDDetachReadFifo(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t rfID)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_DETACH_RFIFO, vdID);

    // Specific
    cmd->cdw12 = rfID;
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Set Num pSLC blocks
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] numPSLC: Number of Dies
 *  @return	None
 */
void SEFAdmSetFeatureVDSetNumPSLC(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint32_t numPSLC)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_SET_NUM_PSLC, vdID);

    // Specific
    cmd->cdw12 = numPSLC;
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Control AEN
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] cwc: Critical Warning Control (bit 2: Cap reduction, bit 1: out of pSLC SBs, bit 0: out of SBs)
 *  @return	None
 */
void SEFAdmSetFeatureVDControlAEN(struct nvme_passthru_cmd64 *cmd, uint16_t vdID, uint8_t cwc)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_CTRL_ASYNC_EVENT, vdID);
    cmd->cdw11 |= ((uint32_t)(cwc & SEF_CAE_WARN_MASK)) << 16;
}

/**
 *  @brief	Generates command for Admin Command VD Set Feature Set Suspend Config
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] vdID: Virtual device ID
 *  @param	[in] minTimeUntilSuspend: minimum time for program/erase operation before suspension may occur
 *  @return	None
 */
void SEFAdmSetFeatureVDSetSuspendConfig(struct nvme_passthru_cmd64 *cmd,
                                        uint16_t vdID,
                                        uint32_t maxTimePerSuspend,
                                        uint32_t minTimeUntilSuspend,
                                        uint32_t maxSuspendInterval)
{
    SEFAdmSetFeatureVDManagement(cmd, SEF_FEATURE_SEL_SET_SUSPEND_CONF, vdID);
    cmd->cdw12 = minTimeUntilSuspend;
}

/**
 *  @brief	Generates command information of Admin Command Set Feature Capacity
 *          Configuration Registration
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to capacity configuration
 *  @param  [in] egcn: Number of endurance group configurations
 *  @param  [in] tchmus: Total number of channel media units
 *  @param	[in] dataSize: size of area that data pointer points to.  Must
 *               be 0x20 + egcn*0x58 + tchmus*8;
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D2h (Capacity
 *                configuration registration)
 *  @details	Target SEFAPI
 *  @details	- SEFCreateVirtualDevices()
 */
void SEFAdmSetFeatureCapCfgReg(
    struct nvme_passthru_cmd64 *cmd, void *data, uint16_t egcn, uint16_t tchmus, uint32_t dataSize)
{
    uint32_t cdw11;

    AdmSetFeature(cmd, NULL, 0, SEF_FID_CAP_CFG_REGISTER);

    assert(0x20 + egcn * 0x58 + tchmus * 8 == dataSize);
    // Specific
    cdw11 = tchmus;                     // [CDW11][Bit15:00] Num CHMUs
    cdw11 |= ((uint32_t)egcn) << 16;    // [CDW11][Bit31:16] Num EGC Descriptors
    cmd->cdw11 = cdw11;
    cmd->addr = (__u64)data;    // Data Pointer
    cmd->data_len = dataSize;
}

/**
 *  @brief	Generates command information of Admin Command Set Features
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data (valid only for Create)
 *  @param	[in] dataSize: size of area that data pointer points to (valid only for Create)
 *  @param	[in] SEL: Select
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] VDId: (valid only for Create/Delete/Reset Encryption Key)
 *  @param	[in] rootPointer: Root Pointer (valid only for Root Pointer)
 *  @param	[in] rootPointerIndex: Root Pointer Index (valid only for Root Pointer)
 *  @param	[in] readDeadLine: Read Dead Line (valid only for Change Read Dead Line)
 *  @param	[in] GCAP: GCAP (valid only for Change Capacity)
 *  @param	[in] QUOTA: QUOTA (valid only for Change Capacity)
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D1h (QoS Domain Management)
 *  @details	Target SEFAPI
 *  @details	- SEFCreateQoSDomain()      : SEL (1)Create
 *  @details	- SEFSetQoSDomainCapacity() : SEL (3)Change Capacity
 *  @details	- SEFDeleteQoSDomain()      : SEL (0)Delete
 *  @details	- SEFSETRootPointer()       : SEL (2)Root Pointer
 *  @details	- SEFSetReadDeadline()      : SEL (4)Change Read Dead Line
 *  @details	- SEFResetEncryptionKey()   : SEL (5)Reset Encryption Key
 */
void SEFAdmSetFeatureQoSDManagement(struct nvme_passthru_cmd64 *cmd,
                                    uint8_t SEL,
                                    uint32_t QoSDId,
                                    uint32_t dword1,
                                    uint32_t dword1shift,
                                    uint64_t qword1,
                                    uint64_t qword2)
{
    AdmSetFeature(cmd, NULL, 0, SEF_FID_QOSDOMAINMANAGEMENT);

    // Specific
    cmd->nsid = QoSDId;          // Namespace Identifier : QoS Domain ID
    cmd->cdw11 = (SEL & 0xF);    // [CDW11][Bit03:00] Select
    cmd->cdw11 |= dword1 << dword1shift;
    cmd->cdw12 = qword1 & 0xffffffff;
    cmd->cdw13 = qword1 >> 32;
    cmd->cdw14 = qword2 & 0xffffffff;
    cmd->cdw15 = qword2 >> 32;
}

void SEFAdmSetFeatureQoSDSetRootPointer(struct nvme_passthru_cmd64 *cmd,
                                        uint32_t QoSDId,
                                        uint32_t rootPointerIndex,
                                        uint64_t rootPointer)
{
    SEFAdmSetFeatureQoSDManagement(cmd, SEF_FEATURE_ROOTPOINTER, QoSDId, rootPointerIndex, 4,
                                   rootPointer, 0);
}

void SEFAdmSetFeatureQoSDChangeCapacity(
    struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, bool pSLC, uint64_t gcap, uint64_t quota)
{
    SEFAdmSetFeatureQoSDManagement(cmd, SEF_FEATURE_CHG_CAPACITY, QoSDId, pSLC, 16, gcap, quota);
}

void SEFAdmSetFeatureQoSDChangeWeights(struct nvme_passthru_cmd64 *cmd,
                                       uint32_t QoSDId,
                                       uint16_t wwt,
                                       uint16_t ewt)
{
    SEFAdmSetFeatureQoSDManagement(cmd, SEF_FEATURE_CHG_WEIGHTS, QoSDId, 0, 0,
                                   ((uint64_t)ewt << 16) | wwt, 0);
}

void SEFAdmSetFeatureQoSDChangeReadDeadline(struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, uint8_t rdl)
{
    SEFAdmSetFeatureQoSDManagement(cmd, SEF_FEATURE_CHG_READDEADLINE, QoSDId, rdl, 8, 0, 0);
}

void SEFAdmSetFeatureQoSDChangeMaxSBS(struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, uint32_t maxosbs)
{
    SEFAdmSetFeatureQoSDManagement(cmd, SEF_FEATURE_CHG_MAXSBS, QoSDId, maxosbs, 8, 0, 0);
}

void SEFAdmSetFeatureQoSDChangeDefRFifo(struct nvme_passthru_cmd64 *cmd, uint32_t QoSDId, uint32_t rfid)
{
    SEFAdmSetFeatureQoSDManagement(cmd, SEF_FEATURE_DEFAULT_RFIFO, QoSDId, 0, 0, rfid, 0);
}

/**
 *  @brief	Generates command information of Admin Command Get Features
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] featureId: Feature Identifier
 *  @return	None
 *  @details	Generates command
 *  @note    Set Select to 0
 */
STATIC void AdmGetFeature(struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint8_t featureId)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_ADMOPC_GETFEATURE;    // Opcode
    cmd->addr = (__u64)data;                // Data Pointer
    cmd->data_len = dataSize;               // Data Length for SEF Driver
    cmd->cdw10 |= featureId;                // [CDW10][Bit07:00] Feature Identifier
    cmd->cdw10 |= (0 << 8);                 // [CDW10][Bit10:08] Select
}

/**
 *  @brief	Generates command information of Admin Command Get Features
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] SEL: Select
 *  @param	[in] VDId: Virtual Device ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D0h (VirtualDevice Management)
 *  @details	Target SEFAPI
 *  @details	- SEFListVirtualDevices()          : SEL: 0h (Get Static Information)
 *  @details	- SEFGetVirtualDeviceInformation() : SEL: 0h (Get Static Information), SEL: 1h (Get Dynamic Information)
 */
void SEFAdmGetFeatureVDManagement(
    struct nvme_passthru_cmd64 *cmd, void *data, uint32_t dataSize, uint8_t SEL, uint32_t VDId)
{
    uint32_t NmofDwSize = (dataSize / SEF_CMD_DWORD) - SEF_CMD_ZERO_BASE;

    AdmGetFeature(cmd, data, dataSize, SEF_FID_VIRTUALDEVOCEMANAGEMENT);

    // Specific
    cmd->nsid = VDId;                    // Namespace Identifier : Virtual Device ID
    cmd->cdw11 |= (SEL & 0xF);           // [CDW11][Bit03-00] Select
    cmd->cdw11 |= (NmofDwSize << 16);    // [CDW11][Bit31-16] Number of Dwords
}

/**
 *  @brief	Generates command information of Admin Command Get Features
 *  @param	[in,out] cmd: pointer to AdminCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] SEL: Select
 *  @param  [in] SSEL: SubSelect
 *  @param	[in] QoSDId: QoS Domain ID
 *  @return	None
 *  @details	Generates command:
 *  @details	- AdminCommand Features Identifier (FID) : D1h (QoS Domain Management)
 *  @details	Target SEFAPI
 *  @details	- SEFGetQoSDomainInformation() : SEL: 0h (Get Information)
 */
void SEFAdmGetFeatureQoSDManagement(struct nvme_passthru_cmd64 *cmd,
                                    void *data,
                                    uint32_t dataSize,
                                    uint8_t SEL,
                                    uint8_t SSEL,
                                    uint32_t QoSDId)
{
    uint32_t NmofDwSize = data ? (dataSize / SEF_CMD_DWORD) - SEF_CMD_ZERO_BASE : 0;

    AdmGetFeature(cmd, data, dataSize, SEF_FID_QOSDOMAINMANAGEMENT);

    // Specific
    cmd->nsid = QoSDId;                  // Namespace Identifier : QoS Domain ID
    cmd->cdw11 |= (SEL & 0xF);           // [CDW11][Bit03:00] Select
    cmd->cdw11 |= (SSEL & 0xF) << 4;     // [CDW11]pBit07:04] Sub-Select
    cmd->cdw11 |= (NmofDwSize << 16);    // [CDW11][Bit31:16] Number of Dwords
}

/**
 *  @brief	Generates command information of Nvm Command Super Block Management
 *  @param	[in,out] cmd: pointer to NvmCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] operation: Operation
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] SBId: Super Block ID
 *  @param	[in] overrides: FMQ parameters
 *  @return	None
 *  @details	Generates command:
 *  @details	- NvmCommand
 *  @details	Target SEFAPI
 *  @details	- SEFReleaseSuperBlock()  : OP: 3h (Free)
 *  @details	- SEFCloseSuperBlock()    : OP: 2h (Close)
 *  @details	- SEFAllocateSuperBlock() : OP: 0h (Erase)
 */
void SEFNvmSuperBlockManagement(struct nvme_passthru_cmd64 *cmd,
                                void *data,
                                uint32_t dataSize,
                                uint8_t operation,
                                uint32_t QoSDId,
                                uint32_t SBId,
                                const struct SEFAllocateOverrides *overrides,
                                uint32_t timeout)
{
    uint32_t NmofDwSize = (dataSize / SEF_CMD_DWORD);

    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_NVMOPC_SUPERBLOCKMANAGEMENT;    // Opcode
    cmd->addr = (__u64)data;                          // Data Pointer
    cmd->data_len = dataSize;                         // Data Length for SEF Driver
    cmd->nsid = QoSDId;                               // Namespace Identifier : QoS Domain ID
    cmd->cdw10 = SBId;                                // [CDW10][Bit31:00] Super Block ID
    cmd->cdw11 = (NmofDwSize & 0xFFFF);               // [CDW11][Bit15:00] Number of Dwords
    cmd->cdw11 |= (operation & 0xF) << 16;            // [CDW11][Bit19:16] Operation
    cmd->timeout_ms = timeout;

    if (operation != SEF_NVMOPE_SBMANAGEMENT_FREE)
    {
        if (overrides != NULL)
        {
            cmd->cdw15 |= (uint32_t)overrides->eraseWeight
                          << 16;    // [CDW12][Bit31:16] Overriding Weight
        }
    }
}

/**
 *  @brief	Generates command information of Nvm Command Flash Address Request
 *  @param	[in,out] cmd: pointer to NvmCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] SBId: Super Block ID
 *  @param	[in] userAddress: User Address
 *  @param	[in] numAdu: number of ADUs
 *  @param	[in] placementId: Placement ID
 *  @param	[in] overrides: FMQ parameters
 *  @return	None
 *  @details	Generates command:
 *  @details	- NvmCommand
 *  @details	Target SEFAPI
 *  @details	- SEFWriteWithoutPhysicalAddress()
 */
void SEFNvmFlashAddressRequest(struct nvme_passthru_cmd64 *cmd,
                               void *data,
                               uint32_t dataSize,
                               uint32_t QoSDId,
                               uint64_t flashAddress,
                               struct SEFPlacementID placementId,
                               uint16_t eraseWeight,
                               uint32_t nlwid)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    uint32_t sbi = (flashAddress != UINT64_C(0xffffffffffffffff));    // 0/1 plid/sbid
    uint32_t cdw14 = sbi ? flashAddress & 0xffffffff : 0;
    uint32_t cdw15 = ((uint32_t)eraseWeight << 16) |
                     (sbi ? (flashAddress >> 32) & 0xffff : placementId.id);

    cmd->opcode = SEF_NVMOPC_FLASHADDRESSREQUEST;    // Opcode
#if CC_USE_FUSED
    cmd->flags = 1;    // fused first
#endif
    cmd->addr = (uint64_t)data;    // Data Pointer
    cmd->data_len = dataSize;      // Number of Dwords in Data Transfer
    cmd->nsid = QoSDId;            // Namespace Identifier : QoS Domain ID
    cmd->cdw12 = (sbi << 25);
    cmd->cdw13 = nlwid;
    cmd->cdw14 = cdw14;
    cmd->cdw15 = cdw15;
    cmd->timeout_ms = 90 * 1000;
}

/**
 *  @brief	Generates command information of Nvm Command Nameless Write
 *  @param	[in,out] cmd: pointer to NvmCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] sbid: Super Block ID
 *  @param	[in] userAddress: User Address
 *  @param	[in] numAdu: number of ADUs
 *  @param	[in] placementId: Placement ID
 *  @param	[in] overrides: FMQ parameters
 *  @param  [in] pSLC: true will set pSLC flag (if SBI == 0)
 *  @return	None
 *  @details	Generates command:
 *  @details	- NvmCommand
 *  @details	Target SEFAPI
 *  @details	- SEFWriteWithoutPhysicalAddress()
 */
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
                                bool pSLC)
{
    uint64_t userAddressBits = le64toh(userAddress.unformatted);

    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    uint32_t sbi = (sbid == 0xffffffff) ? 0 : 1;
    uint32_t cdw12 = (uint32_t)(!!forceUnitAccess << 30) | (sbi << 25) | (numAdu - 1);
    uint64_t fa = sbi ? ((uint64_t)(sbid + 1) << aduBits) - 1 : 0x7fffffff;
    uint32_t cdw14 = (uint32_t)fa;
    uint32_t cdw15 = ((uint32_t)programWeight << 16) | (sbi ? (fa >> 32) & 0xffff : placementId.id);

    if (pSLC && !sbi)
    {
        cdw14 |= 1U << 31;
    }
    cmd->opcode = SEF_NVMOPC_NAMELESSWRITE;    // Opcode
#if CC_USE_FUSED
    cmd->flags = 2;    // fused second
#endif
    cmd->addr = (uint64_t)iovec;                       // Data Pointer
    cmd->data_len = nvecs;                             // Number of Dwords in Data Transfer
    cmd->metadata = (uint64_t)metadata;                // Meta Data Pointer
    cmd->metadata_len = metadata ? numAdu * 16 : 0;    // Number of Dwords in Metadata Transfer
    cmd->nsid = QoSDId;                                // Namespace Identifier : QoS Domain ID
    cmd->cdw10 = ((uint32_t)userAddressBits);          // [CDW10][Bit31:00] Starting User Address
    cmd->cdw11 = (userAddressBits >> 32);              // [CDW11][Bit31:00] Starting User Address
    cmd->cdw12 = cdw12;
    cmd->cdw13 = nlwid;
    cmd->cdw14 = cdw14;
    cmd->cdw15 = cdw15;
    cmd->timeout_ms = timeout;
}

/**
 *  @brief	Generates command information of Nvm Command Read
 *  @param	[in,out] cmd: pointer to NvmCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] userAddress: User Address
 *  @param	[in] numAdu: number of ADUs
 *  @param	[in] flashAddress: Flash Address
 *  @param	[in] overrides: FMQ parameters
 *  @return	None
 *  @details	Generates command:
 *  @details	- NvmCommand
 *  @details	Target SEFAPI
 *  @details	- SEFReadWithoutPhysicalAddress()
 */
void SEFNvmReadCommand(struct nvme_passthru_cmd64 *cmd,
                       const struct iovec *iovec,
                       uint32_t iovcnt,
                       void *metadata,
                       uint32_t QoSDId,
                       struct SEFUserAddress userAddress,
                       uint32_t numAdu,
                       struct SEFFlashAddress flashAddress,
                       uint32_t fifo,
                       uint16_t ioWeight)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    uint64_t userAddressBits = le64toh(userAddress.unformatted);

    cmd->opcode = SEF_NVMOPC_READ;    // Opcode
    cmd->addr = (__u64)iovec;         // Data Pointer
    cmd->data_len = iovcnt;           // Number of Dwords in Data Transfer
    cmd->metadata = (__u64)metadata;
    cmd->metadata_len = metadata ? numAdu * 16 : 0;
    cmd->nsid = QoSDId;    // Namespace Identifier : QoS Domain ID
    cmd->cdw10 = userAddressBits & 0xffffffff;
    cmd->cdw11 = userAddressBits >> 32;
    cmd->cdw12 = numAdu - 1;
    cmd->cdw13 = fifo;
    cmd->cdw14 = flashAddress.bits & 0xffffffff;
    cmd->cdw15 = (ioWeight << 16) | ((flashAddress.bits >> 32) & 0xffff);
}

/**
 *  @brief	Generates command information of Nvm Command NamelessCopy
 *  @param	[in,out] cmd: pointer to NvmCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] copySrc: copy source information
 *  @param	[in] filter: copy source filtering information
 *  @param	[in] overrides: FMQ parameters
 *  @param  [in] nlcid: NLC ID to associate with GCR command
 *  @param	[in] dstFlashAddress: copy destination flash address
 *  @return	None
 *  @details	Generates command:
 *  @details	- NvmCommand
 *  @details	Target SEFAPI
 *  @details	- SEFNamelessCopy()
 */
void SEFNvmNamelessCopy(struct nvme_passthru_cmd64 *cmd,
                        void *data,
                        uint32_t dataSize,
                        uint32_t QoSDId,
                        struct SEFCopySource *copySrc,
                        const struct SEFUserAddressFilter *filter,
                        const struct SEFCopyOverrides *overrides,
                        uint32_t nlcid,
                        struct SEFFlashAddress dstFlashAddress,
                        uint32_t timeout)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_NVMOPC_NAMELESSCOPY;    // Opcode
#if CC_USE_FUSED
    cmd->flags = 1;    // fused first
#endif
    cmd->addr = (__u64)data;     // Data Pointer
    cmd->data_len = dataSize;    // Data Length for SEF Driver
    cmd->nsid = QoSDId;          // Namespace Identifier : QoS Domain ID

    // Bitmap or FLA List
    if (copySrc->format == kBitmap)
    {
        cmd->cdw10 |=
            ((copySrc->arraySize * 2) - SEF_CMD_ZERO_BASE);    // [CDW10][Bit15:00] Number of Elements
        cmd->cdw10 |= ((uint32_t)0 << 16);    // [CDW10][Bit  :16] FLA List or Bitmap [Bitmap]
    }
    else
    {
        cmd->cdw10 |=
            (copySrc->arraySize - SEF_CMD_ZERO_BASE);    // [CDW10][Bit15:00] Number of Elements
        cmd->cdw10 |= ((uint32_t)1 << 16);    // [CDW10][Bit  :16] FLA List or Bitmap [List]
    }

    if ((filter != NULL) && (filter->userAddressRangeType != 0))
    {
        cmd->cdw10 |= ((uint32_t)1 << 17);    // [CDW10][Bit  :17] UA Range
    }

    cmd->cdw13 |= nlcid;    // [CDW13][Bit31:0] NLC ID

    cmd->cdw14 |= (uint32_t)(le64toh(
        dstFlashAddress.bits));    // [CDW14][Bits31:0] Lower 32 bites of src flash address
    cmd->cdw15 |= (le64toh(dstFlashAddress.bits) >> 32) &
                  0xffff;    // [CDW15][Bits15:0] Lower 16 bits of upper 32 bits of src flash address

    if (overrides != NULL)
    {
        cmd->cdw15 |= ((uint32_t)overrides->programWeight)
                      << 16;    // [CDW13][Bit31:16] Overriding Weight for Program
    }
    cmd->timeout_ms = timeout;
}

/**
 *  @brief	Generates command information of Nvm Command GetCopyResult
 *  @param	[in,out] cmd: pointer to NvmCommand
 *  @param	[in] data: pointer to data
 *  @param	[in] dataSize: size of area that data pointer points to
 *  @param	[in] QoSDId: QoS Domain ID
 *  @param	[in] copySrc: copy source information
 *  @param	[in] filter: copy source filtering information
 *  @param	[in] desQoSDId: copy destination QoS Domain ID
 *  @param	[in] desSBId: copy destination Super Block ID
 *  @param	[in] overrides: FMQ parameters
 *  @param	[in] numAddressChangeEntries: number of entries in ACR
 *  @param  [in] nlcid: NLC ID to associate with GCR command
 *  @return	None
 *  @details	Generates command:
 *  @details	- NvmCommand
 *  @details	Target SEFAPI
 *  @details	- SEFNamelessCopy()
 */
void SEFNvmGetCopyResult(struct nvme_passthru_cmd64 *cmd,
                         void *data,
                         uint32_t dataSize,
                         uint32_t QoSDId,
                         int fua,
                         uint32_t numAddressChangeEntries,
                         uint32_t nlcid,
                         uint32_t timeout)
{
    memset(cmd, 0x00, sizeof(struct nvme_passthru_cmd64));

    cmd->opcode = SEF_NVMOPC_GETCOPYRESULT;    // Opcode
#if CC_USE_FUSED
    cmd->flags = 2;    // fused second
#endif
    cmd->addr = (__u64)data;     // Data Pointer
    cmd->data_len = dataSize;    // Data Length for SEF Driver
    cmd->nsid = QoSDId;          // Namespace Identifier : QoS Domain ID
    cmd->cdw10 |= numAddressChangeEntries -
                  1;        // [CDW10][Bit31:0] Number of Address Change Request Entries NACRE
    cmd->cdw13 |= nlcid;    // [CDW13][Bit31:0] NLC ID
    cmd->cdw12 |= (!!fua) << 30;
    cmd->timeout_ms = timeout;
}
