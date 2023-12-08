/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_api.h
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

#ifndef __SEF_API_H__
#define __SEF_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include "SEFAPI.h"
/*
 * SEF-LIB module versioning rule
 *   Format: X.Y
 *     X: Incremented at a major release
 *     Y: Incremented if a feature is changed
 *   * Y returns to zero if X is increased
 */
#define SEFLIB_VER_MAJOR  "0"
#define SEFLIB_VER_MIDDLE "44"

#define SEFLIB_VER_MESG "seflib (Ver %s.%s)\n"

// For the emulated device, this is the expected SEF command set and interface
// version numbers.  If they don't match, an error is logged and the device
// won't be discovered.
#define SEF_CS_MJ_VER 1
#define SEF_CS_MN_VER 13
#if CC_EMU_VALIDATION
#define SEF_QEMU_IF_VER 5    // If emulator has a breaking change, this will increment
#endif

/**
 *  @brief	Identify Controller Data Structure
 */
struct Identify
{
    // Controller Capabilities and Features
    char VID[2];           //!< PCI Vendor ID (VID):
    char SSVID[2];         //!< PCI Subsystem Vendor ID (SSVID):
    char SN[20];           //!< Serial Number (SN): ASCII string
    char MN[40];           //!< Model Number (MN): ASCII string
    char FR[8];            //!< Firmware Revision (FR): ASCII string
    char RAB;              //!< Recommended Arbitration Burst (RAB):
    char IEEE[3];          //!< IEEE OUI Identifier (IEEE):
    char CMIC;             //!< Controller Multi-Path I/O and Namespace Sharing Capabilities (CMIC):
    char MDTS;             //!< Maximum Data Transfer Size (MDTS):
    char CNTLID[2];        //!< Controller ID (CNTLID):
    char VER[4];           //!< Version (VER):
    char RTD3R[4];         //!< RTD3 Resume Latency (RTD3R):
    char RTD3E[4];         //!< RTD3 Entry Latency (RTD3E):
    char OAES[4];          //!< Optional Asynchronous Events Supported (OAES):
    char CTRATT[4];        //!< Controller Attributes (CTRATT):
    char Reserved1[12];    //!< Reserved
    char FGUID[16];        //!< FRU Globally Unique Identifier (FGUID):
    char Reserved2[112];    //!< Reserved
    char NVMeMngIF[16];    //!< Refer to the NVMe Management Interface Specification for definition.

    // Admin Command Set Attributes & Optional Controller Capabilities
    char field2[256];

    // NVM Command Set Attributes
    char field3[512 + 1024];

    // Power State Descriptors
    char field4[1024];

    // Vendor Specific
    char field5[1024];
} __attribute__((packed));

/**
 *  @brief  Capacity Configuration Descriptor
 */
struct MediaUnitDescriptor
{
    uint16_t mu_id;
    uint8_t reserved[4];
    uint16_t mudl;
    uint8_t bytes[];
} __attribute__((packed));

struct ChnlCfgDescriptor
{
    uint16_t ch_id;
    uint16_t chmus;
    struct MediaUnitDescriptor mu_descs[];
} __attribute__((packed));

struct EndGrpDescriptor
{
    uint16_t endgid;
    uint16_t caf;
    uint8_t reserved[76];
    uint16_t egsets;
    uint16_t egchans;
    struct ChnlCfgDescriptor cc_descs[1];
} __attribute__((packed));

struct CapCfgDescriptor
{
    uint16_t cci;
    uint16_t di;
    uint16_t egcn;
    uint8_t reserved[26];
    struct EndGrpDescriptor egc_descs[];
} __attribute__((packed));

struct SupCapCfgList
{
    uint8_t sccn;
    uint8_t reserved[15];
    struct CapCfgDescriptor cag_cfgs[];
} __attribute__((packed));

enum NVME_CAPACITY_MGT_TYPE {
    NVME_SELECT_CAPACITY_CFG,
    NVME_CREATE_ENDURANCE_GROUP,
    NVME_DELETE_ENDURANCE_GROUP,
    NVME_CREATE_NVM_SET,
    NVME_DELETE_NVM_SET,
};

#define MAX_ARR_SBIS 8

struct ArrSbi
{
    uint32_t nfla;
    uint16_t ndp;
    uint8_t reserved6[2];
    struct SEFFlashAddress sfla;
} __attribute__((packed));

/**
 *  @brief	Flash Address Request
 */
struct AddressRecordRequest
{
    uint8_t nsb;
    uint8_t ndpb;
    uint16_t npsp;
    uint32_t nradu;
    uint8_t reserved8[8];
    struct ArrSbi sbis[MAX_ARR_SBIS];
} __attribute__((packed));

/**
 *  @brief	Address Change Order
 */
struct AddressChangeOrder
{
    uint64_t sua;
    uint64_t fla;
    uint16_t nadu;    // 0 based
    uint16_t naco;    // 0 based
    uint32_t nlwid;
    uint32_t nadul;
    uint16_t ndp;
    uint8_t reserved30[2];
    struct AcoEntry
    {
        uint64_t ua;
        uint64_t ofa;
        uint64_t nfa;
    } acoes[];
} __attribute__((packed, aligned(8)));

/**
 *  @brief	GetLogPage SefUnitInformation(D0) - SEF Unit Information Data Structure
 */
struct GetLogPageSefUnitInformation
{
    uint64_t SEFSIG;          //!< Signature indicating it is a SEF device
    uint8_t SEFMJR;           //!< SEF Major version identification
    uint8_t SEFMNR;           //!< SEF Minor version identification
    uint16_t SEFCAP;          //!< SEF Capability
    uint16_t MAXQOSD;         //!< Max Number of QoS Domains
    uint8_t NCH;              //!< Number of Channels
    uint8_t NBNK;             //!< Number of Banks
    uint16_t NBLK;            //!< Number of Blocks per a Die
    uint16_t NPAG;            //!< Number of Pages per a Plane
    uint8_t NPL;              //!< Number of Planes per a Die
    uint8_t NADU;             //!< Number of ADUs per a Page
    uint8_t ADUBW;            //!< ADU Offset Bit Width
    uint8_t SBBW;             //!< Super Block ID Bit Width
    uint16_t EOP;             //!< Expiration Open Period
    uint8_t NPLID;            //!< Number of Placement IDs
    uint8_t NRP;              //!< Number of Root Pointers
    uint8_t NFMQ;             //!< Number of Flash Media Queues per a Die
    uint8_t NPSI;             //!< Number of Pages to Secure Integrity
    uint8_t reserved1[34];    //!< Reserved
    uint16_t NVD;             //!< Number of Virtual Devices Created
    uint16_t NQOSD;           //!< Number of QoS Domains Created
    uint8_t reserved2[60];    //!< Reserved
} __attribute__((packed));

/**
 *  @brief	GetLogPage QoSDomainList(D1) - QoS Domain List
 */
struct GetLogPageQoSDomainList
{
    uint16_t MAX_QOSD;    //!< Maximum QoS Domain ID
    uint16_t NUM_QOSD;    //!< Number of QoS Domain assigned
    uint16_t VDID[];      //!< Virtual Device ID for QoS Domain-n
} __attribute__((packed));

/**
 *  @brief	GetLogPage SuperBlockList(D2) - Super Block Descriptor
 */
struct SuperBlockDescriptor
{
    uint32_t SBID;     //!< Super Block ID
    uint8_t PECI;      //!< P/E Count Index
    uint8_t DIISBS;    //!< Data Integrity Index and Super Block State
    uint8_t reserved6[2];
} __attribute__((packed));

/**
 *  @brief	GetLogPage SuperBlockList(D2) - Super Block List Data
 */
struct GetLogPageSuperBlockList
{
    struct SuperBlockDescriptor SuperBlockDescriptor[1];    //!< n-th Super Block Descriptor
} __attribute__((packed));

/**
 *  @brief	GetLogPage UserAddress List(D4) - User Address Data List
 */
struct GetLogPageUserAddressList
{
    struct SEFUserAddress userAddresses[1];    //!< n-th User Address Collection
} __attribute__((packed));

/**
 *  @brief	GetLogPage SuperBlockInformation(D3) - Super Block Descriptor
 */
struct GetLogPageSuperBlockInformation
{
    uint16_t VDID;      //!< Virtual Device ID
    uint8_t ST;         //!< State transition
    uint8_t SBS;        //!< Super Block State
    uint32_t SBID;      //!< Super Block ID
    uint16_t PID;       //!< Placement ID
    uint16_t QOSDID;    //!< QoS Domain ID
    uint8_t PECI;       //!< P/E Count Index
    uint8_t DII;        //!< Data Integrity Index
    uint16_t TL;        //!< Time Left
    uint32_t CAP;       //!< Capacity in ADUs
    uint32_t ADUPTR;    //!< ADU Pointer
    uint32_t ESN;       //!< Erase Serial Number
    uint16_t DGID;      //!< Die Group ID
    uint16_t NDPL;      //!< Number of Defective Planes
    uint8_t rsvd30[128 - 32];
    uint8_t DBM[128];    //!< Defect Bitmap
} __attribute__((packed));

/**
 *  @brief	GetLogPage SuperBlockInformation(D3) - Super Block State
 */
enum SbState {
    kSbFree = 0x01,                 //!< [bit0] Free
    kSbClosed = 0x02,               //!< [bit1] Closed
    kSbOpenByPlacementID = 0x04,    //!< [bit2] Opened by a Write to a placement id
    kSbOpenByErase = 0x08,          //!< [bit3] Opened by erasing a super block
    kSbPSLC = 0x40,                 //!< [bit6] pSLC
    kSbOffline = 0x80               //!< [bit7] Offline
} __attribute__((packed));

/**
 *  @brief	GetLogPage SuperBlockStateChange(D5h) - State Transition
 */
enum SbStateTransition {
    kSbStateTransToUnkown = 1,             //!< Data Integrity becomes 'Unknown'
    kSbStateTransToClosedFromNlw = 4,      //!< From 'Open for Nameless Write' to 'Closed'
    kSbStateTransToClosedFromNlc = 8,      //!< From 'Open for Nameless Copy' to 'Closed'
    kSbStateTransToClosedFromErase = 16    //!< From 'Open by Erase' to 'Closed'
};

/**
 *  @brief	SetFeatures VirtualDeviceManagement(D0) Create(1) - Weight Settings for FMQ
 */
struct WeightSettings
{
    uint16_t WRR;            //!< Weight for R4R
    uint16_t WPW;            //!< Weight for P4W
    uint16_t WRC;            //!< Weight for R4C
    uint16_t WPC;            //!< Weight for P4C
    uint16_t WEW;            //!< Weight for E4W
    uint16_t WEC;            //!< Weight for E4C
    uint16_t reserved[1];    //!< Reserved
    uint16_t MAX_DTD;        //!< Maximum DT
} __attribute__((packed));

/**
 *  @brief	SetFeatures VirtualDeviceManagement(D0) Create(1) - Virtual Device Information Data for
 * Create a Virtual Device
 */
struct SetFeaturesVDManCreate
{
    uint16_t VDID;                   //!< Virtual Device ID
    uint8_t TCHN;                    //!< Top Channel Number
    uint8_t TBNKN;                   //!< Top Bank Number
    uint8_t NCH;                     //!< Number of Channels
    uint8_t NBK;                     //!< Number of Banks
    uint16_t reserved;               //!< Reserved
    uint8_t MISC;                    //!< Miscellaneous
    uint8_t NFMQ;                    //!< Number of FMQs
    uint8_t reserved2[6];            //!< Reserved
    struct WeightSettings FMQD[];    //!< Weight Settings for FMQ-n
} __attribute__((packed));

/**
 *  @brief	SetFeatures QoSDomainManagement(D1) Create(1) - QoS Domain Information Data Structure for Creation
 */
struct SetFeaturesQDManCreate
{
    uint16_t FS;             //!< Feature Settings
    uint8_t NPLID;           //!< Number of Placement IDs
    uint8_t reserved1[1];    //!< Reserved
    uint8_t QRR;             //!< FMQ for R4R
    uint8_t QPW;             //!< FMQ for P4W
    uint8_t QRC;             //!< FMQ for R4C
    uint8_t QPC;             //!< FMQ for P4C
    uint8_t reserved2[8];    //!< Reserved
    uint64_t GCAP;           //!< Guranteed Capacity
    uint64_t QUOTA;          //!< Quota
} __attribute__((packed));

struct VirtualDeviceSBStats
{
    uint64_t tcap;
    uint64_t tgcap;
    uint64_t ucap;
    uint32_t nfrsb;
    uint32_t nclsb;
    uint32_t neosb;
    uint32_t nwosb;
    uint8_t reserved40[63 - 39];
};

struct NamespaceSBStats
{
    uint64_t quota;
    uint64_t gcap;
    uint64_t ucap;
    uint32_t nsb;
    uint16_t nwosb;
    uint8_t neosb;
    uint8_t reserved40[63 - 30];
};

/**
 *  @brief	Namespace Management (D1) Create(1) - QoS Domain Information Data Structure for Creation
 */
struct NamespaceManagement
{
    uint64_t nsze;                     //!< number of adus
    uint64_t ncap;                     //!< number of adus guarenteed
    uint8_t reserved16[10];            //!< Reserved
    uint8_t flbas;                     //!< formatted lba size
    uint8_t reserved27[28 - 26];       //!< Reserved
    uint8_t dps;                       //!< End-to-end Data protection type settings
    uint8_t reserved30[101 - 29];      //!< FMQ for P4C
    uint16_t endgid;                   //!< Endurance Group ID (Virtual Device ID)
    uint8_t reserved104[767 - 103];    //!< Reserved
    uint32_t qosds;                    //!< QoS Domain Settings
    uint16_t nplid;                    //!< Number of placement ids
    uint16_t mxosb;                    //!< Maximum number of open super blocks
    uint16_t wwt;                      //!< Write Weight
    uint16_t ewt;                      //!< Erase Weight
    uint32_t drfid;                    //!< Default Read Fifo
    uint8_t reserved784[831 - 783];    //!< Reserved
    struct NamespaceSBStats sbsta;     //!< Flash statistics
    struct NamespaceSBStats psbsta;    //!< pSLC statistics
    uint64_t rootptr[8];
    uint8_t reserved1024[4096 - 1024];    //!< Reserved
} __attribute__((packed));

/**
 *  @brief	GetLogPage VirtualDeviceInfo(D0)
 * Structure
 */
typedef struct SefLogVdInfo
{
    uint16_t vdid;
    uint8_t crit_warn;
    uint8_t cwc;
    uint32_t cesn;
    uint8_t max_pe;
    uint8_t avg_pe;
    uint16_t nsbdie;
    uint8_t sbbw;
    uint8_t aobw;
    uint16_t rpc;
    uint32_t mxosb;
    uint32_t drfid;
    uint32_t nrf;
    uint32_t npblk;    // num plc block
    uint64_t vdcap;
    uint64_t vdgcap;
    uint64_t vducap;
    uint8_t reserved56[64 - 56];
    struct VirtualDeviceSBStats sbsta;
    struct VirtualDeviceSBStats psbsta;
    uint16_t ass_qosd;
    uint16_t ndie;
    uint16_t die_ids[];
} __attribute__((packed)) SefLogVdInfo;

static_assert(sizeof(SefLogVdInfo) == 196, "SefLogVdInfo size mismatch");

// when getting ns info for sef cs (CNS 05h CSI 30h)
struct SefNsIdentify
{
    uint64_t nsze;                     //!< number of adus
    uint64_t ncap;                     //!< number of adus guarenteed
    uint8_t reserved16[10];            //!< Reserved
    uint8_t flbas;                     //!< formatted lba size
    uint8_t reserved27[28 - 26];       //!< Reserved
    uint8_t dps;                       //!< End-to-end Data protection type settings
    uint8_t reserved30[101 - 29];      //!< FMQ for P4C
    uint16_t endgid;                   //!< Endurance Group ID (Virtual Device ID)
    uint8_t reserved104[767 - 103];    //!< Reserved
    uint32_t qosds;                    //!< QoS Domain Settings
    uint16_t nplid;                    //!< Number of placement ids
    uint16_t mxosb;                    //!< Maximum number of open super blocks
    uint16_t wwt;                      //!< Write Weight
    uint16_t ewt;                      //!< Erase Weight
    uint32_t drfid;                    //!< Default Read Fifo
    uint8_t reserved784[831 - 783];    //!< Reserved
    struct NamespaceSBStats sbsta;     //!< Flash statistics
    struct NamespaceSBStats psbsta;    //!< pSLC statistics
    uint64_t rootptr[8];
    uint8_t reserved1024[4096 - 1024];    //!< Reserved
};

/**
 *  @brief	Response of GetFeatures QoSDomainManagement(D1) GetInformation(0) - QoS Domain
 * Information Data Structure
 */
struct GetFeaturesQSDManInfo
{
    uint32_t QOSDID : 16;      //!< [DW00][15:00] QoS Domain ID  (QOSDID)
    uint32_t VDID : 16;        //!< [DW00][31:16] Virtual Device ID (VDID)
    uint32_t RDL : 2;          //!< [DW01][01:00] Read Dead-line (RDL)
    uint32_t DMT : 1;          //!< [DW01][   02] Defect Management Type (DMT)
    uint32_t ENC : 1;          //!< [DW01][   03] Encryption (ENC)
    uint32_t HSN : 1;          //!< [DW01][   04] Host S/N (HSN):
    uint32_t reserved : 11;    //!< [DW01][15:05] Reserved
    uint32_t NPLID : 8;        //!< [DW01][23:16] Number of Placement IDs (NPLID)
    uint32_t reserved1 : 8;    //!< [DW01][31:24] Reserved
    uint32_t QRR : 8;          //!< [DW02][07:00] FMQ for R4R (QRR)
    uint32_t QPW : 8;          //!< [DW02][15:08] FMQ for P4W (QPW)
    uint32_t QRC : 8;          //!< [DW02][23:16] FMQ for R4C (QRC)
    uint32_t QPC : 8;          //!< [DW02][31:24] FMQ for P4C (QPC)
    uint32_t reserved2;        //!< [DW03]        Reserved
    uint64_t GCAP;             //!< [DW04-05]     Guaranteed Number of ADUs (GCAP)
    uint64_t QUOTA;            //!< [DW06-07]     Maximum Number of ADUs (QUOTA)
    uint64_t NUMADU;           //!< [DW08-09]     Number of ADUs Used (NUMADU)
    uint32_t NSB : 16;         //!< [DW10][15:00] Number of Super Blocks (NSB)
    uint32_t NWOSB : 8;    //!< [DW10][23:16] Number of Open Super Blocks for Nameless Write (NWOSB)
    uint32_t NCOSB : 8;    //!< [DW10][31:24] Number of Open Super Blocks for Nameless Copy (NCOSB)
    uint32_t NOESB : 8;    //!< [DW11][07:00] Number of Open Super Blocks by Erase (NOESB)
    uint32_t reserved4 : 24;    //!< [DW11][31:08] Reserved
    uint64_t RPTR[];            //!< [DW12-]       Root Pointer-X (RPTRX)
} __attribute__((packed));

/**
 *  @brief	CallbackStructs for Flush Super Block
 *
 *  SEF API doesn't define an IOCB
 */
struct FlushSuperBlockIOCB
{
    struct SEFCommonIOCB common;
    struct SEFFlashAddress flashAddress;      //!< Address of the superblock
    uint32_t distanceToEndOfSuperBlock;       //!< Return value in units of ADUs
    struct SEFAllocateOverrides overrides;    //!< Override parameters for scheduling purposes
};

/**
 *  @brief	CallbackStructs for Patrol Super Block
 */
struct PatrolSuperBlockIOCB
{
    struct SEFCommonIOCB common;
    struct SEFFlashAddress flashAddress;    //!< Address of the superblock
};

/**
 *  @brief	Nameless Copy - User Address Range
 */
struct NamelessCopyUserAddressRange
{
    uint64_t startUserAddress;    //!< Starting User Address (SUA)
    uint64_t length;              //!< Length of Range (LR)
};

/**
 *  @brief	Structure of data retained for NamelessCopy command
 *  @note    Following data is set to DPTR of NamelessCopy command:
 *  @note    Data(1): UserAddressRange (UAR) => struct NamelessCopyUserAddressRange
 *  @note    Data(2): Source Flash Address Information (SFLA) => union NamlessCopySource
 *  @note    Data(3): Address Change Request (ACR) => struct SEFAddressChangeRequest
 */
struct CopyContext
{
    struct SEFAddressChangeRequest* acr;
    char bm_or_fal[];
};

#ifdef __cplusplus
}
#endif

#endif /* __SEF_API_H__ */
