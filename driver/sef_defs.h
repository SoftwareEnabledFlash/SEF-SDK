/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_defs.h
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

#ifndef _SEF_DEFS_H
#define _SEF_DEFS_H

#include <linux/build_bug.h>
#include "sef.h"

enum {
	SEF_LOG_VD_INFO = 0xd0, /* VD info log */
	SEF_LOG_SB_LIST = 0xd2, /* SB list log */
	SEF_LOG_SB_INFO = 0xd3, /* SB info log */
	SEF_LOG_FIFO_INFO = 0xd6, /* Read fifo info log */

	SEF_FEAT_VD = 0xd0, /* virtual device feature id */
	SEF_FEAT_VD_ND = 0, /* get/set num die per super block */
	SEF_FEAT_VD_GRFL = 1, /* get read fifo list */
	SEF_FEAT_VD_RDT = 0xf, /* reset vd die times */

	SEF_FEAT_QOSD = 0xd1, /* qos domain feature id */
	SEF_FEAT_QOSD_CDT = 0 /*2*/, /* set root pointer */
	SEF_FEAT_QOSD_CAP = 1 /*3*/, /* change capacity */
	SEF_FEAT_QOSD_RDL = 3 /*4*/, /* change read deadline */
};

struct sef_sb_info_log {
	__le16 vdid;
	char st;
	char sbs;
	__le32 sbid;
	__le16 pid;
	__le16 qosid;
	char peci;
	char dii;
	__le16 tl;
	__le32 cap;
	__le32 aduptr;
	__le32 esn;
	__le16 dgid;
	__le16 ndpl;
	char rsvd1[128 - 32];
	char dbm[128];
};

struct sef_sb_list_desc {
	u32 sbid;
	u8 peci;
	u8 sbs;
	u8 reserved[2];
};

struct sef_vd_stats {
	__le64 tcap;
	__le64 tgcap;
	__le64 ucap;
	__le32 nfrsb;
	__le32 nclsb;
	__le32 neosb;
	__le32 nowsb;
	char reserved40[64 - 40];
};

static_assert(sizeof(struct sef_vd_stats) == 64, "vd status not 64 bytes");

struct sef_vd_info_log {
	__le16 vdid;
	char crit_warn;
	char cwc;
	__le32 cesn;
	char max_pe;
	char avg_pe;
	__le16 nsbdie;
	char sbbw;
	char aobw;
	__le16 rpc;
	__le32 mxosb;
	__le32 drfid;
	__le32 nrf;
	__le32 npblk; // num plc block
	__le64 vdcap;
	__le64 vdgcap;
	__le64 vducap;
	char reserved56[64 - 56];
	struct sef_vd_stats sbsta;
	struct sef_vd_stats psbsta;
	__le16 ass_qosd;
	__le16 ndie;
	__le16 ids[];
} __attribute__((packed));

static_assert(sizeof(struct sef_vd_info_log) == 196, "not 196 bytes");

struct sef_id_ctrl {
	__le32 cap;
	u8 nch;
	u8 nbnk;
	__le16 nblk;
	__le16 npag;
	u8 npl;
	u8 plds;
	__le16 eop;
	__le16 max_plid;
	__le32 mxosb;
	__le32 nrf;
	u8 npsi;
	u8 rsvd25[39];
	__le32 nurf;
	__le16 nvd;
	__le16 nqosd;
	u8 rsvd72[4096 - 72];
};

static_assert(sizeof(struct sef_id_ctrl) == 4096, "not 4k");

struct sef_qosd_info {
	// todo: only have fields needed for sysfs props.
	//       which are SEFAPI values not requiring an open handle
	u16 qosid;
	u16 vdid;
	u16 nplid;
	u64 gcap;
	u64 quota;
	u64 ucap;
	u64 pgcap;
	u64 pquota;
	u64 pucap;
	u32 nsb;
	u32 pnsb;
	u32 drfid;
	u32 qds;
	u16 msize;
	u16 mxosb;
	u16 wwt;
	u16 ewt;
	u16 nwosb;
	u16 pnwosb;
	u8 neosb;
	u8 pneosb;
	u32 lba_size;
	u64 rootp[8];
};

struct sef_unit_info {
	u32 ctratt;
	u32 cap;
	u32 nblk;
	u32 npag;
	u32 nrf; // num read fifos
	u32 mxosb; // max open super blocks
	u16 max_qosdid;
	u16 nch;
	u16 nbnk;
	u16 npl;
	u16 nadu; // per plane
	u16 eop;
	u16 max_plid;
	u16 nrp;
	u16 nvd;
	u16 nqosd;
	u8 npsi;
};

struct sef_vd {
	struct kobject kobj;
	int vd_id;
	unsigned int sbs; // sb size in units of die
	unsigned int sbc; // sb size in units of adus
	unsigned int nrf; // counted # of fifos
	unsigned int mtps; // max time per suspend
	unsigned int mtus; // min time until suspend
	unsigned int msi; // max suspend interval
	struct sef_vd_info_log *info;
	struct nvme_ctrl *ctrl;
	struct sef_rf_log *fifos;
	struct list_head list;
};

// overlays onto nvme_id_ns.vs[] @ 384 (vendor specific)
struct sef_sb_stats {
	__le64 quota;
	__le64 gcap;
	__le64 ucap;
	__le32 nsb;
	__le16 nwosb;
	__u8 neosb;
	__u8 reserved31[63 - 30];
};

struct sef_vs_id_ns {
	__le64 lbstm;
	__u8 pic;
	__u8 reserved393[395 - 392];
	__le32 elbaf[64];
	__u8 reserved652[767 - 651];
	__u32 qds;
	__u16 nplid;
	__le16 mxosb;
	__le16 wwt;
	__le16 ewt;
	__le32 drfid;
	__u8 reserved784[831 - 783];
	struct sef_sb_stats sbsta; // Flash statistics
	struct sef_sb_stats psbsta; // pSLC statistics
	__le64 rootp[8];
};

struct sef_rf_log {
	__le32 rfid;
	__le16 vdid;
	__le16 weight;
};

struct sef_nvme_identify {
	__u8 opcode;
	__u8 flags;
	__u16 command_id;
	__le32 nsid;
	__u64 rsvd2[2];
	union nvme_data_ptr dptr;
	__u8 cns;
	__u8 rsvd3;
	__le16 ctrlid;
	__le16 nvmsetid;
	__u8 rsvd11;
	__u8 csi;
	__u32 rsvd12[4];
};

struct sef_far_sbi {
	__u32 num_flas;
	__u16 num_dp;
	__u8 reserved6[2];
	__u64 sfa;
};

enum { MAX_SBIS = 8 };

struct sef_arr { // defective bitmaps not included yet
	__u8 num_sbs;
	__u8 dpb_bytes;
	__u16 num_planes;
	__u32 num_adus_left;
	__u8 reserverd8[8];
	struct sef_far_sbi sbis[MAX_SBIS];
};

// async ioctl

/*
 * Check we didn't inadvertently grow the command struct
 */
static inline void _sef_check_size(void)
{
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_identity) != 64);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_ph_rw) != 64);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_erase_blk) != 64);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_getbbtbl) != 64);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_setbbtbl) != 64);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_id12_grp) != 960);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_id12_addrf) != 16);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_id12) != NVME_IDENTIFY_DATA_SIZE);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_bb_tbl) != 64);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_id20_addrf) != 8);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_id20) != NVME_IDENTIFY_DATA_SIZE);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_chk_meta) != 32);
	// BUILD_BUG_ON(sizeof(struct nvme_nvm_chk_meta) !=
	// 					sizeof(struct nvm_chk_meta));
}

#endif
