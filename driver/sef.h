/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef.h
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

#ifndef _SEF_H
#define _SEF_H

#include <linux/atomic.h>
#include <linux/nvme.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/percpu-rwsem.h>
#include <uapi/linux/nvme_ioctl.h>
#include <linux/hashtable.h>
#include <linux/version.h>
#include <linux/timer.h>

#include "../../drivers/nvme/host/nvme.h"

#define SEF_MAX_MINORS 2
#define SEF_WARN_ON_IO_CNT 768 // 0 disables code

// todo: remove - dangerous as it has to match what's in the linux tree so as
//       we move to newer kernels, this will have to change values. Issue #100
#define NVME_QUIRK_SEF_CS (1 << 19)
#define NVME_CSI_SEF 0x30
#define NVME_ID_CNS_EG_LIST 0x19

// clang-format off
#define NVME_CAP_CFG_MGMT		0x20
#define NVME_ADM_CMD_SET_FEATURES	0x9
#define NVME_ADM_CMD__NS_MGMT		0xd
#define NVME_SEF_VD_MANAGEMENT		0xd0
#define NVME_SEF_QD_MANAGEMENT		0xd1
// clang-format on

#define ORDER_SB_BUCKETS 5

struct sef_obj {
	struct file *file;
	struct cdev dev;
	struct device *device;
	char dev_name[32];
	int instance;
	struct kref kref;
};

struct sef_qosd {
	struct sef_obj obj;
	int nsid;
	int sbid_bits;
	struct nvme_ns *ns;
	struct list_head link;
	struct rw_semaphore qosds_lock;
	struct list_head unpaired;
	struct mutex unpaired_lock;
	struct timer_list unpaired_timer;
	struct work_struct unpaired_work;
	struct sef_qosd_info *qinfo;
	struct sef_device *parent;
};

struct sef_device {
	struct sef_obj obj;
	struct nvme_ctrl *ctrl;
	struct list_head qosds;
	struct percpu_rw_semaphore effects_lock;
	struct rw_semaphore qosds_lock;
	struct sef_unit_info *id_info;
	struct rw_semaphore vdevs_lock;
	struct list_head vdevs;
#if SEF_WARN_ON_IO_CNT
	struct ratelimit_state rls;
	atomic_t io_cnt;
#endif
};

struct sef_aen_info {
	struct socket *aen_sock;
};

extern struct workqueue_struct *sef_wq;

int sef_queue_add_qosd(int dev, int nsid);
int sef_queue_del_qosd(int dev, int nsid);
int sef_queue_add_dev(int dev);
int sef_queue_del_dev(int dev);
int sef_configure_sys(struct sef_device *sef_device);
int sef_sys_qosd_add(struct sef_qosd *sef_qosd);
void sef_delete_sys(struct sef_device *sef_device);
void sef_delete_qosd_sys(struct sef_qosd *sef_qosd);
int sef_rescan_vd(struct sef_device *sef_device);
void sef_vd_validate_or_add(struct sef_device *sef_device, int vd_id);
int sef_update_qosd_info(struct sef_qosd *sef_qosd);
int sef_get_log_page(struct nvme_ctrl *ctrl, u32 nsid, u8 log_page, u8 lsp,
		     u8 csi, u16 lsi, void *log, size_t size, u64 offset);
int sef_setf(struct nvme_ctrl *ctrl, u32 nsid, u32 fid, u32 cdw11,
	     void *payload, size_t size);
void sef_qosd_get(struct sef_qosd *);
void sef_qosd_put(struct sef_qosd *);
void sef_device_get(struct sef_device *);
void sef_device_put(struct sef_device *);
struct sef_device *sef_dev_find_and_get(int devidx);
struct sef_qosd *sef_qosd_find_and_get(struct sef_device *sef_device, u16 qosd);

#endif
