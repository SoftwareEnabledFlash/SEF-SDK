/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_core.c
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
#include <linux/module.h>

#include "../../drivers/nvme/host/nvme.h"
#include <linux/io_uring.h>
#include <linux/nvme.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <linux/printk.h>
#include <linux/netlink.h>
#include <linux/idr.h>
#include <uapi/linux/netlink.h>
#include <linux/list.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <uapi/linux/nvme_ioctl.h>

#include "sef.h"
#include "sef_aen.h"
#include "sef_defs.h"

// clang-format off
#define SEF_MAJOR	422
#define SEF_OPCODE_FAR	0xd2
#define SEF_OPCODE_NLW	0xd1
#define SEF_OPCODE_READ	0xd6
#define SEF_OPCODE_NLC	0xdd
#define SEF_OPCODE_GCR	0xde
#define SEF_OPCODE_SBM	0xda
#define SEF_SBM_FLUSH	4
#define SEF_UNPAIRED_CHECK_TIME (jiffies + msecs_to_jiffies(10 * 1000))
#define MAX_UNPAIRED_TIME (msecs_to_jiffies(30 * 1000))
// clang-format on

#ifndef SEFSDKVersion
#define SEFSDKVersion "unavailable"
#endif

static __attribute((used)) char SdkVersion[] = "@(#) SEF_SDK version " SEFSDKVersion;

static bool swap_pair_order = false;
module_param(swap_pair_order, bool, 0644);
MODULE_PARM_DESC(swap_pair_order,
		 "re-order far/nlw and nlc/gcr commands before submission");

struct workqueue_struct *sef_wq;

struct sef_add_qosd_req {
	struct work_struct work;
	int devidx;
	int nsid;
	bool add;
};

struct sef_request {
	struct sef_qosd *sef_qosd;
	struct nvme_ctrl *ctrl;
	struct nvme_command cmd;
	struct request *rq;
	struct work_struct work;
	struct nvme_ns *ns;
	int status;
	__u64 result;
	struct io_uring_cmd *uring_cmd;
	void __user *data;
	unsigned int data_len;
	void __user *metadata;
	unsigned int metadata_len;
	void *metap;
	bool is_vec;
	struct completion comp;
	struct list_head link;
	struct bio *bio;
	struct sef_request *paired_req;
	uint64_t start_time;
	uint64_t timeout;
};

struct sef_pdu {
	struct sef_request *sef_req;
};

static int sef_add_qosd_by_idx(int, int);
static int sef_del_qosd(int, int);
static void sef_qosd_delete(struct kref *kref);
static void sef_device_delete(struct kref *kref);

void sef_qosd_get(struct sef_qosd *sef_qosd)
{
	kref_get(&sef_qosd->obj.kref);
}

void sef_qosd_put(struct sef_qosd *sef_qosd)
{
	kref_put(&sef_qosd->obj.kref, sef_qosd_delete);
}

void sef_device_get(struct sef_device *sef_device)
{
	kref_get(&sef_device->obj.kref);
}

void sef_device_put(struct sef_device *sef_device)
{
	kref_put(&sef_device->obj.kref, sef_device_delete);
}

int sef_chk_cmd_set(struct nvme_ctrl *ctrl)
{
	struct nvme_command c = {};
	__le64 *cmd_set_list = kzalloc(NVME_IDENTIFY_DATA_SIZE, GFP_KERNEL);
	int ret = 0;

	if (!cmd_set_list)
		return -ENOMEM;

	c.identify.opcode = nvme_admin_identify;
	c.identify.cns = cpu_to_le32(0x1c);
	c.identify.ctrlid = 0xffff;

	ret = nvme_submit_sync_cmd(ctrl->admin_q, &c, cmd_set_list,
				   NVME_IDENTIFY_DATA_SIZE);
	if (ret) {
		ret = -ENOTTY;
		goto end;
	}

	if (le64_to_cpu(cmd_set_list[0]) & ((__le64)1 << 0x30))
		ret = 0;
	else
		ret = -ENOTTY;

end:
	kfree(cmd_set_list);
	return ret;
}

int sef_get_log_page(struct nvme_ctrl *ctrl, u32 nsid, u8 log_page, u8 lsp,
		     u8 csi, u16 lsi, void *log, size_t size, u64 offset)
{
	struct nvme_command c = {};
	u32 dwlen = nvme_bytes_to_numd(size);

	c.get_log_page.opcode = nvme_admin_get_log_page;
	c.get_log_page.nsid = cpu_to_le32(nsid);
	c.get_log_page.lid = log_page;
	c.get_log_page.lsp = lsp;
	c.get_log_page.numdl = cpu_to_le16(dwlen & ((1 << 16) - 1));
	c.get_log_page.numdu = cpu_to_le16(dwlen >> 16);
	c.get_log_page.rsvd11 = cpu_to_le16(lsi);
	c.get_log_page.lpol = cpu_to_le32(lower_32_bits(offset));
	c.get_log_page.lpou = cpu_to_le32(upper_32_bits(offset));
	c.get_log_page.csi = csi;

	return nvme_submit_sync_cmd(ctrl->admin_q, &c, log, size);
}

int sef_setf(struct nvme_ctrl *ctrl, u32 nsid, u32 fid, u32 cdw11,
	     void *payload, size_t size)
{
	struct nvme_command c = {};

	c.features.opcode = nvme_admin_set_features;
	c.features.nsid = cpu_to_le32(nsid);
	c.features.fid = cpu_to_le32(fid);
	c.features.dword11 = cpu_to_le32(cdw11);

	return nvme_submit_sync_cmd(ctrl->admin_q, &c, payload, size);
}

struct sef_qosd *sef_qosd_find_and_get(struct sef_device *sef_device, u16 qosd)
{
	struct sef_qosd *sef_qosd = NULL;
	struct list_head *l;
	down_read(&sef_device->qosds_lock);
	list_for_each(l, &sef_device->qosds) {
		struct sef_qosd *s = list_entry(l, struct sef_qosd, link);
		if (s->nsid == qosd) {
			sef_qosd = s;
			sef_qosd_get(s);
			break;
		}
	}
	up_read(&sef_device->qosds_lock);
	return sef_qosd;
}

struct sef_qosd *sef_qosd_find_and_get_by_idx(struct sef_device *sef_device,
					      int index)
{
	struct sef_qosd *sef_qosd = NULL;
	struct list_head *l;
	down_read(&sef_device->qosds_lock);
	list_for_each(l, &sef_device->qosds) {
		struct sef_qosd *s = list_entry(l, struct sef_qosd, link);
		if (s->obj.instance == index) {
			sef_qosd = s;
			sef_qosd_get(s);
			break;
		}
	}
	up_read(&sef_device->qosds_lock);
	return sef_qosd;
}

struct sef_request *sef_uring_cmd_req(struct io_uring_cmd *ioucmd)
{
	struct sef_pdu *pdu = (struct sef_pdu *)&ioucmd->pdu;
	return pdu->sef_req;
}

static void sef_passthru_execute_cmd_work(struct work_struct *work)
{
	struct sef_request *sef_req =
		container_of(work, struct sef_request, work);
	struct request *rq = sef_req->rq;
	int status;
	u64 result;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	u32 effects;
	effects = nvme_passthru_start(sef_req->ctrl, sef_req->ns,
				      sef_req->cmd.common.opcode);
	status = nvme_execute_rq(rq, false);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
	u32 effects;
	status = nvme_execute_passthru_rq(rq, &effects);
#else
	status = nvme_execute_passthru_rq(rq);
#endif
	result = le64_to_cpu(nvme_req(sef_req->rq)->result.u64);

	blk_mq_free_request(rq);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
	if (effects)
		nvme_passthru_end(sef_req->ctrl, sef_req->ns, effects,
				 &sef_req->cmd, status);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	if (effects)
		nvme_passthru_end(sef_req->ctrl, effects, &sef_req->cmd,
				  status);
#endif
	if (sef_req->uring_cmd) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		io_uring_cmd_done(sef_req->uring_cmd, status, result,
				  IO_URING_F_UNLOCKED);
#else
		io_uring_cmd_done(sef_req->uring_cmd, status, result);
#endif
		kfree(sef_req);
	} else
		complete(&sef_req->comp);
}

static void *sef_add_user_metadata(struct bio *bio, void __user *ubuf,
				   unsigned len, u32 seed, bool write)
{
	struct bio_integrity_payload *bip;
	int ret = -ENOMEM;
	void *buf;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		goto out;

	ret = -EFAULT;
	if (write && copy_from_user(buf, ubuf, len))
		goto out_free_meta;

	bip = bio_integrity_alloc(bio, GFP_KERNEL, 1);
	if (IS_ERR(bip)) {
		ret = PTR_ERR(bip);
		goto out_free_meta;
	}

	bip->bip_iter.bi_size = len;
	bip->bip_iter.bi_sector = seed;
	ret = bio_integrity_add_page(bio, virt_to_page(buf), len,
				     offset_in_page(buf));
	if (ret == len)
		return buf;
	ret = -ENOMEM;
out_free_meta:
	kfree(buf);
out:
	return ERR_PTR(ret);
}

static int sef_map_user_buffers(struct request_queue *q,
				struct sef_request *sef_req,
				struct block_device *bdev, bool write)
{
	struct request *rq = sef_req->rq;
	void __user *data = sef_req->data;
	unsigned data_len = sef_req->data_len;
	void __user *metadata = sef_req->metadata;
	unsigned metadata_len = sef_req->metadata_len;
	struct bio *bio = NULL;
	void *meta = NULL;
	int ret;

	if (data && data_len) {
		if (!sef_req->is_vec) {
			ret = blk_rq_map_user(q, rq, NULL, data, data_len,
					      GFP_KERNEL);
		} else {
			struct iovec fast_iov[UIO_FASTIOV];
			struct iovec *iov = fast_iov;
			struct iov_iter iter;
			ret = import_iovec(rq_data_dir(rq), data, data_len,
					   UIO_FASTIOV, &iov, &iter);
			if (ret < 0)
				goto out;
			ret = blk_rq_map_user_iov(q, rq, NULL, &iter,
						  GFP_KERNEL);
			if (iov != fast_iov)
				kfree(iov);
		}
		if (ret)
			goto out;
		bio = rq->bio;
		if (bdev)
			bio_set_dev(bio, bdev);
	}

	if (bdev && metadata && metadata_len) {
		if (!bio) {
			ret = -ENOMEM;
			// metadata fits in a page so 1 segment
			bio = bio_kmalloc(1, GFP_KERNEL);
			if (!bio)
				goto out;
			bio_init(bio, NULL, NULL, 1, req_op(rq));
			bio_set_dev(bio, bdev);
			rq->bio = bio;
		}
		meta = sef_add_user_metadata(bio, metadata, metadata_len, 0,
					     write);
		if (IS_ERR(meta)) {
			ret = PTR_ERR(meta);
			goto out_unmap;
		}
		rq->cmd_flags |= REQ_INTEGRITY;
		sef_req->metap = meta;
	}

	sef_req->bio = bio;

	return 0;

out_unmap:
	if (bio)
		blk_rq_unmap_user(bio);
out:
	blk_mq_free_request(rq);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static void sef_uring_task_cb(struct io_uring_cmd *ioucmd, unsigned issue_flags)
#else
static void sef_uring_task_cb(struct io_uring_cmd *ioucmd)
#endif
{
	struct sef_request *sef_req = sef_uring_cmd_req(ioucmd);
	struct request *rq = sef_req->rq;
	struct sef_qosd *sef_qosd = sef_req->sef_qosd;

	if (rq) {
		if (sef_req->metadata) {
			if (!sef_req->status && req_op(rq) == REQ_OP_DRV_IN &&
			    copy_to_user(sef_req->metadata, sef_req->metap,
					 sef_req->metadata_len))
				sef_req->status = -EFAULT;
			kfree(sef_req->metap);
		}

		if (sef_req->bio)
			blk_rq_unmap_user(sef_req->bio);
		blk_mq_free_request(rq);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	io_uring_cmd_done(ioucmd, sef_req->status, sef_req->result,
			  issue_flags);
#else
	io_uring_cmd_done(ioucmd, sef_req->status, sef_req->result);
#endif
	sef_qosd_put(sef_qosd);
	kfree(sef_req);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
#define SEF_IO_CB_RET_TYPE enum rq_end_io_ret
#define SEF_END_IO_NONE RQ_END_IO_NONE
#else
#define SEF_IO_CB_RET_TYPE void
#define SEF_END_IO_NONE
#endif
static SEF_IO_CB_RET_TYPE sef_passthru_req_done(struct request *req,
						blk_status_t err)
{
	struct sef_request *sef_req;
	struct io_uring_cmd *ioucmd;
	int status;
	u64 result;

	sef_req = req->end_io_data;
#if SEF_WARN_ON_IO_CNT
	if (sef_req->cmd.common.opcode != SEF_OPCODE_READ)
		atomic_dec(&sef_req->sef_qosd->parent->io_cnt);
#endif
	if (nvme_req(req)->flags & NVME_REQ_CANCELLED)
		status = -EINTR;
	else
		status = nvme_req(req)->status;

	result = le64_to_cpu(nvme_req(req)->result.u64);

	sef_req->status = status;
	sef_req->result = result;

	ioucmd = sef_req->uring_cmd;
	if (ioucmd) {
		/* extract bio before reusing the same field for request */
		/* this takes care of moving rest of completion-work to task context */
		io_uring_cmd_complete_in_task(ioucmd, sef_uring_task_cb);
	} else {
		complete(&sef_req->comp);
	}

	return SEF_END_IO_NONE;
}

static int sef_init_passthru_cmd(struct sef_request *req, unsigned int flags)
{
	struct nvme_ctrl *ctrl = req->ctrl;
	struct request_queue *q = ctrl->admin_q;
	struct nvme_ns *ns = NULL;
	struct request *rq = NULL;
	struct block_device *bdev = NULL;
	unsigned int timeout;
	u16 status;
	int rq_flags = 0;
	blk_mq_req_flags_t blk_flags = 0;
	int ret = -EINVAL;

	if (likely(req->sef_qosd)) {
		ns = req->sef_qosd->ns;
		bdev = ns->disk->part0;
		q = ns->queue;
		timeout = 60;
	} else {
		timeout = 60;
	}

	if (flags & IO_URING_F_NONBLOCK) {
		rq_flags = REQ_NOWAIT;
		blk_flags = BLK_MQ_REQ_NOWAIT;
	}

	rq = blk_mq_alloc_request(q, nvme_req_op(&req->cmd) | rq_flags,
				  blk_flags);
	if (IS_ERR(rq)) {
		status = NVME_SC_INTERNAL;
		ret = PTR_ERR(rq);
		goto out;
	}
	nvme_init_request(rq, &req->cmd);
	nvme_req(rq)->flags |= NVME_REQ_USERCMD;

	if (req->timeout)
		rq->timeout = req->timeout;
	else if (timeout)
		rq->timeout = timeout * HZ;

	req->rq = rq;

	ret = sef_map_user_buffers(q, req, bdev, true);
	if (ret)
		goto out_put_req;

	return 0;

out_put_req:
	blk_mq_free_request(rq);
out:
	req->status = status;

	return ret;
}

static void sef_submit_passthru_cmd(struct sef_request *req)
{
	struct nvme_ctrl *ctrl = req->ctrl;
	struct nvme_ns *ns = NULL;
	struct request *rq = req->rq;
	u32 effects;

	/*
	 * If there are effects for the command we are about to execute, or
	 * an end_req function we need to use nvme_execute_passthru_rq()
	 * synchronously in a work item seeing the end_req function and
	 * nvme_passthru_end() can't be called in the request done callback
	 * which is typically in interrupt context.
	 */
	effects = nvme_command_effects(ctrl, ns, req->cmd.common.opcode);
	if (effects) {
		INIT_WORK(&req->work, sef_passthru_execute_cmd_work);
		req->rq = rq;
		queue_work(sef_wq, &req->work);
	} else {
#if SEF_WARN_ON_IO_CNT
		if (req->cmd.common.opcode != SEF_OPCODE_READ) {
			int io_cnt = atomic_fetch_inc(
					     &req->sef_qosd->parent->io_cnt) +
				     1;

			if (io_cnt > SEF_WARN_ON_IO_CNT &&
			    __ratelimit(&req->sef_qosd->parent->rls))
				printk(KERN_WARNING
				       "%s: active i/o cnt %d is > %d\n",
				       __func__, io_cnt, SEF_WARN_ON_IO_CNT);
		}
#endif
		rq->end_io = sef_passthru_req_done;
		rq->end_io_data = req;
		blk_execute_rq_nowait(rq, false);
	}
}

void sef_init_request(struct sef_request *sef_req, struct sef_qosd *sef_qosd,
		      const struct nvme_uring_cmd *cmd, bool vec)
{
	struct nvme_command *c = &sef_req->cmd;

	memset(c, 0, sizeof(c));
	c->common.opcode = cmd->opcode;
	c->common.flags = cmd->flags;
	c->common.nsid = cpu_to_le32(cmd->nsid);
	c->common.cdw2[0] = cpu_to_le32(cmd->cdw2);
	c->common.cdw2[1] = cpu_to_le32(cmd->cdw3);
	c->common.cdw10 = cpu_to_le32(cmd->cdw10);
	c->common.cdw11 = cpu_to_le32(cmd->cdw11);
	c->common.cdw12 = cpu_to_le32(cmd->cdw12);
	c->common.cdw13 = cpu_to_le32(cmd->cdw13);
	c->common.cdw14 = cpu_to_le32(cmd->cdw14);
	c->common.cdw15 = cpu_to_le32(cmd->cdw15);

	if (cmd->timeout_ms)
		sef_req->timeout = msecs_to_jiffies(cmd->timeout_ms);
	sef_req->data = (void __user *)(cmd->addr);
	sef_req->data_len = cmd->data_len;
	sef_req->metadata = (void __user *)(cmd->metadata);
	sef_req->metadata_len = cmd->metadata_len;
	sef_req->is_vec = vec;
	sef_req->sef_qosd = sef_qosd;
	sef_req->ctrl = sef_qosd->parent->ctrl;
}

struct sef_request *sef_get_match_or_save(struct sef_qosd *sef_qosd,
					  struct sef_request *sef_req)
{
	struct sef_request *matching = NULL;
	u32 opcode = sef_req->cmd.common.opcode;
	u32 id = sef_req->cmd.common.cdw13;
	u32 nsid = sef_req->cmd.common.nsid;
	struct list_head *l, *t;
	u32 opcode_match;

	switch (opcode) {
	case SEF_OPCODE_FAR:
		opcode_match = SEF_OPCODE_NLW;
		break;
	case SEF_OPCODE_NLW:
		opcode_match = SEF_OPCODE_FAR;
		break;
	case SEF_OPCODE_NLC:
		opcode_match = SEF_OPCODE_GCR;
		break;
	case SEF_OPCODE_GCR:
		opcode_match = SEF_OPCODE_NLC;
		break;
	default:
		printk(KERN_ERR "Invalid opcode seeking match\n");
		return NULL;
	}

	mutex_lock(&sef_qosd->unpaired_lock);
	list_for_each_safe(l, t, &sef_qosd->unpaired) {
		struct sef_request *req =
			list_entry(l, struct sef_request, link);
		struct nvme_command *n = &req->cmd;
		if (n->common.cdw13 == id && nsid == n->common.nsid &&
		    opcode_match == n->common.opcode) {
			list_del(&req->link);
			matching = req;
			break;
		}
	}
	if (!matching) {
		sef_req->start_time = jiffies;
		list_add_tail(&sef_req->link, &sef_qosd->unpaired);
	}
	mutex_unlock(&sef_qosd->unpaired_lock);
	return matching;
}

static int sef_submit_uring_cmd(struct io_uring_cmd *ioucmd,
				unsigned int issue_flags,
				struct sef_device *sef_device,
				struct sef_qosd *sef_qosd)
{
	struct sef_request *sef_req;
	struct sef_pdu *pdu = (struct sef_pdu *)&ioucmd->pdu;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
	struct nvme_uring_cmd const *cmd = io_uring_sqe_cmd(ioucmd->sqe);
#else
	struct nvme_uring_cmd const *cmd = ioucmd->cmd;
#endif
	int ret;
	bool is_vec = false;
	struct sef_request *matching = NULL;
	bool swap_cmds = false;
	int req_size = sizeof(struct sef_request);

	switch (cmd->opcode) {
	case SEF_OPCODE_NLW:
		req_size += sizeof(struct sef_arr);
	}

	sef_req = kzalloc(req_size, GFP_KERNEL);
	if (!sef_req)
		return -ENOMEM;

	switch (ioucmd->cmd_op) {
	case NVME_URING_CMD_IO:
		is_vec = false;
		break;
	case NVME_URING_CMD_IO_VEC:
		is_vec = true;
		break;
	default:
		ret = -ENOTTY;
		goto free_req;
	}

	sef_init_request(sef_req, sef_qosd, cmd, is_vec);
	sef_req->uring_cmd = (struct io_uring_cmd *)ioucmd;
	pdu->sef_req = sef_req;

	switch (sef_req->cmd.common.opcode) {
	case SEF_OPCODE_FAR:
	case SEF_OPCODE_NLC:
		swap_cmds = true;
		fallthrough;
	case SEF_OPCODE_NLW:
	case SEF_OPCODE_GCR: {
		matching = sef_get_match_or_save(sef_qosd, sef_req);
		if (matching) {
			if (swap_pair_order)
				swap_cmds = !swap_cmds;

			ret = sef_init_passthru_cmd(matching, issue_flags);
			if (ret)
				goto fail;
		} else {
			return -EIOCBQUEUED;
		}
	} break;
	default:
		break;
	}

	ret = sef_init_passthru_cmd(sef_req, issue_flags);
	if (ret)
		goto fail;

	if (matching) {
		struct blk_plug plug;

		struct sef_request *first = swap_cmds ? sef_req : matching;
		struct sef_request *second = swap_cmds ? matching : sef_req;

		blk_start_plug(&plug);
		percpu_down_read(&sef_device->effects_lock);
		sef_submit_passthru_cmd(first);
		sef_submit_passthru_cmd(second);
		percpu_up_read(&sef_device->effects_lock);
		blk_finish_plug(&plug);
	} else
		sef_submit_passthru_cmd(sef_req);

	return -EIOCBQUEUED;

fail:

	if (ret == -EAGAIN)
		ret = -EBUSY;

	if (matching) {
		matching->status = ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		sef_uring_task_cb(matching->uring_cmd, IO_URING_F_UNLOCKED);
#else
		sef_uring_task_cb(matching->uring_cmd);
#endif
	}

free_req:
	kfree(sef_req);

	return ret;
}

static int sef_qosd_open(struct inode *inode, struct file *file)
{
	struct sef_qosd *sef_qosd =
		container_of(inode->i_cdev, struct sef_qosd, obj.dev);
	file->private_data = sef_qosd;
	sef_qosd_get(sef_qosd);
	return 0;
}

static long sef_qosd_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct sef_qosd *sef_qosd = file->private_data;
	struct sef_device *sef_device = sef_qosd->parent;
	bool is_admin = false;
	bool update_qosd_info = false;
	struct nvme_passthru_cmd c;

	switch (cmd) {
	case NVME_IOCTL_ADMIN_CMD:
	case NVME_IOCTL_ADMIN64_CMD:
		if (copy_from_user(&c, (void __user *)arg, sizeof(c)))
			return -EFAULT;
		switch (c.opcode) {
		case NVME_ADM_CMD_SET_FEATURES:
			if ((c.cdw10 & 0x7fffffff) == NVME_SEF_QD_MANAGEMENT)
				update_qosd_info = true;
			break;
		}

		is_admin = true;
		fallthrough;

	case NVME_IOCTL_IO_CMD:
	case NVME_IOCTL_IO64_CMD: {
		long status;
		if (is_admin) {
			percpu_down_write(&sef_device->effects_lock);
		}
		status = sef_qosd->obj.file->f_op->unlocked_ioctl(
			sef_qosd->obj.file, cmd, arg);
		if (is_admin) {
			percpu_up_write(&sef_device->effects_lock);
			if (status == 0) {
				flush_workqueue(sef_wq);
				if (update_qosd_info) {
					sef_update_qosd_info(sef_qosd);
				}
			}
		}
		return status;
	}
	default:
		return -EINVAL;
	}
}

static int sef_qosd_release(struct inode *inode, struct file *file)
{
	struct sef_qosd *sef_qosd = file->private_data;
	sef_qosd_put(sef_qosd);
	return 0;
}

static int sef_qosd_uring_cmd(struct io_uring_cmd *ioucmd,
			      unsigned int issue_flags)
{
	struct sef_qosd *sef_qosd = ioucmd->file->private_data;
	int ret;
	sef_qosd_get(sef_qosd);
	ret = sef_submit_uring_cmd(ioucmd, issue_flags, sef_qosd->parent,
				   sef_qosd);
	if (ret != -EIOCBQUEUED)
		sef_qosd_put(sef_qosd);
	return ret;
}

const struct file_operations sef_qosd_fops = {
	.owner = THIS_MODULE,
	.open = sef_qosd_open,
	.release = sef_qosd_release,
	.unlocked_ioctl = sef_qosd_ioctl,
	.uring_cmd = sef_qosd_uring_cmd,
};

static SEF_IO_CB_RET_TYPE sef_flush_complete(struct request *rq,
					     blk_status_t err)
{
	if (err)
		printk(KERN_ERR "Flush request failed %d\n", err);
	blk_mq_free_request(rq);
	return SEF_END_IO_NONE;
}

#define MAX_SB_LIST_COUNT (4096 / sizeof(struct sef_sb_list_desc))

static void sef_qosd_flush(struct sef_qosd *sef_qosd)
{
	size_t size = sizeof(struct sef_sb_list_desc) * MAX_SB_LIST_COUNT;
	struct sef_sb_list_desc *sb_list = kzalloc(size, GFP_KERNEL);
	struct sef_sb_list_desc *sb_desc = sb_list;
	int i, ret;

	if (!sb_list) {
		printk(KERN_ERR "Unable to allocate sb list buffer\n");
		return;
	}

	ret = sef_get_log_page(sef_qosd->parent->ctrl, sef_qosd->nsid,
			       SEF_LOG_SB_LIST, 3 << 4, 0, 0, sb_list, size, 0);

	if (ret) {
		printk(KERN_ERR "unable to get sb_list qosd=%d, ret=%d\n",
		       sef_qosd->nsid, ret);
		return;
	}

	for (i = 0; i < MAX_SB_LIST_COUNT; ++i, ++sb_desc) {
		struct request *rq;

		// submit async flush request
		struct nvme_command cmd = {
			.common.opcode = SEF_OPCODE_SBM,
			.common.nsid = sef_qosd->nsid,
			.common.cdw10 = sb_desc->sbid,
			.common.cdw11 = SEF_SBM_FLUSH << 16,
		};

		if (sb_desc->sbid == 0xffffffff)
			goto done;

		if (sb_desc->sbs & 0x3)
			continue;

		rq = blk_mq_alloc_request(sef_qosd->ns->queue, REQ_OP_DRV_OUT,
					  BLK_MQ_REQ_NOWAIT);
		if (IS_ERR(rq)) {
			printk(KERN_ERR "Unable to flush qosd %d sb %d\n",
			       sef_qosd->nsid, sb_desc->sbid);
			continue;
		}
		nvme_init_request(rq, &cmd);
		rq->timeout = 10 * HZ;
		rq->end_io = sef_flush_complete;
		rq->end_io_data = NULL;
		blk_execute_rq_nowait(rq, false);
	}

done:
	kfree(sb_list);
}

static void sef_dev_flush(struct sef_device *sef_device)
{
	struct list_head *l;
	down_read(&sef_device->qosds_lock);
	list_for_each(l, &sef_device->qosds) {
		struct sef_qosd *s = list_entry(l, struct sef_qosd, link);
		sef_qosd_flush(s);
	}
	up_read(&sef_device->qosds_lock);
}

static int sef_dev_open(struct inode *inode, struct file *file)
{
	struct sef_device *sef_device =
		container_of(inode->i_cdev, struct sef_device, obj.dev);
	file->private_data = sef_device;
	sef_device_get(sef_device);
	return 0;
}

static long sef_dev_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct sef_device *sef_device = file->private_data;
	bool is_admin = false;
	bool rescan_vd = false;
	bool update_qosd_info = false;
	bool update_vd_info = false;
	bool flush_sb = false;
	struct nvme_passthru_cmd c;

	switch (cmd) {
	case NVME_IOCTL_ADMIN_CMD:
	case NVME_IOCTL_ADMIN64_CMD: {
		__u32 effects;
		if (copy_from_user(&c, (void __user *)arg, sizeof(c)))
			return -EFAULT;
		effects =
			nvme_command_effects(sef_device->ctrl, NULL, c.opcode);
		if (effects & (NVME_CMD_EFFECTS_NIC | NVME_CMD_EFFECTS_NCC))
			flush_sb = true;
		switch (c.opcode) {
		case NVME_CAP_CFG_MGMT:
		case NVME_ADM_CMD__NS_MGMT:
			rescan_vd = true;
			break;
		case NVME_ADM_CMD_SET_FEATURES:
			if ((c.cdw10 & 0x7fffffff) == NVME_SEF_QD_MANAGEMENT)
				update_qosd_info = true;
			else if ((c.cdw10 & 0x7fffffff) ==
				 NVME_SEF_VD_MANAGEMENT)
				update_vd_info = true;
			break;
		}

		is_admin = true;
		fallthrough;
	}
	case NVME_IOCTL_IO_CMD:
	case NVME_IOCTL_IO64_CMD: {
		long status;
		if (is_admin) {
			percpu_down_write(&sef_device->effects_lock);
		}
		if (flush_sb) {
			sef_dev_flush(sef_device);
		}
		status = sef_device->obj.file->f_op->unlocked_ioctl(
			sef_device->obj.file, cmd, arg);
		if (is_admin) {
			percpu_up_write(&sef_device->effects_lock);
			if (status == 0) {
				flush_workqueue(sef_wq);
				if (rescan_vd)
					sef_rescan_vd(sef_device);
				else if (update_vd_info)
					sef_vd_validate_or_add(sef_device,
							       c.nsid);
				if (update_qosd_info) {
					struct sef_qosd *sef_qosd =
						sef_qosd_find_and_get(
							sef_device, c.nsid);
					if (sef_qosd) {
						sef_update_qosd_info(sef_qosd);
						sef_qosd_put(sef_qosd);
					} else
						printk(KERN_ERR
						       "No qosd %d found\n",
						       c.nsid);
				}
			}
		}
		return status;
	}
	default:
		return -EINVAL;
	}
}

static int sef_dev_release(struct inode *inode, struct file *file)
{
	struct sef_device *sef_device = file->private_data;
	sef_device_put(sef_device);
	return 0;
}

static int sef_dev_uring_cmd(struct io_uring_cmd *ioucmd,
			     unsigned int issue_flags)
{
	struct sef_device *sef_device = ioucmd->file->private_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
	struct nvme_uring_cmd const *cmd = io_uring_sqe_cmd(ioucmd->sqe);
#else
	const struct nvme_uring_cmd *cmd = ioucmd->cmd;
#endif
	struct sef_qosd *sef_qosd =
		sef_qosd_find_and_get(sef_device, cmd->nsid);
	int ret;

	if (!sef_qosd) {
		return -ENODEV;
	}
	ret = sef_submit_uring_cmd(ioucmd, issue_flags, sef_device, sef_qosd);
	if (ret != -EIOCBQUEUED)
		sef_qosd_put(sef_qosd);
	return ret;
}

const struct file_operations sef_dev_fops = {
	.owner = THIS_MODULE,
	.open = sef_dev_open,
	.release = sef_dev_release,
	.unlocked_ioctl = sef_dev_ioctl,
	.uring_cmd = sef_dev_uring_cmd,
};

#define MAX_SEF_DEVICES 256
#define MAX_SEF_NS_MINORS (1 << 20)
static struct sef_device *l_sef_devices[MAX_SEF_DEVICES];
static struct sef_aen_info l_sef_aen_info;
static DECLARE_RWSEM(l_sef_devices_lock);

static DEFINE_IDA(sef_minor_ida);
static dev_t sef_devt = SEF_MAJOR;
static struct class *sef_class;

static DEFINE_IDA(sef_qosd_minor_ida);
static dev_t sef_qosd_devt;
static struct class *sef_qosd_class;

struct sef_device *sef_dev_find_and_get(int devidx)
{
	struct sef_device *sef_device = NULL;
	if (devidx < MAX_SEF_DEVICES) {
		down_read(&l_sef_devices_lock);
		sef_device = l_sef_devices[devidx];
		if (sef_device) {
			sef_device_get(sef_device);
		}
		up_read(&l_sef_devices_lock);
	}
	return sef_device;
}

static void sef_qosd_cdev_rel(struct device *dev)
{
	ida_free(&sef_qosd_minor_ida, MINOR(dev->devt));
}

static void sef_cdev_rel(struct device *dev)
{
	ida_free(&sef_minor_ida, MINOR(dev->devt));
}

void sef_cdev_del(struct cdev *cdev, struct device *cdev_device)
{
	cdev_device_del(cdev, cdev_device);
	put_device(cdev_device);
}

int sef_cdev_add(struct sef_obj *sef_obj, const struct file_operations *fops,
		 void (*release)(struct device *), struct class *class,
		 struct ida *ida, dev_t devt)
{
	int minor, ret;
	struct cdev *cdev = &sef_obj->dev;
	struct device *device;

	minor = ida_alloc(ida, GFP_KERNEL);
	if (minor < 0)
		return minor;

	cdev_init(cdev, fops);
	ret = cdev_add(&sef_obj->dev, MKDEV(devt, minor), 1);
	if (ret)
		goto ida_free;

	device = device_create(class, NULL, MKDEV(devt, minor), sef_obj,
			       sef_obj->dev_name);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto cdev_del;
	}

	sef_obj->device = device;

	return 0;

cdev_del:
	cdev_del(&sef_obj->dev);
ida_free:
	ida_free(ida, minor);
	return ret;
}

int sef_obj_init(struct sef_obj *obj, const char *nvme_fmt, const char *sef_fmt,
		 int devidx, int qidx, const struct file_operations *fops,
		 void (*release)(struct device *), struct class *class,
		 struct ida *ida, dev_t devt)
{
	struct file *file;
	char nvme_dev_name[32];
	int ret;

	kref_init(&obj->kref);
	obj->instance = qidx ?: devidx;

	snprintf(nvme_dev_name, sizeof(nvme_dev_name), nvme_fmt, devidx, qidx);

	file = filp_open(nvme_dev_name, O_RDWR, 0);
	if (IS_ERR(file)) {
		ret = PTR_ERR(file);
		goto out;
	}

	snprintf(obj->dev_name, sizeof(obj->dev_name), sef_fmt, devidx, qidx);
	ret = sef_cdev_add(obj, fops, release, class, ida, devt);
	if (ret)
		goto close_file;

	obj->file = file;

	return 0;

close_file:
	filp_close(file, NULL);
out:
	return ret;
}

void sef_obj_del(struct sef_obj *obj)
{
	sef_cdev_del(&obj->dev, obj->device);
	filp_close(obj->file, NULL);
}

static int sef_qosd_file_get_id(struct sef_obj *sef_obj)
{
	struct sef_qosd *sef_qosd = (struct sef_qosd *)sef_obj;
	struct file *file = sef_obj->file;
	struct nvme_ns *ns;
	int nsid;

	nsid = file->f_op->unlocked_ioctl(file, NVME_IOCTL_ID, 0);
	if (nsid <= 0) {
		return nsid;
	}

	ns = nvme_find_get_ns(sef_qosd->parent->ctrl, nsid);
	if (!ns) {
		return -ENODEV;
	}

	sef_qosd->nsid = nsid;
	sef_qosd->ns = ns;

	return 0;
}

int sef_qosd_obj_init(struct sef_qosd *sef_qosd, int devidx, int qidx)
{
	return sef_obj_init(&sef_qosd->obj, "/dev/ng%dn%d", "sef%dn%d", devidx,
			    qidx, &sef_qosd_fops, sef_qosd_cdev_rel,
			    sef_qosd_class, &sef_qosd_minor_ida,
			    MAJOR(sef_qosd_devt));
}

int sef_dev_obj_init(struct sef_device *sef_device, int devidx)
{
	return sef_obj_init(&sef_device->obj, "/dev/nvme%d", "sef%d", devidx, 0,
			    &sef_dev_fops, sef_cdev_rel, sef_class,
			    &sef_minor_ida, sef_devt);
}

static void sef_qosd_unpaired_check(struct timer_list *timer)
{
	struct sef_qosd *sef_qosd =
		container_of(timer, struct sef_qosd, unpaired_timer);
	queue_work(sef_wq, &sef_qosd->unpaired_work);
	mod_timer(&sef_qosd->unpaired_timer, SEF_UNPAIRED_CHECK_TIME);
}

static void sef_qosd_unpaired_check_work(struct work_struct *work)
{
	struct sef_qosd *sef_qosd =
		container_of(work, struct sef_qosd, unpaired_work);
	unsigned long now = jiffies;
	struct list_head *l, *t;

	mutex_lock(&sef_qosd->unpaired_lock);
	list_for_each_safe(l, t, &sef_qosd->unpaired) {
		struct sef_request *sef_req =
			list_entry(l, struct sef_request, link);
		if (now - sef_req->start_time > MAX_UNPAIRED_TIME) {
			list_del(&sef_req->link);
			sef_req->status = -ETIMEDOUT;
			sef_req->result = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			sef_uring_task_cb(sef_req->uring_cmd,
					  IO_URING_F_UNLOCKED);
#else
			sef_uring_task_cb(sef_req->uring_cmd);
#endif
		}
	}
	mutex_unlock(&sef_qosd->unpaired_lock);
}

int sef_qosd_init(struct sef_qosd *sef_qosd, struct sef_device *parent,
		  int devnum, int instance)
{
	int ret = 0;

	INIT_LIST_HEAD(&sef_qosd->unpaired);
	mutex_init(&sef_qosd->unpaired_lock);

	sef_qosd->parent = parent;
	sef_qosd->sbid_bits = 16;

	INIT_WORK(&sef_qosd->unpaired_work, sef_qosd_unpaired_check_work);
	timer_setup(&sef_qosd->unpaired_timer, sef_qosd_unpaired_check, 0);

	ret = sef_qosd_obj_init(sef_qosd, devnum, instance);
	if (ret)
		goto out;

	ret = sef_qosd_file_get_id(&sef_qosd->obj);
	if (ret)
		goto del_obj;

	down_write(&parent->qosds_lock);
	list_add_tail(&sef_qosd->link, &parent->qosds);
	up_write(&parent->qosds_lock);
	ret = sef_sys_qosd_add(sef_qosd);
	if (ret)
		goto out_put_ns;

	sef_device_get(parent);

	mod_timer(&sef_qosd->unpaired_timer, SEF_UNPAIRED_CHECK_TIME);

	return 0;

out_put_ns:
	nvme_put_ns(sef_qosd->ns);
del_obj:
	sef_obj_del(&sef_qosd->obj);
out:
	return ret;
}

static void sef_qosd_delete(struct kref *kref)
{
	struct sef_qosd *sef_qosd =
		container_of(kref, struct sef_qosd, obj.kref);
	del_timer(&sef_qosd->unpaired_timer);
	flush_work(&sef_qosd->unpaired_work);
	nvme_put_ns(sef_qosd->ns);
	sef_delete_qosd_sys(sef_qosd);
	sef_obj_del(&sef_qosd->obj);
	sef_device_put(sef_qosd->parent);
	if (sef_qosd->qinfo)
		kfree(sef_qosd->qinfo);
	kfree(sef_qosd);
}

int sef_dev_init(struct sef_device *sef_device, int devnum)
{
	struct nvme_ctrl *ctrl;
	int ret = 0;

	INIT_LIST_HEAD(&sef_device->qosds);
	INIT_LIST_HEAD(&sef_device->vdevs);
	init_rwsem(&sef_device->qosds_lock);
	percpu_init_rwsem(&sef_device->effects_lock);
#if SEF_WARN_ON_IO_CNT
	ratelimit_state_init(&sef_device->rls, 30 * HZ, 1);
#endif

	ret = sef_dev_obj_init(sef_device, devnum);
	if (ret)
		goto out;

	ctrl = nvme_ctrl_from_file(sef_device->obj.file);
	if (!ctrl) {
		pr_err("failed to open nvme controller %s\n",
		       sef_device->obj.dev_name);
		ret = -EINVAL;
		goto out_del_obj;
	}

	ret = sef_chk_cmd_set(ctrl);
	if (ret)
		goto out_del_obj;

	nvme_get_ctrl(ctrl);
	__module_get(ctrl->ops->module);

	sef_device->ctrl = ctrl;

	ret = sef_configure_sys(sef_device);
	if (ret)
		goto mod_put;

	return 0;

mod_put:
	module_put(ctrl->ops->module);
	nvme_put_ctrl(ctrl);
out_del_obj:
	sef_obj_del(&sef_device->obj);
out:
	return ret;
}

static void sef_device_delete(struct kref *kref)
{
	struct sef_device *sef_device =
		container_of(kref, struct sef_device, obj.kref);

	if (!list_empty(&sef_device->qosds)) {
		printk("Qosd list not empty on delete\n");
		BUG();
	}

	module_put(sef_device->ctrl->ops->module);
	nvme_put_ctrl(sef_device->ctrl);
	sef_delete_sys(sef_device);
	kfree(sef_device->id_info);
	sef_obj_del(&sef_device->obj);
	kfree(sef_device);
}

static void sef_device_remove(struct sef_device *sef_device)
{
	if (sef_device->obj.instance >= MAX_SEF_DEVICES) {
		printk(KERN_ERR "Invalid sef device being removed\n");
		return;
	}

	down_write(&l_sef_devices_lock);
	if (sef_device != l_sef_devices[sef_device->obj.instance])
		printk(KERN_ERR "Sef device in wrong slot\n");
	l_sef_devices[sef_device->obj.instance] = NULL;
	up_write(&l_sef_devices_lock);

	while (down_write(&sef_device->qosds_lock),
	       !list_empty(&sef_device->qosds)) {
		struct sef_qosd *sef_qosd = list_first_entry(
			&sef_device->qosds, struct sef_qosd, link);
		list_del(&sef_qosd->link);
		up_write(&sef_device->qosds_lock);
		sef_qosd_put(sef_qosd);
	}
	up_write(&sef_device->qosds_lock);

	sef_device_put(sef_device);
}

static int sef_add_qosd(struct sef_device *sef_device, int nsid)
{
	struct sef_qosd *sef_qosd;
	int ret;

	sef_qosd = sef_qosd_find_and_get_by_idx(sef_device, nsid);
	if (sef_qosd) {
		sef_qosd_put(sef_qosd);
		printk(KERN_DEBUG "SEF QoSD %d already exists on device %d\n",
		       nsid, sef_device->obj.instance);
		return -EEXIST;
	}

	sef_qosd = kzalloc(sizeof(*sef_qosd), GFP_KERNEL);
	if (!sef_qosd) {
		ret = -ENOMEM;
		goto end;
	}

	if ((ret = sef_qosd_init(sef_qosd, sef_device, sef_device->obj.instance,
				 nsid)))
		goto free_info;

	goto end;

free_info:
	kfree(sef_qosd);
end:
	return ret;
}

static int sef_add_qosd_by_idx(int devidx, int nsid)
{
	struct sef_device *sef_device = sef_dev_find_and_get(devidx);
	int ret;

	if (!sef_device)
		return -ENODEV;

	ret = sef_add_qosd(sef_device, nsid);
	sef_device_put(sef_device);

	return ret;
}

static int sef_del_qosd(int devidx, int nsid)
{
	struct sef_device *sef_device = sef_dev_find_and_get(devidx);
	int ret = -ENODEV;
	char dev_name[32];
	struct list_head *l, *t;

	if (!sef_device)
		return -ENODEV;
	snprintf(dev_name, sizeof(dev_name), "sef%dn%d", devidx, nsid);
	down_write(&sef_device->qosds_lock);
	list_for_each_safe(l, t, &sef_device->qosds) {
		struct sef_qosd *sef_qosd =
			list_entry(l, struct sef_qosd, link);
		if (strncmp(sef_qosd->obj.dev_name, dev_name,
			    sizeof(sef_qosd->obj.dev_name)) == 0) {
			list_del(&sef_qosd->link);
			up_write(&sef_device->qosds_lock);
			sef_qosd_put(sef_qosd);
			ret = 0;
			goto put_dev;
		}
	}
	up_write(&sef_device->qosds_lock);
put_dev:
	sef_device_put(sef_device);
	return ret;
}

static int sef_add_device(int devidx)
{
	int ret;
	struct sef_device *sef_device = sef_dev_find_and_get(devidx);
	if (sef_device) {
		sef_device_put(sef_device);
		return -EEXIST;
	}
	sef_device = kzalloc(sizeof(*sef_device), GFP_KERNEL);
	if (!sef_device)
		return -ENOMEM;

	if ((ret = sef_dev_init(sef_device, devidx)) != 0)
		kfree(sef_device);
	else {
		int i;
		for (i = 0; i < 1024; ++i) {
			ret = sef_add_qosd(sef_device, i);
		}
		down_write(&l_sef_devices_lock);
		l_sef_devices[sef_device->obj.instance] = sef_device;
		up_write(&l_sef_devices_lock);
	}
	return ret;
}

static int sef_del_device(int devidx)
{
	struct sef_device *sef_device = sef_dev_find_and_get(devidx);
	if (!sef_device)
		return -ENODEV;
	sef_device_remove(sef_device);
	sef_device_put(sef_device);
	return 0;
}

void sef_qosd_work(struct work_struct *work)
{
	struct sef_add_qosd_req *req = (struct sef_add_qosd_req *)work;
	if (req->nsid == -1) {
		if (req->add)
			sef_add_device(req->devidx);
		else
			sef_del_device(req->devidx);
	} else {
		if (req->add)
			sef_add_qosd_by_idx(req->devidx, req->nsid);
		else
			sef_del_qosd(req->devidx, req->nsid);
	}
	kfree(req);
}

int sef_queue_qosd_op(int devidx, int nsid, bool add)
{
	struct sef_add_qosd_req *req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	req->devidx = devidx;
	req->nsid = nsid;
	req->add = add;
	INIT_WORK(&req->work, sef_qosd_work);
	queue_work(sef_wq, &req->work);
	return 0;
}

int sef_queue_add_qosd(int devidx, int nsid)
{
	return sef_queue_qosd_op(devidx, nsid, true);
}

int sef_queue_del_qosd(int devidx, int nsid)
{
	return sef_queue_qosd_op(devidx, nsid, false);
}

int sef_queue_add_dev(int devidx)
{
	return sef_queue_qosd_op(devidx, -1, true);
}

int sef_queue_del_dev(int devidx)
{
	return sef_queue_qosd_op(devidx, -1, false);
}

static void sef_dev_scan_work(struct work_struct *work)
{
	int i;

	for (i = 0; i < MAX_SEF_DEVICES; ++i)
		sef_add_device(i);
}

static int __init sef_init(void)
{
	static struct work_struct sef_dev_work;
	int ret;

	pr_info("SDK version %s\n", SEFSDKVersion);
	sef_wq = alloc_workqueue("sef-wq",
				 WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_SYSFS, 0);
	if (!sef_wq)
		return -ENOMEM;

	ret = register_chrdev_region(MKDEV(SEF_MAJOR, 0), SEF_MAX_MINORS,
				     "sef_driver");
	if (ret != 0) {
		/* report error */
		goto fail;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	sef_class = class_create("sef");
#else
	sef_class = class_create(THIS_MODULE, "sef");
#endif
	if (IS_ERR(sef_class)) {
		ret = PTR_ERR(sef_class);
		goto unregister_sef;
	}

	ret = alloc_chrdev_region(&sef_qosd_devt, 1, MAX_SEF_NS_MINORS - 1,
				  "sef-qosd");
	if (ret < 0)
		goto delete_sef_class;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	sef_qosd_class = class_create("sef-qosd");
#else
	sef_qosd_class = class_create(THIS_MODULE, "sef-qosd");
#endif
	if (IS_ERR(sef_qosd_class)) {
		ret = PTR_ERR(sef_qosd_class);
		goto unregister_ns;
	}

	if ((ret = sef_aen_init(&l_sef_aen_info)) != 0) {
		goto destroy_class;
	}

	INIT_WORK(&sef_dev_work, sef_dev_scan_work);
	queue_work(sef_wq, &sef_dev_work);

	return 0;

destroy_class:
	class_destroy(sef_qosd_class);
unregister_ns:
	unregister_chrdev_region(sef_qosd_devt, MAX_SEF_NS_MINORS - 1);
delete_sef_class:
	class_destroy(sef_class);
unregister_sef:
	unregister_chrdev_region(MKDEV(SEF_MAJOR, 0), SEF_MAX_MINORS);
fail:
	destroy_workqueue(sef_wq);
	return ret;
}

static void __exit sef_exit(void)
{
	int i;

	sef_aen_exit(&l_sef_aen_info);
	flush_workqueue(sef_wq);
	for (i = 0; i < MAX_SEF_DEVICES; ++i) {
		if (l_sef_devices[i])
			sef_device_remove(l_sef_devices[i]);
	}
	unregister_chrdev_region(sef_qosd_devt, MAX_SEF_NS_MINORS - 1);
	unregister_chrdev_region(MKDEV(SEF_MAJOR, 0), SEF_MAX_MINORS);
	class_destroy(sef_class);
	class_destroy(sef_qosd_class);
	destroy_workqueue(sef_wq);
}

module_init(sef_init);
module_exit(sef_exit);
MODULE_IMPORT_NS(NVME_TARGET_PASSTHRU);
MODULE_LICENSE("GPL v2");
