/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_aen.c
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
#include "../../drivers/nvme/host/nvme.h"

#include "sef.h"
#include <linux/nvme.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <linux/printk.h>
#include <linux/netlink.h>
#include <uapi/linux/netlink.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <uapi/linux/nvme_ioctl.h>
#include "sef_defs.h"

#define __UEVENT_BUFFER_SIZE (2048 * 2)

// clang-format off
#define AEN_TYPE_NS_ADD		1
#define AEN_TYPE_NS_REMOVE	2
#define AEN_TYPE_SEF		3
#define WATCH_TYPE_NVME_AEN	2
// clang-format on

static void sef_send_vd_uevent(struct sef_device *sef_device, u32 aen_result,
			       u16 vdid, u8 crit)
{
	char *envp[4] = {};

	envp[0] = kasprintf(GFP_KERNEL, "SEF_AEN=%#08x", aen_result);
	envp[1] = kasprintf(GFP_KERNEL, "SEF_VDID=%#08x", vdid);
	envp[2] = kasprintf(GFP_KERNEL, "SEF_CRIT=%#08x", crit);
	if (envp[0] && envp[1])
		kobject_uevent_env(&sef_device->obj.device->kobj, KOBJ_CHANGE,
				   envp);
	kfree(envp[0]);
	kfree(envp[1]);
	kfree(envp[2]);
}

static void sef_handle_vd_aen(struct sef_device *sef_device, u32 aen_result)
{
	int ret = -1;
	u16 vdid;
	size_t size;
	struct sef_vd_info_log *log = NULL;
	struct nvme_ctrl *ctrl = sef_device->ctrl;
	u8 crit_warn, cwc;

	size = struct_size(log, ids, 0);
	log = kmalloc(size, GFP_KERNEL);
	if (!log)
		goto Exit; // notifications will stop

	ret = sef_get_log_page(ctrl, 0, SEF_LOG_VD_INFO, 1 << 7, 0, 0xffff, log,
			       size, 0);
	if (ret) // todo: check ret for "abort" status to break
		goto Exit; // assume it's because there's nothing to report

	vdid = le16_to_cpu(log->vdid);
	crit_warn = log->crit_warn;
	cwc = ~crit_warn & 0x7;
	ret = sef_setf(ctrl, vdid, 0xD0, (cwc << 16) | 5, 0, 0);
	if (ret)
		goto Exit;

	ret = sef_get_log_page(ctrl, 0, SEF_LOG_VD_INFO, 0, 0, 0xffff, log,
			       size, 0);
	if (ret) // todo: check ret for "abort" status to break
		goto Exit; // assume it's because there's nothing to report

	sef_send_vd_uevent(sef_device, aen_result, vdid, crit_warn);
Exit:
	kfree(log);
	if (ret)
		printk(KERN_ERR
		       "Err %d reading VD log page - AENs will no longer be processed\n",
		       ret);
}

static void sef_send_sb_uevent(struct sef_device *sef_device, u32 aen_result,
			       u16 qosd, u16 sbid, u8 sbst, u32 nvadu)
{
	char *envp[6] = {};

	envp[0] = kasprintf(GFP_KERNEL, "SEF_AEN=%#08x", aen_result);
	envp[1] = kasprintf(GFP_KERNEL, "SEF_QOSD=%#08x", qosd);
	envp[2] = kasprintf(GFP_KERNEL, "SEF_SBID=%#08x", sbid);
	envp[3] = kasprintf(GFP_KERNEL, "SEF_SBST=%#08x", sbst);
	envp[4] = kasprintf(GFP_KERNEL, "SEF_NVADU=%#08x", nvadu);
	if (envp[0] && envp[1] && envp[2] && envp[3] && envp[4])
		kobject_uevent_env(&sef_device->obj.device->kobj, KOBJ_CHANGE,
				   envp);
	kfree(envp[0]);
	kfree(envp[1]);
	kfree(envp[2]);
	kfree(envp[3]);
	kfree(envp[4]);
}

static void sef_handle_sb_aen(struct sef_device *sef_device, u32 aen_result)
{
	int ret = -1;
	u16 qosd;
	u32 sbid;
	u32 nvadu;
	size_t size;
	struct sef_sb_info_log *log = NULL;
	struct nvme_ctrl *ctrl = sef_device->ctrl;

	size = sizeof(*log);
	log = kmalloc(size, GFP_KERNEL);
	if (!log)
		goto Exit; // notifications will stop

	ret = sef_get_log_page(ctrl, 0, SEF_LOG_SB_INFO, 0, 0, 0xffff, log,
			       size, 0);
	if (ret) // todo: check ret for "abort" status to break
		goto Exit; // assume it's because there's nothing to report

	qosd = le16_to_cpu(log->qosid);
	sbid = le32_to_cpu(log->sbid);
	nvadu = le32_to_cpu(log->cap);
	sef_send_sb_uevent(sef_device, aen_result, qosd, sbid, log->st, nvadu);

Exit:
	kfree(log);
	if (ret)
		printk(KERN_ERR
		       "Err %d reading SB log page - AENs will no longer be processed\n",
		       ret);
}

static void sef_handle_aen(struct sef_device *sef_device, u32 aen_result)
{
	u32 log_page = (aen_result >> 16) & 0xff;

	if (log_page == SEF_LOG_VD_INFO)
		sef_handle_vd_aen(sef_device, aen_result);
	else if (log_page == SEF_LOG_SB_INFO)
		sef_handle_sb_aen(sef_device, aen_result);
}

static inline u32 netlink_group_mask(u32 group)
{
	if (group > 32)
		return 0;
	return group ? 1 << (group - 1) : 0;
}

struct sef_aen_work_struct {
	struct work_struct work;
	int devidx;
	int aen_type;
};

void sef_aen_work(struct work_struct *work)
{
	struct sef_aen_work_struct *aen_req =
		container_of(work, struct sef_aen_work_struct, work);
	struct sef_device *sef_device = sef_dev_find_and_get(aen_req->devidx);
	if (!sef_device)
		return;
	sef_handle_aen(sef_device, aen_req->aen_type);
	sef_device_put(sef_device);

	kfree(aen_req);
}

int sef_queue_aen(int devidx, int aen_type)
{
	struct sef_aen_work_struct *req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	INIT_WORK(&req->work, sef_aen_work);
	req->devidx = devidx;
	req->aen_type = aen_type;
	queue_work(sef_wq, &req->work);
	return 0;
}

static int sef_recvmsg(struct sock *sk, struct msghdr *msg, size_t len,
		       int flags)
{
	size_t copied;
	struct sk_buff *skb, *data_skb;
	int err;
	char *ptr = NULL;
	int type = 0;
	char *devname = NULL;
	bool is_nvme = false;
	int aen_type = 0;

	copied = 0;

	skb = skb_recv_datagram(sk, flags, &err);
	if (skb == NULL)
		return copied;

	data_skb = skb;

	/* Record the max length of recvmsg() calls for future allocations */
	copied = data_skb->len;
	if (len < copied) {
		msg->msg_flags |= MSG_TRUNC;
		copied = len;
	}

	err = skb_copy_datagram_msg(data_skb, 0, msg, copied);
	if (err < 0)
		goto out;

	if (msg->msg_name) {
		DECLARE_SOCKADDR(struct sockaddr_nl *, addr, msg->msg_name);
		addr->nl_family = AF_NETLINK;
		addr->nl_pad = 0;
		addr->nl_pid = NETLINK_CB(skb).portid;
		addr->nl_groups = netlink_group_mask(NETLINK_CB(skb).dst_group);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	ptr = (char *)iter_iov(&msg->msg_iter)->iov_base;
#else
	ptr = (char *)msg->msg_iter.iov->iov_base;
#endif
	while (*ptr && !(type && devname && is_nvme)) {
		if (strcmp("libudev", ptr) == 0)
			goto out;
		if (strncmp("ACTION", ptr, 6) == 0) {
			if (strcmp("ACTION=add", ptr) == 0) {
				type = AEN_TYPE_NS_ADD;
			} else if (strcmp("ACTION=remove", ptr) == 0) {
				type = AEN_TYPE_NS_REMOVE;
			}
		} else if (strncmp("DEVNAME", ptr, 6) == 0) {
			devname = ptr + 8;
		} else if (strcmp("SUBSYSTEM=nvme-generic", ptr) == 0) {
			is_nvme = true;
		} else if (strcmp("SUBSYSTEM=nvme", ptr) == 0) {
			is_nvme = true;
		} else if (strncmp("NVME_AEN", ptr, 8) == 0) {
			int cnt;
			if ((cnt = sscanf(ptr, "NVME_AEN=%x", &aen_type)) ==
			    1) {
				if (((aen_type >> 8) & 0xff) == 0x40)
					type = AEN_TYPE_SEF;
			}
		}

		ptr += strlen(ptr) + 1;
	}

	switch (type) {
	case AEN_TYPE_NS_ADD:
	case AEN_TYPE_NS_REMOVE:
		if (devname && is_nvme) {
			int devidx, nsid, cnt;
			if ((cnt = sscanf(devname, "ng%dn%d", &devidx,
					  &nsid)) == 2) {
				if (type == AEN_TYPE_NS_ADD)
					sef_queue_add_qosd(devidx, nsid);
				else
					sef_queue_del_qosd(devidx, nsid);
			} else if (sscanf(devname, "nvme%d", &devidx) == 1) {
				if (type == AEN_TYPE_NS_ADD)
					sef_queue_add_dev(devidx);
				else
					sef_queue_del_dev(devidx);
			}
		}
		break;
	case AEN_TYPE_SEF: {
		int devidx, cnt;
		if ((cnt = sscanf(devname, "nvme%d", &devidx)) == 1) {
			sef_queue_aen(devidx, aen_type);
		}
		break;
	}
	}

out:
	skb_free_datagram(sk, skb);

	return err ?: copied;
}

char buf[__UEVENT_BUFFER_SIZE] = { 0 };

static void netlink_data_ready(struct sock *sk)
{
	int ret;
	struct sockaddr_nl rcv_addr = { 0 };
	struct kvec iov = { buf, __UEVENT_BUFFER_SIZE };
	struct msghdr hdr;
	memset(&hdr, 0, sizeof(hdr));
	memset(buf, 0, sizeof(buf));
	hdr.msg_name = &rcv_addr;
	hdr.msg_namelen = sizeof(rcv_addr);
	iov_iter_kvec(&hdr.msg_iter, READ, &iov, 1, iov.iov_len);
	if ((ret = sef_recvmsg(sk, &hdr, __UEVENT_BUFFER_SIZE, 0)) <= 0) {
		printk(KERN_ERR "recv err: %d\n", ret);
	}
}

int sef_aen_init(struct sef_aen_info *aen_info)
{
	int ret;
	struct sock *sock = NULL;
	struct sockaddr_nl addr = { 0 };

	ret = sock_create_kern(&init_net, AF_NETLINK, SOCK_RAW,
			       NETLINK_KOBJECT_UEVENT, &aen_info->aen_sock);
	if (ret < 0)
		goto fail;

	addr.nl_family = AF_NETLINK;
	addr.nl_groups = -1;

	ret = kernel_bind(aen_info->aen_sock, (struct sockaddr *)&addr,
			  sizeof(addr));
	if (ret) {
		sock_release(aen_info->aen_sock);
		goto fail;
	}

	sock = aen_info->aen_sock->sk;
	sock->sk_data_ready = netlink_data_ready;
	sock->sk_allocation = GFP_KERNEL;

fail:
	return ret;
}

void sef_aen_exit(struct sef_aen_info *aen_info)
{
	struct sock *sk = aen_info->aen_sock->sk;

	sk->sk_data_ready = NULL;
	sock_release(aen_info->aen_sock);
}
