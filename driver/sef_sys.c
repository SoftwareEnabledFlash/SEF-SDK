/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * sef_sys.c
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
#include <linux/bitmap.h>
#include <linux/blkdev.h>
#include <linux/nvme_ioctl.h>
#include "sef.h"
#include "sef_defs.h"

int sef_update_qosd_info(struct sef_qosd *sef_qosd)
{
	// todo: code getting nvme_id_ns is nearly the same as nvme_identify_ns
	//       but that's static in core.c so maybe caller of this can
	//       call this for us and pass in the nvme_id_ns struct
	struct nvme_command c = {};
	struct nvme_id_ns *id;
	struct sef_vs_id_ns *sid;
	struct sef_qosd_info *qinfo = sef_qosd->qinfo;
	int ret, i;
	int nsid = sef_qosd->nsid;
	struct nvme_ctrl *ctrl = sef_qosd->parent->ctrl;
	unsigned lbaf;

	if (!qinfo) {
		qinfo = kzalloc(sizeof(*qinfo), GFP_KERNEL);
		if (!qinfo)
			return -ENOMEM;
	}

	c.identify.opcode = nvme_admin_identify;
	c.identify.nsid = cpu_to_le32(nsid);
	c.identify.cns = NVME_ID_CNS_CS_NS;
	c.identify.csi = NVME_CSI_SEF;

	id = kmalloc(sizeof(*id), GFP_KERNEL);
	if (!id) {
		ret = -ENOMEM;
		goto err;
	}

	ret = nvme_submit_sync_cmd(ctrl->admin_q, &c, id, sizeof(*id));
	if (ret) {
		dev_err(ctrl->device, "Identify namepsace failed (%d)\n", ret);
		ret = -EIO;
		goto err;
	}

	lbaf = nvme_lbaf_index(id->flbas);
	sid = (void *)id->vs;
	qinfo->qosid = nsid;
	qinfo->vdid = le16_to_cpu(id->endgid);
	qinfo->qds = sid->qds;
	qinfo->nplid = le16_to_cpu(sid->nplid + 1);
	qinfo->gcap = le64_to_cpu(sid->sbsta.gcap);
	qinfo->quota = le64_to_cpu(sid->sbsta.quota);
	qinfo->ucap = le64_to_cpu(sid->sbsta.ucap);
	qinfo->nsb = le16_to_cpu(sid->sbsta.nsb);
	qinfo->nwosb = le16_to_cpu(sid->sbsta.nwosb);
	qinfo->neosb = sid->sbsta.neosb;
	qinfo->qds = le32_to_cpu(sid->qds);
	// pSLC version
	qinfo->pgcap = le64_to_cpu(sid->psbsta.gcap);
	qinfo->pquota = le64_to_cpu(sid->psbsta.quota);
	qinfo->pucap = le64_to_cpu(sid->psbsta.ucap);
	qinfo->pnsb = le16_to_cpu(sid->psbsta.nsb);
	qinfo->pnwosb = le16_to_cpu(sid->psbsta.nwosb);
	qinfo->pneosb = sid->psbsta.neosb;
	qinfo->msize = le16_to_cpu(id->lbaf[id->flbas].ms);
	qinfo->mxosb = le16_to_cpu(sid->mxosb);
	qinfo->wwt = le16_to_cpu(sid->wwt);
	qinfo->ewt = le16_to_cpu(sid->ewt);
	qinfo->drfid = le32_to_cpu(sid->drfid);
	qinfo->lba_size = 1 << id->lbaf[lbaf].ds;

	for (i = 0; i < ARRAY_SIZE(qinfo->rootp); i++)
		qinfo->rootp[i] = sid->rootp[i];

	sef_qosd->qinfo = qinfo;
	goto done;

err:
	sef_qosd->qinfo = NULL;
	if (qinfo)
		kfree(qinfo);

done:
	kfree(id);
	return ret;
};

static ssize_t sef_dev_attr_show(struct device *dev,
				 struct device_attribute *dattr, char *page)
{
	struct sef_device *sef_device = dev_get_drvdata(dev);
	struct attribute *attr = &dattr->attr;
	struct sef_unit_info *sinfo = sef_device->id_info;

	if (!sinfo)
		return 0;

	if (strcmp(attr->name, "ctratt") == 0)
		return scnprintf(page, PAGE_SIZE, "0x%x\n", sinfo->ctratt);
	else if (strcmp(attr->name, "capability") == 0)
		return scnprintf(page, PAGE_SIZE, "0x%x\n", sinfo->cap);
	else if (strcmp(attr->name, "max_qosd") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->max_qosdid);
	else if (strcmp(attr->name, "max_rootp") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->nrp);
	else if (strcmp(attr->name, "max_placement") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->max_plid);
	else if (strcmp(attr->name, "max_osb") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->mxosb);
	else if (strcmp(attr->name, "num_ch") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->nch);
	else if (strcmp(attr->name, "num_bank") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->nbnk);
	else if (strcmp(attr->name, "num_page") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->npag);
	else if (strcmp(attr->name, "num_plane") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->npl);
	else if (strcmp(attr->name, "num_blocks") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->nblk);
	else if (strcmp(attr->name, "num_adus") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->nadu);
	else if (strcmp(attr->name, "num_rf") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->nrf);
	else if (strcmp(attr->name, "eop") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->eop);
	else if (strcmp(attr->name, "num_psi") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", sinfo->npsi);
	else if (strcmp(attr->name, "qosd_list") == 0) {
		int count = 0;
		char *del = "";
		struct sef_qosd *sef_qosd;

		down_read(&sef_device->qosds_lock);
		list_for_each_entry(sef_qosd, &sef_device->qosds, link) {
			count += scnprintf(page + count, PAGE_SIZE - count,
					   "%s%u", del, sef_qosd->nsid);
			del = ",";
		}
		up_read(&sef_device->qosds_lock);
		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		return count;
	} else if (strcmp(attr->name, "vd_list") == 0) {
		int count = 0;
		char *del = "";
		struct sef_vd *vd;

		// @todo: consider it's own sem?  But the list is updated when
		//        the namespaces are so that lock will be held already
		down_read(&sef_device->vdevs_lock);
		list_for_each_entry(vd, &sef_device->vdevs, list) {
			count += scnprintf(page + count, PAGE_SIZE - count,
					   "%s%u", del, vd->vd_id);
			del = ",";
		}
		up_read(&sef_device->vdevs_lock);
		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		return count;
	} else {
		return scnprintf(page, PAGE_SIZE,
				 "Unhandled attr(%s) in `%s`\n", attr->name,
				 __func__);
	}
}

static ssize_t sef_ns_attr_show(struct device *dev,
				struct device_attribute *dattr, char *page)
{
	struct attribute *attr;
	struct sef_qosd *sef_qosd = dev_get_drvdata(dev);
	struct sef_qosd_info *qinfo = sef_qosd->qinfo;

	if (!qinfo) {
		return 0;
	}
	attr = &dattr->attr;

	if (strcmp(attr->name, "vdid") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->vdid);
	else if (strcmp(attr->name, "id") == 0)
		return scnprintf(page, PAGE_SIZE, "%d\n", sef_qosd->nsid);
	else if (strcmp(attr->name, "features") == 0)
		return scnprintf(page, PAGE_SIZE, "0x%x\n", qinfo->qds);
	else if (strcmp(attr->name, "nplid") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->nplid);
	else if (strcmp(attr->name, "gcap") == 0)
		return scnprintf(page, PAGE_SIZE, "%llu\n", qinfo->gcap);
	else if (strcmp(attr->name, "ucap") == 0)
		return scnprintf(page, PAGE_SIZE, "%llu\n", qinfo->ucap);
	else if (strcmp(attr->name, "quota") == 0)
		return scnprintf(page, PAGE_SIZE, "%llu\n", qinfo->quota);
	else if (strcmp(attr->name, "pgcap") == 0)
		return scnprintf(page, PAGE_SIZE, "%llu\n", qinfo->pgcap);
	else if (strcmp(attr->name, "pquota") == 0)
		return scnprintf(page, PAGE_SIZE, "%llu\n", qinfo->pquota);
	else if (strcmp(attr->name, "pucap") == 0)
		return scnprintf(page, PAGE_SIZE, "%llu\n", qinfo->pucap);
	else if (strcmp(attr->name, "msize") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->msize);
	else if (strcmp(attr->name, "mxosb") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->mxosb);
	else if (strcmp(attr->name, "ewt") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->ewt);
	else if (strcmp(attr->name, "wwt") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->wwt);
	else if (strcmp(attr->name, "drfid") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->drfid);
	else if (strcmp(attr->name, "lba_size") == 0)
		return scnprintf(page, PAGE_SIZE, "%u\n", qinfo->lba_size);
	else if (strcmp(attr->name, "rootp") == 0) {
		// todo: need to trigger qinfo reload on set rootp  instead
		//       or removed using qinfo to read root pointers in SEFAPI
		char *del = "";
		int i, count = 0;
		struct sef_unit_info *sinfo = sef_qosd->parent->id_info;

		for (i = 0; i < sinfo->nrp; i++) {
			count += scnprintf(page + count, PAGE_SIZE - count,
					   "%s%llu", del, qinfo->rootp[i]);
			del = ",";
		}
		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		return count;
	} else {
		return scnprintf(page, PAGE_SIZE,
				 "Unhandled attr(%s) in `%s`\n", attr->name,
				 __func__);
	}
}

#define SEF_DEV_ATTR_RO(_name) \
	DEVICE_ATTR(_name, S_IRUGO, sef_dev_attr_show, NULL)
#define SEF_DEV_ATTR_RW(_name) \
	DEVICE_ATTR(_name, S_IRUGO | S_IWUSR, sef_dev_attr_show, NULL)
#define SEF_NS_ATTR_RO(_name) \
	DEVICE_ATTR(_name, S_IRUGO, sef_ns_attr_show, NULL)

static SEF_DEV_ATTR_RO(ctratt);
static SEF_DEV_ATTR_RO(capability);
static SEF_DEV_ATTR_RO(max_qosd);
static SEF_DEV_ATTR_RO(max_rootp);
static SEF_DEV_ATTR_RO(max_placement);
static SEF_DEV_ATTR_RO(max_osb);
static SEF_DEV_ATTR_RO(num_ch);
static SEF_DEV_ATTR_RO(num_bank);
static SEF_DEV_ATTR_RO(num_page);
static SEF_DEV_ATTR_RO(num_plane);
static SEF_DEV_ATTR_RO(num_blocks);
static SEF_DEV_ATTR_RO(num_adus);
static SEF_DEV_ATTR_RO(num_rf);
static SEF_DEV_ATTR_RO(eop);
//static SEF_DEV_ATTR_RO(num_fmq);
static SEF_DEV_ATTR_RO(num_psi);
static SEF_DEV_ATTR_RO(qosd_list);
static SEF_DEV_ATTR_RO(vd_list);

static SEF_NS_ATTR_RO(vdid);
// no qosid as it's the same as nsid;
static SEF_NS_ATTR_RO(id);
static SEF_NS_ATTR_RO(features);
static SEF_NS_ATTR_RO(nplid);
static SEF_NS_ATTR_RO(gcap);
static SEF_NS_ATTR_RO(ucap);
static SEF_NS_ATTR_RO(quota);
static SEF_NS_ATTR_RO(pgcap);
static SEF_NS_ATTR_RO(pquota);
static SEF_NS_ATTR_RO(pucap);
static SEF_NS_ATTR_RO(msize);
static SEF_NS_ATTR_RO(mxosb);
static SEF_NS_ATTR_RO(ewt);
static SEF_NS_ATTR_RO(wwt);
static SEF_NS_ATTR_RO(drfid);
static SEF_NS_ATTR_RO(rootp);
static SEF_NS_ATTR_RO(lba_size);

static struct attribute *sef_dev_attrs[] = {
	&dev_attr_ctratt.attr,	      &dev_attr_capability.attr,
	&dev_attr_max_qosd.attr,      &dev_attr_max_rootp.attr,
	&dev_attr_max_placement.attr, &dev_attr_max_osb.attr,
	&dev_attr_num_ch.attr,	      &dev_attr_num_bank.attr,
	&dev_attr_num_page.attr,      &dev_attr_num_plane.attr,
	&dev_attr_num_blocks.attr,    &dev_attr_num_adus.attr,
	&dev_attr_num_rf.attr,	      &dev_attr_eop.attr,
	&dev_attr_num_psi.attr,	      &dev_attr_qosd_list.attr,
	&dev_attr_vd_list.attr,	      NULL
};

static struct attribute *sef_ns_attrs[] = {
	&dev_attr_vdid.attr,  &dev_attr_id.attr,       &dev_attr_features.attr,
	&dev_attr_nplid.attr, &dev_attr_gcap.attr,     &dev_attr_ucap.attr,
	&dev_attr_quota.attr, &dev_attr_pgcap.attr,    &dev_attr_pquota.attr,
	&dev_attr_pucap.attr, &dev_attr_msize.attr,    &dev_attr_mxosb.attr,
	&dev_attr_ewt.attr,   &dev_attr_wwt.attr,      &dev_attr_drfid.attr,
	&dev_attr_rootp.attr, &dev_attr_lba_size.attr, NULL
};

static umode_t sef_dev_attrs_visible(struct kobject *kobj,
				     struct attribute *attr, int index)
{
	return attr->mode;
}

static umode_t sef_ns_attrs_visible(struct kobject *kobj,
				    struct attribute *attr, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct sef_qosd *sef_qosd = dev_get_drvdata(dev);

	if (!sef_qosd || !sef_qosd->qinfo)
		return 0;

	return attr->mode;
}

static const struct attribute_group sef_ns_attr_group = {
	.attrs = sef_ns_attrs,
	.is_visible = sef_ns_attrs_visible,
};

static const struct attribute_group sef_dev_attr_group = {
	.attrs = sef_dev_attrs,
	.is_visible = sef_dev_attrs_visible,
};

static struct sef_rf_log *sef_rf_log(struct nvme_ctrl *ctrl, int vd_id, int nrf)
{
	struct sef_rf_log *log;
	size_t log_size = array_size(sizeof(*log), nrf ?: 1);
	int ret;

	log = kzalloc(log_size, GFP_KERNEL);
	if (!log)
		return ERR_PTR(-ENOMEM);

	ret = sef_get_log_page(ctrl, 0, SEF_LOG_FIFO_INFO, 0, 0,
			       le16_to_cpu(vd_id), log, log_size, 0);
	if (ret)
		goto out;
	return log;
out:
	kfree(log);
	if (ret > 0)
		ret = -EINVAL;
	return ERR_PTR(ret);
}

static struct sef_vd_info_log *sef_vd_info_log(struct sef_device *sef_device,
					       int vd_id)
{
	struct sef_unit_info *sinfo = sef_device->id_info;
	struct sef_vd_info_log *log;
	u16 ndie = sinfo->nch * sinfo->nbnk;
	size_t log_size = struct_size(log, ids, ndie);
	int ret;

	log = kzalloc(log_size, GFP_KERNEL);
	if (!log)
		return ERR_PTR(-ENOMEM);

	ret = sef_get_log_page(sef_device->ctrl, 0, SEF_LOG_VD_INFO, 0, 0,
			       le16_to_cpu(vd_id), log, log_size, 0);
	if (ret)
		goto out;
	return log;
out:
	kfree(log);
	if (ret > 0)
		ret = -EINVAL;
	return ERR_PTR(ret);
}

__u16 *sef_eg_list_id(struct nvme_ctrl *ctrl)
{
	struct nvme_command c = {};
	struct sef_nvme_identify *identify =
		(struct sef_nvme_identify *)&c.identify;
	__u16 *list;
	int ret;

	list = kzalloc(4096, GFP_KERNEL);
	if (!list)
		return ERR_PTR(-ENOMEM);

	identify->opcode = nvme_admin_identify;
	identify->cns = NVME_ID_CNS_EG_LIST;
	identify->nvmsetid = 1;

	ret = nvme_submit_sync_cmd(ctrl->admin_q, &c, list, 4096);
	if (ret) {
		kfree(list);
		return ERR_PTR(ret);
	}
	return list;
}

static unsigned long *sef_vd_list(struct sef_device *sef_device)
{
	unsigned long *vds;
	int num_dies = sef_device->id_info->nch * sef_device->id_info->nbnk;
	int i;
	__u16 *list;
	__u16 nvd;

	vds = bitmap_zalloc(num_dies, GFP_KERNEL);
	if (!vds)
		return ERR_PTR(-ENOMEM);

	list = sef_eg_list_id(sef_device->ctrl);
	if (!list)
		return vds;
	nvd = __le16_to_cpu(list[0]);
	for (i = 1; i <= nvd; i++) {
		bitmap_set(vds, __le16_to_cpu(list[i]) - 1, 1);
	}
	kfree(list);
	return vds;
}

static void sef_vd_release(struct kobject *kobj)
{
	struct sef_vd *vd = container_of(kobj, struct sef_vd, kobj);

	kfree(vd->info);
	kfree(vd->fifos);
	kfree(vd);
}

static ssize_t sef_vd_attr_show(struct kobject *kobj, struct attribute *attr,
				char *page)
{
	struct sef_vd *vd = container_of(kobj, struct sef_vd, kobj);
	struct nvme_ctrl *ctrl = vd->ctrl;
	struct sef_device *sef_device = sef_dev_find_and_get(ctrl->instance);
	int ret = 0;

	if (!sef_device)
		return 0;

	if (strcmp(attr->name, "vid") == 0)
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->vd_id);
	else if (strcmp(attr->name, "cap") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%llu\n",
				le64_to_cpu(vd->info->sbsta.tcap));
	} else if (strcmp(attr->name, "gcap") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%llu\n",
				le64_to_cpu(vd->info->sbsta.tgcap));
	} else if (strcmp(attr->name, "mxosb") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n",
				le32_to_cpu(vd->info->mxosb));
	} else if (strcmp(attr->name, "ucap") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%llu\n",
				le64_to_cpu(vd->info->sbsta.ucap));
	} else if (strcmp(attr->name, "pcap") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%llu\n",
				le64_to_cpu(vd->info->psbsta.tcap));
	} else if (strcmp(attr->name, "pgcap") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%llu\n",
				le64_to_cpu(vd->info->psbsta.tgcap));
	} else if (strcmp(attr->name, "pucap") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%llu\n",
				le64_to_cpu(vd->info->psbsta.ucap));
	} else if (strcmp(attr->name, "adu_bits") == 0)
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->info->aobw + 1);
	else if (strcmp(attr->name, "sb_bits") == 0)
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->info->sbbw + 1);
	else if (strcmp(attr->name, "ndie") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n",
				le16_to_cpu(vd->info->ndie) + 1);
	} else if (strcmp(attr->name, "sb_size") == 0)
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->sbs);
	else if (strcmp(attr->name, "die_list") == 0) {
		int i, count = 0;
		char *del = "";

		for (i = 0; i < le16_to_cpu(vd->info->ndie) + 1; i++) {
			count += scnprintf(page + count, PAGE_SIZE - count,
					   "%s%u", del,
					   le16_to_cpu(vd->info->ids[i]));
			del = ", ";
		}
		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		ret = count;
	} else if (strcmp(attr->name, "qosd_list") == 0) {
		int count = 0;
		char *del = "";
		struct list_head *l;
		down_read(&sef_device->qosds_lock);
		list_for_each(l, &sef_device->qosds) {
			struct sef_qosd *sef_qosd =
				list_entry(l, struct sef_qosd, link);
			if (sef_qosd->qinfo &&
			    sef_qosd->qinfo->vdid == vd->vd_id) {
				count += scnprintf(page + count,
						   PAGE_SIZE - count, "%s%u",
						   del, sef_qosd->nsid);
				del = ",";
			}
		}
		up_read(&sef_device->qosds_lock);

		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		ret = count;
	} else if (strcmp(attr->name, "nrf") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->nrf);
	} else if (strcmp(attr->name, "npblk") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n",
				le16_to_cpu(vd->info->npblk));
	} else if (strcmp(attr->name, "mtps") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->mtps);
	} else if (strcmp(attr->name, "mtus") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->mtus);
	} else if (strcmp(attr->name, "msi") == 0) {
		ret = scnprintf(page, PAGE_SIZE, "%u\n", vd->msi);
	} else if (strcmp(attr->name, "rfids") == 0) {
		int i, count = 0;
		char *del = "";

		for (i = 0; i < le16_to_cpu(vd->nrf); i++) {
			count += scnprintf(page + count, PAGE_SIZE - count,
					   "%s%u", del,
					   le16_to_cpu(vd->fifos[i].rfid));
			del = ", ";
		}
		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		ret = count;
	} else if (strcmp(attr->name, "rfws") == 0) {
		int i, count = 0;
		char *del = "";

		for (i = 0; i < le16_to_cpu(vd->nrf); i++) {
			count += scnprintf(page + count, PAGE_SIZE - count,
					   "%s%u", del,
					   le16_to_cpu(vd->fifos[i].weight));
			del = ", ";
		}
		count += scnprintf(page + count, PAGE_SIZE - count, "\n");
		ret = count;
	} else {
		ret = scnprintf(page, PAGE_SIZE, "Unhandled attr(%s) in `%s`\n",
				attr->name, __func__);
	}
	sef_device_put(sef_device);
	return ret;
}

static struct sysfs_ops sef_vd_sysfs_ops = { .show = sef_vd_attr_show };

static struct attribute sef_vd_vid_attr = { .name = "vid", .mode = S_IRUGO };
static struct attribute sef_vd_cap_attr = { .name = "cap", .mode = S_IRUGO };
static struct attribute sef_vd_gcap_attr = { .name = "gcap", .mode = S_IRUGO };
static struct attribute sef_vd_ucap_attr = { .name = "ucap", .mode = S_IRUGO };
static struct attribute sef_vd_pcap_attr = { .name = "pcap", .mode = S_IRUGO };
static struct attribute sef_vd_pgcap_attr = { .name = "pgcap",
					      .mode = S_IRUGO };
static struct attribute sef_vd_pucap_attr = { .name = "pucap",
					      .mode = S_IRUGO };
static struct attribute sef_vd_aobw_attr = { .name = "adu_bits",
					     .mode = S_IRUGO };
static struct attribute sef_vd_sbbw = { .name = "sb_bits", .mode = S_IRUGO };
static struct attribute sef_vd_ndie_attr = { .name = "ndie", .mode = S_IRUGO };
static struct attribute sef_vd_sbs_attr = { .name = "sb_size",
					    .mode = S_IRUGO };
static struct attribute sef_vd_die_attr = { .name = "die_list",
					    .mode = S_IRUGO };
static struct attribute sef_vd_qosd_attr = { .name = "qosd_list",
					     .mode = S_IRUGO };
static struct attribute sef_vd_mxosb_attr = { .name = "mxosb",
					      .mode = S_IRUGO };
static struct attribute sef_vd_nrf_attr = { .name = "nrf", .mode = S_IRUGO };
static struct attribute sef_vd_rfids_attr = { .name = "rfids",
					      .mode = S_IRUGO };
static struct attribute sef_vd_rfws_attr = { .name = "rfws", .mode = S_IRUGO };
static struct attribute sef_vd_npblk_attr = { .name = "npblk",
					      .mode = S_IRUGO };
static struct attribute sef_vd_mtps_attr = { .name = "mtps", .mode = S_IRUGO };
static struct attribute sef_vd_mtus_attr = { .name = "mtus", .mode = S_IRUGO };
static struct attribute sef_vd_msi_attr = { .name = "msi", .mode = S_IRUGO };

static struct attribute *sef_vd_attrs[] = {
	&sef_vd_vid_attr,   &sef_vd_cap_attr,
	&sef_vd_gcap_attr,  &sef_vd_ucap_attr,
	&sef_vd_pcap_attr,  &sef_vd_pgcap_attr,
	&sef_vd_pucap_attr, &sef_vd_aobw_attr,
	&sef_vd_sbbw,	    &sef_vd_ndie_attr,
	&sef_vd_sbs_attr,   &sef_vd_die_attr,
	&sef_vd_qosd_attr,  &sef_vd_mxosb_attr,
	&sef_vd_nrf_attr,   &sef_vd_rfids_attr,
	&sef_vd_rfws_attr,  &sef_vd_npblk_attr,
	&sef_vd_mtps_attr,  &sef_vd_mtus_attr,
	&sef_vd_msi_attr,   NULL
};

ATTRIBUTE_GROUPS(sef_vd);

static struct kobj_type sef_vd_ktype = {
	.release = sef_vd_release,
	.sysfs_ops = &sef_vd_sysfs_ops,
	.default_groups = sef_vd_groups,
};

static int sef_vd_update(struct sef_device *sef_device, struct sef_vd *vd)
{
	struct nvme_ctrl *ctrl = sef_device->ctrl;
	struct sef_vd_info_log *info = NULL;
	struct sef_rf_log *fifos = NULL;

	info = sef_vd_info_log(sef_device, vd->vd_id);
	if (IS_ERR(info))
		return PTR_ERR(info);
	kfree(vd->info);
	vd->info = info;
	vd->sbs = le16_to_cpu(vd->info->nsbdie);
	vd->mtps = le32_to_cpu(0); // max time per suspend
	vd->mtus = le32_to_cpu(0); // min time until suspend
	vd->msi = le32_to_cpu(0); // max suspend interval

	// todo: when CS revs, fetch log page or copy from vd info

	fifos = sef_rf_log(ctrl, vd->vd_id, le32_to_cpu(info->nrf));
	if (IS_ERR(fifos))
		return PTR_ERR(fifos);
	kfree(vd->fifos);
	vd->fifos = fifos;
	for (vd->nrf = 0; vd->nrf < __le32_to_cpu(vd->info->nrf); vd->nrf++)
		if (fifos[vd->nrf].rfid == 0)
			break;
	return 0;
}

static int sef_vd_add(struct sef_device *sef_device, int vd_id)
{
	struct nvme_ctrl *ctrl = sef_device->ctrl;
	struct sef_vd *vd;
	int ret;

	ret = -ENOMEM;
	vd = kzalloc(sizeof(*vd), GFP_KERNEL);
	if (!vd)
		goto out;
	vd->vd_id = vd_id;
	kobject_init(&vd->kobj, &sef_vd_ktype);
	ret = sef_vd_update(sef_device, vd);
	if (ret)
		goto out;
	ret = kobject_add(&vd->kobj, &sef_device->obj.device->kobj, "vd%u",
			  vd_id);
	if (ret)
		goto out;
	down_read(&sef_device->vdevs_lock);
	list_add_tail(&vd->list, &sef_device->vdevs);
	up_read(&sef_device->vdevs_lock);
	vd->ctrl = ctrl;
	return ret;
out:
	kfree(vd);
	return ret;
}

void sef_vd_validate_or_add(struct sef_device *sef_device, int vd_id)
{
	struct list_head *i, *tmp;
	struct sef_vd *vd;

	down_read(&sef_device->vdevs_lock);
	list_for_each_safe(i, tmp, &sef_device->vdevs) {
		vd = list_entry(i, struct sef_vd, list);
		if (vd->vd_id == vd_id) {
			sef_vd_update(sef_device, vd);
			up_read(&sef_device->vdevs_lock);
			return;
		}
	}
	// todo: during itest (once), this triggered a stack dump because the
	// vd<n> dir was already there.  So the above test failed to find it in
	// ctrl->vdevs.
	up_read(&sef_device->vdevs_lock);
	sef_vd_add(sef_device, vd_id);
	return;
}

static void sef_vd_remove(struct sef_device *sef_device, int vd_id)
{
	struct list_head *i, *tmp;
	struct sef_vd *vd;

	down_read(&sef_device->vdevs_lock);
	list_for_each_safe(i, tmp, &sef_device->vdevs) {
		vd = list_entry(i, struct sef_vd, list);
		if (vd->vd_id == vd_id) {
			list_del(i);
			up_read(&sef_device->vdevs_lock);
			kobject_del(&vd->kobj);
			kobject_put(&vd->kobj);
			vd = NULL;
			return;
		}
	}
	up_read(&sef_device->vdevs_lock);
	return;
}

static void sef_scan_vd(struct sef_device *sef_device)
{
	int num_dies = sef_device->id_info->nch * sef_device->id_info->nbnk;
	int vd_id;
	unsigned long *vd_list;

	vd_list = sef_vd_list(sef_device);
	if (IS_ERR(vd_list))
		return;
	for (vd_id = 0; vd_id < num_dies; vd_id++) {
		if (test_bit(vd_id, vd_list))
			sef_vd_validate_or_add(sef_device, vd_id + 1);
		else
			sef_vd_remove(sef_device, vd_id + 1);
	}
	bitmap_free(vd_list);
}

int sef_sys_qosd_add(struct sef_qosd *sef_qosd)
{
	struct sef_device *sef_device = sef_qosd->parent;
	int ret;

	_sef_check_size();

	ret = sef_update_qosd_info(sef_qosd);
	if (ret)
		return ret;
	sef_vd_validate_or_add(sef_device, sef_qosd->qinfo->vdid);
	ret = sysfs_create_group(&sef_qosd->obj.device->kobj,
				 &sef_ns_attr_group);
	if (ret)
		return ret;
	ret = sysfs_create_link(&sef_device->obj.device->kobj,
				&sef_qosd->obj.device->kobj,
				sef_qosd->obj.dev_name);
	return ret;
}

struct sef_id_ctrl *sef_id_ctrl(struct nvme_ctrl *ctrl)
{
	struct nvme_command c = {};
	struct sef_id_ctrl *id;
	int ret;

	id = kzalloc(sizeof(*id), GFP_KERNEL);
	if (!id)
		return ERR_PTR(-ENOMEM);

	c.identify.opcode = nvme_admin_identify;
	c.identify.cns = NVME_ID_CNS_CS_CTRL;
	c.identify.csi = NVME_CSI_SEF;

	ret = nvme_submit_sync_cmd(ctrl->admin_q, &c, id, sizeof(*id));
	if (ret) {
		kfree(id);
		return ERR_PTR(ret);
	}
	return id;
}

static struct nvme_id_ctrl *nvme_ctrl_id(struct nvme_ctrl *ctrl)
{
	struct nvme_command c = {};
	struct nvme_id_ctrl *id;
	int ret;

	id = kzalloc(sizeof(*id), GFP_KERNEL);
	if (!id)
		return ERR_PTR(-ENOMEM);

	c.identify.opcode = nvme_admin_identify;
	c.identify.cns = NVME_ID_CNS_CTRL;
	c.identify.csi = NVME_CSI_NVM;

	ret = nvme_submit_sync_cmd(ctrl->admin_q, &c, id, sizeof(*id));
	if (ret) {
		kfree(id);
		return ERR_PTR(ret);
	}
	return id;
}

static struct sef_unit_info *sef_unit_info(struct nvme_ctrl *ctrl)
{
	struct nvme_id_ctrl *id;
	struct sef_id_ctrl *sid;
	struct sef_unit_info *sinfo = NULL;
	int ret = 0;

	id = nvme_ctrl_id(ctrl);
	if (IS_ERR(id))
		return (void *)id;
	sid = sef_id_ctrl(ctrl);
	if (IS_ERR(sid)) {
		kfree(id);
		return (void *)sid;
	}
	sinfo = kmalloc(sizeof(*sinfo), GFP_KERNEL);
	if (!sinfo) {
		ret = -ENOMEM;
		goto out;
	}
	sinfo->mxosb = __le32_to_cpu(sid->mxosb);
	sinfo->max_plid = __le16_to_cpu(sid->max_plid) + 1;
	sinfo->nrp = 8;
	sinfo->nch = sid->nch + 1;
	sinfo->nbnk = sid->nbnk + 1;
	sinfo->npl = sid->npl + 1;
	sinfo->nadu = (1 << (sid->plds - 12));
	sinfo->nrf = le32_to_cpu(sid->nrf);
	sinfo->npsi = sid->npsi;
	sinfo->ctratt = le32_to_cpu(id->ctratt);
	sinfo->cap = le32_to_cpu(sid->cap);
	sinfo->max_qosdid = le32_to_cpu(id->nn);
	sinfo->nblk = le16_to_cpu(sid->nblk) + 1;
	sinfo->npag = le16_to_cpu(sid->npag) + 1;
	sinfo->eop = le16_to_cpu(sid->eop);
	sinfo->nvd = le16_to_cpu(sid->nvd);
	sinfo->nqosd = le16_to_cpu(sid->nqosd);
out:
	kfree(sid);
	kfree(id);
	if (ret)
		return ERR_PTR(ret);
	return sinfo;
};

int sef_rescan_vd(struct sef_device *sef_device)
{
	sef_scan_vd(sef_device);
	return 0;
}

void sef_delete_qosd_sys(struct sef_qosd *sef_qosd)
{
	sysfs_remove_link(&sef_qosd->parent->obj.device->kobj,
			  sef_qosd->obj.dev_name);
	sysfs_remove_group(&sef_qosd->obj.device->kobj, &sef_ns_attr_group);
}

int sef_configure_sys(struct sef_device *sef_device)
{
	struct sef_unit_info *id_info = sef_device->id_info;
	struct nvme_ctrl *ctrl = sef_device->ctrl;
	int ret;
	if (id_info)
		kfree(id_info);
	sef_device->id_info = NULL;
	id_info = sef_unit_info(ctrl);
	if (IS_ERR(id_info))
		return PTR_ERR(id_info);
	sef_device->id_info = id_info;
	ret = sysfs_create_group(&sef_device->obj.device->kobj,
				 &sef_dev_attr_group);
	if (ret)
		return ret;
	sef_scan_vd(sef_device);
	return 0;
}

void sef_delete_sys(struct sef_device *sef_device)
{
	struct list_head *i, *tmp;
	struct sef_vd *vd;

	down_read(&sef_device->vdevs_lock);
	list_for_each_safe(i, tmp, &sef_device->vdevs) {
		vd = list_entry(i, struct sef_vd, list);
		list_del(i);
		kobject_del(&vd->kobj);
		kobject_put(&vd->kobj);
		vd = NULL;
	}
	up_read(&sef_device->vdevs_lock);
	sysfs_remove_group(&sef_device->obj.device->kobj, &sef_dev_attr_group);
}
