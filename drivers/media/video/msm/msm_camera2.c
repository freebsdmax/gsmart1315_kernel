/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
//FIXME: most allocations need not be GFP_ATOMIC
/* FIXME: management of mutexes */
/* FIXME: msm_pmem_region_lookup return values */
/* FIXME: way too many copy to/from user */
/* FIXME: does region->active mean free */
/* FIXME: check limits on command lenghts passed from userspace */
/* FIXME: __msm_release: which queues should we flush when opencnt != 0 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/board.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <linux/syscalls.h>
#include <linux/hrtimer.h>
DEFINE_MUTEX(ctrl_cmd_lock);

spinlock_t pp_prev_spinlock;
spinlock_t pp_snap_spinlock;
spinlock_t pp_thumb_spinlock;

#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

static struct class *msm_class;
static dev_t msm_devno;
static LIST_HEAD(msm_sensors);
struct  msm_control_device *g_v4l2_control_device;
int g_v4l2_opencnt;
static int camera_node;
static enum msm_camera_type camera_type[MSM_MAX_CAMERA_SENSORS];
static uint32_t sensor_mount_angle[MSM_MAX_CAMERA_SENSORS];

static const char *vfe_config_cmd[] = {
	"CMD_GENERAL",  /* 0 */
	"CMD_AXI_CFG_OUT1",
	"CMD_AXI_CFG_SNAP_O1_AND_O2",
	"CMD_AXI_CFG_OUT2",
	"CMD_PICT_T_AXI_CFG",
	"CMD_PICT_M_AXI_CFG",  /* 5 */
	"CMD_RAW_PICT_AXI_CFG",
	"CMD_FRAME_BUF_RELEASE",
	"CMD_PREV_BUF_CFG",
	"CMD_SNAP_BUF_RELEASE",
	"CMD_SNAP_BUF_CFG",  /* 10 */
	"CMD_STATS_DISABLE",
	"CMD_STATS_AEC_AWB_ENABLE",
	"CMD_STATS_AF_ENABLE",
	"CMD_STATS_AEC_ENABLE",
	"CMD_STATS_AWB_ENABLE",  /* 15 */
	"CMD_STATS_ENABLE",
	"CMD_STATS_AXI_CFG",
	"CMD_STATS_AEC_AXI_CFG",
	"CMD_STATS_AF_AXI_CFG",
	"CMD_STATS_AWB_AXI_CFG",  /* 20 */
	"CMD_STATS_RS_AXI_CFG",
	"CMD_STATS_CS_AXI_CFG",
	"CMD_STATS_IHIST_AXI_CFG",
	"CMD_STATS_SKIN_AXI_CFG",
	"CMD_STATS_BUF_RELEASE",  /* 25 */
	"CMD_STATS_AEC_BUF_RELEASE",
	"CMD_STATS_AF_BUF_RELEASE",
	"CMD_STATS_AWB_BUF_RELEASE",
	"CMD_STATS_RS_BUF_RELEASE",
	"CMD_STATS_CS_BUF_RELEASE",  /* 30 */
	"CMD_STATS_IHIST_BUF_RELEASE",
	"CMD_STATS_SKIN_BUF_RELEASE",
	"UPDATE_STATS_INVALID",
	"CMD_AXI_CFG_SNAP_GEMINI",
	"CMD_AXI_CFG_SNAP",  /* 35 */
	"CMD_AXI_CFG_PREVIEW",
	"CMD_AXI_CFG_VIDEO",
	"CMD_STATS_IHIST_ENABLE",
	"CMD_STATS_RS_ENABLE",
	"CMD_STATS_CS_ENABLE",  /* 40 */
	"CMD_VPE",
	"CMD_AXI_CFG_VPE"
};
#define __CONTAINS(r, v, l, field) ({				\
	typeof(r) __r = r;					\
	typeof(v) __v = v;					\
	typeof(v) __e = __v + l;				\
	int res = __v >= __r->field &&				\
		__e <= __r->field + __r->len;			\
	res;							\
})

#define CONTAINS(r1, r2, field) ({				\
	typeof(r2) __r2 = r2;					\
	__CONTAINS(r1, __r2->field, __r2->len, field);		\
})

#define IN_RANGE(r, v, field) ({				\
	typeof(r) __r = r;					\
	typeof(v) __vv = v;					\
	int res = ((__vv >= __r->field) &&			\
		(__vv < (__r->field + __r->len)));		\
	res;							\
})

#define OVERLAPS(r1, r2, field) ({				\
	typeof(r1) __r1 = r1;					\
	typeof(r2) __r2 = r2;					\
	typeof(__r2->field) __v = __r2->field;			\
	typeof(__v) __e = __v + __r2->len - 1;			\
	int res = (IN_RANGE(__r1, __v, field) ||		\
		   IN_RANGE(__r1, __e, field));                 \
	res;							\
})

static inline void free_qcmd(struct msm_queue_cmd *qcmd)
{
	if (!qcmd || !atomic_read(&qcmd->on_heap))
		return;
	if (!atomic_sub_return(1, &qcmd->on_heap))
		kfree(qcmd);
}

static void msm_region_init(struct msm_sync *sync)
{
	INIT_HLIST_HEAD(&sync->pmem_frames);
	INIT_HLIST_HEAD(&sync->pmem_stats);
	spin_lock_init(&sync->pmem_frame_spinlock);
	spin_lock_init(&sync->pmem_stats_spinlock);
}

static void msm_queue_init(struct msm_device_queue *queue, const char *name)
{
	spin_lock_init(&queue->lock);
	queue->len = 0;
	queue->max = 0;
	queue->name = name;
	INIT_LIST_HEAD(&queue->list);
	init_waitqueue_head(&queue->wait);
}

static void msm_enqueue(struct msm_device_queue *queue,
		struct list_head *entry)
{
	unsigned long flags;
	spin_lock_irqsave(&queue->lock, flags);
	queue->len++;
	if (queue->len > queue->max) {
		queue->max = queue->len;
		pr_info("%s: queue %s new max is %d\n", __func__,
			queue->name, queue->max);
	}
	list_add_tail(entry, &queue->list);
	wake_up(&queue->wait);
	CDBG("%s: woke up %s\n", __func__, queue->name);
	spin_unlock_irqrestore(&queue->lock, flags);
}

static void msm_enqueue_vpe(struct msm_device_queue *queue,
		struct list_head *entry)
{
	unsigned long flags;
	spin_lock_irqsave(&queue->lock, flags);
	queue->len++;
	if (queue->len > queue->max) {
		queue->max = queue->len;
		pr_info("%s: queue %s new max is %d\n", __func__,
			queue->name, queue->max);
	}
	list_add_tail(entry, &queue->list);
    CDBG("%s: woke up %s\n", __func__, queue->name);
	spin_unlock_irqrestore(&queue->lock, flags);
}

#define msm_dequeue(queue, member) ({				\
	unsigned long flags;					\
	struct msm_device_queue *__q = (queue);			\
	struct msm_queue_cmd *qcmd = 0;				\
	spin_lock_irqsave(&__q->lock, flags);			\
	if (!list_empty(&__q->list)) {				\
		__q->len--;					\
		qcmd = list_first_entry(&__q->list,		\
				struct msm_queue_cmd, member);	\
		if ((qcmd) && (&qcmd->member) && (&qcmd->member.next))	\
			list_del_init(&qcmd->member);			\
	}							\
	spin_unlock_irqrestore(&__q->lock, flags);	\
	qcmd;							\
})

#define msm_delete_entry(queue, member, q_cmd) ({		\
	unsigned long flags;					\
	struct msm_device_queue *__q = (queue);			\
	struct msm_queue_cmd *qcmd = 0;				\
	pr_info("msm_delete_entry\n");			\
	spin_lock_irqsave(&__q->lock, flags);			\
	if (!list_empty(&__q->list)) {				\
		list_for_each_entry(qcmd, &__q->list, member)	\
		if (qcmd == q_cmd) {				\
			__q->len--;				\
			list_del_init(&qcmd->member);		\
			CDBG("msm_delete_entry, match found\n");\
			kfree(q_cmd);				\
			q_cmd = NULL;				\
			break;					\
		}						\
	}							\
	spin_unlock_irqrestore(&__q->lock, flags);		\
	q_cmd;		\
})

#define msm_queue_drain(queue, member) do {			\
	unsigned long flags;					\
	struct msm_device_queue *__q = (queue);			\
	struct msm_queue_cmd *qcmd;				\
	spin_lock_irqsave(&__q->lock, flags);			\
	pr_info("%s: draining queue %s\n", __func__, __q->name);	\
	while (!list_empty(&__q->list)) {			\
		__q->len--;					\
		pr_info("%s,q->len = %d\n", __func__, __q->len);	\
		qcmd = list_first_entry(&__q->list,		\
			struct msm_queue_cmd, member);		\
		if (qcmd) {					\
			if ((&qcmd->member) && (&qcmd->member.next))	\
				list_del_init(&qcmd->member);		\
			free_qcmd(qcmd);				\
		}							\
	}							\
	spin_unlock_irqrestore(&__q->lock, flags);		\
} while (0)

static int check_overlap(struct hlist_head *ptype,
			unsigned long paddr,
			unsigned long len)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region t = { .paddr = paddr, .len = len };
	struct hlist_node *node;

	hlist_for_each_entry(region, node, ptype, list) {
		if (CONTAINS(region, &t, paddr) ||
				CONTAINS(&t, region, paddr) ||
				OVERLAPS(region, &t, paddr)) {
			CDBG(" region (PHYS %p len %ld)"
				" clashes with registered region"
				" (paddr %p len %ld)\n",
				(void *)t.paddr, t.len,
				(void *)region->paddr, region->len);
			return -1;
		}
	}

	return 0;
}

static int check_pmem_info(struct msm_pmem_info *info, int len)
{
	if (info->offset < len &&
	    info->offset + info->len <= len &&
	    info->y_off < len &&
	    info->cbcr_off < len)
		return 0;

	pr_err("%s: check failed: off %d len %d y %d cbcr %d (total len %d)\n",
		__func__,
		info->offset,
		info->len,
		info->y_off,
		info->cbcr_off,
		len);
	return -EINVAL;
}
static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info, spinlock_t* pmem_spinlock)
{
	struct file *file;
	unsigned long paddr;
	unsigned long kvstart;
	unsigned long len;
	int rc;
	struct msm_pmem_region *region;
	unsigned long flags;


	rc = get_pmem_file(info->fd, &paddr, &kvstart, &len, &file);
	if (rc < 0) {
		pr_err("%s: get_pmem_file fd %d error %d\n",
			__func__,
			info->fd, rc);
		return rc;
	}

	if (!info->len)
		info->len = len;

	rc = check_pmem_info(info, len);
	if (rc < 0)
		return rc;

	paddr += info->offset;
	len = info->len;

	spin_lock_irqsave(pmem_spinlock, flags);
	if (check_overlap(ptype, paddr, len) < 0) {
		spin_unlock_irqrestore(pmem_spinlock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(pmem_spinlock, flags);


	region = kmalloc(sizeof(struct msm_pmem_region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	spin_lock_irqsave(pmem_spinlock, flags);
	INIT_HLIST_NODE(&region->list);

	region->paddr = paddr;
	region->len = len;
	region->file = file;
	memcpy(&region->info, info, sizeof(region->info));

	hlist_add_head(&(region->list), ptype);
	spin_unlock_irqrestore(pmem_spinlock, flags);
	pr_info("%s: type %d, paddr 0x%lx, vaddr 0x%lx\n",
		__func__, info->type, paddr, (unsigned long)info->vaddr);


	return 0;
}

/* return of 0 means failure */
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	int pmem_type, struct msm_pmem_region *reg, uint8_t maxcount,
	spinlock_t *pmem_spinlock)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node, *n;
	unsigned long flags = 0;

	uint8_t rc = 0;

	regptr = reg;
	spin_lock_irqsave(pmem_spinlock, flags);
	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		if (region->info.type == pmem_type && region->info.active) {
			*regptr = *region;
			rc += 1;
			if (rc >= maxcount)
				break;
			regptr++;
		}
	}
	spin_unlock_irqrestore(pmem_spinlock, flags);
	return rc;
}

static uint8_t msm_pmem_region_lookup_2(struct hlist_head *ptype,
					int pmem_type,
					struct msm_pmem_region *reg,
					uint8_t maxcount,
					spinlock_t *pmem_spinlock)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node, *n;
	uint8_t rc = 0;
	unsigned long flags = 0;
	regptr = reg;
	spin_lock_irqsave(pmem_spinlock, flags);
	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		CDBG("%s:info.type=%d, pmem_type = %d,"
						"info.active = %d\n",
		__func__, region->info.type, pmem_type, region->info.active);

		if (region->info.type == pmem_type && region->info.active) {
			CDBG("%s:info.type=%d, pmem_type = %d,"
							"info.active = %d,\n",
				__func__, region->info.type, pmem_type,
				region->info.active);
			*regptr = *region;
			region->info.type = MSM_PMEM_VIDEO;
			rc += 1;
			if (rc >= maxcount)
				break;
			regptr++;
		}
	}
	spin_unlock_irqrestore(pmem_spinlock, flags);
	return rc;
}

static int msm_pmem_frame_ptov_lookup(struct msm_sync *sync,
		unsigned long pyaddr,
		unsigned long pcbcraddr,
		struct msm_pmem_info *pmem_info,
		int clear_active)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;
	unsigned long flags = 0;

	spin_lock_irqsave(&sync->pmem_frame_spinlock, flags);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_frames, list) {
		if (pyaddr == (region->paddr + region->info.y_off) &&
				pcbcraddr == (region->paddr +
						region->info.cbcr_off) &&
				region->info.active) {
			/* offset since we could pass vaddr inside
			 * a registerd pmem buffer
			 */
			memcpy(pmem_info, &region->info, sizeof(*pmem_info));
			if (clear_active)
				region->info.active = 0;
			spin_unlock_irqrestore(&sync->pmem_frame_spinlock,
				flags);
			return 0;
		}
	}
	/* After lookup failure, dump all the list entries... */
	pr_err("%s, for pyaddr 0x%lx, pcbcraddr 0x%lx\n",
			__func__, pyaddr, pcbcraddr);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_frames, list) {
		pr_err("listed pyaddr 0x%lx, pcbcraddr 0x%lx, active = %d",
				(region->paddr + region->info.y_off),
				(region->paddr + region->info.cbcr_off),
				region->info.active);
	}

	spin_unlock_irqrestore(&sync->pmem_frame_spinlock, flags);
	return -EINVAL;
}

static unsigned long msm_pmem_stats_ptov_lookup(struct msm_sync *sync,
		unsigned long addr, int *fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;
	unsigned long flags = 0;

	spin_lock_irqsave(&sync->pmem_stats_spinlock, flags);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (addr == region->paddr && region->info.active) {
			/* offset since we could pass vaddr inside a
			 * registered pmem buffer */
			*fd = region->info.fd;
			region->info.active = 0;
			spin_unlock_irqrestore(&sync->pmem_stats_spinlock,
				flags);
			return (unsigned long)(region->info.vaddr);
		}
	}
	/* After lookup failure, dump all the list entries... */
	pr_err("%s, for paddr 0x%lx\n",
			__func__, addr);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		pr_err("listed paddr 0x%lx, active = %d",
				region->paddr,
				region->info.active);
	}
	spin_unlock_irqrestore(&sync->pmem_stats_spinlock, flags);

	return 0;
}

static unsigned long msm_pmem_frame_vtop_lookup(struct msm_sync *sync,
		unsigned long buffer,
		uint32_t yoff, uint32_t cbcroff, int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;
	unsigned long flags = 0;

	spin_lock_irqsave(&sync->pmem_frame_spinlock, flags);
	hlist_for_each_entry_safe(region,
		node, n, &sync->pmem_frames, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
				(region->info.y_off == yoff) &&
				(region->info.cbcr_off == cbcroff) &&
				(region->info.fd == fd) &&
				(region->info.active == 0)) {
			region->info.active = 1;
			spin_unlock_irqrestore(&sync->pmem_frame_spinlock,
				flags);
			return region->paddr;
		}
	}
	/* After lookup failure, dump all the list entries... */
	pr_err("%s, for vaddr 0x%lx, yoff %d cbcroff %d\n",
			__func__, buffer, yoff, cbcroff);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_frames, list) {
		pr_err("listed vaddr 0x%p, cbcroff %d, active = %d",
				(region->info.vaddr),
				(region->info.cbcr_off),
				region->info.active);
	}

	spin_unlock_irqrestore(&sync->pmem_frame_spinlock, flags);

	return 0;
}

static unsigned long msm_pmem_stats_vtop_lookup(
		struct msm_sync *sync,
		unsigned long buffer,
		int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;
	unsigned long flags = 0;

	spin_lock_irqsave(&sync->pmem_stats_spinlock, flags);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
				(region->info.fd == fd) &&
				region->info.active == 0) {
			region->info.active = 1;
			spin_unlock_irqrestore(&sync->pmem_stats_spinlock,
				flags);
			return region->paddr;
		}
	}
	/* After lookup failure, dump all the list entries... */
	pr_err("%s, for vaddr %ld\n",
			__func__, buffer);
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		pr_err("listed vaddr 0x%p, active = %d",
				region->info.vaddr,
				region->info.active);
	}
	spin_unlock_irqrestore(&sync->pmem_stats_spinlock, flags);

	return 0;
}

static int __msm_pmem_table_del(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
{
	int rc = 0;
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;
	unsigned long flags = 0;

	switch (pinfo->type) {
	case MSM_PMEM_VIDEO:
	case MSM_PMEM_PREVIEW:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
	case MSM_PMEM_VIDEO_VPE:
		spin_lock_irqsave(&sync->pmem_frame_spinlock, flags);
		hlist_for_each_entry_safe(region, node, n,
			&sync->pmem_frames, list) {

			if (pinfo->type == region->info.type &&
					pinfo->vaddr == region->info.vaddr &&
					pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
				pr_info("%s: type %d, vaddr  0x%p\n",
					__func__, pinfo->type, pinfo->vaddr);
			}
		}
		spin_unlock_irqrestore(&sync->pmem_frame_spinlock, flags);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		spin_lock_irqsave(&sync->pmem_stats_spinlock, flags);
		hlist_for_each_entry_safe(region, node, n,
			&sync->pmem_stats, list) {

			if (pinfo->type == region->info.type &&
					pinfo->vaddr == region->info.vaddr &&
					pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
				pr_info("%s: type %d, vaddr  0x%p\n",
					__func__, pinfo->type, pinfo->vaddr);
			}
		}
		spin_unlock_irqrestore(&sync->pmem_stats_spinlock, flags);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_pmem_table_del(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_pmem_table_del(sync, &info);
}

static int __msm_get_frame(struct msm_sync *sync,
		struct msm_frame *frame)
{
	int rc = 0;

	struct msm_pmem_info pmem_info;
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_vfe_resp *vdata;
	struct msm_vfe_phy_info *pphy;

	qcmd = msm_dequeue(&sync->frame_q, list_frame);

	if (!qcmd) {
		pr_err("%s: no preview frame.\n", __func__);
		return -EAGAIN;
	}

	if ((!qcmd->command) && (qcmd->error_code & MSM_CAMERA_ERR_MASK)) {
		frame->error_code = qcmd->error_code;
		pr_info("%s: fake frame with camera error code = %d\n",
			__func__, frame->error_code);
		goto err;
	}

	vdata = (struct msm_vfe_resp *)(qcmd->command);
	pphy = &vdata->phy;

	rc = msm_pmem_frame_ptov_lookup(sync,
			pphy->y_phy,
			pphy->cbcr_phy,
			&pmem_info,
			1); /* mark frame in use */

	if (rc < 0) {
		pr_err("%s: cannot get frame, invalid lookup address "
			"y %x cbcr %x\n",
			__func__,
			pphy->y_phy,
			pphy->cbcr_phy);
		goto err;
	}

	frame->ts = qcmd->ts;
	frame->buffer = (unsigned long)pmem_info.vaddr;
	frame->y_off = pmem_info.y_off;
	frame->cbcr_off = pmem_info.cbcr_off;
	frame->fd = pmem_info.fd;
	frame->path = vdata->phy.output_id;
	CDBG("%s: y %x, cbcr %x, qcmd %x, virt_addr %x\n",
		__func__,
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);

err:
	free_qcmd(qcmd);
	return rc;
}

static int msm_get_frame(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_frame frame;

	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	rc = __msm_get_frame(sync, &frame);
	if (rc < 0)
		return rc;

	/* read the frame after the status has been read */
	rmb();

	mutex_lock(&sync->lock);
	if (sync->croplen) {
		if (frame.croplen != sync->croplen) {
			pr_err("%s: invalid frame croplen %d,"
				"expecting %d\n",
				__func__,
				frame.croplen,
				sync->croplen);
			mutex_unlock(&sync->lock);
			return -EINVAL;
		}

		if (copy_to_user((void *)frame.cropinfo,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			mutex_unlock(&sync->lock);
			return -EFAULT;
		}
	}

	if (sync->fdroiinfo.info) {
		if (copy_to_user((void *)frame.roi_info.info,
			sync->fdroiinfo.info,
			sync->fdroiinfo.info_len)) {
			ERR_COPY_TO_USER();
			mutex_unlock(&sync->lock);
			return -EFAULT;
		}
	}

	if (copy_to_user((void *)arg,
				&frame, sizeof(struct msm_frame))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	mutex_unlock(&sync->lock);
	CDBG("%s: got frame\n", __func__);

	return rc;
}

static int msm_enable_vfe(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
			arg,
			sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (sync->vfefn.vfe_enable)
		rc = sync->vfefn.vfe_enable(&cfg);

	pr_info("%s: rc %d\n", __func__, rc);
	return rc;
}

static int msm_disable_vfe(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
			arg,
			sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (sync->vfefn.vfe_disable)
		rc = sync->vfefn.vfe_disable(&cfg, NULL);

	pr_info("%s: rc %d\n", __func__, rc);
	return rc;
}

static struct msm_queue_cmd *__msm_control(struct msm_sync *sync,
		struct msm_device_queue *queue,
		struct msm_queue_cmd *qcmd,
		int timeout)
{
	int rc;

	CDBG("Inside __msm_control\n");
	if (sync->event_q.len <= 100 && sync->frame_q.len <= 100) {
		/* wake up config thread */
		msm_enqueue(&sync->event_q, &qcmd->list_config);
	} else {
		pr_err("%s, Error Queue limit exceeded e_q = %d, f_q = %d\n",
			__func__, sync->event_q.len, sync->frame_q.len);
		free_qcmd(qcmd);
		return NULL;
	}
	if (!queue)
		return NULL;

	/* wait for config status */
	CDBG("Waiting for config status \n");
	rc = wait_event_interruptible_timeout(
			queue->wait,
			!list_empty_careful(&queue->list),
			timeout);
	CDBG("Waiting over for config status\n");
	if (list_empty_careful(&queue->list)) {
		if (!rc) {
			rc = -ETIMEDOUT;
			pr_err("%s: wait_event error %d\n", __func__, rc);
			return ERR_PTR(rc);
		} else if (rc < 0) {
			pr_err("%s: wait_event error %d\n", __func__, rc);
			if (msm_delete_entry(&sync->event_q,
				list_config, qcmd)) {
				sync->ignore_qcmd = true;
				sync->ignore_qcmd_type =
					(int16_t)((struct msm_ctrl_cmd *)
					(qcmd->command))->type;
			}
			return ERR_PTR(rc);
		}
	}
	qcmd = msm_dequeue(queue, list_control);
	BUG_ON(!qcmd);
	CDBG("__msm_control done \n");
	return qcmd;
}

static struct msm_queue_cmd *__msm_control_nb(struct msm_sync *sync,
					struct msm_queue_cmd *qcmd_to_copy)
{
	/* Since this is a non-blocking command, we cannot use qcmd_to_copy and
	 * its data, since they are on the stack.  We replicate them on the heap
	 * and mark them on_heap so that they get freed when the config thread
	 * dequeues them.
	 */

	struct msm_ctrl_cmd *udata;
	struct msm_ctrl_cmd *udata_to_copy = qcmd_to_copy->command;

	struct msm_queue_cmd *qcmd =
			kmalloc(sizeof(*qcmd_to_copy) +
				sizeof(*udata_to_copy) +
				udata_to_copy->length,
				GFP_KERNEL);
	if (!qcmd) {
		pr_err("%s: out of memory\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	*qcmd = *qcmd_to_copy;
	udata = qcmd->command = qcmd + 1;
	memcpy(udata, udata_to_copy, sizeof(*udata));
	udata->value = udata + 1;
	memcpy(udata->value, udata_to_copy->value, udata_to_copy->length);

	atomic_set(&qcmd->on_heap, 1);

	/* qcmd_resp will be set to NULL */
	return __msm_control(sync, NULL, qcmd, 0);
}

static int msm_control(struct msm_control_device *ctrl_pmsm,
			int block,
			void __user *arg)
{
	int rc = 0;

	struct msm_sync *sync = ctrl_pmsm->pmsm->sync;
	void __user *uptr;
	struct msm_ctrl_cmd udata_resp;
	struct msm_queue_cmd *qcmd_resp = NULL;
	uint8_t data[max_control_command_size];
	struct msm_ctrl_cmd *udata;
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) +
			sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!qcmd) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}
	udata = (struct msm_ctrl_cmd *)(qcmd + 1);
	atomic_set(&(qcmd->on_heap), 1);
	CDBG("Inside msm_control\n");
	if (copy_from_user(udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto end;
	}

	uptr = udata->value;
	udata->value = data;
	qcmd->type = MSM_CAM_Q_CTRL;
	qcmd->command = udata;

	if (udata->length) {
		if (udata->length > sizeof(data)) {
			pr_err("%s: user data too large (%d, max is %d)\n",
					__func__,
					udata->length,
					sizeof(data));
			rc = -EIO;
			goto end;
		}
		if (copy_from_user(udata->value, uptr, udata->length)) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
			goto end;
		}
	}

	if (unlikely(!block)) {
		qcmd_resp = __msm_control_nb(sync, qcmd);
		goto end;
	}

	qcmd_resp = __msm_control(sync,
				  &ctrl_pmsm->ctrl_q,
				  qcmd, msecs_to_jiffies(10000));

	/* ownership of qcmd will be transfered to event queue */
	qcmd = NULL;

	if (!qcmd_resp || IS_ERR(qcmd_resp)) {
		/* Do not free qcmd_resp here.  If the config thread read it,
		 * then it has already been freed, and we timed out because
		 * we did not receive a MSM_CAM_IOCTL_CTRL_CMD_DONE.  If the
		 * config thread itself is blocked and not dequeueing commands,
		 * then it will either eventually unblock and process them,
		 * or when it is killed, qcmd will be freed in
		 * msm_release_config.
		 */
		rc = PTR_ERR(qcmd_resp);
		qcmd_resp = NULL;
		goto end;
	}

	if (qcmd_resp->command) {
		udata_resp = *(struct msm_ctrl_cmd *)qcmd_resp->command;
		if (udata_resp.length > 0) {
			if (copy_to_user(uptr,
					 udata_resp.value,
					 udata_resp.length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto end;
			}
		}
		udata_resp.value = uptr;

		if (copy_to_user((void *)arg, &udata_resp,
				sizeof(struct msm_ctrl_cmd))) {
			ERR_COPY_TO_USER();
			rc = -EFAULT;
			goto end;
		}
	}

end:
	free_qcmd(qcmd);
	CDBG("%s: done rc = %d\n", __func__, rc);
	return rc;
}

/* Divert frames for post-processing by delivering them to the config thread;
 * when post-processing is done, it will return the frame to the frame thread.
 */
static int msm_divert_frame(struct msm_sync *sync,
		struct msm_vfe_resp *data,
		struct msm_stats_event_ctrl *se)
{
	struct msm_pmem_info pinfo;
	struct msm_postproc buf;
	int rc;

	pr_info("%s: Frame PP sync->pp_mask %d\n", __func__, sync->pp_mask);

	if (!(sync->pp_mask & PP_PREV)  && !(sync->pp_mask & PP_SNAP)) {
		pr_err("%s: diverting frame, not in PP_PREV or PP_SNAP!\n",
			__func__);
		return -EINVAL;
	}

	rc = msm_pmem_frame_ptov_lookup(sync, data->phy.y_phy,
			data->phy.cbcr_phy, &pinfo,
			0);  /* do clear the active flag */
	if (rc < 0) {
		pr_err("%s: msm_pmem_frame_ptov_lookup failed\n", __func__);
		return rc;
	}

	buf.fmain.buffer = (unsigned long)pinfo.vaddr;
	buf.fmain.y_off = pinfo.y_off;
	buf.fmain.cbcr_off = pinfo.cbcr_off;
	buf.fmain.fd = pinfo.fd;

	CDBG("%s: buf %ld fd %d\n", __func__, buf.fmain.buffer, buf.fmain.fd);
	if (copy_to_user((void *)(se->stats_event.data),
			&(buf.fmain), sizeof(struct msm_frame))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}
	return 0;
}

static int msm_get_stats(struct msm_sync *sync, void __user *arg)
{
	int timeout;
	int rc = 0;

	struct msm_stats_event_ctrl se;

	struct msm_queue_cmd *qcmd = NULL;
	struct msm_ctrl_cmd  *ctrl = NULL;
	struct msm_vfe_resp  *data = NULL;
	struct msm_stats_buf stats;

	if (copy_from_user(&se, arg,
			sizeof(struct msm_stats_event_ctrl))) {
		ERR_COPY_FROM_USER();
		pr_err("%s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	timeout = (int)se.timeout_ms;

	CDBG("%s: timeout %d\n", __func__, timeout);
	rc = wait_event_interruptible_timeout(
			sync->event_q.wait,
			!list_empty_careful(&sync->event_q.list),
			msecs_to_jiffies(timeout));
	if (list_empty_careful(&sync->event_q.list)) {
		if (rc == 0)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("%s: error %d\n", __func__, rc);
			return rc;
		}
	}
	CDBG("%s: returned from wait: %d\n", __func__, rc);

	rc = 0;

	qcmd = msm_dequeue(&sync->event_q, list_config);
	if (!qcmd) {
		/* Should be associated with wait_event
			error -512 from __msm_control*/
		pr_info("%s, qcmd is Null\n", __func__);
		rc = -ETIMEDOUT;
		return rc;
	}

	CDBG("%s: received from DSP %d\n", __func__, qcmd->type);

	/* order the reads of stat/snapshot buffers */
	rmb();

	switch (qcmd->type) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		data = (struct msm_vfe_resp *)(qcmd->command);

		/* adsp event and message */
		se.resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		se.stats_event.type   = data->evt_msg.type;
		se.stats_event.msg_id = data->evt_msg.msg_id;
		se.stats_event.len    = data->evt_msg.len;

		CDBG("%s: qcmd->type %d length %d msd_id %d\n", __func__,
			qcmd->type,
			se.stats_event.len,
			se.stats_event.msg_id);

		if ((data->type >= VFE_MSG_STATS_AEC) &&
		    (data->type <=  VFE_MSG_STATS_WE)) {
			/* the check above includes all stats type. */
			stats.buffer =
			msm_pmem_stats_ptov_lookup(sync,
					data->phy.sbuf_phy,
					&(stats.fd));
			if (!stats.buffer) {
				pr_err("%s: msm_pmem_stats_ptov_lookup error\n",
					__func__);
				rc = -EINVAL;
				goto failure;
			}

			if (copy_to_user((void *)(se.stats_event.data),
					&stats,
					sizeof(struct msm_stats_buf))) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		} else if ((data->evt_msg.len > 0) &&
				(data->type == VFE_MSG_GENERAL)) {
			if (copy_to_user((void *)(se.stats_event.data),
					data->evt_msg.data,
					data->evt_msg.len)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		} else {
			if ((sync->pp_frame_avail == 1) &&
				(sync->pp_mask & PP_PREV) &&
				(data->type == VFE_MSG_OUTPUT_P)) {
					CDBG("%s:%d:preiew PP\n",
					__func__, __LINE__);
					rc = msm_divert_frame(sync, data, &se);
					sync->pp_frame_avail = 0;
			} else {
				if ((sync->pp_mask & PP_PREV) &&
					(data->type == VFE_MSG_OUTPUT_P)) {
					free_qcmd(qcmd);
					return 0;
				} else
					CDBG("%s:indication type is %d\n",
						__func__, data->type);
			}
			if (sync->pp_mask & PP_SNAP)
				if (data->type == VFE_MSG_OUTPUT_S ||
					data->type == VFE_MSG_OUTPUT_T)
					rc = msm_divert_frame(sync, data, &se);
		}
		break;

	case MSM_CAM_Q_CTRL:
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("%s: qcmd->type %d length %d\n", __func__,
			qcmd->type, ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
						ctrl->value,
						ctrl->length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		}

		se.resptype = MSM_CAM_RESP_CTRL;

		/* what to control */
		se.ctrl_cmd.type = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
		se.ctrl_cmd.resp_fd = ctrl->resp_fd;
		break;

	case MSM_CAM_Q_V4L2_REQ:
		/* control command from v4l2 client */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);
		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
					ctrl->value, ctrl->length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		}

		/* 2 tells config thread this is v4l2 request */
		se.resptype = MSM_CAM_RESP_V4L2;

		/* what to control */
		se.ctrl_cmd.type   = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
		break;

	default:
		rc = -EFAULT;
		goto failure;
	} /* switch qcmd->type */
	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
		goto failure;
	}

failure:
	free_qcmd(qcmd);

	CDBG("%s: %d\n", __func__, rc);
	return rc;
}

static int msm_ctrl_cmd_done(struct msm_control_device *ctrl_pmsm,
		void __user *arg)
{
	void __user *uptr;
	struct msm_queue_cmd *qcmd = &ctrl_pmsm->qcmd;
	struct msm_ctrl_cmd *command = &ctrl_pmsm->ctrl;

	if (copy_from_user(command, arg, sizeof(*command))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	atomic_set(&qcmd->on_heap, 0);
	qcmd->command = command;
	uptr = command->value;

	if (command->length > 0) {
		command->value = ctrl_pmsm->ctrl_data;
		if (command->length > sizeof(ctrl_pmsm->ctrl_data)) {
			pr_err("%s: user data %d is too big (max %d)\n",
				__func__, command->length,
				sizeof(ctrl_pmsm->ctrl_data));
			return -EINVAL;
	}

		if (copy_from_user(command->value,
					uptr,
					command->length)) {
			ERR_COPY_FROM_USER();
			return -EFAULT;
		}
	} else
		command->value = NULL;

	/* Ignore the command if the ctrl cmd has
	   return back due to signaling */
	/* Should be associated with wait_event
	   error -512 from __msm_control*/
	if (ctrl_pmsm->pmsm->sync->ignore_qcmd == true &&
	   ctrl_pmsm->pmsm->sync->ignore_qcmd_type == (int16_t)command->type) {
		ctrl_pmsm->pmsm->sync->ignore_qcmd = false;
		ctrl_pmsm->pmsm->sync->ignore_qcmd_type = -1;
	} else /* wake up control thread */
	msm_enqueue(&ctrl_pmsm->ctrl_q, &qcmd->list_control);
	return 0;
}

static int msm_config_vpe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vpe_cfg_cmd cfgcmd;
	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}
	CDBG("%s: cmd_type %s\n", __func__, vfe_config_cmd[cfgcmd.cmd_type]);
	switch (cfgcmd.cmd_type) {
	case CMD_VPE:
		return sync->vpefn.vpe_config(&cfgcmd, NULL);
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd.cmd_type);
	}
	return -EINVAL;
}

static int msm_config_vfe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;
	struct msm_pmem_region region[8];
	struct axidata axi_data;

	if (!sync->vfefn.vfe_config) {
		pr_err("%s: no vfe_config!\n", __func__);
		return -EIO;
	}

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	memset(&axi_data, 0, sizeof(axi_data));
	CDBG("%s: cmd_type %s\n", __func__, vfe_config_cmd[cfgcmd.cmd_type]);
	switch (cfgcmd.cmd_type) {
	case CMD_STATS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
				MSM_PMEM_AEC_AWB, &region[0],
				NUM_STAT_OUTPUT_BUFFERS,
				&sync->pmem_stats_spinlock);
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_stats,
				MSM_PMEM_AF, &region[axi_data.bufnum1],
				NUM_STAT_OUTPUT_BUFFERS,
				&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1 || !axi_data.bufnum2) {
			pr_err("%s: pmem region lookup error\n", __func__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AF_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
				MSM_PMEM_AF, &region[0],
				NUM_STAT_OUTPUT_BUFFERS,
				&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
				MSM_PMEM_AEC_AWB, &region[0],
				NUM_STAT_OUTPUT_BUFFERS,
				&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC, &region[0],
			NUM_STAT_OUTPUT_BUFFERS,
			&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS,
			&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);


	case CMD_STATS_IHIST_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_IHIST, &region[0],
			NUM_STAT_OUTPUT_BUFFERS,
			&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_RS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_RS, &region[0],
			NUM_STAT_OUTPUT_BUFFERS,
			&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_CS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_CS, &region[0],
			NUM_STAT_OUTPUT_BUFFERS,
			&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_GENERAL:
	case CMD_STATS_DISABLE:
		return sync->vfefn.vfe_config(&cfgcmd, NULL);
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd.cmd_type);
	}

	return -EINVAL;
}
static int msm_vpe_frame_cfg(struct msm_sync *sync,
				void *cfgcmdin)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;

	struct msm_vpe_cfg_cmd *cfgcmd;
	cfgcmd = (struct msm_vpe_cfg_cmd *)cfgcmdin;

	memset(&axi_data, 0, sizeof(axi_data));
	CDBG("In vpe_frame_cfg cfgcmd->cmd_type = %s\n",
		vfe_config_cmd[cfgcmd->cmd_type]);
	switch (cfgcmd->cmd_type) {
	case CMD_AXI_CFG_VPE:
		pmem_type = MSM_PMEM_VIDEO_VPE;
		axi_data.bufnum1 =
			msm_pmem_region_lookup_2(&sync->pmem_frames, pmem_type,
				&region[0], 8, &sync->pmem_frame_spinlock);
		CDBG("axi_data.bufnum1 = %d\n", axi_data.bufnum1);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		pmem_type = MSM_PMEM_VIDEO;
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		break;
	}
	axi_data.region = &region[0];
	CDBG("out vpe_frame_cfg cfgcmd->cmd_type = %s\n",
		vfe_config_cmd[cfgcmd->cmd_type]);
	/* send the AXI configuration command to driver */
	if (sync->vpefn.vpe_config)
		rc = sync->vpefn.vpe_config(cfgcmd, data);
	return rc;
}

static int msm_frame_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {

	case CMD_AXI_CFG_PREVIEW:
		pmem_type = MSM_PMEM_PREVIEW;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8, &sync->pmem_frame_spinlock);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error (empty %d)\n",
				__func__, __LINE__,
				hlist_empty(&sync->pmem_frames));
			return -EINVAL;
		}
		break;

	case CMD_AXI_CFG_VIDEO:
		pmem_type = MSM_PMEM_PREVIEW;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8, &sync->pmem_frame_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_VIDEO;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[axi_data.bufnum1],
				(8-(axi_data.bufnum1)),
				&sync->pmem_frame_spinlock);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;


	case CMD_AXI_CFG_SNAP:
		pmem_type = MSM_PMEM_THUMBNAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8, &sync->pmem_frame_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[axi_data.bufnum1],
				(8-(axi_data.bufnum1)),
				 &sync->pmem_frame_spinlock);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_RAW_PICT_AXI_CFG:
		pmem_type = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8, &sync->pmem_frame_spinlock);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_GENERAL:
		data = NULL;
		break;

	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	axi_data.region = &region[0];

	/* send the AXI configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, data);

	return rc;
}

static int msm_get_sensor_info(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_sensor_info *sdata;

	if (copy_from_user(&info,
			arg,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	sdata = sync->pdev->dev.platform_data;
	CDBG("%s: sensor_name %s\n", __func__, sdata->sensor_name);

	memcpy(&info.name[0],
		sdata->sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled = sdata->flash_data->flash_type !=
		MSM_CAMERA_FLASH_NONE;

	/* copy back to user space */
	if (copy_to_user((void *)arg,
			&info,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	return rc;
}

static int msm_get_camera_info(void __user *arg)
{
	int rc = 0;
	int i = 0;
	struct msm_camera_info info;

	if (copy_from_user(&info, arg, sizeof(struct msm_camera_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	CDBG("%s: camera_node %d\n", __func__, camera_node);
	info.num_cameras = camera_node;

	for (i = 0; i < camera_node; i++) {
		info.has_3d_support[i] = 0;
		info.is_internal_cam[i] = 0;
		info.s_mount_angle[i] = sensor_mount_angle[i];
		switch (camera_type[i]) {
		case FRONT_CAMERA_2D:
			info.is_internal_cam[i] = 1;
			break;
		case BACK_CAMERA_3D:
			info.has_3d_support[i] = 1;
			break;
		case BACK_CAMERA_2D:
		default:
			break;
		}
	}
	/* copy back to user space */
	if (copy_to_user((void *)arg, &info, sizeof(struct msm_camera_info))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}
	return rc;
}

static int __msm_put_frame_buf(struct msm_sync *sync,
		struct msm_frame *pb)
{
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	int rc = -EIO;

	pphy = msm_pmem_frame_vtop_lookup(sync,
		pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd);

	if (pphy != 0) {
		CDBG("%s: rel: vaddr %lx, paddr %lx\n",
			__func__,
			pb->buffer, pphy);
		cfgcmd.cmd_type = CMD_FRAME_BUF_RELEASE;
		cfgcmd.value    = (void *)pb;
		if (sync->vfefn.vfe_config)
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
	} else {
		pr_err("%s: msm_pmem_frame_vtop_lookup failed\n",
			__func__);
		rc = -EINVAL;
	}

	return rc;
}

static int msm_put_frame_buffer(struct msm_sync *sync, void __user *arg)
{
	struct msm_frame buf_t;

	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_put_frame_buf(sync, &buf_t);
}

static int __msm_register_pmem(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
{
	int rc = 0;

	switch (pinfo->type) {
	case MSM_PMEM_VIDEO:
	case MSM_PMEM_PREVIEW:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
	case MSM_PMEM_VIDEO_VPE:
		rc = msm_pmem_table_add(&sync->pmem_frames, pinfo,
			&sync->pmem_frame_spinlock);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
	case MSM_PMEM_AEC:
	case MSM_PMEM_AWB:
	case MSM_PMEM_RS:
	case MSM_PMEM_CS:
	case MSM_PMEM_IHIST:
	case MSM_PMEM_SKIN:

		rc = msm_pmem_table_add(&sync->pmem_stats, pinfo,
			 &sync->pmem_stats_spinlock);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_register_pmem(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_register_pmem(sync, &info);
}

static int msm_stats_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;

	struct msm_pmem_region region[3];
	int pmem_type = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {
	case CMD_STATS_AXI_CFG:
		pmem_type = MSM_PMEM_AEC_AWB;
		break;
	case CMD_STATS_AF_AXI_CFG:
		pmem_type = MSM_PMEM_AF;
		break;
	case CMD_GENERAL:
		data = NULL;
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	if (cfgcmd->cmd_type != CMD_GENERAL) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats, pmem_type,
				&region[0], NUM_STAT_OUTPUT_BUFFERS,
				&sync->pmem_stats_spinlock);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	axi_data.region = &region[0];
	}

	/* send the AEC/AWB STATS configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, &axi_data);

	return rc;
}

static int msm_put_stats_buffer(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	CDBG("%s\n", __func__);
	pphy = msm_pmem_stats_vtop_lookup(sync, buf.buffer, buf.fd);

	if (pphy != 0) {
		if (buf.type == STAT_AEAW)
			cfgcmd.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else if (buf.type == STAT_AEC)
			cfgcmd.cmd_type = CMD_STATS_AEC_BUF_RELEASE;
		else if (buf.type == STAT_AWB)
			cfgcmd.cmd_type = CMD_STATS_AWB_BUF_RELEASE;
		else if (buf.type == STAT_IHIST)
			cfgcmd.cmd_type = CMD_STATS_IHIST_BUF_RELEASE;
		else if (buf.type == STAT_RS)
			cfgcmd.cmd_type = CMD_STATS_RS_BUF_RELEASE;
		else if (buf.type == STAT_CS)
			cfgcmd.cmd_type = CMD_STATS_CS_BUF_RELEASE;

		else {
			pr_err("%s: invalid buf type %d\n",
				__func__,
				buf.type);
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd.value = (void *)&buf;

		if (sync->vfefn.vfe_config) {
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
			if (rc < 0)
				pr_err("%s: vfe_config error %d\n",
					__func__, rc);
		} else
			pr_err("%s: vfe_config is NULL\n", __func__);
	} else {
		pr_err("%s: NULL physical address\n", __func__);
		rc = -EINVAL;
	}

put_done:
	return rc;
}

static int msm_axi_config(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	switch (cfgcmd.cmd_type) {
	case CMD_AXI_CFG_VIDEO:
	case CMD_AXI_CFG_PREVIEW:
	case CMD_AXI_CFG_SNAP:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(sync, &cfgcmd);
	case CMD_AXI_CFG_VPE:
		return msm_vpe_frame_cfg(sync, (void *)&cfgcmd);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(sync, &cfgcmd);

	default:
		pr_err("%s: unknown command type %d\n",
			__func__,
			cfgcmd.cmd_type);
		return -EINVAL;
	}

	return 0;
}

static int __msm_get_pic(struct msm_sync *sync, struct msm_ctrl_cmd *ctrl)
{
	int rc = 0;
	int tm;
	unsigned long flags = 0;

	struct msm_queue_cmd *qcmd = NULL;

	tm = (int)ctrl->timeout_ms;

	spin_lock_irqsave(&sync->abort_pict_lock, flags);
	sync->get_pic_abort = 0;
	spin_unlock_irqrestore(&sync->abort_pict_lock, flags);

	rc = wait_event_interruptible_timeout(
			sync->pict_q.wait,
			!list_empty_careful(
				&sync->pict_q.list) || sync->get_pic_abort,
			msecs_to_jiffies(tm));

	spin_lock_irqsave(&sync->abort_pict_lock, flags);
	if (sync->get_pic_abort) {
		sync->get_pic_abort = 0;
		spin_unlock_irqrestore(&sync->abort_pict_lock, flags);
		return -ENODATA;
	}
	spin_unlock_irqrestore(&sync->abort_pict_lock, flags);

	if (list_empty_careful(&sync->pict_q.list)) {
		if (rc == 0)
			return -ETIMEDOUT;
		if (rc < 0) {
			pr_err("%s: rc %d\n", __func__, rc);
			return rc;
		}
	}

	rc = 0;

	qcmd = msm_dequeue(&sync->pict_q, list_pict);
	BUG_ON(!qcmd);

	if (qcmd->command != NULL) {
		struct msm_ctrl_cmd *q =
			(struct msm_ctrl_cmd *)qcmd->command;
		ctrl->type = q->type;
		ctrl->status = q->status;
	} else {
		ctrl->type = -1;
		ctrl->status = -1;
	}

	free_qcmd(qcmd);

	return rc;
}

static int msm_get_pic(struct msm_sync *sync, void __user *arg)
{
	struct msm_ctrl_cmd ctrlcmd_t;
	int rc;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	rc = __msm_get_pic(sync, &ctrlcmd_t);
	if (rc < 0) {
		pr_err("%s, failed\n", __func__);
		return rc;
	}

	if (sync->croplen) {
		if (ctrlcmd_t.length != sync->croplen) {
			pr_err("%s: invalid len %d < %d\n",
				__func__,
				ctrlcmd_t.length,
				sync->croplen);
			return -EINVAL;
		}
		if (copy_to_user(ctrlcmd_t.value,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			return -EFAULT;
		}
	}
	pr_info("%s: copy snapshot frame to user\n", __func__);
	if (copy_to_user((void *)arg,
		&ctrlcmd_t,
		sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}
	return 0;
}

static int msm_set_crop(struct msm_sync *sync, void __user *arg)
{
	struct crop_info crop;

	mutex_lock(&sync->lock);
	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info))) {
		ERR_COPY_FROM_USER();
		mutex_unlock(&sync->lock);
		return -EFAULT;
	}

	if (crop.len != CROP_LEN) {
		mutex_unlock(&sync->lock);
		return -EINVAL;
	}

	if (!sync->croplen) {
		sync->cropinfo = kmalloc(crop.len, GFP_KERNEL);
		if (!sync->cropinfo) {
			mutex_unlock(&sync->lock);
			return -ENOMEM;
		}
	}

	if (copy_from_user(sync->cropinfo,
				crop.info,
				crop.len)) {
		ERR_COPY_FROM_USER();
		sync->croplen = 0;
		kfree(sync->cropinfo);
		mutex_unlock(&sync->lock);
		return -EFAULT;
	}

	sync->croplen = crop.len;

	mutex_unlock(&sync->lock);
	return 0;
}

static int msm_error_config(struct msm_sync *sync, void __user *arg)
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);

	qcmd->command = NULL;

	if (qcmd)
		atomic_set(&(qcmd->on_heap), 1);

	if (copy_from_user(&(qcmd->error_code), arg, sizeof(uint32_t))) {
		ERR_COPY_FROM_USER();
		free_qcmd(qcmd);
		return -EFAULT;
	}

	pr_info("%s: Enqueue Fake Frame with error code = %d\n", __func__,
		qcmd->error_code);
	msm_enqueue(&sync->frame_q, &qcmd->list_frame);
	return 0;
}

static int msm_set_fd_roi(struct msm_sync *sync, void __user *arg)
{
	struct fd_roi_info fd_roi;

	mutex_lock(&sync->lock);
	if (copy_from_user(&fd_roi,
			arg,
			sizeof(struct fd_roi_info))) {
		ERR_COPY_FROM_USER();
		mutex_unlock(&sync->lock);
		return -EFAULT;
	}
	if (fd_roi.info_len <= 0) {
		mutex_unlock(&sync->lock);
		return -EFAULT;
	}

	if (!sync->fdroiinfo.info) {
		sync->fdroiinfo.info = kmalloc(fd_roi.info_len, GFP_KERNEL);
		if (!sync->fdroiinfo.info) {
			mutex_unlock(&sync->lock);
			return -ENOMEM;
		}
		sync->fdroiinfo.info_len = fd_roi.info_len;
	} else if (sync->fdroiinfo.info_len < fd_roi.info_len) {
		mutex_unlock(&sync->lock);
		return -EINVAL;
    }

	if (copy_from_user(sync->fdroiinfo.info,
			fd_roi.info,
			fd_roi.info_len)) {
		ERR_COPY_FROM_USER();
		kfree(sync->fdroiinfo.info);
		sync->fdroiinfo.info = NULL;
		mutex_unlock(&sync->lock);
		return -EFAULT;
	}
	mutex_unlock(&sync->lock);
	return 0;
}

static int msm_pp_grab(struct msm_sync *sync, void __user *arg)
{
	uint32_t enable;
	if (copy_from_user(&enable, arg, sizeof(enable))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
		enable &= PP_MASK;
		if (enable & (enable - 1)) {
			pr_err("%s: error: more than one PP request!\n",
				__func__);
			return -EINVAL;
		}
		if (sync->pp_mask) {
			if (enable) {
				pr_err("%s: postproc %x is already enabled\n",
					__func__, sync->pp_mask & enable);
				return -EINVAL;
			} else {
				sync->pp_mask &= enable;
				CDBG("%s: sync->pp_mask %d enable %d\n",
					__func__, sync->pp_mask, enable);
				return 0;
			}
		}

		CDBG("%s: sync->pp_mask %d enable %d\n", __func__,
			sync->pp_mask, enable);
		sync->pp_mask |= enable;
	}

	return 0;
}

static int msm_pp_release(struct msm_sync *sync, void __user *arg)
{
	unsigned long flags;

	if (!sync->pp_mask) {
		pr_warning("%s: pp not in progress for\n", __func__);
		return -EINVAL;
	}
	if (sync->pp_mask & PP_PREV) {


		spin_lock_irqsave(&pp_prev_spinlock, flags);
		if (!sync->pp_prev) {
			pr_err("%s: no preview frame to deliver!\n",
				__func__);
			spin_unlock_irqrestore(&pp_prev_spinlock,
				flags);
			return -EINVAL;
		}
		CDBG("%s: delivering pp_prev\n", __func__);

			if (sync->frame_q.len <= 100 &&
				sync->event_q.len <= 100) {
					msm_enqueue(&sync->frame_q,
						&sync->pp_prev->list_frame);
			} else {
				pr_err("%s, Error Queue limit exceeded f_q=%d,\
					e_q = %d\n",
					__func__, sync->frame_q.len,
					sync->event_q.len);
				free_qcmd(sync->pp_prev);
				goto done;
			}

			sync->pp_prev = NULL;
			spin_unlock_irqrestore(&pp_prev_spinlock, flags);
		goto done;
	}

	if ((sync->pp_mask & PP_SNAP) ||
		(sync->pp_mask & PP_RAW_SNAP)) {
		spin_lock_irqsave(&pp_snap_spinlock, flags);
		if (!sync->pp_snap) {
			pr_err("%s: no snapshot to deliver!\n", __func__);
			spin_unlock_irqrestore(&pp_snap_spinlock, flags);
			return -EINVAL;
		}
		CDBG("%s: delivering pp_snap\n", __func__);
		msm_enqueue(&sync->pict_q, &sync->pp_snap->list_pict);
		sync->pp_snap = NULL;
		sync->pp_thumb = NULL;
		spin_unlock_irqrestore(&pp_snap_spinlock, flags);
	}

done:
	return 0;
}

static long msm_ioctl_common(struct msm_cam_device *pmsm,
		unsigned int cmd,
		void __user *argp)
{
	CDBG("%s\n", __func__);
	switch (cmd) {
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(pmsm->sync, argp);
	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(pmsm->sync, argp);
	default:
		return -EINVAL;
	}
}

static long msm_ioctl_config(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_cam_device *pmsm = filep->private_data;

	CDBG("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		rc = msm_config_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_CONFIG_VPE:
		/* Coming from config thread for update */
		rc = msm_config_vpe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_GET_STATS:
		/* Coming from config thread wait
		 * for vfe statistics and control requests */
		rc = msm_get_stats(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_ENABLE_VFE:
		/* This request comes from control thread:
		 * enable either QCAMTASK or VFETASK */
		rc = msm_enable_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_DISABLE_VFE:
		/* This request comes from control thread:
		 * disable either QCAMTASK or VFETASK */
		rc = msm_disable_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_VFE_APPS_RESET:
		msm_camio_vfe_blk_reset();
		rc = 0;
		break;

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		rc = msm_put_stats_buffer(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_AXI_CONFIG:
	case MSM_CAM_IOCTL_AXI_VPE_CONFIG:
		rc = msm_axi_config(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SET_CROP:
		rc = msm_set_crop(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SET_FD_ROI:
		rc = msm_set_fd_roi(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_PICT_PP:
		/* Grab one preview frame or one snapshot
		 * frame.
		 */
		rc = msm_pp_grab(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_PICT_PP_DONE:
		/* Release the preview of snapshot frame
		 * that was grabbed.
		 */
		rc = msm_pp_release(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		rc = pmsm->sync->sctrl.s_config(argp);
		break;

	case MSM_CAM_IOCTL_FLASH_LED_CFG: {
		uint32_t led_state;
		if (copy_from_user(&led_state, argp, sizeof(led_state))) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else
			rc = msm_camera_flash_set_led_state(pmsm->sync->
					sdata->flash_data, led_state);
		break;
	}

	case MSM_CAM_IOCTL_STROBE_FLASH_CFG: {
		uint32_t flash_type;
		if (copy_from_user(&flash_type, argp, sizeof(flash_type))) {
			pr_err("msm_strobe_flash_init failed");
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else {
			CDBG("msm_strobe_flash_init enter");
			rc = msm_strobe_flash_init(pmsm->sync, flash_type);
		}
		break;
	}

	case MSM_CAM_IOCTL_STROBE_FLASH_RELEASE:
		if (pmsm->sync->sdata->strobe_flash_data) {
			rc = pmsm->sync->sfctrl.strobe_flash_release(
				pmsm->sync->sdata->strobe_flash_data, 0);
		}
		break;

	case MSM_CAM_IOCTL_STROBE_FLASH_CHARGE: {
		uint32_t charge_en;
		if (copy_from_user(&charge_en, argp, sizeof(charge_en))) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else
			rc = pmsm->sync->sfctrl.strobe_flash_charge(
			pmsm->sync->sdata->strobe_flash_data->flash_charge,
			charge_en, pmsm->sync->sdata->strobe_flash_data->
				flash_recharge_duration);
		break;
	}

	case MSM_CAM_IOCTL_FLASH_CTRL: {
		struct flash_ctrl_data flash_info;
		if (copy_from_user(&flash_info, argp, sizeof(flash_info))) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else
			rc = msm_flash_ctrl(pmsm->sync->sdata, &flash_info);

		break;
	}

	case MSM_CAM_IOCTL_ERROR_CONFIG:
		rc = msm_error_config(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_ABORT_CAPTURE: {
		unsigned long flags = 0;
		pr_info("get_pic:MSM_CAM_IOCTL_ABORT_CAPTURE\n");
		spin_lock_irqsave(&pmsm->sync->abort_pict_lock, flags);
		pmsm->sync->get_pic_abort = 1;
		spin_unlock_irqrestore(&pmsm->sync->abort_pict_lock, flags);
		wake_up(&(pmsm->sync->pict_q.wait));
		rc = 0;
		break;
	}

	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	CDBG("%s: cmd %d DONE\n", __func__, _IOC_NR(cmd));
	return rc;
}

static int msm_unblock_poll_frame(struct msm_sync *);

static long msm_ioctl_frame(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_cam_device *pmsm = filep->private_data;


	switch (cmd) {
	case MSM_CAM_IOCTL_GETFRAME:
		/* Coming from frame thread to get frame
		 * after SELECT is done */
		rc = msm_get_frame(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER:
		rc = msm_put_frame_buffer(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME:
		rc = msm_unblock_poll_frame(pmsm->sync);
		break;
	default:
		break;
	}

	return rc;
}


static long msm_ioctl_control(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_cam_device *pmsm = ctrl_pmsm->pmsm;

	switch (cmd) {
	case MSM_CAM_IOCTL_CTRL_COMMAND:
		/* Coming from control thread, may need to wait for
		 * command status */
		CDBG("calling msm_control kernel msm_ioctl_control\n");
		mutex_lock(&ctrl_cmd_lock);
		rc = msm_control(ctrl_pmsm, 1, argp);
		mutex_unlock(&ctrl_cmd_lock);
		break;
	case MSM_CAM_IOCTL_CTRL_COMMAND_2:
		/* Sends a message, returns immediately */
		rc = msm_control(ctrl_pmsm, 0, argp);
		break;
	case MSM_CAM_IOCTL_CTRL_CMD_DONE:
		/* Config thread calls the control thread to notify it
		 * of the result of a MSM_CAM_IOCTL_CTRL_COMMAND.
		 */
		rc = msm_ctrl_cmd_done(ctrl_pmsm, argp);
		break;
	case MSM_CAM_IOCTL_GET_PICTURE:
		rc = msm_get_pic(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_GET_CAMERA_INFO:
		rc = msm_get_camera_info(argp);
		break;
	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	return rc;
}

static int __msm_release(struct msm_sync *sync)
{
	struct msm_pmem_region *region;
	struct hlist_node *hnode;
	struct hlist_node *n;

	mutex_lock(&sync->lock);
	if (sync->opencnt)
		sync->opencnt--;
	pr_info("%s, open count =%d\n", __func__, sync->opencnt);
	if (!sync->opencnt) {
		/* need to clean up system resource */
		pr_info("%s, vfe_release\n", __func__);
		if (sync->core_powered_on) {
			if (sync->vfefn.vfe_release)
				sync->vfefn.vfe_release(sync->pdev);
			/*sensor release */
			pr_info("%s, s_release\n", __func__);
			sync->sctrl.s_release();
			pr_info("%s, msm_camio_sensor_clk_off\n", __func__);
			msm_camio_sensor_clk_off(sync->pdev);
			if (sync->sfctrl.strobe_flash_release) {
				pr_info("%s, strobe_flash_release\n", __func__);
				sync->sfctrl.strobe_flash_release(
				sync->sdata->strobe_flash_data, 1);
			}
		}
		kfree(sync->cropinfo);
		sync->cropinfo = NULL;
		sync->croplen = 0;
		pr_info("%s, free frame pmem region\n", __func__);
		hlist_for_each_entry_safe(region, hnode, n,
				&sync->pmem_frames, list) {
			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}
		pr_info("%s, free stats pmem region\n", __func__);
		hlist_for_each_entry_safe(region, hnode, n,
				&sync->pmem_stats, list) {
			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}
		msm_queue_drain(&sync->pict_q, list_pict);

		wake_unlock(&sync->wake_lock);
		sync->apps_id = NULL;
		pr_info("%s: completed\n", __func__);
		sync->core_powered_on = 0;
	}
	mutex_unlock(&sync->lock);

	return 0;
}

static int msm_release_config(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_cam_device *pmsm = filep->private_data;
	pr_info("%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		msm_queue_drain(&pmsm->sync->event_q, list_config);
		atomic_set(&pmsm->opened, 0);
	}
	pr_info("%s, completed\n", __func__);
	return rc;
}

static int msm_release_control(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_cam_device *pmsm = ctrl_pmsm->pmsm;
	pr_info("%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	g_v4l2_opencnt--;
	mutex_lock(&pmsm->sync->lock);
	if (pmsm->sync->core_powered_on && pmsm->sync->vfefn.vfe_stop) {
		pr_info("%s, stop vfe if active\n", __func__);
		pmsm->sync->vfefn.vfe_stop();
	}
	mutex_unlock(&pmsm->sync->lock);
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		msm_queue_drain(&ctrl_pmsm->ctrl_q, list_control);
		kfree(ctrl_pmsm);
	}
		pr_info("%s, completed\n", __func__);
	return rc;
}

static int msm_release_frame(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_cam_device *pmsm = filep->private_data;
	pr_info("%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		msm_queue_drain(&pmsm->sync->frame_q, list_frame);
		atomic_set(&pmsm->opened, 0);
	}
	pr_info("%s, completed\n", __func__);
	return rc;
}

static int msm_unblock_poll_frame(struct msm_sync *sync)
{
	unsigned long flags;
	CDBG("%s\n", __func__);
	spin_lock_irqsave(&sync->frame_q.lock, flags);
	sync->unblock_poll_frame = 1;
	wake_up(&sync->frame_q.wait);
	spin_unlock_irqrestore(&sync->frame_q.lock, flags);
	return 0;
}

static unsigned int __msm_poll_frame(struct msm_sync *sync,
		struct file *filep,
		struct poll_table_struct *pll_table)
{
	int rc = 0;
	unsigned long flags;

	poll_wait(filep, &sync->frame_q.wait, pll_table);

	spin_lock_irqsave(&sync->frame_q.lock, flags);
	if (!list_empty_careful(&sync->frame_q.list))
		/* frame ready */
		rc = POLLIN | POLLRDNORM;
	if (sync->unblock_poll_frame) {
		CDBG("%s: sync->unblock_poll_frame is true\n", __func__);
		rc |= POLLPRI;
		sync->unblock_poll_frame = 0;
	}
	spin_unlock_irqrestore(&sync->frame_q.lock, flags);

	return rc;
}

static unsigned int msm_poll_frame(struct file *filep,
	struct poll_table_struct *pll_table)
{
	struct msm_cam_device *pmsm = filep->private_data;
	return __msm_poll_frame(pmsm->sync, filep, pll_table);
}

/*
 * This function executes in interrupt context.
 */

static void *msm_vfe_sync_alloc(int size,
			void *syncdata __attribute__((unused)),
			gfp_t gfp)
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) + size, gfp);
	if (qcmd) {
		atomic_set(&qcmd->on_heap, 1);
		return qcmd + 1;
	}
	return NULL;
}

static void *msm_vpe_sync_alloc(int size,
			void *syncdata __attribute__((unused)),
			gfp_t gfp)
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) + size, gfp);
	if (qcmd) {
		atomic_set(&qcmd->on_heap, 1);
		return qcmd + 1;
	}
	return NULL;
}

static void msm_vfe_sync_free(void *ptr)
{
	if (ptr) {
		struct msm_queue_cmd *qcmd =
			(struct msm_queue_cmd *)ptr;
		qcmd--;
		if (atomic_read(&qcmd->on_heap))
			kfree(qcmd);
	}
}

static void msm_vpe_sync_free(void *ptr)
{
	if (ptr) {
		struct msm_queue_cmd *qcmd =
			(struct msm_queue_cmd *)ptr;
		qcmd--;
		if (atomic_read(&qcmd->on_heap))
			kfree(qcmd);
	}
}

/*
 * This function executes in interrupt context.
 */

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
		enum msm_queue qtype, void *syncdata,
		gfp_t gfp)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_sync *sync = (struct msm_sync *)syncdata;
	unsigned long flags;

	if (!sync) {
		pr_err("%s: no context in dsp callback.\n", __func__);
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;
	qcmd->command = vdata;

	ktime_get_ts(&(qcmd->ts));

	if (qtype != MSM_CAM_Q_VFE_MSG)
		goto vfe_for_config;

	CDBG("%s: vdata->type %d\n", __func__, vdata->type);

	switch (vdata->type) {
	case VFE_MSG_OUTPUT_P:
		if (sync->pp_mask & PP_PREV) {
			CDBG("%s: PP_PREV in progress: phy_y %x phy_cbcr %x\n",
				__func__,
				vdata->phy.y_phy,
				vdata->phy.cbcr_phy);
			spin_lock_irqsave(&pp_prev_spinlock, flags);
			if (sync->pp_prev)
				CDBG("%s: overwriting pp_prev!\n",
					__func__);
			CDBG("%s: sending preview to config\n", __func__);
			sync->pp_prev = qcmd;
			spin_unlock_irqrestore(&pp_prev_spinlock, flags);
			sync->pp_frame_avail = 1;
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
			break;
		}
		CDBG("%s: msm_enqueue frame_q\n", __func__);
		if (sync->frame_q.len <= 100 && sync->event_q.len <= 100) {
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
			msm_enqueue(&sync->frame_q, &qcmd->list_frame);
		} else {
			pr_err("%s, Error Queue limit exceeded f_q = %d, e_q = %d\n",
				__func__, sync->frame_q.len, sync->event_q.len);
			free_qcmd(qcmd);
			return;
		}

		break;

	case VFE_MSG_OUTPUT_T:
		if (sync->pp_mask & PP_SNAP) {
			spin_lock_irqsave(&pp_thumb_spinlock, flags);
			if (!sync->pp_thumb) {
				CDBG("%s: pp sending thumbnail to config\n",
					__func__);
				sync->pp_thumb = qcmd;
			}
			spin_unlock_irqrestore(&pp_thumb_spinlock, flags);
			break;
		} else {
		/* this is for normal snapshot case. right now we only have
		single shot. still keeping the old way. therefore no need
		to send anything to user.*/
			if (atomic_read(&qcmd->on_heap))
				free_qcmd(qcmd);
			return;
		}

	case VFE_MSG_OUTPUT_S:
		if (sync->pp_mask & PP_SNAP) {
			spin_lock_irqsave(&pp_snap_spinlock, flags);
			if (!sync->pp_snap) {
				CDBG("%s: pp sending main image to config\n",
					__func__);
				sync->pp_snap = qcmd;
				spin_unlock_irqrestore(&pp_snap_spinlock,
					flags);
				if (atomic_read(&qcmd->on_heap))
					atomic_add(1, &qcmd->on_heap);
			}
			spin_unlock_irqrestore(&pp_snap_spinlock, flags);
			break;
		} else {
		/* this is for normal snapshot case. right now we only have
		  single shot. still keeping the old way. therefore no need
		  to send anything to user.*/
			if (atomic_read(&qcmd->on_heap))
				free_qcmd(qcmd);
			return;
		}

	case VFE_MSG_OUTPUT_V:
		if (sync->vpefn.vpe_cfg_update) {
			CDBG("dis_en = %d\n", *sync->vpefn.dis);
			if (*(sync->vpefn.dis)) {
				memset(&(vdata->vpe_bf), 0,
					sizeof(vdata->vpe_bf));

				if (sync->cropinfo != NULL)
					vdata->vpe_bf.vpe_crop =
				*(struct video_crop_t *)(sync->cropinfo);

				vdata->vpe_bf.y_phy = vdata->phy.y_phy;
				vdata->vpe_bf.cbcr_phy = vdata->phy.cbcr_phy;
				vdata->vpe_bf.ts = (qcmd->ts);
				vdata->vpe_bf.frame_id = vdata->phy.frame_id;
				qcmd->command = vdata;
				msm_enqueue_vpe(&sync->vpe_q,
					&qcmd->list_vpe_frame);
				return;
			} else if (sync->vpefn.vpe_cfg_update(sync->cropinfo)) {
				CDBG("%s: msm_enqueue video frame to vpe time "
					"= %ld\n", __func__, qcmd->ts.tv_nsec);

				sync->vpefn.send_frame_to_vpe(
					vdata->phy.y_phy,
					vdata->phy.cbcr_phy,
					&(qcmd->ts));

				free_qcmd(qcmd);
				return;
			} else {
				CDBG("%s: msm_enqueue video frame_q\n",
					__func__);
				if (sync->liveshot_enabled) {
					CDBG("%s: msm_enqueue liveshot\n",
						__func__);
					vdata->phy.output_id |= OUTPUT_TYPE_L;
					sync->liveshot_enabled = false;
				}
				if (sync->frame_q.len <= 100 &&
					sync->event_q.len <= 100) {
						msm_enqueue(&sync->frame_q,
							&qcmd->list_frame);
				} else {
					pr_err("%s, Error Queue limit exceeded\
						f_q = %d, e_q = %d\n",
						__func__, sync->frame_q.len,
						sync->event_q.len);
					free_qcmd(qcmd);
				}

				return;
			}
		} else {
			CDBG("%s: msm_enqueue video frame_q\n",	__func__);
			if (sync->frame_q.len <= 100 &&
				sync->event_q.len <= 100) {
				msm_enqueue(&sync->frame_q, &qcmd->list_frame);
			} else {
				pr_err("%s, Error Queue limit exceeded\
					f_q = %d, e_q = %d\n",
					__func__, sync->frame_q.len,
					sync->event_q.len);
				free_qcmd(qcmd);
			}

			return;
		}

	case VFE_MSG_SNAPSHOT:
		if (sync->pp_mask & (PP_SNAP | PP_RAW_SNAP)) {
			CDBG("%s: PP_SNAP in progress: pp_mask %x\n",
				__func__, sync->pp_mask);
			spin_lock_irqsave(&pp_snap_spinlock, flags);
			if (sync->pp_snap)
				pr_warning("%s: overwriting pp_snap!\n",
					__func__);
			CDBG("%s: sending snapshot to config\n",
				__func__);
			spin_unlock_irqrestore(&pp_snap_spinlock, flags);
		} else {
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
			msm_enqueue(&sync->pict_q, &qcmd->list_pict);
		}
		break;

	case VFE_MSG_STATS_AWB:
		CDBG("%s: qtype %d, AWB stats, enqueue event_q.\n",
			__func__, vdata->type);
		break;

	case VFE_MSG_STATS_AEC:
		CDBG("%s: qtype %d, AEC stats, enqueue event_q.\n",
			__func__, vdata->type);
		break;

	case VFE_MSG_STATS_IHIST:
		CDBG("%s: qtype %d, ihist stats, enqueue event_q.\n",
			__func__, vdata->type);
		break;

	case VFE_MSG_STATS_RS:
		CDBG("%s: qtype %d, rs stats, enqueue event_q.\n",
			__func__, vdata->type);
		break;

	case VFE_MSG_STATS_CS:
		CDBG("%s: qtype %d, cs stats, enqueue event_q.\n",
			__func__, vdata->type);
		break;


	case VFE_MSG_GENERAL:
		CDBG("%s: qtype %d, general msg, enqueue event_q.\n",
			__func__, vdata->type);
		break;

	default:
		CDBG("%s: qtype %d not handled\n", __func__, vdata->type);
		/* fall through, send to config. */
	}

vfe_for_config:
	CDBG("%s: msm_enqueue event_q\n", __func__);
	if (sync->frame_q.len <= 100 && sync->event_q.len <= 100) {
		msm_enqueue(&sync->event_q, &qcmd->list_config);
	} else {
		pr_err("%s, Error Queue limit exceeded f_q = %d, e_q = %d\n",
			__func__, sync->frame_q.len, sync->event_q.len);
		free_qcmd(qcmd);
	}

}

static void msm_vpe_sync(struct msm_vpe_resp *vdata,
						enum msm_queue qtype,
						void *syncdata,
						void *ts, gfp_t gfp)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_sync *sync = (struct msm_sync *)syncdata;
	if (!sync) {
		pr_err("%s: no context in dsp callback.\n", __func__);
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;
	qcmd->command = vdata;
	qcmd->ts = *((struct timespec *)ts);

	if (qtype != MSM_CAM_Q_VPE_MSG) {
		pr_err("%s: Invalid qcmd type = %d.\n", __func__, qcmd->type);
		free_qcmd(qcmd);
		return;
	}

	CDBG("%s: vdata->type %d\n", __func__, vdata->type);
	switch (vdata->type) {
	case VFE_MSG_OUTPUT_V:
		CDBG("%s: msm_enqueue video frame_q from VPE\n", __func__);
		if (sync->liveshot_enabled) {
			CDBG("%s: msm_enqueue liveshot %d\n", __func__,
				sync->liveshot_enabled);
			vdata->phy.output_id |= OUTPUT_TYPE_L;
			sync->liveshot_enabled = false;
		}

		if (sync->frame_q.len <= 100 && sync->event_q.len <= 100) {
			msm_enqueue(&sync->frame_q, &qcmd->list_frame);
		} else {
			pr_err("%s, Error Queue limit exceeded f_q = %d, e_q = %d\n",
				__func__, sync->frame_q.len, sync->event_q.len);
			free_qcmd(qcmd);
		}

		return;
	default:
		CDBG("%s: qtype %d not handled\n", __func__, vdata->type);
		/* fall through, send to config. */
	}
	CDBG("%s: msm_enqueue event_q\n", __func__);
	if (sync->frame_q.len <= 100 && sync->event_q.len <= 100) {
		msm_enqueue(&sync->event_q, &qcmd->list_config);
	} else {
		pr_err("%s, Error Queue limit exceeded f_q = %d, e_q = %d\n",
			__func__, sync->frame_q.len, sync->event_q.len);
		free_qcmd(qcmd);
	}

}

static struct msm_vpe_callback msm_vpe_s = {
	.vpe_resp = msm_vpe_sync,
	.vpe_alloc = msm_vpe_sync_alloc,
	.vpe_free = msm_vpe_sync_free,
};

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
	.vfe_alloc = msm_vfe_sync_alloc,
	.vfe_free = msm_vfe_sync_free,
};

static int __msm_open(struct msm_sync *sync, const char *const apps_id,
			int is_controlnode)
{
	int rc = 0;

	mutex_lock(&sync->lock);
	if (sync->apps_id && strcmp(sync->apps_id, apps_id)
				&& (!strcmp(MSM_APPS_ID_V4L2, apps_id))) {
		pr_err("%s(%s): sensor %s is already opened for %s\n",
			__func__,
			apps_id,
			sync->sdata->sensor_name,
			sync->apps_id);
		rc = -EBUSY;
		goto msm_open_done;
	}

	sync->apps_id = apps_id;

	if (!sync->core_powered_on && !is_controlnode) {
		wake_lock(&sync->wake_lock);

		msm_camvfe_fn_init(&sync->vfefn, sync);
		if (sync->vfefn.vfe_init) {
			sync->pp_frame_avail = 0;
			sync->get_pic_abort = 0;
			rc = msm_camio_sensor_clk_on(sync->pdev);
			if (rc < 0) {
				pr_err("%s: setting sensor clocks failed: %d\n",
					__func__, rc);
				goto msm_open_done;
			}
			rc = sync->sctrl.s_init(sync->sdata);
			if (rc < 0) {
				pr_err("%s: sensor init failed: %d\n",
					__func__, rc);
				goto msm_open_done;
			}
			rc = sync->vfefn.vfe_init(&msm_vfe_s,
				sync->pdev);
			if (rc < 0) {
				pr_err("%s: vfe_init failed at %d\n",
					__func__, rc);
				goto msm_open_done;
			}
		} else {
			pr_err("%s: no sensor init func\n", __func__);
			rc = -ENODEV;
			goto msm_open_done;
		}
		msm_camvpe_fn_init(&sync->vpefn, sync);

		spin_lock_init(&sync->abort_pict_lock);
		if (rc >= 0) {
			msm_region_init(sync);
			if (sync->vpefn.vpe_reg)
				sync->vpefn.vpe_reg(&msm_vpe_s);
			sync->unblock_poll_frame = 0;
		}
		sync->core_powered_on = 1;
	}
	sync->opencnt++;

msm_open_done:
	mutex_unlock(&sync->lock);
	return rc;
}

static int msm_open_common(struct inode *inode, struct file *filep,
			int once, int is_controlnode)
{
	int rc;
	struct msm_cam_device *pmsm =
		container_of(inode->i_cdev, struct msm_cam_device, cdev);

	CDBG("%s: open %s\n", __func__, filep->f_path.dentry->d_name.name);

	if (atomic_cmpxchg(&pmsm->opened, 0, 1) && once) {
		pr_err("%s: %s is already opened.\n",
			__func__,
			filep->f_path.dentry->d_name.name);
		return -EBUSY;
	}

	rc = nonseekable_open(inode, filep);
	if (rc < 0) {
		pr_err("%s: nonseekable_open error %d\n", __func__, rc);
		return rc;
	}

	rc = __msm_open(pmsm->sync, MSM_APPS_ID_PROP, is_controlnode);
	if (rc < 0)
		return rc;
	filep->private_data = pmsm;
	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static int msm_open(struct inode *inode, struct file *filep)
{
	return msm_open_common(inode, filep, 1, 0);
}

static int msm_open_control(struct inode *inode, struct file *filep)
{
	int rc;

	struct msm_control_device *ctrl_pmsm =
		kmalloc(sizeof(struct msm_control_device), GFP_KERNEL);
	if (!ctrl_pmsm)
		return -ENOMEM;

	rc = msm_open_common(inode, filep, 0, 1);
	if (rc < 0) {
		kfree(ctrl_pmsm);
		return rc;
	}
	ctrl_pmsm->pmsm = filep->private_data;
	filep->private_data = ctrl_pmsm;

	msm_queue_init(&ctrl_pmsm->ctrl_q, "control");

	if (!g_v4l2_opencnt)
		g_v4l2_control_device = ctrl_pmsm;
	g_v4l2_opencnt++;
	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static int __msm_v4l2_control(struct msm_sync *sync,
		struct msm_ctrl_cmd *out)
{
	int rc = 0;

	struct msm_queue_cmd *qcmd = NULL, *rcmd = NULL;
	struct msm_ctrl_cmd *ctrl;
	struct msm_device_queue *v4l2_ctrl_q = &g_v4l2_control_device->ctrl_q;

	/* wake up config thread, 4 is for V4L2 application */
	qcmd = kmalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!qcmd) {
		pr_err("%s: cannot allocate buffer\n", __func__);
		rc = -ENOMEM;
		goto end;
	}
	qcmd->type = MSM_CAM_Q_V4L2_REQ;
	qcmd->command = out;
	atomic_set(&qcmd->on_heap, 1);

	if (out->type == V4L2_CAMERA_EXIT) {
		rcmd = __msm_control(sync, NULL, qcmd, out->timeout_ms);
		if (rcmd == NULL) {
			rc = PTR_ERR(rcmd);
			goto end;
		}
	}

	rcmd = __msm_control(sync, v4l2_ctrl_q, qcmd, out->timeout_ms);

	if (IS_ERR(rcmd)) {
		rc = PTR_ERR(rcmd);
		goto end;
	}

	ctrl = (struct msm_ctrl_cmd *)(rcmd->command);
	/* FIXME: we should just set out->length = ctrl->length; */
	BUG_ON(out->length < ctrl->length);
	memcpy(out->value, ctrl->value, ctrl->length);

end:
	free_qcmd(rcmd);
	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static const struct file_operations msm_fops_config = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl_config,
	.release = msm_release_config,
};

static const struct file_operations msm_fops_control = {
	.owner = THIS_MODULE,
	.open = msm_open_control,
	.unlocked_ioctl = msm_ioctl_control,
	.release = msm_release_control,
};

static const struct file_operations msm_fops_frame = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl_frame,
	.release = msm_release_frame,
	.poll = msm_poll_frame,
};

static int msm_setup_cdev(struct msm_cam_device *msm,
			int node,
			dev_t devno,
			const char *suffix,
			const struct file_operations *fops)
{
	int rc = -ENODEV;

	struct device *device =
		device_create(msm_class, NULL,
			devno, NULL,
			"%s%d", suffix, node);

	if (IS_ERR(device)) {
		rc = PTR_ERR(device);
		pr_err("%s: error creating device: %d\n", __func__, rc);
		return rc;
	}

	cdev_init(&msm->cdev, fops);
	msm->cdev.owner = THIS_MODULE;

	rc = cdev_add(&msm->cdev, devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev: %d\n", __func__, rc);
		device_destroy(msm_class, devno);
		return rc;
	}

	return rc;
}

static int msm_tear_down_cdev(struct msm_cam_device *msm, dev_t devno)
{
	cdev_del(&msm->cdev);
	device_destroy(msm_class, devno);
	return 0;
}

int msm_v4l2_register(struct msm_v4l2_driver *drv)
{
	/* FIXME: support multiple sensors */
	if (list_empty(&msm_sensors))
		return -ENODEV;

	drv->sync = list_first_entry(&msm_sensors, struct msm_sync, list);
	drv->open      = __msm_open;
	drv->release   = __msm_release;
	drv->ctrl      = __msm_v4l2_control;
	drv->reg_pmem  = __msm_register_pmem;
	drv->get_frame = __msm_get_frame;
	drv->put_frame = __msm_put_frame_buf;
	drv->get_pict  = __msm_get_pic;
	drv->drv_poll  = __msm_poll_frame;

	return 0;
}
EXPORT_SYMBOL(msm_v4l2_register);

int msm_v4l2_unregister(struct msm_v4l2_driver *drv)
{
	drv->sync = NULL;
	return 0;
}
EXPORT_SYMBOL(msm_v4l2_unregister);

static int msm_sync_init(struct msm_sync *sync,
		struct platform_device *pdev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
				struct msm_sensor_ctrl *))
{
	int rc = 0;
	struct msm_sensor_ctrl sctrl;
	sync->sdata = pdev->dev.platform_data;

	msm_queue_init(&sync->event_q, "event");
	msm_queue_init(&sync->frame_q, "frame");
	msm_queue_init(&sync->pict_q, "pict");
	msm_queue_init(&sync->vpe_q, "vpe");

	wake_lock_init(&sync->wake_lock, WAKE_LOCK_IDLE, "msm_camera");

	rc = msm_camio_probe_on(pdev);
	if (rc < 0) {
		wake_lock_destroy(&sync->wake_lock);
		return rc;
	}
	rc = sensor_probe(sync->sdata, &sctrl);
	if (rc >= 0) {
		sync->pdev = pdev;
		sync->sctrl = sctrl;
	}
	msm_camio_probe_off(pdev);
	if (rc < 0) {
		pr_err("%s: failed to initialize %s\n",
			__func__,
			sync->sdata->sensor_name);
		wake_lock_destroy(&sync->wake_lock);
		return rc;
	}

	sync->opencnt = 0;
	sync->core_powered_on = 0;
	sync->ignore_qcmd = false;
	sync->ignore_qcmd_type = -1;
	mutex_init(&sync->lock);
	if (sync->sdata->strobe_flash_data) {
		sync->sdata->strobe_flash_data->state = 0;
		spin_lock_init(&sync->sdata->strobe_flash_data->spin_lock);
	}
	pr_info("%s: initialized %s\n", __func__, sync->sdata->sensor_name);
	return rc;
}

static int msm_sync_destroy(struct msm_sync *sync)
{
	wake_lock_destroy(&sync->wake_lock);
	return 0;
}

static int msm_device_init(struct msm_cam_device *pmsm,
		struct msm_sync *sync,
		int node)
{
	int dev_num = 3 * node;
	int rc = msm_setup_cdev(pmsm, node,
		MKDEV(MAJOR(msm_devno), dev_num),
		"control", &msm_fops_control);
	if (rc < 0) {
		pr_err("%s: error creating control node: %d\n", __func__, rc);
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 1, node,
		MKDEV(MAJOR(msm_devno), dev_num + 1),
		"config", &msm_fops_config);
	if (rc < 0) {
		pr_err("%s: error creating config node: %d\n", __func__, rc);
		msm_tear_down_cdev(pmsm, MKDEV(MAJOR(msm_devno),
				dev_num));
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 2, node,
		MKDEV(MAJOR(msm_devno), dev_num + 2),
		"frame", &msm_fops_frame);
	if (rc < 0) {
		pr_err("%s: error creating frame node: %d\n", __func__, rc);
		msm_tear_down_cdev(pmsm,
			MKDEV(MAJOR(msm_devno), dev_num));
		msm_tear_down_cdev(pmsm + 1,
			MKDEV(MAJOR(msm_devno), dev_num + 1));
		return rc;
	}

	atomic_set(&pmsm[0].opened, 0);
	atomic_set(&pmsm[1].opened, 0);
	atomic_set(&pmsm[2].opened, 0);

	pmsm[0].sync = sync;
	pmsm[1].sync = sync;
	pmsm[2].sync = sync;

	return rc;
}

int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
			struct msm_sensor_ctrl *))
{
	struct msm_cam_device *pmsm = NULL;
	struct msm_sync *sync;
	int rc = -ENODEV;

	if (camera_node >= MSM_MAX_CAMERA_SENSORS) {
		pr_err("%s: too many camera sensors\n", __func__);
		return rc;
	}

	if (!msm_class) {
		/* There are three device nodes per sensor */
		rc = alloc_chrdev_region(&msm_devno, 0,
				3 * MSM_MAX_CAMERA_SENSORS,
				"msm_camera");
		if (rc < 0) {
			pr_err("%s: failed to allocate chrdev: %d\n", __func__,
				rc);
			return rc;
		}

		msm_class = class_create(THIS_MODULE, "msm_camera");
		if (IS_ERR(msm_class)) {
			rc = PTR_ERR(msm_class);
			pr_err("%s: create device class failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

	pmsm = kzalloc(sizeof(struct msm_cam_device) * 3 +
			sizeof(struct msm_sync), GFP_ATOMIC);
	if (!pmsm)
		return -ENOMEM;
	sync = (struct msm_sync *)(pmsm + 3);

	rc = msm_sync_init(sync, dev, sensor_probe);
	if (rc < 0) {
		kfree(pmsm);
		return rc;
	}

	CDBG("%s: setting camera node %d\n", __func__, camera_node);
	rc = msm_device_init(pmsm, sync, camera_node);
	if (rc < 0) {
		msm_sync_destroy(sync);
		kfree(pmsm);
		return rc;
	}

	camera_type[camera_node] = sync->sctrl.s_camera_type;
	sensor_mount_angle[camera_node] = sync->sctrl.s_mount_angle;
	camera_node++;

	list_add(&sync->list, &msm_sensors);
	return rc;
}
EXPORT_SYMBOL(msm_camera_drv_start);
