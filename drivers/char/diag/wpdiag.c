/* drivers/misc//wpdiag.c
 *
 * Device node of modem diag service
 *
 * Copyright (C) 2009 Richard Rau
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/slab.h>

#include <mach/dbgcfgtool.h>

#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>  //SW-2-5-1-MP-DbgCfgTool-03+

//SW-2-5-1-MP-DbgCfgTool-03+[
#define  APPS_MODE_ANDROID   0x1
#define  APPS_MODE_RECOVERY  0x2
#define  APPS_MODE_FTM       0x3
#define  APPS_MODE_UNKNOWN   0xFF
//SW-2-5-1-MP-DbgCfgTool-03+]

#if 1
#define TRACE(tag,data,len,decode) do {} while(0)
#else
static void TRACE(const char *tag, const void *_data, int len, int decode)
{
	const unsigned char *data = _data;
	int escape = 0;

	printk(KERN_INFO "wp_diag: %s", tag);
	if (decode) {
		while (len-- > 0) {
			unsigned x = *data++;
			if (x == 0x7e) {
				printk(" $$");
				escape = 0;
				continue;
			}
			if (x == 0x7d) {
				escape = 1;
				continue;
			}
			if (escape) {
				escape = 0;
				printk(" %02x", x ^ 0x20);
			} else {
				printk(" %02x", x);
			}
		}
	} else {
		while (len-- > 0) {
			printk(" %02x", *data++);
		}
		printk(" $$");
	}
	printk("\n");
}
#endif

//#define DYNAMIC_ALLOC

#define HDLC_MAX		512 // Only used in TX
#define RX_MAX			8192

#ifdef DYNAMIC_ALLOC
#define READQ_MAX		40
#else //DYNAMIC_ALLOC
#define RX_SMALL_MAX		256
#define RX_MEDIUM_MAX		4352
#define RX_LARGE_MAX		RX_MAX

#define FREE_SMALL_MAX		5
#define FREE_MEDIUM_MAX		40
#define FREE_LARGE_MAX		2
#endif // DYNAMIC_ALLOC


struct diag_packet
{
	struct list_head list;
	unsigned int len;
	unsigned char *content;
#ifndef DYNAMIC_ALLOC
	unsigned char type;
#endif	
};

struct wp_diag_context
{
	smd_channel_t *ch;

#ifdef DYNAMIC_ALLOC
	struct list_head read_q;
#else
	struct list_head s_free_q;
	unsigned char s_free_q_count;
	struct list_head m_free_q;
	unsigned char m_free_q_count;
	struct list_head l_free_q;
	unsigned char l_free_q_count;

	struct list_head used_q;
	unsigned char _discard_buf[RX_LARGE_MAX];
#endif

	unsigned char read_q_count;
	spinlock_t read_q_lock;

	wait_queue_head_t wait_q;

	/* assembly buffer for A11->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned char send_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;

	unsigned configured;
};

int wp_diag_major;
static struct wp_diag_context _wp_context;

static void wp_diag_process_hdlc(struct wp_diag_context *ctxt, void *_data, unsigned len)
{
	unsigned char *data = _data;
	unsigned count = ctxt->hdlc_count;
	unsigned escape = ctxt->hdlc_escape;
	unsigned char *hdlc = ctxt->hdlc_buf;

	while (len-- > 0) {
		unsigned char x = *data++;
		if (x == 0x7E) {
			if (count > 2) {
				/* we're just ignoring the crc here */
				TRACE("A11>", hdlc, count - 2, 0);
				if (ctxt->ch)
					smd_write(ctxt->ch, hdlc, count - 2);
			}
			count = 0;
			escape = 0;
		} else if (x == 0x7D) {
			escape = 1;
		} else {
			if (escape) {
				x = x ^ 0x20;
				escape = 0;
			}
			hdlc[count++] = x;

			/* discard frame if we overflow */
			if (count == HDLC_MAX)
				count = 0;
		}
	}

	ctxt->hdlc_count = count;
	ctxt->hdlc_escape = escape;
}

static void wp_smd_diag_notify(void *priv, unsigned event)
{
	struct wp_diag_context *ctxt = priv;
	struct diag_packet *dp;
	unsigned long flags;
#ifdef DYNAMIC_ALLOC
	unsigned char *discard_buf;
#endif

	//printk(KERN_INFO "wp_diag: %s, event %d\n", __FUNCTION__, event);

	if (ctxt->ch && event == SMD_EVENT_DATA)
	{
		int r = smd_read_avail(ctxt->ch);
		//printk(KERN_INFO "wp_diag: smd_read_avail num bytes = %d\n", r);
		if (r > RX_MAX || r <= 0)	{
			printk(KERN_ERR "wp_diag: diag dropped num bytes = %d or invaild num to read\n", r);
			return;
		}

		spin_lock_irqsave(&ctxt->read_q_lock, flags);
#ifdef DYNAMIC_ALLOC
		if (_wp_context.read_q_count < READQ_MAX) {
			dp = kmalloc(sizeof(struct diag_packet), GFP_ATOMIC);
			if (!dp) {
				printk(KERN_ERR "wp_diag: kmalloc dp failed\n");
				spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
				return;
			}

			dp->content = kmalloc(r, GFP_ATOMIC);
			if (!dp->content) {
				kfree(dp);
				printk(KERN_ERR "wp_diag: kmalloc dp->content failed\n");
				spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
				return;
			}

			dp->len = smd_read_from_cb(ctxt->ch, dp->content, r);
			//printk(KERN_INFO "wp_diag: smd_read_from_cb %d\n", dp->len);

			list_add_tail(&dp->list, &ctxt->read_q);
			_wp_context.read_q_count++;
			spin_unlock_irqrestore(&ctxt->read_q_lock, flags);

			TRACE("A9>", dp->content, dp->len, 0);
			//printk(KERN_ERR "wp_diag: put readq count %d\n", _wp_context.read_q_count);
		} else {
			printk(KERN_ERR "wp_diag: exceed max num in read q: %d\n", _wp_context.read_q_count);

			discard_buf = kmalloc(r, GFP_ATOMIC);
			smd_read_from_cb(ctxt->ch, discard_buf, r);
			kfree(discard_buf);
			spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
			return;
		}
#else
		// note if small buffer run out, we can use larger buffer
		if (r <= RX_SMALL_MAX && ctxt->s_free_q_count > 0) {
			dp = list_first_entry(&ctxt->s_free_q, struct diag_packet, list);
			list_del(&dp->list);
			ctxt->s_free_q_count--;
		} else if (r <= RX_MEDIUM_MAX && ctxt->m_free_q_count > 0) {
			dp = list_first_entry(&ctxt->m_free_q, struct diag_packet, list);
			list_del(&dp->list);
			ctxt->m_free_q_count--;
		} else if (r <= RX_LARGE_MAX && ctxt->l_free_q_count > 0) {
			dp = list_first_entry(&ctxt->l_free_q, struct diag_packet, list);
			list_del(&dp->list);
			ctxt->l_free_q_count--;
		} else {
			printk(KERN_ERR "wp_diag: no free space, r %d, s %d, m %d, l %d, rq %d\n", r, ctxt->s_free_q_count,
				 ctxt->m_free_q_count, ctxt->l_free_q_count, ctxt->read_q_count);

			smd_read_from_cb(ctxt->ch, _wp_context._discard_buf, r);

			spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
			return;
		}

		dp->len = smd_read_from_cb(ctxt->ch, dp->content, r);
		//printk(KERN_INFO "wp_diag: smd_read_from_cb %d\n", dp->len);
		list_add_tail(&dp->list, &ctxt->used_q);
		_wp_context.read_q_count++;
		spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
#endif
		wake_up_interruptible(&ctxt->wait_q);
	}
}

static ssize_t wp_diag_device_op_read(struct file *filp, char __user *buff,
	size_t count, loff_t *offp)
{
	ssize_t ret;
	struct wp_diag_context *ctxt = &_wp_context;
	struct diag_packet *dp;
	unsigned long flags;
	//printk(KERN_INFO "wp_diag: %s\n", __FUNCTION__);

#ifdef DYNAMIC_ALLOC
	if (wait_event_interruptible(ctxt->wait_q, !list_empty(&ctxt->read_q)))
		return -ERESTARTSYS;

	spin_lock_irqsave(&ctxt->read_q_lock, flags);
	if (list_empty(&ctxt->read_q)) {
		spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
		return -EAGAIN;
	}
	dp = list_first_entry(&ctxt->read_q, struct diag_packet, list);
	if (dp->len > count) {
		printk(KERN_ERR "wp_diag: buffer to small, discard! count %d, pk_len %d\n",
			count, dp->len);
		list_del(&dp->list);
		_wp_context.read_q_count--;
		spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
		kfree(dp->content);
		kfree(dp);
		return -ETOOSMALL;
	}
	list_del(&dp->list);
	_wp_context.read_q_count--;
	spin_unlock_irqrestore(&ctxt->read_q_lock, flags);

	if (copy_to_user(buff, dp->content, dp->len)) {
		printk(KERN_ERR "wp_diag: copy_to_user return non-zero\n");
		kfree(dp->content);
		kfree(dp);
		return -EFAULT;
	}
	ret = dp->len;
	kfree(dp->content);
	kfree(dp);
#else
	if (wait_event_interruptible(ctxt->wait_q, !list_empty(&ctxt->used_q)))
		return -ERESTARTSYS;

	spin_lock_irqsave(&ctxt->read_q_lock, flags);
	if (list_empty(&ctxt->used_q)) {
		spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
		return -EAGAIN;
	}
	dp = list_first_entry(&ctxt->used_q, struct diag_packet, list);
	if (dp->len > count) {
		printk(KERN_ERR "wp_diag: buffer to small, discard! count %d, pk_len %d\n",
			count, dp->len);
		switch (dp->type)
		{
		case 0:
			list_move_tail(&dp->list, &ctxt->s_free_q);
			ctxt->s_free_q_count++;
			break;
		case 1:
			list_move_tail(&dp->list, &ctxt->m_free_q);
			ctxt->m_free_q_count++;
			break;
		case 2:
			list_move_tail(&dp->list, &ctxt->l_free_q);
			ctxt->l_free_q_count++;
			break;
		}
		_wp_context.read_q_count--;

		spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
		return -ETOOSMALL;
	}

	list_del(&dp->list);
	_wp_context.read_q_count--;
	spin_unlock_irqrestore(&ctxt->read_q_lock, flags);

	if (copy_to_user(buff, dp->content, dp->len)) {
		printk(KERN_ERR "wp_diag: copy_to_user return non-zero\n");
		ret = -EFAULT;
	} else {
		ret = dp->len;
	}

	spin_lock_irqsave(&ctxt->read_q_lock, flags);
	// give buffer back
	switch (dp->type)
	{
	case 0:
		list_add_tail(&dp->list, &ctxt->s_free_q);
		ctxt->s_free_q_count++;
		break;
	case 1:
		list_add_tail(&dp->list, &ctxt->m_free_q);
		ctxt->m_free_q_count++;
		break;
	case 2:
		list_add_tail(&dp->list, &ctxt->l_free_q);
		ctxt->l_free_q_count++;
		break;
	}
	spin_unlock_irqrestore(&ctxt->read_q_lock, flags);
#endif
	//printk(KERN_INFO "wp_diag: readq count in read %d\n", _wp_context.read_q_count);

	return ret;
}

static ssize_t wp_diag_device_op_write (struct file *filp, const char __user *buff,
	size_t count, loff_t *offp)
{
	struct wp_diag_context *ctxt = &_wp_context;

	//printk(KERN_INFO "wp_diag: %s, count %d\n", __FUNCTION__, count);

	if (count > HDLC_MAX) {
		printk(KERN_ERR "wp_diag: write size over HDLC_MAX\n");
		return -ENOMEM;
	}

	if (copy_from_user(ctxt->send_buf, buff, count)) {
		printk(KERN_ERR "wp_diag: copy_from_user return non-zero\n");
		return -EFAULT;
	}

	wp_diag_process_hdlc(ctxt, ctxt->send_buf, count);

	//printk(KERN_INFO "wp_diag: %s leaving\n", __FUNCTION__);

	return count;
}

static unsigned int wp_diag_device_op_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct wp_diag_context *ctxt = &_wp_context;
	unsigned int mask = 0;
	unsigned long flags;

	//printk(KERN_INFO "wp_diag: %s\n", __FUNCTION__);

	poll_wait(filp, &ctxt->wait_q, wait);

	spin_lock_irqsave(&ctxt->read_q_lock, flags);
#ifdef DYNAMIC_ALLOC
	if (!list_empty(&ctxt->read_q))
#else
	if (!list_empty(&ctxt->used_q))
#endif
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&ctxt->read_q_lock, flags);

	mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static int wp_diag_device_op_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	//printk(KERN_INFO "wp_diag: %s\n", __FUNCTION__);
	return 0;
}

#ifndef DYNAMIC_ALLOC
static void clean_up(void)
{
	struct wp_diag_context *ctxt = &_wp_context;
	struct diag_packet *dp;

	while (!list_empty(&ctxt->used_q)) {
		dp = list_first_entry(&ctxt->used_q, struct diag_packet, list);
		kfree(dp->content);
		kfree(dp);
		list_del(&dp->list);
	}

	while (!list_empty(&ctxt->s_free_q)) {
		dp = list_first_entry(&ctxt->s_free_q, struct diag_packet, list);
		kfree(dp->content);
		kfree(dp);
		list_del(&dp->list);
	}

	while (!list_empty(&ctxt->m_free_q)) {
		dp = list_first_entry(&ctxt->m_free_q, struct diag_packet, list);
		kfree(dp->content);
		kfree(dp);
		list_del(&dp->list);
	}
	while (!list_empty(&ctxt->l_free_q)) {
		dp = list_first_entry(&ctxt->l_free_q, struct diag_packet, list);
		kfree(dp->content);
		kfree(dp);
		list_del(&dp->list);
	}

}
#endif

static int wp_diag_device_op_open(struct inode *inode, struct file *filp)
{
	int rc;

	int cfg_val;
//SW2-5-1-MP-DbgCfgTool-03*[
	int boot_mode;

	struct wp_diag_context *ctxt = &_wp_context;
#ifndef DYNAMIC_ALLOC
	struct diag_packet *dp;
	int i;
#endif

	rc = nonseekable_open(inode, filp);
	if (rc < 0)
		return rc;

	boot_mode = fih_read_boot_mode_from_smem();

	if (boot_mode != APPS_MODE_FTM) {
		rc = DbgCfgGetByBit(DEBUG_MODEM_LOGGER_CFG, (int*)&cfg_val);
		if ((rc == 0) && (cfg_val == 1) && (boot_mode != APPS_MODE_RECOVERY)) {
#ifdef DYNAMIC_ALLOC
			printk(KERN_INFO "wp_diag: embedded QXDM enabled, %s, dynamic allocate." , __FUNCTION__);
			INIT_LIST_HEAD(&_wp_context.read_q);
#else
			printk(KERN_INFO "wp_diag: embedded QXDM enabled, %s, static allocate." , __FUNCTION__);
			INIT_LIST_HEAD(&_wp_context.s_free_q);
			INIT_LIST_HEAD(&_wp_context.m_free_q);
			INIT_LIST_HEAD(&_wp_context.l_free_q);
			INIT_LIST_HEAD(&_wp_context.used_q);
#endif
			spin_lock_init(&_wp_context.read_q_lock);
			init_waitqueue_head(&_wp_context.wait_q);
			_wp_context.read_q_count = 0;

#ifndef DYNAMIC_ALLOC
			for (i = 0;i < FREE_SMALL_MAX;i++) {
				dp = kmalloc(sizeof(struct diag_packet), GFP_ATOMIC);
				if (!dp) {
					printk(KERN_ERR "wp_diag: open kmalloc dp failed\n");
					clean_up();
					return -ENOMEM;
				}
				dp->type = 0;
				dp->content = kmalloc(RX_SMALL_MAX, GFP_ATOMIC);
				if (!dp->content) {
					kfree(dp);
					printk(KERN_ERR "wp_diag: open kmalloc dp->content failed\n");
					clean_up();
					return -ENOMEM;
				}
				list_add_tail(&dp->list, &ctxt->s_free_q);
			}
			ctxt->s_free_q_count = FREE_SMALL_MAX;
			for (i = 0;i < FREE_MEDIUM_MAX;i++) {
				dp = kmalloc(sizeof(struct diag_packet), GFP_ATOMIC);
				if (!dp) {
					printk(KERN_ERR "wp_diag: open kmalloc dp failed\n");
					clean_up();
					return -ENOMEM;
				}
				dp->type = 1;
				dp->content = kmalloc(RX_MEDIUM_MAX, GFP_ATOMIC);
				if (!dp->content) {
					kfree(dp);
					printk(KERN_ERR "wp_diag: open kmalloc dp->content failed\n");
					clean_up();
					return -ENOMEM;
				}
				list_add_tail(&dp->list, &ctxt->m_free_q);
			}
			ctxt->m_free_q_count = FREE_MEDIUM_MAX;
			for (i = 0;i < FREE_LARGE_MAX;i++) {
				dp = kmalloc(sizeof(struct diag_packet), GFP_ATOMIC);
				if (!dp) {
					printk(KERN_ERR "wp_diag: open kmalloc dp failed\n");
					clean_up();
					return -ENOMEM;
				}
				dp->type = 2;
				dp->content = kmalloc(RX_LARGE_MAX, GFP_ATOMIC);
				if (!dp->content) {
					kfree(dp);
					printk(KERN_ERR "wp_diag: open kmalloc dp->content failed\n");
					clean_up();
					return -ENOMEM;
				}
				list_add_tail(&dp->list, &ctxt->l_free_q);
			}
			ctxt->l_free_q_count = FREE_LARGE_MAX;
			printk(KERN_INFO "wp_diag: static alloc %s, s: %d, m: %d, l: %d", __FUNCTION__,
				ctxt->s_free_q_count, ctxt->m_free_q_count, ctxt->l_free_q_count);
#endif

			rc = smd_open("DIAG", &ctxt->ch, ctxt, wp_smd_diag_notify);
			_wp_context.configured = 1;
			return 0;
		} else {
			printk(KERN_INFO "wp_diag: embedded QxDM Disabled. %s", __FUNCTION__);
			return -EFAULT;
		}
	} else {
		printk(KERN_INFO "wp_diag: embedded QXDM Disabled due to FTM image. %s", __FUNCTION__);
		return -EFAULT;
	}
//SW2-5-1-MP-DbgCfgTool-03*]
}

static int wp_diag_device_op_release(struct inode *inode, struct file *filp)
{
	//printk(KERN_INFO "wp_diag: %s, i_rdev %d\n", __FUNCTION__, inode->i_rdev);
	if (_wp_context.configured) {
		smd_close(_wp_context.ch);
#ifdef DYNAMIC_ALLOC
		list_del(&_wp_context.read_q);
#else
		clean_up();
		list_del(&_wp_context.s_free_q);
		list_del(&_wp_context.m_free_q);
		list_del(&_wp_context.l_free_q);
		list_del(&_wp_context.used_q);
		_wp_context.read_q_count = 0;
#endif
		_wp_context.configured = 0;
	}
	return 0;
}

const struct file_operations wp_diag_device_ops = {
	.owner		= THIS_MODULE,
	.read		= wp_diag_device_op_read,
	.write		= wp_diag_device_op_write,
	.poll		= wp_diag_device_op_poll,
	.ioctl		= wp_diag_device_op_ioctl,
	.open		= wp_diag_device_op_open,
	.release	= wp_diag_device_op_release,
};

struct class *wp_diag_class;
dev_t wp_diag_devno;

static struct cdev wp_diag_cdev;
static struct device *wp_diag_device;

static int __init wp_diag_init(void)
{
	struct wp_diag_context *ctxt;
	int rc;

	printk(KERN_INFO "wp_diag: %s\n", __FUNCTION__);

	ctxt = &_wp_context;
	/* Create the device nodes */
	wp_diag_class = class_create(THIS_MODULE, "wp_diag");
	if (IS_ERR(wp_diag_class)) {
		rc = -ENOMEM;
		printk(KERN_ERR "wp_diag: failed to create wp_diag class\n");
		goto fail;
	}

	rc = alloc_chrdev_region(&wp_diag_devno, 0, 1, "wp_diag");
	if (rc < 0) {
		printk(KERN_ERR "wp_diag: Failed to alloc chardev region (%d)\n", rc);
		goto fail_destroy_class;
	}

	printk(KERN_INFO "wp_diag: got major %d\n", MAJOR(wp_diag_devno));

	wp_diag_device = device_create(wp_diag_class, NULL, wp_diag_devno, NULL, "wp_diag");
	if (IS_ERR(wp_diag_device)) {
		rc = -ENOMEM;
		goto fail_unregister_cdev_region;
	}

	cdev_init(&wp_diag_cdev, &wp_diag_device_ops);
	wp_diag_cdev.owner = THIS_MODULE;

	rc = cdev_add(&wp_diag_cdev, wp_diag_devno, 1);
	if (rc < 0)
		goto fail_destroy_device;

	_wp_context.configured = 0;
	return 0;

fail_destroy_device:
	device_destroy(wp_diag_class, wp_diag_devno);
fail_unregister_cdev_region:
	unregister_chrdev_region(wp_diag_devno, 1);
fail_destroy_class:
	class_destroy(wp_diag_class);
fail:
	return rc;
}

module_init(wp_diag_init);

static void __exit wp_diag_exit(void)
{
	struct wp_diag_context *ctxt;

	printk(KERN_INFO "wp_diag: %s\n", __FUNCTION__);

	ctxt = &_wp_context;
	if (!ctxt)
		return;

	smd_close(ctxt->ch);
	cdev_del(&wp_diag_cdev);
	device_destroy(wp_diag_class, wp_diag_devno);
	unregister_chrdev_region(wp_diag_devno, 1);
	class_destroy(wp_diag_class);

	return;
}

module_exit(wp_diag_exit);

MODULE_LICENSE("GPL v2");
