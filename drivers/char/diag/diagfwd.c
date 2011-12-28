/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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
 
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/diagchar.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include <mach/msm_smd.h>
#include <mach/socinfo.h>
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagchar_hdlc.h"
#ifdef CONFIG_DIAG_SDIO_PIPE
#include "diagfwd_sdio.h"
#endif
#include "../../../arch/arm/mach-msm/proc_comm.h"    //Div2D5-LC-BSP-Porting_RecoveryMode-00 +
//+{PS3-RR-ON_DEVICE_QXDM-01
#include <mach/dbgcfgtool.h>
//PS3-RR-ON_DEVICE_QXDM-01}+

//Div2D5-LC-BSP-Porting_RecoveryMode-00 +[
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/rtc.h>
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +]

MODULE_DESCRIPTION("Diag Char Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

//SW-2-5-1-MP-DbgCfgTool-03+[
#define  APPS_MODE_ANDROID   0x1
#define  APPS_MODE_RECOVERY  0x2
#define  APPS_MODE_FTM       0x3
#define  APPS_MODE_UNKNOWN   0xFF
//SW-2-5-1-MP-DbgCfgTool-03+]

/*
//SW2-5-1-MP-DbgCfgTool-00+[
#ifdef CONFIG_FIH_EFS2SD
#define SAVE_MODEM_EFS_LOG 1
#else
#define SAVE_MODEM_EFS_LOG 0
#endif

#if SAVE_MODEM_EFS_LOG
#include <linux/delay.h>
#include <asm/uaccess.h>  //get_fs, set_fs
#include <linux/fs.h>  //filp_open

#define EFS_READ_FILE_BUF_SIZE 600
#define EFS_FILE_PATH_STR_MAX 64

struct diag_read_buf
{
    unsigned char read_buf[EFS_READ_FILE_BUF_SIZE];
    unsigned int bytes_read;
}diag_device;

static struct workqueue_struct *diag_wq;
#endif

//SW2-5-1-MP-DbgCfgTool-00+]
*/
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +]
static DEFINE_SPINLOCK(smd_lock);
//static DECLARE_WAIT_QUEUE_HEAD(diag_wait_queue);
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +]

int diag_debug_buf_idx;
unsigned char diag_debug_buf[1024];
static unsigned int buf_tbl_size = 8; /*Number of entries in table of buffers */

struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };

#define ENCODE_RSP_AND_SEND(buf_length)				\
do {									\
	send.state = DIAG_STATE_START;					\
	send.pkt = driver->apps_rsp_buf;				\
	send.last = (void *)(driver->apps_rsp_buf + buf_length);	\
	send.terminate = 1;						\
	if (!driver->in_busy_1) {					\
		enc.dest = driver->buf_in_1;				\
		enc.dest_last = (void *)(driver->buf_in_1 + 499);	\
		diag_hdlc_encode(&send, &enc);				\
		driver->write_ptr_1->buf = driver->buf_in_1;		\
		driver->write_ptr_1->length = buf_length + 4;		\
		usb_diag_write(driver->legacy_ch, driver->write_ptr_1);	\
	}								\
} while (0)

#define CHK_OVERFLOW(bufStart, start, end, length) \
((bufStart <= start) && (end - start >= length)) ? 1 : 0

/*
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +[
#define WRITE_NV_4719_TO_ENTER_RECOVERY 1
#ifdef WRITE_NV_4719_TO_ENTER_RECOVERY
#define NV_FTM_MODE_BOOT_COUNT_I    4719

unsigned int switch_efslog = 1;

//static ssize_t write_nv4179(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
//{
//    unsigned int nv_value = 0;
//    unsigned int NVdata[3] = {NV_FTM_MODE_BOOT_COUNT_I, 0x1, 0x0};	
//
//    sscanf(buf, "%u\n", &nv_value);
//	
//    printk("paul %s: %d %u\n", __func__, count, nv_value);
//    NVdata[1] = nv_value;
//    fih_write_nv4719((unsigned *) NVdata);
//
//    return count;
//}
//DEVICE_ATTR(boot2recovery, 0644, NULL, write_nv4179);
#endif

static ssize_t OnOffEFS(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int nv_value = 0;

    sscanf(buf, "%u\n", &nv_value);
    switch_efslog = nv_value;

    printk("paul %s: %d %u\n", __func__, count, switch_efslog);
    return count;
}
DEVICE_ATTR(turnonofffefslog, 0644, NULL, OnOffEFS);
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +]
*/

#define CHK_APQ_GET_ID() \
(socinfo_get_id() == 86) ? 4062 : 0

void __diag_smd_send_req(void)
{
	void *buf = NULL;
	int *in_busy_ptr = NULL;
	struct diag_request *write_ptr_modem = NULL;

	if (!driver->in_busy_1) {
		buf = driver->buf_in_1;
		write_ptr_modem = driver->write_ptr_1;
		in_busy_ptr = &(driver->in_busy_1);
	} else if (!driver->in_busy_2) {
		buf = driver->buf_in_2;
		write_ptr_modem = driver->write_ptr_2;
		in_busy_ptr = &(driver->in_busy_2);
	}

	if (driver->ch && buf) {
		int r = smd_read_avail(driver->ch);

		if (r > IN_BUF_SIZE) {
			if (r < MAX_IN_BUF_SIZE) {
				printk(KERN_ALERT "\n diag: SMD sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				printk(KERN_ALERT "\n diag: SMD sending in "
				"packets more than %d bytes", MAX_IN_BUF_SIZE);
				return;
			}
		}
		if (r > 0) {
			if (!buf)
				pr_info("Out of diagmem for Modem\n");
			else {
				APPEND_DEBUG('i');
				smd_read(driver->ch, buf, r);
				APPEND_DEBUG('j');
				write_ptr_modem->length = r;
				*in_busy_ptr = 1;
				diag_device_write(buf, MODEM_DATA,
							 write_ptr_modem);
			}
		}
	}
}

int diag_device_write(void *buf, int proc_num, struct diag_request *write_ptr)
{
	int i, err = 0;

	if (driver->logging_mode == MEMORY_DEVICE_MODE) {
		if (proc_num == APPS_DATA) {
			for (i = 0; i < driver->poolsize_write_struct; i++)
				if (driver->buf_tbl[i].length == 0) {
					driver->buf_tbl[i].buf = buf;
					driver->buf_tbl[i].length =
								 driver->used;
#ifdef DIAG_DEBUG
					printk(KERN_INFO "\n ENQUEUE buf ptr"
						   " and length is %x , %d\n",
						   (unsigned int)(driver->buf_
				tbl[i].buf), driver->buf_tbl[i].length);
#endif
					break;
				}
		}
		for (i = 0; i < driver->num_clients; i++)
			if (driver->client_map[i].pid ==
						 driver->logging_process_id)
				break;
		if (i < driver->num_clients) {
			driver->data_ready[i] |= MEMORY_DEVICE_LOG_TYPE;
			wake_up_interruptible(&driver->wait_q);
		} else
			return -EINVAL;
	} else if (driver->logging_mode == NO_LOGGING_MODE) {
		if (proc_num == MODEM_DATA) {
			driver->in_busy_1 = 0;
			driver->in_busy_2 = 0;
			queue_work(driver->diag_wq, &(driver->
							diag_read_smd_work));
		} else if (proc_num == QDSP_DATA) {
			driver->in_busy_qdsp_1 = 0;
			driver->in_busy_qdsp_2 = 0;
			queue_work(driver->diag_wq, &(driver->
						diag_read_smd_qdsp_work));
		}
		err = -1;
	}
#ifdef CONFIG_DIAG_OVER_USB
	else if (driver->logging_mode == USB_MODE) {
		if (proc_num == APPS_DATA) {
			driver->write_ptr_svc = (struct diag_request *)
			(diagmem_alloc(driver, sizeof(struct diag_request),
				 POOL_TYPE_WRITE_STRUCT));
			if (driver->write_ptr_svc) {
				driver->write_ptr_svc->length = driver->used;
				driver->write_ptr_svc->buf = buf;
				err = usb_diag_write(driver->legacy_ch,
						driver->write_ptr_svc);
			} else
				err = -1;
		} else if (proc_num == MODEM_DATA) {
			write_ptr->buf = buf;
#ifdef DIAG_DEBUG
			printk(KERN_INFO "writing data to USB,"
				"pkt length %d\n", write_ptr->length);
			print_hex_dump(KERN_DEBUG, "Written Packet Data to"
					   " USB: ", 16, 1, DUMP_PREFIX_ADDRESS,
					    buf, write_ptr->length, 1);
#endif /* DIAG DEBUG */
			err = usb_diag_write(driver->legacy_ch, write_ptr);
		} else if (proc_num == QDSP_DATA) {
			write_ptr->buf = buf;
			err = usb_diag_write(driver->legacy_ch, write_ptr);
		}
#ifdef CONFIG_DIAG_SDIO_PIPE
		else if (proc_num == SDIO_DATA) {
			if (machine_is_msm8x60_charm_surf() ||
					machine_is_msm8x60_charm_ffa()) {
				write_ptr->buf = buf;
				err = usb_diag_write(driver->mdm_ch, write_ptr);
			} else
				pr_err("diag: Incorrect data while USB write");
		}
#endif
		APPEND_DEBUG('d');
	}
#endif /* DIAG OVER USB */
    return err;
}

void __diag_smd_qdsp_send_req(void)
{
	void *buf = NULL;
	int *in_busy_qdsp_ptr = NULL;
	struct diag_request *write_ptr_qdsp = NULL;

	if (!driver->in_busy_qdsp_1) {
		buf = driver->buf_in_qdsp_1;
		write_ptr_qdsp = driver->write_ptr_qdsp_1;
		in_busy_qdsp_ptr = &(driver->in_busy_qdsp_1);
	} else if (!driver->in_busy_qdsp_2) {
		buf = driver->buf_in_qdsp_2;
		write_ptr_qdsp = driver->write_ptr_qdsp_2;
		in_busy_qdsp_ptr = &(driver->in_busy_qdsp_2);
	}

	if (driver->chqdsp && buf) {
		int r = smd_read_avail(driver->chqdsp);

		if (r > IN_BUF_SIZE) {
			if (r < MAX_IN_BUF_SIZE) {
				printk(KERN_ALERT "\n diag: SMD sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				printk(KERN_ALERT "\n diag: SMD sending in "
				"packets more than %d bytes", MAX_IN_BUF_SIZE);
				return;
			}
		}
		if (r > 0) {
			if (!buf)
				printk(KERN_INFO "Out of diagmem for QDSP\n");
			else {
				APPEND_DEBUG('i');
				smd_read(driver->chqdsp, buf, r);
				APPEND_DEBUG('j');
				write_ptr_qdsp->length = r;
				*in_busy_qdsp_ptr = 1;
				diag_device_write(buf, QDSP_DATA,
							 write_ptr_qdsp);
			}
		}
	}
}

static void diag_print_mask_table(void)
{
/* Enable this to print mask table when updated */
#ifdef MASK_DEBUG
	int first;
	int last;
	uint8_t *ptr = driver->msg_masks;
	int i = 0;

	while (*(uint32_t *)(ptr + 4)) {
		first = *(uint32_t *)ptr;
		ptr += 4;
		last = *(uint32_t *)ptr;
		ptr += 4;
		printk(KERN_INFO "SSID %d - %d\n", first, last);
		for (i = 0 ; i <= last - first ; i++)
			printk(KERN_INFO "MASK:%x\n", *((uint32_t *)ptr + i));
		ptr += ((last - first) + 1)*4;

	}
#endif
}

static void diag_update_msg_mask(int start, int end , uint8_t *buf)
{
	int found = 0;
	int first;
	int last;
	uint8_t *ptr = driver->msg_masks;
	uint8_t *ptr_buffer_start = &(*(driver->msg_masks));
	uint8_t *ptr_buffer_end = &(*(driver->msg_masks)) + MSG_MASK_SIZE;

	mutex_lock(&driver->diagchar_mutex);
	/* First SSID can be zero : So check that last is non-zero */

	while (*(uint32_t *)(ptr + 4)) {
		first = *(uint32_t *)ptr;
		ptr += 4;
		last = *(uint32_t *)ptr;
		ptr += 4;
		if (start >= first && start <= last) {
			ptr += (start - first)*4;
			if (end <= last)
				if (CHK_OVERFLOW(ptr_buffer_start, ptr,
						  ptr_buffer_end,
						  (((end - start)+1)*4)))
					memcpy(ptr, buf , ((end - start)+1)*4);
				else
					printk(KERN_CRIT "Not enough"
							 " buffer space for"
							 " MSG_MASK\n");
			else
				printk(KERN_INFO "Unable to copy"
						 " mask change\n");

			found = 1;
			break;
		} else {
			ptr += ((last - first) + 1)*4;
		}
	}
	/* Entry was not found - add new table */
	if (!found) {
		if (CHK_OVERFLOW(ptr_buffer_start, ptr, ptr_buffer_end,
				  8 + ((end - start) + 1)*4)) {
			memcpy(ptr, &(start) , 4);
			ptr += 4;
			memcpy(ptr, &(end), 4);
			ptr += 4;
			memcpy(ptr, buf , ((end - start) + 1)*4);
		} else
			printk(KERN_CRIT " Not enough buffer"
					 " space for MSG_MASK\n");
	}
	mutex_unlock(&driver->diagchar_mutex);
	diag_print_mask_table();

}

static void diag_update_event_mask(uint8_t *buf, int toggle, int num_bits)
{
	uint8_t *ptr = driver->event_masks;
	uint8_t *temp = buf + 2;

	mutex_lock(&driver->diagchar_mutex);
	if (!toggle)
		memset(ptr, 0 , EVENT_MASK_SIZE);
	else
		if (CHK_OVERFLOW(ptr, ptr,
				 ptr+EVENT_MASK_SIZE,
				  num_bits/8 + 1))
			memcpy(ptr, temp , num_bits/8 + 1);
		else
			printk(KERN_CRIT "Not enough buffer space "
					 "for EVENT_MASK\n");
	mutex_unlock(&driver->diagchar_mutex);
}

static void diag_update_log_mask(int equip_id, uint8_t *buf, int num_items)
{
	uint8_t *temp = buf;
	struct mask_info {
		int equip_id;
		int index;
	};
	int i = 0;
	unsigned char *ptr_data;
	int offset = 8*MAX_EQUIP_ID;
	struct mask_info *ptr = (struct mask_info *)driver->log_masks;

	mutex_lock(&driver->diagchar_mutex);
	/* Check if we already know index of this equipment ID */
	for (i = 0; i < MAX_EQUIP_ID; i++) {
		if ((ptr->equip_id == equip_id) && (ptr->index != 0)) {
			offset = ptr->index;
			break;
		}
		if ((ptr->equip_id == 0) && (ptr->index == 0)) {
			/*Reached a null entry */
			ptr->equip_id = equip_id;
			ptr->index = driver->log_masks_length;
			offset = driver->log_masks_length;
			driver->log_masks_length += ((num_items+7)/8);
			break;
		}
		ptr++;
	}
	ptr_data = driver->log_masks + offset;
	if (CHK_OVERFLOW(driver->log_masks, ptr_data, driver->log_masks
					 + LOG_MASK_SIZE, (num_items+7)/8))
		memcpy(ptr_data, temp , (num_items+7)/8);
	else
		printk(KERN_CRIT " Not enough buffer space for LOG_MASK\n");
	mutex_unlock(&driver->diagchar_mutex);
}

static void diag_update_pkt_buffer(unsigned char *buf)
{
	unsigned char *ptr = driver->pkt_buf;
	unsigned char *temp = buf;

	mutex_lock(&driver->diagchar_mutex);
	if (CHK_OVERFLOW(ptr, ptr, ptr + PKT_SIZE, driver->pkt_length))
		memcpy(ptr, temp , driver->pkt_length);
	else
		printk(KERN_CRIT " Not enough buffer space for PKT_RESP\n");
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_userspace_clients(unsigned int type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid != 0)
			driver->data_ready[i] |= type;
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_sleeping_process(int process_id)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid == process_id) {
			driver->data_ready[i] |= PKT_TYPE;
			break;
		}
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

static int diag_process_apps_pkt(unsigned char *buf, int len)
{
	uint16_t start;
	uint16_t end, subsys_cmd_code;
	int i, cmd_code, subsys_id;
	int packet_type = 1;
	unsigned char *temp = buf;

	/* event mask */
	if ((*buf == 0x60) && (*(++buf) == 0x0)) {
		diag_update_event_mask(buf, 0, 0);
		diag_update_userspace_clients(EVENT_MASKS_TYPE);
	}
	/* check for set event mask */
	else if (*buf == 0x82) {
		buf += 4;
		diag_update_event_mask(buf, 1, *(uint16_t *)buf);
		diag_update_userspace_clients(
		EVENT_MASKS_TYPE);
	}
	/* log mask */
	else if (*buf == 0x73) {
		buf += 4;
		if (*(int *)buf == 3) {
			buf += 4;
			/* Read Equip ID and pass as first param below*/
			diag_update_log_mask(*(int *)buf, buf+8,
							 *(int *)(buf+4));
			diag_update_userspace_clients(LOG_MASKS_TYPE);
		}
	}
	/* Check for set message mask  */
	else if ((*buf == 0x7d) && (*(++buf) == 0x4)) {
		buf++;
		start = *(uint16_t *)buf;
		buf += 2;
		end = *(uint16_t *)buf;
		buf += 4;
		diag_update_msg_mask((uint32_t)start, (uint32_t)end , buf);
		diag_update_userspace_clients(MSG_MASKS_TYPE);
	}
	/* Set all run-time masks
	if ((*buf == 0x7d) && (*(++buf) == 0x5)) {
		TO DO
	} */
#if defined(CONFIG_DIAG_OVER_USB)
	 /* Check for ID for APQ8060 AND NO MODEM present */
	else if (!(driver->ch) && CHK_APQ_GET_ID()) {
		/* Respond to polling for Apps only DIAG */
		if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
							 (*(buf+2) == 0x03)) {
			for (i = 0; i < 3; i++)
				driver->apps_rsp_buf[i] = *(buf+i);
			for (i = 0; i < 13; i++)
				driver->apps_rsp_buf[i+3] = 0;

			ENCODE_RSP_AND_SEND(15);
			return 0;
		}
		/* respond to 0x0 command */
		else if (*buf == 0x00) {
			for (i = 0; i < 55; i++)
				driver->apps_rsp_buf[i] = 0;

			ENCODE_RSP_AND_SEND(54);
			return 0;
		}
		/* respond to 0x7c command */
		else if (*buf == 0x7c) {
			driver->apps_rsp_buf[0] = 0x7c;
			for (i = 1; i < 8; i++)
				driver->apps_rsp_buf[i] = 0;
			/* Tools ID for APQ 8060 */
			*(int *)(driver->apps_rsp_buf + 8) = CHK_APQ_GET_ID();
			*(unsigned char *)(driver->apps_rsp_buf + 12) = '\0';
			*(unsigned char *)(driver->apps_rsp_buf + 13) = '\0';
			ENCODE_RSP_AND_SEND(13);
			return 0;
		}
	}
#endif
	/* Check for registered clients and forward packet to user-space */
	cmd_code = (int)(*(char *)buf);
	temp++;
	subsys_id = (int)(*(char *)temp);
	temp++;
	subsys_cmd_code = *(uint16_t *)temp;
	temp += 2;

	for (i = 0; i < diag_max_registration; i++) {
		if (driver->table[i].process_id != 0) {
			if (driver->table[i].cmd_code ==
				 cmd_code && driver->table[i].subsys_id ==
				 subsys_id &&
				driver->table[i].cmd_code_lo <=
				 subsys_cmd_code &&
				  driver->table[i].cmd_code_hi >=
				 subsys_cmd_code){
				driver->pkt_length = len;
				diag_update_pkt_buffer(buf);
				diag_update_sleeping_process(
					driver->table[i].process_id);
					return 0;
				} /* end of if */
			else if (driver->table[i].cmd_code == 255
				  && cmd_code == 75) {
				if (driver->table[i].subsys_id ==
					subsys_id &&
				   driver->table[i].cmd_code_lo <=
					subsys_cmd_code &&
					 driver->table[i].cmd_code_hi >=
					subsys_cmd_code){
					driver->pkt_length = len;
					diag_update_pkt_buffer(buf);
					diag_update_sleeping_process(
						driver->table[i].
						process_id);
					return 0;
				}
			} /* end of else-if */
			else if (driver->table[i].cmd_code == 255 &&
				  driver->table[i].subsys_id == 255) {
				if (driver->table[i].cmd_code_lo <=
						 cmd_code &&
						 driver->table[i].
						cmd_code_hi >= cmd_code){
					driver->pkt_length = len;
					diag_update_pkt_buffer(buf);
					diag_update_sleeping_process
						(driver->table[i].
						 process_id);
					return 0;
				}
			} /* end of else-if */
		} /* if(driver->table[i].process_id != 0) */
	}  /* for (i = 0; i < diag_max_registration; i++) */
	return packet_type;
}

void diag_process_hdlc(void *data, unsigned len)
{
	struct diag_hdlc_decode_type hdlc;
	int ret, type = 0;
#ifdef DIAG_DEBUG
	int i;
	printk(KERN_INFO "\n HDLC decode function, len of data  %d\n", len);
#endif
	hdlc.dest_ptr = driver->hdlc_buf;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);

	if (ret)
		type = diag_process_apps_pkt(driver->hdlc_buf,
							  hdlc.dest_idx - 3);
	else if (driver->debug_flag) {
		printk(KERN_ERR "Packet dropped due to bad HDLC coding/CRC"
				" errors or partial packet received, packet"
				" length = %d\n", len);
		print_hex_dump(KERN_DEBUG, "Dropped Packet Data: ", 16, 1,
					   DUMP_PREFIX_ADDRESS, data, len, 1);
		driver->debug_flag = 0;
	}
	/* implies this packet is NOT meant for apps */
	if (!(driver->ch) && type == 1 && CHK_APQ_GET_ID()) {
		if (driver->chqdsp)
			smd_write(driver->chqdsp, driver->hdlc_buf,
							 hdlc.dest_idx - 3);
		type = 0;
	}

#ifdef DIAG_DEBUG
	printk(KERN_INFO "\n hdlc.dest_idx = %d", hdlc.dest_idx);
	for (i = 0; i < hdlc.dest_idx; i++)
		printk(KERN_DEBUG "\t%x", *(((unsigned char *)
							driver->hdlc_buf)+i));
#endif /* DIAG DEBUG */
	/* ignore 2 bytes for CRC, one for 7E and send */
	if ((driver->ch) && (ret) && (type) && (hdlc.dest_idx > 3)) {
		APPEND_DEBUG('g');
		smd_write(driver->ch, driver->hdlc_buf, hdlc.dest_idx - 3);
		APPEND_DEBUG('h');
#ifdef DIAG_DEBUG
		printk(KERN_INFO "writing data to SMD, pkt length %d\n", len);
		print_hex_dump(KERN_DEBUG, "Written Packet Data to SMD: ", 16,
			       1, DUMP_PREFIX_ADDRESS, data, len, 1);
#endif /* DIAG DEBUG */
	}

}

#ifdef CONFIG_DIAG_OVER_USB
#define N_LEGACY_WRITE	(driver->poolsize + 5) /* 2+1 for modem ; 2 for q6 */
#define N_LEGACY_READ	1

int diagfwd_connect(void)
{
	int err;

	printk(KERN_DEBUG "diag: USB connected\n");
	err = usb_diag_alloc_req(driver->legacy_ch, N_LEGACY_WRITE,
			N_LEGACY_READ);
	if (err)
		printk(KERN_ERR "diag: unable to alloc USB req on legacy ch");

	driver->usb_connected = 1;
	driver->in_busy_1 = 0;
	driver->in_busy_2 = 0;
	driver->in_busy_qdsp_1 = 0;
	driver->in_busy_qdsp_2 = 0;

	/* Poll SMD channels to check for data*/
	queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	/* Poll USB channel to check for data*/
	queue_work(driver->diag_wq, &(driver->diag_read_work));
#ifdef CONFIG_DIAG_SDIO_PIPE
	if (machine_is_msm8x60_charm_surf() || machine_is_msm8x60_charm_ffa()) {
		if (driver->mdm_ch && !IS_ERR(driver->mdm_ch))
			diagfwd_connect_sdio();
		else
			printk(KERN_INFO "diag: No USB MDM ch");
	}
#endif
	return 0;
}

int diagfwd_disconnect(void)
{
	printk(KERN_DEBUG "diag: USB disconnected\n");
	driver->usb_connected = 0;
	driver->in_busy_1 = 1;
	driver->in_busy_2 = 1;
	driver->in_busy_qdsp_1 = 1;
	driver->in_busy_qdsp_2 = 1;
	driver->debug_flag = 1;
	usb_diag_free_req(driver->legacy_ch);
#ifdef CONFIG_DIAG_SDIO_PIPE
	if (machine_is_msm8x60_charm_surf() || machine_is_msm8x60_charm_ffa())
		if (driver->mdm_ch && !IS_ERR(driver->mdm_ch))
			diagfwd_disconnect_sdio();
#endif
	/* TBD - notify and flow control SMD */
	return 0;
}

int diagfwd_write_complete(struct diag_request *diag_write_ptr)
{
	unsigned char *buf = diag_write_ptr->buf;
	/*Determine if the write complete is for data from modem/apps/q6 */
	/* Need a context variable here instead */
	if (buf == (void *)driver->buf_in_1) {
		driver->in_busy_1 = 0;
		APPEND_DEBUG('o');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	} else if (buf == (void *)driver->buf_in_2) {
		driver->in_busy_2 = 0;
		APPEND_DEBUG('O');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	} else if (buf == (void *)driver->buf_in_qdsp_1) {
		driver->in_busy_qdsp_1 = 0;
		APPEND_DEBUG('p');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	} else if (buf == (void *)driver->buf_in_qdsp_2) {
		driver->in_busy_qdsp_2 = 0;
		APPEND_DEBUG('P');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	}
#ifdef CONFIG_DIAG_SDIO_PIPE
	else if (buf == (void *)driver->buf_in_sdio)
		if (machine_is_msm8x60_charm_surf() ||
					 machine_is_msm8x60_charm_ffa())
			diagfwd_write_complete_sdio();
		else
			pr_err("diag: Incorrect buffer pointer while WRITE");
#endif
	else {
		diagmem_free(driver, (unsigned char *)buf, POOL_TYPE_HDLC);
		diagmem_free(driver, (unsigned char *)diag_write_ptr,
						 POOL_TYPE_WRITE_STRUCT);
		APPEND_DEBUG('q');
	}
	return 0;
}

int diagfwd_read_complete(struct diag_request *diag_read_ptr)
{
	int status = diag_read_ptr->status;
	unsigned char *buf = diag_read_ptr->buf;

	/* Determine if the read complete is for data on legacy/mdm ch */
	if (buf == (void *)driver->usb_buf_out) {
		driver->read_len_legacy = diag_read_ptr->actual;
		APPEND_DEBUG('s');
#ifdef DIAG_DEBUG
		printk(KERN_INFO "read data from USB, pkt length %d",
		    diag_read_ptr->actual);
		print_hex_dump(KERN_DEBUG, "Read Packet Data from USB: ", 16, 1,
		       DUMP_PREFIX_ADDRESS, diag_read_ptr->buf,
		       diag_read_ptr->actual, 1);
#endif /* DIAG DEBUG */
		if (driver->logging_mode == USB_MODE) {
			if (status != -ECONNRESET && status != -ESHUTDOWN)
				queue_work(driver->diag_wq,
					&(driver->diag_proc_hdlc_work));
			else
				queue_work(driver->diag_wq,
						 &(driver->diag_read_work));
		}
	}
#ifdef CONFIG_DIAG_SDIO_PIPE
	else if (buf == (void *)driver->usb_buf_mdm_out) {
		if (machine_is_msm8x60_charm_surf() ||
					 machine_is_msm8x60_charm_ffa()) {
			driver->read_len_mdm = diag_read_ptr->actual;
			diagfwd_read_complete_sdio();
		} else
			pr_err("diag: Incorrect buffer pointer while READ");
	}
#endif
	else
		printk(KERN_ERR "diag: Unknown buffer ptr from USB");

	return 0;
}

void diag_read_work_fn(struct work_struct *work)
{
	APPEND_DEBUG('d');
	driver->usb_read_ptr->buf = driver->usb_buf_out;
	driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
	usb_diag_read(driver->legacy_ch, driver->usb_read_ptr);
	APPEND_DEBUG('e');
}

void diag_process_hdlc_fn(struct work_struct *work)
{
	APPEND_DEBUG('D');
	diag_process_hdlc(driver->usb_buf_out, driver->read_len_legacy);
	diag_read_work_fn(work);
	APPEND_DEBUG('E');
}

void diag_usb_legacy_notifier(void *priv, unsigned event,
			struct diag_request *d_req)
{
	switch (event) {
	case USB_DIAG_CONNECT:
		diagfwd_connect();
		break;
	case USB_DIAG_DISCONNECT:
		diagfwd_disconnect();
		break;
	case USB_DIAG_READ_DONE:
		diagfwd_read_complete(d_req);
		break;
	case USB_DIAG_WRITE_DONE:
		diagfwd_write_complete(d_req);
		break;
	default:
		printk(KERN_ERR "Unknown event from USB diag\n");
		break;
	}
}

#endif /* DIAG OVER USB */

//Div2D5-LC-BSP-Porting_RecoveryMode-00 +[
#define DIAG_READ_RETRY_COUNT    100  //100 * 5 ms = 500ms
int diag_read_from_smd(uint8_t * res_buf, int16_t* res_size)
{
    unsigned long flags;

    int sz;
    int gg = 0;
    int retry = 0;
    int rc = -1;

    for(retry = 0; retry < DIAG_READ_RETRY_COUNT; )
    {
        sz = smd_cur_packet_size(driver->ch);

        if (sz == 0)
        {
            msleep(5);
            retry++;
            continue;
        }
        gg = smd_read_avail(driver->ch);

        if (sz > gg)
        {
            continue;
        }

        spin_lock_irqsave(&smd_lock, flags);//mutex_lock(&nmea_rx_buf_lock);
        if (smd_read(driver->ch, res_buf, sz) == sz) {
            spin_unlock_irqrestore(&smd_lock, flags);//mutex_unlock(&nmea_rx_buf_lock);
            break;
        }
        //nmea_devp->bytes_read = sz;
        spin_unlock_irqrestore(&smd_lock, flags);//mutex_unlock(&nmea_rx_buf_lock);
    }
    *res_size = sz;
    if (retry >= DIAG_READ_RETRY_COUNT)
    {
        rc = -2;
        goto lbExit;
    }
    rc = 0;
lbExit:
    return rc;
}
EXPORT_SYMBOL(diag_read_from_smd);

void diag_write_to_smd(uint8_t * cmd_buf, int cmd_size)
{
	unsigned long flags;
	int need;
	//paul// need = sizeof(cmd_buf);
	need = cmd_size;

	spin_lock_irqsave(&smd_lock, flags);
	while (smd_write_avail(driver->ch) < need) {
		spin_unlock_irqrestore(&smd_lock, flags);
		msleep(10);
		spin_lock_irqsave(&smd_lock, flags);
	}
       smd_write(driver->ch, cmd_buf, cmd_size);
       spin_unlock_irqrestore(&smd_lock, flags);
}
EXPORT_SYMBOL(diag_write_to_smd);
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +]
/*
//SW2-5-1-MP-DbgCfgTool-00+[
#if SAVE_MODEM_EFS_LOG
static DEFINE_MUTEX(diag_rx_buf_lock);
static struct workqueue_struct *diag_wq;

static void diag_work_func(struct work_struct *ws)
{
	int sz;

	for (;;) {
		sz = smd_cur_packet_size(driver->ch);
		if (sz == 0)
			break;
		if (sz > smd_read_avail(driver->ch))
			break;
		if (sz > 600) {
			smd_read(driver->ch, 0, sz);
			continue;
		}

		mutex_lock(&diag_rx_buf_lock);
		if (smd_read(driver->ch, diag_device.read_buf, sz) != sz) {
			mutex_unlock(&diag_rx_buf_lock);
			printk(KERN_ERR "diag: not enough data?!\n");
			continue;
		}
		diag_device.bytes_read = sz;
		mutex_unlock(&diag_rx_buf_lock);
		wake_up_interruptible(&diag_wait_queue);
	}
}
static DECLARE_WORK(diag_work, diag_work_func);
#endif
//SW2-5-1-MP-DbgCfgTool-00+]
*/

static void diag_smd_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
/*
//SW2-5-1-MP-DbgCfgTool-00+[
#if SAVE_MODEM_EFS_LOG
    if(!diag_usb_configure()) {
        switch (event) {
        case SMD_EVENT_DATA: {
            int sz;
            sz = smd_cur_packet_size(driver->ch);
            if ((sz > 0) && (sz <= smd_read_avail(driver->ch)))
                queue_work(diag_wq, &diag_work);
            break;
        }
        case SMD_EVENT_OPEN:
            printk(KERN_INFO "diag: smd opened\n");
            break;
        case SMD_EVENT_CLOSE:
            printk(KERN_INFO "diag: smd closed\n");
            break;
        }
    }
#endif
//SW2-5-1-MP-DbgCfgTool-00+]
*/
}

#if defined(CONFIG_MSM_N_WAY_SMD)
static void diag_smd_qdsp_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
}
#endif

/*
//SW2-5-1-MP-DbgCfgTool-00+[
#if SAVE_MODEM_EFS_LOG
#define DEBUG_EFS_COMMAND 1
static void efs_send_req(char * cmd_buf ,int cmd_size)
{
    unsigned long flags;
    int need = cmd_size;
    int smd_size = 0;
    int write_size = 0;
    spin_lock_irqsave(&smd_lock, flags);
    while ((smd_size = smd_write_avail(driver->ch)) < need) {
        spin_unlock_irqrestore(&smd_lock, flags);
        msleep(250);
        spin_lock_irqsave(&smd_lock, flags);
    }
    write_size = smd_write(driver->ch, cmd_buf, cmd_size);
    spin_unlock_irqrestore(&smd_lock, flags);

	//print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET,
	//			16, 1, cmd_buf, cmd_size, 0);
}
static void efs_recv_res(char * res_buf ,int res_size)
{
    struct diag_hdlc_decode_type hdlc = {
        .dest_ptr = res_buf,  //driver->hdlc_buf;
        .dest_size = res_size,  //USB_MAX_OUT_BUF;
        .src_ptr = diag_device.read_buf,  //data;
        .src_size = res_size, //len;
        .src_idx = 0,
        .dest_idx = 0,
        .escaping = 0,
    };
    int r = 0;

    r = wait_event_interruptible_timeout(diag_wait_queue, diag_device.bytes_read,msecs_to_jiffies(1000));
//    printk(KERN_INFO "%s: bytes_read=%d", __func__, diag_device.bytes_read);
*/

//	if (r < 0) {
		/* qualify error message */
//		if (r != -ERESTARTSYS) {
			/* we get this anytime a signal comes in */
/*
			printk(KERN_ERR "ERROR:%s:%i:%s: "
				"wait_event_interruptible ret %i\n",
				__FILE__,
				__LINE__,
				__func__,
				r
				);
		}
		return;
	} else if(r==0){
	    printk(KERN_INFO "%s: timeout", __func__);
	    return;
	}

	mutex_lock(&diag_rx_buf_lock);
	diag_device.bytes_read = 0;
	r = diag_hdlc_decode(&hdlc);
	mutex_unlock(&diag_rx_buf_lock);
}

static void efs_send_command(char * cmd_buf ,int cmd_size, char * res_buf ,int res_size)
{
    efs_send_req(cmd_buf, cmd_size);
    efs_recv_res(res_buf, res_size);
}

struct efs_packet_header
{
    u8 cmd_code;
    u8 subsys_id;
    u16 subsys_cmd_code;
};

struct efs_packet_readdir_req
{
    struct efs_packet_header header;
    u32 dirp;
    u32 seqno;
};

struct efs_packet_closedir_req
{
    struct efs_packet_header header;
    u32 dirp;
};

struct efs_packet_closefile_req
{
    struct efs_packet_header header;
    u32 fd;
};

struct efs_packet_readfile_req
{
    struct efs_packet_header header;
    u32 fd;
    u32 nbyte;
    u32 offset;
};

struct efs_packet_openfile_req
{
    struct efs_packet_header header;
    u32 oflag;
    u32 mode;
};

struct efs_packet_opendir_req
{
    struct efs_packet_header header;
    char path[1];
};

static void getlog(char *dir_name, unsigned int dir_name_size)
{
    struct efs_packet_readdir_req EFS_readdir_req = {
        .header = {
            .cmd_code = 75,
            .subsys_id = 19,
            .subsys_cmd_code = 12,
        },
        .dirp = 1,
        .seqno = 1,
    }, *pEFS_readdir_req = &EFS_readdir_req;

    struct efs_packet_openfile_req EFS_openfile_oem_req = {
        .header = {
            .cmd_code = 75,
            .subsys_id = 19,
            .subsys_cmd_code = 2,
        },
        .oflag = 0,
        .mode = 0,
    }, *pEFS_openfile_oem_req = &EFS_openfile_oem_req;

    struct efs_packet_readfile_req EFS_readfile_req = {
        .header = {
            .cmd_code = 75,
            .subsys_id = 19,
            .subsys_cmd_code = 4,
        },
        .fd = 0,
        .nbyte = 512,
        .offset = 0,
    }, *pEFS_readfile_req = &EFS_readfile_req;

    struct efs_packet_closedir_req EFS_closedir_req = {
        .header = {
            .cmd_code = 75,
            .subsys_id = 19,
            .subsys_cmd_code = 13,
        },
        .dirp = 1,
    }, *pEFS_closedir_req = &EFS_closedir_req;

    struct efs_packet_closefile_req EFS_closefile_req = {
        .header = {
            .cmd_code = 75,
            .subsys_id = 19,
            .subsys_cmd_code = 3,
        },
        .fd = 0,
    }, *pEFS_closefile_req = &EFS_closefile_req;

    struct file *efs_file_filp = NULL;
    struct efs_packet_opendir_req* efs_opendir_req = NULL;

    char efs_log_root_path[] = "/data/efslog/";
    char efs_file_path[EFS_FILE_PATH_STR_MAX];
    char responseBuf[EFS_READ_FILE_BUF_SIZE];

    char * efs_req_buf = NULL;
    char *write_p = NULL;
    char *efs_dir_path = NULL;    //store string /data/efslog/OEMDBG_LOG/ or /data/efslog/err/

    unsigned int readdir_i=2;
    unsigned int efs_file_name_length = 0;
    unsigned int bEOF = 0;
    unsigned int read_offset = 0;
    unsigned int open_file_req_size = 0;
    unsigned int efs_read_res_size = 0;
    unsigned int efs_opendir_req_size = sizeof(struct efs_packet_header)+ dir_name_size;
    unsigned int efs_dir_str_len = sizeof(efs_log_root_path)+dir_name_size;  // sizeof():"/data/efslog/" + '\0'  dir_name_size: "err" + '\0'

    efs_dir_path = kzalloc(efs_dir_str_len, GFP_KERNEL);
    if(efs_dir_path == NULL) {
        printk(KERN_INFO "allocate efs_dir_path\n");
        goto End;
    }
    snprintf(efs_dir_path, efs_dir_str_len, "%s%s/", efs_log_root_path,dir_name);
    printk(KERN_INFO "dir_name=%s, dir_name_size=%d, efs_dir_path=%s\n", dir_name, dir_name_size, efs_dir_path);

    efs_opendir_req = (struct efs_packet_opendir_req*)kzalloc(efs_opendir_req_size, GFP_KERNEL);   //efs_opendir_req: open dir command
    if(efs_opendir_req == NULL) {
        printk(KERN_INFO "allocate dir_name buffer fails\n");
        goto End;
    }

    efs_opendir_req->header.cmd_code = 75;
    efs_opendir_req->header.subsys_id = 19;
    efs_opendir_req->header.subsys_cmd_code = 11;
    memcpy(efs_opendir_req->path, dir_name, dir_name_size);  //include the last '\0'
    efs_send_command((char *)efs_opendir_req, efs_opendir_req_size, responseBuf, sizeof(responseBuf));
    printk(KERN_INFO "open %s dir, the dirp=0x%x\n", efs_opendir_req->path, responseBuf[4]);
    kfree(efs_opendir_req);

    EFS_readdir_req.dirp = responseBuf[4];

    while(1){
        efs_send_command((char*)pEFS_readdir_req, sizeof(EFS_readdir_req), responseBuf, sizeof(responseBuf));
        EFS_readdir_req.seqno = readdir_i++;
        if(responseBuf[40] == 0x00) { //the file name is null
            printk(KERN_INFO "No additional files\n");
            break;
        }

        printk(KERN_INFO "%s dir, file name: %s\n", dir_name, &responseBuf[40]);

        snprintf(efs_file_path, EFS_FILE_PATH_STR_MAX, "%s%s", efs_dir_path, &responseBuf[40]);
        efs_file_filp = filp_open(efs_file_path, O_WRONLY | O_CREAT |O_LARGEFILE, 0777);

        efs_file_name_length = strlen(&responseBuf[40]);
//        printk(KERN_INFO "efs_file_name_length=%d\n", efs_file_name_length);

//length of dir_name_size includes the last '\0', length of efs_file_name_length does not include the last '\0'
        open_file_req_size = sizeof(EFS_openfile_oem_req) + dir_name_size + efs_file_name_length + 1;  //command + dir name length + '\0' + file name length +  '\0'  // the first '\0' will be replaced by '/'
        efs_req_buf = kzalloc(open_file_req_size, GFP_KERNEL);
        memcpy(efs_req_buf, pEFS_openfile_oem_req, sizeof(EFS_openfile_oem_req));
        snprintf(efs_req_buf + sizeof(EFS_openfile_oem_req), open_file_req_size, "%s/%s", dir_name, &responseBuf[40]);
        efs_send_command(efs_req_buf, open_file_req_size, responseBuf, sizeof(responseBuf));
        kfree(efs_req_buf);

        EFS_readfile_req.fd = responseBuf[4];
        EFS_closefile_req.fd = responseBuf[4];

        bEOF = 0;
        read_offset = 0;
        while(!bEOF) //write file until EOF
        {
            EFS_readfile_req.offset = read_offset << 9;
            efs_send_command((char *)pEFS_readfile_req, sizeof(EFS_readfile_req), responseBuf, EFS_READ_FILE_BUF_SIZE);
            read_offset++;

            write_p = &responseBuf[20];
            efs_read_res_size = (responseBuf[13] << 8) + responseBuf[12] ; 

            if(efs_read_res_size == 0) {
                bEOF = 1;
            }else if(efs_read_res_size < 512) {
                efs_file_filp->f_op->write(efs_file_filp,(char __user *)(write_p), efs_read_res_size ,&efs_file_filp->f_pos);
                bEOF = 1;
            } else {
                efs_file_filp->f_op->write(efs_file_filp,(char __user *)(write_p), efs_read_res_size ,&efs_file_filp->f_pos);
            }
        }
        filp_close(efs_file_filp, NULL);
        efs_send_command((char *)pEFS_closefile_req, sizeof(EFS_closefile_req), responseBuf, sizeof(responseBuf));
    };//while(1) read content until the pathname is null
    efs_send_command((char *)pEFS_closedir_req, sizeof(EFS_closedir_req), responseBuf, sizeof(responseBuf));

End:

    if(efs_dir_path)
        kfree(efs_dir_path);

    return;
}

static ssize_t save_efs2sd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char EFS_opendir_oemdbg_req[15] = {0x4b, 0x13, 0x0b, 0x00, 'O', 'E', 'M', 'D', 'B', 'G', '_', 'L', 'O', 'G', 0x00};//open OEMDBG_LOG folder
    char responseBuf[EFS_READ_FILE_BUF_SIZE];
    mm_segment_t oldfs;

    if(diag_usb_configure()) {
        printk(KERN_INFO "%s: DIAG Port is open, and USB is plugged, so efs2sd is diabled.\n", __func__);
        goto End;
    }

    oldfs=get_fs();
    set_fs(KERNEL_DS);
*/
//**********************BUG****  The first command will fail. Below is a command to fail on purpose, or dirp is not an correct value
//    efs_send_req(EFS_opendir_oemdbg_req, sizeof(EFS_opendir_oemdbg_req));
//    efs_recv_res(responseBuf, sizeof(responseBuf));
//**********************BUG****
/*
    getlog("OEMDBG_LOG", sizeof("OEMDBG_LOG"));
    getlog("err", sizeof("err"));

    set_fs(oldfs);

End:
    return count;
}

DEVICE_ATTR(efs2sd, 0777, NULL, save_efs2sd);
#endif
//SW2-5-1-MP-DbgCfgTool-00+]
*/

/* FIH, JiaHao, 2011/09/30 { */ /* fver */

#define ESF2_CMD_RETRY_CONT     100  //100 * 5 ms = 500ms
#define EFS2_CMD_REQUEST_SIZE   256
#define EFS2_CMD_RESPONSE_SIZE  544  // 512+32=544

#define EFS2_FILE_READ_SIZE  0x200

#define EFS2_DIAG_OPENDIR_RES_SIZE   12  // Response 12 bytes
#define EFS2_DIAG_READDIR_RES_SIZE   (40 + 64)  // Response 40 bytes + variable string
#define EFS2_DIAG_OPEN_RES_SIZE      12  // Response 12 bytes
#define EFS2_DIAG_READ_RES_SIZE      20  // Response 20 bytes + variable data
#define EFS2_DIAG_CLOSE_RES_SIZE     8   // Response 8 bytes
#define EFS2_DIAG_CLOSEDIR_RES_SIZE  8   // Response 8 bytes

static DEFINE_SPINLOCK(diag_smd_lock);

enum {
    EFS_ERR_NONE,
    EFS_EPERM,
    EFS_ENOENT,
    EFS_EEXIST        =6, 
    EFS_EBADF         =9,
    EFS_ENOMEM        =12,
    EFS_EACCES        =13,
    EFS_EBUSY         =16,
    EFS_EXDEV         =18,
    EFS_ENODEV        =19,
    EFS_ENOTDIR       =20,
    EFS_EISDIR        =21,
    EFS_EINVAL        =22,
    EFS_EMFILE        =24,
    EFS_ETXTBSY       =26,
    EFS_ENOSPC        =28,
    EFS_ESPIPE        =29,
    EFS_FS_ERANGE     =34,
    EFS_ENAMETOOLONG  =36,
    EFS_ENOTEMPTY     =39,
    EFS_ELOOP         =40,
    EFS_ESTALE        =116,
    EFS_EDQUOT        =122,
    EFS_ENOCARD       =301,
    EFS_EBADFMT       =302,
    EFS_ENOTITM       =303,
    EFS_EROLLBACK     =304,
    EFS_ERR_UNKNOWN   =999,
};

enum {
    EFS_DIAG_TRANSACTION_OK = 0,
    EFS_DIAG_TRANSACTION_NO_DATA,
    EFS_DIAG_TRANSACTION_WRONG_DATA,
    EFS_DIAG_TRANSACTION_NO_FILE_DIR,
    EFS_DIAG_TRANSACTION_CMD_ERROR,
    EFS_DIAG_TRANSACTION_ABORT,
};

static void efs_send_req(unsigned char * cmd_buf ,int cmd_size)
{
    unsigned long flags;
    int need;

    need = cmd_size;
    spin_lock_irqsave(&diag_smd_lock, flags);

    while (smd_write_avail(driver->ch) < need) {
        spin_unlock_irqrestore(&diag_smd_lock, flags);
        msleep(250);
        spin_lock_irqsave(&diag_smd_lock, flags);
    }

    smd_write(driver->ch, cmd_buf, cmd_size);
    spin_unlock_irqrestore(&diag_smd_lock, flags);
}

static int efs_recv_res(unsigned char * res_buf ,int res_size)
{
    unsigned long flags;
    static unsigned char responseBuf_temp[EFS2_CMD_RESPONSE_SIZE];
    struct diag_hdlc_decode_type hdlc;
    int ret = 0;
    int sz;
    int gg = 0;
    int retry = 0;
    int rc = -EFS_DIAG_TRANSACTION_NO_DATA;

    hdlc.dest_ptr = res_buf;
    hdlc.dest_size = res_size;
    hdlc.src_ptr = responseBuf_temp; //data;
    hdlc.src_size = sizeof(responseBuf_temp); //len
    hdlc.src_idx = 0;
    hdlc.dest_idx = 0;
    hdlc.escaping = 0;
    memset(responseBuf_temp, 0, sizeof(responseBuf_temp));

    for (retry = 0; retry < ESF2_CMD_RETRY_CONT; )
    {
        sz = smd_cur_packet_size(driver->ch);
        if (sz == 0) {
            retry++;
            msleep(5);
            continue;
        }

        gg = smd_read_avail(driver->ch);
        if (sz > gg) {
            continue;
        }

        if (sz >= sizeof(responseBuf_temp)) {
            goto lbExit_efs_recv_res;
        }

        spin_lock_irqsave(&diag_smd_lock, flags);
        if (smd_read(driver->ch, responseBuf_temp, sz) == sz) {
            spin_unlock_irqrestore(&diag_smd_lock, flags);
            break;
        }
        spin_unlock_irqrestore(&diag_smd_lock, flags);
    }

    if (retry >= ESF2_CMD_RETRY_CONT) {
        goto lbExit_efs_recv_res;
    }

    ret = diag_hdlc_decode(&hdlc);
    rc = EFS_DIAG_TRANSACTION_OK;

lbExit_efs_recv_res:
    return rc;
}

static int efs_diag_func(unsigned char * TxBuf, int TxBufSize, unsigned char * RxBuf, int RxBufSize)
{
    int retry = 5;
    int rc = -1;
    int error_code = EFS_ERR_NONE;

    do
    {
        efs_send_req(TxBuf, TxBufSize);

lbReadDataAgain:
        rc = efs_recv_res(RxBuf, RxBufSize);

        if (rc < 0)
        {
            if (retry-- < 0)
                break;
            else
                continue;
        }

        if (TxBuf[0] != RxBuf[0] || TxBuf[1] != RxBuf[1] || TxBuf[2] != RxBuf[2])
        {
            rc = -EFS_DIAG_TRANSACTION_WRONG_DATA;
            print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 16, 1, RxBuf, RxBufSize, 0);
            if (retry-- < 0)
                break;
            else
                goto lbReadDataAgain;
        }
        else
        {
            if (RxBuf[0] == 0x4b && RxBuf[1] == 0x13)
            {
                switch (RxBuf[2])
                {
                    case 0x0d:  //Close dir
                    case 0x03:  //Close file
                        error_code = *(uint32_t *)&RxBuf[4];
                        break;
                    case 0x0c:  //Read dir
                        error_code = *(uint32_t *)&RxBuf[12];
                        break;
                    case 0x0b:  //Open dir
                    case 0x02:  //Open file
                        error_code = *(uint32_t *)&RxBuf[8];
                        break;
                    case 0x04:  //Read file
                        error_code = *(uint32_t *)&RxBuf[16];
                        break;
                    default:
                        error_code = EFS_ERR_UNKNOWN;
                        break;
                }
            }

            if (error_code)
            {
                if (error_code == EFS_ENOENT)
                {
                    printk(KERN_ERR "No such file or directory," "Cmd:0x%x Error:0x%x\n", *(uint32_t *)&RxBuf[0], error_code);
                    rc = -EFS_DIAG_TRANSACTION_NO_FILE_DIR;
                    break;
                }
                else
                {
                    printk(KERN_ERR "Cmd:0x%x Error:0x%x\n", *(uint32_t *)&RxBuf[0], error_code);
                    rc = -EFS_DIAG_TRANSACTION_CMD_ERROR;
                }
            }
        }
    } while(rc < 0 && retry-- > 0);

    return rc;
}

// efs_page = the index of page in fver which want to read
// efs_buf  = store the read back page of BSP/fver 
int read_efsfver(int efs_page, char *efs_buf)
{
    char src[] = "BSP/fver";

    char req_cmd[EFS2_CMD_REQUEST_SIZE];
    int  req_size;
    char res_dat[EFS2_CMD_RESPONSE_SIZE];
    int  res_size;

    int rc;
    int fd;
    int offset;
    int bEOF;
    char *chr = NULL;
    int len;
    int total;

    char CMD_OpenFile[]  = {0x4b, 0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x01, 0x00, 0x00};
    char CMD_CloseFile[] = {0x4b, 0x13, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
    char CMD_ReadFile[]  = {0x4b, 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//    if (!ftm_smd_driver) {
//        printk(KERN_ERR "efs_fver_read: [ERROR] ftm_smd_driver is NULL\n");
//        return (-3);
//    }

    // open DIAG
//    mutex_lock(&ftm_smd_driver->ftm_smd_mutex);
//    smd_open("DIAG", &ftm_smd_driver->ch, ftm_smd_driver, ftm_smd_notify);
//    mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);

//    if (!ftm_smd_driver->ch) {
//        printk(KERN_ERR "efs_fver_read: [ERROR] ftm_smd_driver->ch is NULL\n");
//        return (-4);
//    }

    if (NULL == efs_buf) {
        printk(KERN_ERR "efs_fver_read: [WARNING] efs_buf is NULL\n");
        printk(KERN_ERR "efs_fver_read: [INFO] return the all byte(s) of %s\n", src);
    }
    printk("efs_fver_read: [INFO] efs_page = %d\n", efs_page);

    // open BSP/fver file
    printk(KERN_ERR "efs_fver_read: [INFO] Open %s (src)\n", src);
    memcpy(req_cmd, CMD_OpenFile, sizeof(CMD_OpenFile));
    memcpy((req_cmd + sizeof(CMD_OpenFile)), src, sizeof(src));
    req_size = sizeof(CMD_OpenFile) + sizeof(src) + 1; // +1 for null end of string
    memset(res_dat, sizeof(res_dat), 0);
    res_size = EFS2_DIAG_OPEN_RES_SIZE;

    rc = efs_diag_func(req_cmd, req_size, res_dat, res_size);
    if (rc < 0) {
        printk(KERN_ERR "efs_fver_read: [ERRPR] Open %s Failed #(-1)\n", src);
        total = (-1);
        goto lbExit_efs_fver_read;
    }
    fd = res_dat[4];
    offset = PAGE_SIZE * efs_page;
    printk("efs_fver_read: [INFO] offset = %d\n", offset);
    bEOF = 0;
    total = 0;

    // read BSP/fver file
    while (!bEOF)
    {
        memset(req_cmd, sizeof(req_cmd), 0);
        memcpy(req_cmd, CMD_ReadFile, sizeof(CMD_ReadFile));
        req_cmd[4] = fd;
        *(uint32_t *) &req_cmd[12] = offset;
        req_size = sizeof(CMD_ReadFile);
        memset(res_dat, sizeof(res_dat), 0);
        res_size = EFS2_CMD_RESPONSE_SIZE;

        rc = efs_diag_func(req_cmd, req_size, res_dat, res_size);
        if (rc < 0) {
            printk(KERN_ERR "efs_fver_read: [ERROR] Read %s Failed #(-2)\n", src);
            total = (-2);
            break;
        }

        offset += EFS2_FILE_READ_SIZE;
        chr = &res_dat[20];
        len = (res_dat[13] << 8) + res_dat[12];
        printk("efs_fver_read: [INFO] len = %d\n", len);
        total += len;

        // write BSP/fver into efs_buf
        if (0 == len)
        {
            bEOF = 1;
        }
        else if (len < EFS2_FILE_READ_SIZE)
        {
            bEOF = 1;
            if ((NULL != efs_buf)&&(total <= PAGE_SIZE)) {
                memcpy((efs_buf+(total-len)), chr, len);
            }
        }
        else
        {
            if ((NULL != efs_buf)&&(total <= PAGE_SIZE)) {
                memcpy((efs_buf+(total-len)), chr, len);
            }
        }

        if ((PAGE_SIZE <= total)&&(NULL != efs_buf)) {
            bEOF = 1;
        }
    } // while (!bEOF)

    // close BSP/fver file
    printk("efs_fver_read: [INFO] Close %s\n", src);
    memset(req_cmd, sizeof(req_cmd), 0);
    memcpy(req_cmd, CMD_CloseFile, sizeof(CMD_CloseFile));
    req_size = sizeof(CMD_CloseFile);
    memset(res_dat, sizeof(res_dat), 0);
    res_size = EFS2_DIAG_CLOSE_RES_SIZE;

    rc = efs_diag_func(req_cmd, req_size, res_dat, res_size);
    if (rc < 0) {
        printk("efs_fver_read: [ERROR] Close %s Failed\n", src);
    }

    printk("efs_fver_read: [INFO] total = %d\n", total);

lbExit_efs_fver_read:

    // close DIAG
//    if (ftm_smd_driver->ch) {
//        mutex_lock(&ftm_smd_driver->ftm_smd_mutex);
//        smd_close(ftm_smd_driver->ch);
//        mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
//    }

    return total;
}
/* FIH, JiaHao, 2011/09/30 } */ /* fver */


static int diag_smd_probe(struct platform_device *pdev)
{
	int r = 0;

//SW2-5-1-MP-DbgCfgTool-03*[
//+{PS3-RR-ON_DEVICE_QXDM-01
#if 0		
	int cfg_val;
	int boot_mode;
	
	boot_mode = fih_read_boot_mode_from_smem();
	
	if(boot_mode != APPS_MODE_FTM)
	{
		r = DbgCfgGetByBit(DEBUG_MODEM_LOGGER_CFG, (int*)&cfg_val);
		if ((r == 0) && (cfg_val == 1) && (boot_mode != APPS_MODE_RECOVERY))
		{
			printk(KERN_INFO "FIH:Embedded QXDM Enabled. %s", __FUNCTION__);
		}
		else
		{
			printk(KERN_ERR "FIH:Fail to call DbgCfgGetByBit(), ret=%d or Embedded QxDM disabled, cfg_val=%d.", r, cfg_val);
			printk(KERN_ERR "FIH:Default: Embedded QXDM Disabled.\n");
			
			if (pdev->id == 0)
				r = smd_open("DIAG", &driver->ch, driver, diag_smd_notify);
		}
	}
	else
	{
		if (pdev->id == 0)
			r = smd_open("DIAG", &driver->ch, driver, diag_smd_notify);
	}
#endif
	if (pdev->id == 0)
	r = smd_open("DIAG", &driver->ch, driver, diag_smd_notify);
#if defined(CONFIG_MSM_N_WAY_SMD)
	if (pdev->id == 1)
		r = smd_named_open_on_edge("DIAG", SMD_APPS_QDSP
			, &driver->chqdsp, driver, diag_smd_qdsp_notify);
#endif
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	printk(KERN_INFO "diag opened SMD port ; r = %d\n", r);

/*
//SW2-5-1-MP-DbgCfgTool-00+[
#if SAVE_MODEM_EFS_LOG
	r = device_create_file(&pdev->dev, &dev_attr_efs2sd);
	printk(KERN_INFO "device create file r=%d\n", r);
	if (r < 0)	{
		dev_err(&pdev->dev, "%s: Create efs2sd attribute \"efs2sd\" failed!! <%d>", __func__, r);
	}

	diag_wq = create_singlethread_workqueue("diag");
	if (diag_wq == 0)
		return -ENOMEM;

#endif
//SW2-5-1-MP-DbgCfgTool-00+]
*/

//Div2D5-LC-BSP-Porting_RecoveryMode-00 +[
#if 0
	r = device_create_file(&pdev->dev, &dev_attr_boot2recovery);

	if (r < 0)
	{
		dev_err(&pdev->dev, "%s: Create nv4719 attribute failed!! <%d>", __func__, r);
	}

	r = device_create_file(&pdev->dev, &dev_attr_turnonofffefslog);

	if (r < 0)
	{
		dev_err(&pdev->dev, "%s: Create turnonofffefslog attribute failed!! <%d>", __func__, r);
	}
#endif
//Div2D5-LC-BSP-Porting_RecoveryMode-00 +]

	return 0;
}

/*
//SW2-5-1-MP-DbgCfgTool-00+[
#if SAVE_MODEM_EFS_LOG
static int diag_smd_remove(struct platform_device *pdev)
{
    int ret = 0;
    device_remove_file(&pdev->dev, &dev_attr_efs2sd);
    return ret;
}
#endif
//SW2-5-1-MP-DbgCfgTool-00+]
*/

static int diagfwd_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_dev_pm_ops = {
	.runtime_suspend = diagfwd_runtime_suspend,
	.runtime_resume = diagfwd_runtime_resume,
};

static struct platform_driver msm_smd_ch1_driver = {

	.probe = diag_smd_probe,
	.driver = {
		   .name = "DIAG",
		   .owner = THIS_MODULE,
		   .pm   = &diagfwd_dev_pm_ops,
		   },
/*
//SW2-5-1-MP-DbgCfgTool-00+[
#if SAVE_MODEM_EFS_LOG
	.remove = diag_smd_remove,
#endif
//SW2-5-1-MP-DbgCfgTool-00+]
*/
};

void diagfwd_init(void)
{
	diag_debug_buf_idx = 0;
	driver->read_len_legacy = 0;
	if (driver->buf_in_1 == NULL)
		driver->buf_in_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_1 == NULL)
			goto err;
	if (driver->buf_in_2 == NULL)
		driver->buf_in_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_2 == NULL)
			goto err;
	if (driver->buf_in_qdsp_1 == NULL)
		driver->buf_in_qdsp_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_qdsp_1 == NULL)
			goto err;
	if (driver->buf_in_qdsp_2 == NULL)
		driver->buf_in_qdsp_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_qdsp_2 == NULL)
			goto err;
	if (driver->usb_buf_out  == NULL &&
	     (driver->usb_buf_out = kzalloc(USB_MAX_OUT_BUF,
					 GFP_KERNEL)) == NULL)
		goto err;
	if (driver->hdlc_buf == NULL
	    && (driver->hdlc_buf = kzalloc(HDLC_MAX, GFP_KERNEL)) == NULL)
		goto err;
	if (driver->msg_masks == NULL
	    && (driver->msg_masks = kzalloc(MSG_MASK_SIZE,
					     GFP_KERNEL)) == NULL)
		goto err;
	if (driver->log_masks == NULL &&
	    (driver->log_masks = kzalloc(LOG_MASK_SIZE, GFP_KERNEL)) == NULL)
		goto err;
	driver->log_masks_length = 8*MAX_EQUIP_ID;
	if (driver->event_masks == NULL &&
	    (driver->event_masks = kzalloc(EVENT_MASK_SIZE,
					    GFP_KERNEL)) == NULL)
		goto err;
	if (driver->client_map == NULL &&
	    (driver->client_map = kzalloc
	     ((driver->num_clients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
	if (driver->buf_tbl == NULL)
			driver->buf_tbl = kzalloc(buf_tbl_size *
			  sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->buf_tbl == NULL)
		goto err;
	if (driver->data_ready == NULL &&
	     (driver->data_ready = kzalloc(driver->num_clients * sizeof(struct
					 diag_client_map), GFP_KERNEL)) == NULL)
		goto err;
	if (driver->table == NULL &&
	     (driver->table = kzalloc(diag_max_registration*
		      sizeof(struct diag_master_table),
		       GFP_KERNEL)) == NULL)
		goto err;
	if (driver->write_ptr_1 == NULL)
		driver->write_ptr_1 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_1 == NULL)
			goto err;
	if (driver->write_ptr_2 == NULL)
		driver->write_ptr_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_2 == NULL)
			goto err;
	if (driver->write_ptr_qdsp_1 == NULL)
		driver->write_ptr_qdsp_1 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_qdsp_1 == NULL)
			goto err;
	if (driver->write_ptr_qdsp_2 == NULL)
		driver->write_ptr_qdsp_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_qdsp_2 == NULL)
			goto err;
	if (driver->usb_read_ptr == NULL)
		driver->usb_read_ptr = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_read_ptr == NULL)
			goto err;
	if (driver->pkt_buf == NULL &&
	     (driver->pkt_buf = kzalloc(PKT_SIZE,
			 GFP_KERNEL)) == NULL)
		goto err;
	if (driver->apps_rsp_buf == NULL)
			driver->apps_rsp_buf = kzalloc(150, GFP_KERNEL);
		if (driver->apps_rsp_buf == NULL)
			goto err;
	driver->diag_wq = create_singlethread_workqueue("diag_wq");
#ifdef CONFIG_DIAG_OVER_USB
	INIT_WORK(&(driver->diag_proc_hdlc_work), diag_process_hdlc_fn);
	INIT_WORK(&(driver->diag_read_work), diag_read_work_fn);
	driver->legacy_ch = usb_diag_open(DIAG_LEGACY, driver,
			diag_usb_legacy_notifier);
	if (IS_ERR(driver->legacy_ch)) {
		printk(KERN_ERR "Unable to open USB diag legacy channel\n");
		goto err;
	}
#ifdef CONFIG_DIAG_SDIO_PIPE
	if (machine_is_msm8x60_charm_surf() || machine_is_msm8x60_charm_ffa())
		diagfwd_sdio_init();
#endif
#endif
	platform_driver_register(&msm_smd_ch1_driver);

	return;
err:
		printk(KERN_INFO "\n Could not initialize diag buffers\n");
		kfree(driver->buf_in_1);
		kfree(driver->buf_in_2);
		kfree(driver->buf_in_qdsp_1);
		kfree(driver->buf_in_qdsp_2);
		kfree(driver->usb_buf_out);
		kfree(driver->hdlc_buf);
		kfree(driver->msg_masks);
		kfree(driver->log_masks);
		kfree(driver->event_masks);
		kfree(driver->client_map);
		kfree(driver->buf_tbl);
		kfree(driver->data_ready);
		kfree(driver->table);
		kfree(driver->pkt_buf);
		kfree(driver->write_ptr_1);
		kfree(driver->write_ptr_2);
		kfree(driver->write_ptr_qdsp_1);
		kfree(driver->write_ptr_qdsp_2);
		kfree(driver->usb_read_ptr);
		kfree(driver->apps_rsp_buf);
		if (driver->diag_wq)
			destroy_workqueue(driver->diag_wq);
}

void diagfwd_exit(void)
{
	smd_close(driver->ch);
	smd_close(driver->chqdsp);
	driver->ch = 0;		/*SMD can make this NULL */
	driver->chqdsp = 0;
#ifdef CONFIG_DIAG_OVER_USB
	if (driver->usb_connected)
		usb_diag_free_req(driver->legacy_ch);
#endif
	platform_driver_unregister(&msm_smd_ch1_driver);
#ifdef CONFIG_DIAG_OVER_USB
	usb_diag_close(driver->legacy_ch);
#endif

	kfree(driver->buf_in_1);
	kfree(driver->buf_in_2);
	kfree(driver->buf_in_qdsp_1);
	kfree(driver->buf_in_qdsp_2);
	kfree(driver->usb_buf_out);
	kfree(driver->hdlc_buf);
	kfree(driver->msg_masks);
	kfree(driver->log_masks);
	kfree(driver->event_masks);
	kfree(driver->client_map);
	kfree(driver->buf_tbl);
	kfree(driver->data_ready);
	kfree(driver->table);
	kfree(driver->pkt_buf);
	kfree(driver->write_ptr_1);
	kfree(driver->write_ptr_2);
	kfree(driver->write_ptr_qdsp_1);
	kfree(driver->write_ptr_qdsp_2);
	kfree(driver->usb_read_ptr);
	kfree(driver->apps_rsp_buf);
	destroy_workqueue(driver->diag_wq);
}
