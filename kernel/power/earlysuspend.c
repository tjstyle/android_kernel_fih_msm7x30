/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"

//Div2-SW2-BSP-pmlog, HenryMCWang +
#include "linux/pmlog.h"
#include "mach/fih_msm_battery.h"
//Div2-SW2-BSP-pmlog, HenryMCWang -

/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/kallsyms.h>
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

/* FIHTDC, Div2-SW2-BSP, Penho, fast dormancy { */
#ifdef CONFIG_FIH_FAST_DORMANCY
extern void open_dorm_fifo(void);
#endif	// CONFIG_FIH_FAST_DORMANCY
/* } FIHTDC, Div2-SW2-BSP, Penho, fast dormancy */

/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
struct task_struct *gts_suspend = NULL;
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
	ktime_t calltime, delta, rettime;
	unsigned long long duration;
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */

#if defined(CONFIG_FIH_POWER_LOG) && defined(CONFIG_BATTERY_FIH_MSM)
	struct batt_info_interface* batt_info_if = get_batt_info_if();
#endif

/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
	gts_suspend = current;
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");

//Div2-SW2-BSP-pmlog, HenryMCWang +
#if defined(CONFIG_FIH_POWER_LOG) && defined(CONFIG_BATTERY_FIH_MSM)
	pmlog("early_suspend(): batt capacity=%dmAh, usb charging type=%d\n", batt_info_if->get_batt_voltage()/1000, batt_info_if->get_chg_source());
#endif
//Div2-SW2-BSP-pmlog, HenryMCWang -
	
	list_for_each_entry(pos, &early_suspend_handlers, link) {
/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
		if (pos->suspend != NULL) {
			calltime = ktime_get();
			print_symbol("early_suspend function: %s\n", (unsigned long)pos->suspend);

			pos->suspend(pos);

			rettime = ktime_get();
			delta = ktime_sub(rettime, calltime);
			duration = (unsigned long long) ktime_to_ns(delta) >> 10;
			pr_info("takes %Ld usecs\n", duration);
		}
#else	// CONFIG_FIH_SUSPEND_RESUME_LOG
		if (pos->suspend != NULL)
			pos->suspend(pos);
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */
	}
	mutex_unlock(&early_suspend_lock);

/* FIHTDC, Div2-SW2-BSP, Penho, fast dormancy { */
#ifdef CONFIG_FIH_FAST_DORMANCY
	open_dorm_fifo();
#endif	// CONFIG_FIH_FAST_DORMANCY
/* } FIHTDC, Div2-SW2-BSP, Penho, fast dormancy */


/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
	calltime = ktime_get();
	suspend_sys_sync_queue();
	rettime = ktime_get();
	delta = ktime_sub(rettime, calltime);
	duration = (unsigned long long) ktime_to_ns(delta) >> 10;
	pr_info("early suspend sync: takes %Ld usecs\n", duration);
#else	// CONFIG_FIH_SUSPEND_RESUME_LOG
	suspend_sys_sync_queue();
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
	ktime_t calltime, delta, rettime;
	unsigned long long duration;
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG
/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */

#if defined(CONFIG_FIH_POWER_LOG) && defined(CONFIG_BATTERY_FIH_MSM)
	struct batt_info_interface* batt_info_if = get_batt_info_if();
#endif

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
/* FIHTDC, Div2-SW2-BSP, Penho, SuspendLog { */
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
		if (pos->resume != NULL) {
			calltime = ktime_get();
			print_symbol("late_resume function: %s\n", (unsigned long)pos->resume);

			pos->resume(pos);

			rettime = ktime_get();
			delta = ktime_sub(rettime, calltime);
			duration = (unsigned long long) ktime_to_ns(delta) >> 10;
			pr_info("takes %Ld usecs\n", duration);
		}
#else	// CONFIG_FIH_SUSPEND_RESUME_LOG
		if (pos->resume != NULL)
			pos->resume(pos);
#endif	// CONFIG_FIH_SUSPEND_RESUME_LOG

//Div2-SW2-BSP-pmlog, HenryMCWang +
#if defined(CONFIG_FIH_POWER_LOG) && defined(CONFIG_BATTERY_FIH_MSM)
	pmlog("late_resume(): batt capacity=%dmAh, usb charging type=%d\n", batt_info_if->get_batt_voltage()/1000, batt_info_if->get_chg_source());
#endif
//Div2-SW2-BSP-pmlog, HenryMCWang -

/* } FIHTDC, Div2-SW2-BSP, Penho, SuspendLog */
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
