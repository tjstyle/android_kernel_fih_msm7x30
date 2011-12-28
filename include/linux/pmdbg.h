#ifndef _PMDBG_H_
#define _PMDBG_H_

enum {
#ifdef CONFIG_FIH_POWER_LOG
	MSM_PM_DEBUG_FIH_MODULE = 1U << 7,		/* 128 */ /* fast dormancy, packet filter */
	MSM_PM_DEBUG_FIH_PM = 1U << 8,			/* 256 */ /* wakeup irq */
	MSM_PM_DEBUG_FIH_CALL_STACK = 1U << 9,	/* 512 */ /* clock, irq */
#endif	// CONFIG_FIH_POWER_LOG
};

extern int msm_pm_debug_mask;

#define PMDBG(mask, msg ...) \
	do { \
		if ((mask) & msm_pm_debug_mask) \
			printk(KERN_INFO msg); \
	} while (0)

extern int g_fih_trace_pcclk;
extern int g_fih_trace_irq;

#define PMDBG_INIT_T_PCCLK	-1
#define PMDBG_INIT_T_IRQ	-1

#endif	// _PMDBG_H_
