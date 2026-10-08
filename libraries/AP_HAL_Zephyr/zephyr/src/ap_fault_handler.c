/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
/*
  Fault handling and the synchronous console it reports through.

  Split out of the old uart_probe_diag.c, which began as throwaway RT1176
  boot-hang scaffolding and was never removed. Every board needs what is here:
  Zephyr calls k_sys_fatal_error_handler() on any fault, Scheduler.cpp reads
  the fault back through ap_persistent_save_fault(), and
  Tools/zephyr/zephyr_read_fatal.py reads the g_ap_fatal_* symbols over SWD.
  The boot-stage checkpoint tracing that file also carried is optional and now
  lives in boot_checkpoints.c.
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/fatal.h>

#include "ap_diag_console.h"
#include "ap_hooks.h"

static const struct device *console_dev;

/* PRE_KERNEL_2/0: earliest point the console device is available, and ahead of
   every checkpoint and driver init that may want to report. The fault handler
   depends on this having run, so it lives here and not with the checkpoints. */
static int ap_diag_console_init(void)
{
	console_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_console));
	return 0;
}
SYS_INIT(ap_diag_console_init, PRE_KERNEL_2, 0);

void ap_diag_puts(const char *s)
{
	if (!console_dev || !device_is_ready(console_dev)) {
		return;
	}
	for (const char *p = s; *p; p++) {
		uart_poll_out(console_dev, (unsigned char)*p);
	}
	uart_poll_out(console_dev, '\r');
	uart_poll_out(console_dev, '\n');
}

void ap_diag_puthex(const char *label, uint32_t val)
{
	if (!console_dev || !device_is_ready(console_dev)) {
		return;
	}
	for (const char *p = label; *p; p++) {
		uart_poll_out(console_dev, (unsigned char)*p);
	}
	uart_poll_out(console_dev, '0');
	uart_poll_out(console_dev, 'x');
	for (int i = 28; i >= 0; i -= 4) {
		uint8_t nib = (val >> i) & 0xF;
		uart_poll_out(console_dev, nib < 10 ? ('0' + nib) : ('A' + nib - 10));
	}
	uart_poll_out(console_dev, '\r');
	uart_poll_out(console_dev, '\n');
}

/* Crash forensics, readable over SWD when the fault handler cannot print.
   Tools/zephyr/zephyr_read_fatal.py reads these five symbols by name. */
#if defined(CONFIG_CPU_CORTEX_M)
/* __noinit so they SURVIVE the watchdog reset the fault leads to.
   They used to be ordinary .bss, zeroed on the way back up, so the only way to
   read them was over SWD while the board sat halted. With no probe on the bench
   that left the persistent-data WDG statustext as the sole channel, and that
   carries neither a usable PC nor a guarantee of being the LATEST fault (see
   ap_persistent_save_fault()). g_ap_fatal_magic separates "we wrote this" from
   uninitialised RAM, the same trick the bootloader's g_jump_* record uses. */
#define AP_FATAL_MAGIC 0x46415441u   /* 'FATA' */
__noinit volatile uint32_t g_ap_fatal_magic;
__noinit volatile unsigned int g_ap_fatal_reason;
__noinit volatile uint32_t g_ap_fatal_pc;
__noinit volatile uint32_t g_ap_fatal_lr;
__noinit volatile uint32_t g_ap_fatal_cfsr;
__noinit volatile uint32_t g_ap_fatal_count;
__noinit volatile uint32_t g_ap_fatal_icsr;
__noinit volatile uint32_t g_ap_fatal_thd_prio;

/* WATCHDOG STALL RECORD. A watchdog reset is not a fault: no exception is
   taken, so the record above stays empty and AP's WDG line reads FT0 FLR0
   FICSR0 - which says only "the main loop stopped patting", never why. The
   hardware watchdog raises an interrupt about half a period before it resets
   the SoC, and that interrupt is the one chance to look at the system while
   it is still stuck. Observed on mr_vmu_rt1176 as resets at 88 s to 190 s
   with FT0 every time, even with the whole fault path resident in ITCM.

   Same __noinit + magic scheme as the fault record, and reported the same way
   by the monitor thread once a GCS is listening. */
#define AP_WDG_MAGIC 0x57444721u   /* 'WDG!' */
__noinit volatile uint32_t g_ap_wdg_magic;
__noinit volatile uint32_t g_ap_wdg_stall_ms;    /* since the last main-loop pat */
__noinit volatile int32_t  g_ap_wdg_sched_task;  /* -1 = between tasks */
__noinit volatile uint32_t g_ap_wdg_cur_prio;    /* thread the interrupt hit */
__noinit volatile uint32_t g_ap_wdg_main_state;  /* main's Zephyr state bits */
__noinit volatile uint32_t g_ap_wdg_main_pended; /* wait queue main is blocked on */
__noinit volatile uint32_t g_ap_wdg_main_pc;     /* main's PC, from its own stack */
__noinit volatile char     g_ap_wdg_cur_name[12];

void ap_wdg_record_put(uint32_t stall_ms, int32_t sched_task, uint32_t cur_prio,
		       const char *cur_name, uint32_t main_state,
		       uint32_t main_pended, uint32_t main_pc)
{
	g_ap_wdg_stall_ms    = stall_ms;
	g_ap_wdg_sched_task  = sched_task;
	g_ap_wdg_cur_prio    = cur_prio;
	g_ap_wdg_main_state  = main_state;
	g_ap_wdg_main_pended = main_pended;
	g_ap_wdg_main_pc     = main_pc;
	unsigned int i = 0;
	if (cur_name != NULL) {
		for (; i < sizeof(g_ap_wdg_cur_name) - 1U && cur_name[i] != '\0'; i++) {
			g_ap_wdg_cur_name[i] = cur_name[i];
		}
	}
	g_ap_wdg_cur_name[i] = '\0';
	__asm__ volatile("dsb");
	g_ap_wdg_magic = AP_WDG_MAGIC;   /* last, so a torn record is not believed */
	__asm__ volatile("dsb");
}

bool ap_wdg_record_take(uint32_t *stall_ms, int32_t *sched_task,
			uint32_t *cur_prio, char *cur_name, size_t cur_name_len,
			uint32_t *main_state, uint32_t *main_pended,
			uint32_t *main_pc)
{
	if (g_ap_wdg_magic != AP_WDG_MAGIC) {
		return false;
	}
	*stall_ms    = g_ap_wdg_stall_ms;
	*sched_task  = g_ap_wdg_sched_task;
	*cur_prio    = g_ap_wdg_cur_prio;
	*main_state  = g_ap_wdg_main_state;
	*main_pended = g_ap_wdg_main_pended;
	*main_pc     = g_ap_wdg_main_pc;
	unsigned int i = 0;
	if (cur_name != NULL && cur_name_len > 0U) {
		for (; i < cur_name_len - 1U && i < sizeof(g_ap_wdg_cur_name) - 1U &&
		       g_ap_wdg_cur_name[i] != '\0'; i++) {
			cur_name[i] = g_ap_wdg_cur_name[i];
		}
		cur_name[i] = '\0';
	}
	g_ap_wdg_magic = 0;   /* report once per reset */
	return true;
}

/* Crash-forensics bridge into the C++ HAL (AP_HAL_Zephyr/Scheduler.cpp):
   ap_persistent_save_fault(), declared in ap_hooks.h. */

bool ap_fault_record_peek(unsigned int *reason, uint32_t *pc, uint32_t *lr,
			  uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
			  uint32_t *count)
{
	if (g_ap_fatal_magic != AP_FATAL_MAGIC) {
		return false;
	}
	*reason   = g_ap_fatal_reason;
	*pc       = g_ap_fatal_pc;
	*lr       = g_ap_fatal_lr;
	*cfsr     = g_ap_fatal_cfsr;
	*icsr     = g_ap_fatal_icsr;
	*thd_prio = g_ap_fatal_thd_prio;
	*count    = g_ap_fatal_count;
	return true;   /* left in place: the WDG line and the monitor both want it */
}

bool ap_fault_record_take(unsigned int *reason, uint32_t *pc, uint32_t *lr,
			  uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
			  uint32_t *count)
{
	if (g_ap_fatal_magic != AP_FATAL_MAGIC) {
		return false;
	}
	*reason   = g_ap_fatal_reason;
	*pc       = g_ap_fatal_pc;
	*lr       = g_ap_fatal_lr;
	*cfsr     = g_ap_fatal_cfsr;
	*icsr     = g_ap_fatal_icsr;
	*thd_prio = g_ap_fatal_thd_prio;
	*count    = g_ap_fatal_count;
	g_ap_fatal_magic = 0;   /* report once per reset */
	return true;
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	g_ap_fatal_reason = reason;
	g_ap_fatal_pc = esf ? esf->basic.pc : 0;
	g_ap_fatal_lr = esf ? esf->basic.lr : 0;
	g_ap_fatal_cfsr = *(volatile uint32_t *)0xE000ED28;  // ARM SCB CFSR
	g_ap_fatal_icsr = *(volatile uint32_t *)0xE000ED04;  // ARM SCB ICSR
	{
		k_tid_t tid = k_current_get();
		g_ap_fatal_thd_prio = (tid != NULL) ? (uint32_t)k_thread_priority_get(tid)
						    : 0xFFFFFFFFu;
	}
	__asm__ volatile("dsb");
	if (g_ap_fatal_magic != AP_FATAL_MAGIC) {
		g_ap_fatal_count = 0;        /* first fault since a cold boot */
	}
	g_ap_fatal_count++;
	g_ap_fatal_magic = AP_FATAL_MAGIC;
	__asm__ volatile("dsb");

#ifndef AP_ZEPHYR_BOOTLOADER_BUILD
	/* fault_addr = faulting PC, fault_icsr = SCB->ICSR (0xE000ED04), matching
	   the fields ChibiOS's save_fault_watchdog() records. reason is Zephyr's
	   k_fatal_error code, carried in the fault_type slot. */
	ap_persistent_save_fault(0, (uint8_t)reason,
				 esf ? esf->basic.pc : 0,
				 esf ? esf->basic.lr : 0,
				 *(volatile uint32_t *)0xE000ED04);
#endif

	ap_diag_puts("### FATAL ERROR ###");
	ap_diag_puthex("reason=", reason);
	if (esf) {
		ap_diag_puthex("pc=", esf->basic.pc);
		ap_diag_puthex("lr=", esf->basic.lr);
		ap_diag_puthex("r0=", esf->basic.a1);
		ap_diag_puthex("xpsr=", esf->basic.xpsr);
	}
	ap_diag_puthex("CFSR=", *(volatile uint32_t *)0xE000ED28);   // ARM SCB CFSR
	ap_diag_puthex("HFSR=", *(volatile uint32_t *)0xE000ED2C);   // ARM SCB HFSR
	ap_diag_puthex("MMFAR=", *(volatile uint32_t *)0xE000ED34); // ARM SCB MMFAR
	ap_diag_puthex("BFAR=", *(volatile uint32_t *)0xE000ED38);  // ARM SCB BFAR
	ap_diag_puts("### HALTING (not resetting) ###");
	arch_irq_lock();
	for (;;) {
		/* spin so state is inspectable / message is the last thing sent */
	}
}
#else
/* No persistent fault/watchdog record outside Cortex-M: the callers in
   Scheduler.cpp see "nothing recorded". */
void ap_wdg_record_put(uint32_t stall_ms, int32_t sched_task, uint32_t cur_prio,
                       const char *cur_name, uint32_t main_state,
                       uint32_t main_pended, uint32_t main_pc)
{
}

bool ap_wdg_record_take(uint32_t *stall_ms, int32_t *sched_task,
                        uint32_t *cur_prio, char *cur_name, size_t cur_name_len,
                        uint32_t *main_state, uint32_t *main_pended,
                        uint32_t *main_pc)
{
        return false;
}

bool ap_fault_record_peek(unsigned int *reason, uint32_t *pc, uint32_t *lr,
                          uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
                          uint32_t *count)
{
        return false;
}

bool ap_fault_record_take(unsigned int *reason, uint32_t *pc, uint32_t *lr,
                          uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
                          uint32_t *count)
{
        return false;
}
#endif  /* CONFIG_CPU_CORTEX_M */

