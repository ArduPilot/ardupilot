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
  Tools/scripts/zephyr_read_fatal.py reads the g_ap_fatal_* symbols over SWD.
  The boot-stage checkpoint tracing that file also carried is optional and now
  lives in boot_checkpoints.c.
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/fatal.h>

#include "ap_diag_console.h"

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
   Tools/scripts/zephyr_read_fatal.py reads these five symbols by name. */
#if defined(CONFIG_CPU_CORTEX_M)
volatile unsigned int g_ap_fatal_reason;
volatile uint32_t g_ap_fatal_pc;
volatile uint32_t g_ap_fatal_lr;
volatile uint32_t g_ap_fatal_cfsr;
volatile uint32_t g_ap_fatal_count;

/* Crash-forensics bridge into the C++ HAL (AP_HAL_Zephyr/Scheduler.cpp). */
#ifndef AP_ZEPHYR_BOOTLOADER_BUILD
extern void ap_persistent_save_fault(uint16_t line, uint8_t fault_type,
				     uint32_t fault_addr, uint32_t fault_lr,
				     uint32_t fault_icsr);
#endif

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	g_ap_fatal_reason = reason;
	g_ap_fatal_pc = esf ? esf->basic.pc : 0;
	g_ap_fatal_lr = esf ? esf->basic.lr : 0;
	g_ap_fatal_cfsr = *(volatile uint32_t *)0xE000ED28;  // ARM SCB CFSR
	__asm__ volatile("dsb");
	g_ap_fatal_count++;

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
#endif  /* CONFIG_CPU_CORTEX_M */
