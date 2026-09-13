/*
 * Veto WFI in Zephyr's idle thread.
 *
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
 */

/* Veto WFI so Zephyr's idle thread never sleeps, matching ChibiOS-ArduPilot:
 * there CORTEX_ENABLE_WFI_IDLE defaults FALSE and ArduPilot never sets it TRUE,
 * so port_wait_for_interrupt() is an empty function and the flying firmware
 * never issues the instruction at all. Both boards this HAL supports follow
 * that, for that reason.
 *
 * Each also has a hardware reason found before that was understood. On
 * mr_vmu_rt1176 a WFI gates the whole CM7 clock domain - SysTick and DWT both
 * freeze - so the kernel clock stops whenever all threads sleep. On
 * CubeOrangeZephyr it looked like the same fault through TIM5, the clock behind
 * micros(), but was not: WFI does not stop TIM5. The H7 gates Sleep-mode clocks
 * through RCC_APB1LLPENR, separately from the Run-mode gate in RCC_APB1LENR, and
 * with TIM5LPEN set the counter runs straight through WFI. That bit comes out of
 * reset set; something earlier in this board's boot chain had cleared it.
 * hrt_init() now sets it explicitly, so that board's time base survives WFI -
 * which changes nothing here, because we still do not sleep.
 *
 * The veto used to be a bare `return false`, which sends arch_cpu_idle() back
 * to the kernel's idle loop for another pass: per pass a cpsid, two BASEPRI
 * writes, three isb, a cpsie, a call and a return - 13 instructions, four of
 * them barriers. On silicon that is merely a busy idle thread. Under Renode it
 * is the single most expensive thing the guest does: every barrier ends a
 * translation block, so the emulator re-dispatches four times per 13
 * instructions, and the idle thread is 80 % of the guest's virtual time
 * (threads-*-renode-*.txt at the repo root). The emulated CubeOrange ran at
 * 1/9.5 real time, with ~7.6 of every 9.5 host seconds spent in this loop.
 *
 * So: unmask interrupts, spin 256 nops, and then return false exactly as
 * before. The kernel's idle loop still gets control back every pass - if a
 * board ever turns on CONFIG_PM its hooks still run - but the expensive pass
 * is amortised over 269 instructions instead of 13, and interrupts are open
 * for the spin instead of masked at the top of every pass. A never-returning
 * variant (one 257-instruction block, about a tenth cheaper again for the
 * emulator) was measured first and set aside: it works only because nothing
 * in the idle loop needs control back today.
 *
 * Renode could go further still by executing the real WFI - patching this
 * function's first two halfwords to `movs r0, #1; bx lr` in the emulated image
 * only - and skipping idle time altogether. Not done: the 2026-09-10 record of
 * a WFI-enabled emulated guest with dt short by its idle fraction has to be
 * understood first (TODO 2.23). */

#include <stdbool.h>

bool z_arm_on_enter_cpu_idle(void)
{
	/* Entered with PRIMASK set (arch_cpu_idle() did cpsid i) and BASEPRI 0.
	   arch_cpu_idle() runs its own cpsie/isb after we return, so opening
	   interrupts here early is harmless there and lets an interrupt land
	   anywhere in the spin. */
	__asm__ volatile(
		"cpsie i\n\t"
		".rept 256\n\t"
		"nop\n\t"
		".endr\n\t"
		::: "memory");
	return false;
}
