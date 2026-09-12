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
 * On-board crash dump for mr_vmu_rt1176: a custom Zephyr `coredump` backend
 * (CONFIG_DEBUG_COREDUMP_BACKEND_OTHER), the ChibiOS-parity target being
 * CrashCatcher / AP_HAL_ChibiOS/hwdef/common/crashdump.c.
 *
 * WHY NOT CRASHCATCHER DIRECTLY: its capture engine
 * (modules/CrashDebug/CrashCatcher/Core/src/CrashCatcher_armv7m.S) is
 * hand-written ARMv7-M hard-fault-vector assembly that expects to own the
 * vector table. Zephyr already owns fault entry via its own portable
 * z_fatal_error() -> k_sys_fatal_error_handler() path (already used by
 * ap_fault_handler.c on this board) - fighting CrashCatcher for the vector
 * would mean re-deriving Zephyr's own fault plumbing from scratch. Zephyr's
 * `coredump` subsystem is the architecture-independent equivalent instead:
 * kernel/fatal.c's z_fatal_error() calls coredump() immediately BEFORE
 * k_sys_fatal_error_handler(), so this backend and ap_fault_handler.c's
 * handler coexist with zero conflict.
 *
 * WHY A CUSTOM BACKEND, NOT THE STOCK coredump_backend_flash_partition.c:
 * that backend needs CONFIG_FLASH + CONFIG_FLASH_MAP + CONFIG_STREAM_FLASH,
 * all deliberately =n on this board (see prj.mr_vmu_rt1176.conf) - Zephyr's
 * FlexSPI NOR driver reconfigures the controller the CPU is executing
 * through and traps the core in BootROM (see rt1176_romapi_flash.c's own
 * header comment). This backend reuses those same ROM-API primitives
 * Storage.cpp already proved instead.
 *
 * ============================================================================
 * WATCHDOG DANGER - read this before changing the erase/write footprint.
 * ============================================================================
 * z_fatal_error() calls arch_irq_lock() and holds it for the ENTIRE
 * duration of the coredump() call, i.e. across this backend's .start(),
 * every .buffer_output(), and .end(). NO thread can run in that window -
 * not even the monitor thread that feeds the hardware watchdog
 * (Scheduler.cpp's watchdog_pat() path). The watchdog is armed
 * unconditionally on this board (CONFIG_WATCHDOG=y, CONFIG_WDT_DISABLE_AT_
 * BOOT=n) with a 2000ms timeout - left unfed for that long, THE WATCHDOG
 * ITSELF WILL RESET THE BOARD MID-DUMP, destroying the exact artifact this
 * backend exists to preserve.
 *
 * rt1176_flash_erase()'s own per-4KB-chunk irq_lock()/irq_unlock() pairs
 * (rt1176_romapi_flash.c) do NOT rescue this: irq_unlock(key) restores
 * whatever state was current when THAT irq_lock() was taken. Since
 * z_fatal_error() already disabled interrupts before calling in here, the
 * "restored" state after each chunk is still "disabled" - the chunking only
 * shortens each locked interval when called from ordinary thread context
 * (which is the case for every other rt1176_flash_erase() caller, e.g.
 * Storage.cpp's normal writes). Inside a fault handler it buys nothing.
 *
 * The only real safety margin here is keeping the TOTAL erased+programmed
 * footprint tiny by construction, and explicitly refreshing the watchdog
 * ourselves at the two points where this backend is guaranteed to run
 * (start and end) - mirroring AP_HAL_ChibiOS/hwdef/common/crashdump.c,
 * which calls stm32_watchdog_pat() between its own write chunks for the
 * identical reason. rt1176_coredump_wdt_pat() below bypasses Zephyr's
 * watchdog device/driver machinery entirely (WDOG_Refresh() is a direct
 * two-word MMIO write, the same primitive Scheduler.cpp's monitor thread
 * ultimately calls via wdt_feed() - see wdt_mcux_imx_wdog.c) so this file
 * has no dependency on any other subsystem being in a sane state during a
 * fault.
 *
 * Budget: ONE 4KB erase (RT1176_COREDUMP_ERASE_SIZE, ~30-50ms measured
 * elsewhere in this tree for a chunk this size - see rt1176_romapi_flash.c)
 * plus however many 256B page-program calls the payload needs (a handful,
 * each on the order of single-digit ms). Total well under 200ms against a
 * 2000ms timeout. This budget depends on CONFIG_DEBUG_COREDUMP_MEMORY_
 * DUMP_MIN and a bounded CONFIG_DEBUG_COREDUMP_THREAD_STACK_TOP_LIMIT_FOR_
 * CURRENT (see prj.mr_vmu_rt1176.conf) - re-check it before switching to a
 * richer dump mode (_THREADS / _LINKER_RAM dump the whole system and would
 * blow this budget by orders of magnitude).
 *
 * NOT YET DONE: a genuine hardware test of a real fault -> reset ->
 * @SYS/crash_dump.bin retrieval cycle. Per this project's own doctrine nothing
 * here counts as verified until that specific test has been run - this file
 * builds and its logic has been reasoned through carefully (given this
 * project's own history of ROM-API flash mistakes bricking this exact board),
 * but "reasoned through carefully" is not "hardware-verified".
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

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/debug/coredump.h>
#include <errno.h>

#include <fsl_wdog.h>

#include "rt1176_coredump.h"
#include "rt1176_romapi_flash.h"

static uint32_t total_written;   /* bytes of payload committed so far */
static bool     backend_ok;      /* erase succeeded, programming still allowed */
static bool     truncated;       /* payload exceeded RT1176_COREDUMP_PAYLOAD_MAX */

/* Direct MMIO watchdog refresh - no Zephyr device or thread machinery, because
 * this runs from the fault handler. */
static inline void rt1176_coredump_wdt_pat(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdog1), okay)
	WDOG_Refresh((WDOG_Type *)DT_REG_ADDR(DT_NODELABEL(wdog1)));
#endif
}

static void rt1176_coredump_start(void)
{
	total_written = 0;
	truncated = false;

	/* rt1176_flash_init() is idempotent (romapi_ensure_init() guards on a
	 * static flag) - safe to call again even though Storage.cpp already
	 * called it once during normal boot. */
	backend_ok = (rt1176_flash_init() == 0) &&
		     (rt1176_flash_erase(RT1176_COREDUMP_PARTITION_OFFSET,
					  RT1176_COREDUMP_ERASE_SIZE) == 0);

	rt1176_coredump_wdt_pat();
}

static void rt1176_coredump_buffer_output(uint8_t *buf, size_t buflen)
{
	if (!backend_ok || truncated || buflen == 0) {
		return;
	}

	if (buflen > (size_t)(RT1176_COREDUMP_PAYLOAD_MAX - total_written)) {
		/* Over budget: stop accepting more rather than program past the
		 * one sector we erased. Keep what's already committed - a
		 * truncated dump is still a diagnosable artifact, and refusing
		 * to write past erased flash is the only way to avoid either
		 * corrupting an adjacent sector or faulting again in here. */
		truncated = true;
		return;
	}

	const uint32_t offset = RT1176_COREDUMP_PARTITION_OFFSET +
				 RT1176_COREDUMP_HDR_SIZE + total_written;

	if (rt1176_flash_program(offset, buf, (uint32_t)buflen) != 0) {
		backend_ok = false;
		return;
	}

	total_written += (uint32_t)buflen;
}

static void rt1176_coredump_end(void)
{
	if (backend_ok && total_written > 0) {
		const struct rt1176_coredump_hdr hdr = {
			.magic = RT1176_COREDUMP_MAGIC,
			.size = total_written,
		};

		(void)rt1176_flash_program(RT1176_COREDUMP_PARTITION_OFFSET,
					    (const uint8_t *)&hdr, sizeof(hdr));
	}

	rt1176_coredump_wdt_pat();
}

static int rt1176_coredump_query(enum coredump_query_id query_id, void *arg)
{
	ARG_UNUSED(arg);

	const struct rt1176_coredump_hdr *hdr = (const struct rt1176_coredump_hdr *)
		(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + RT1176_COREDUMP_PARTITION_OFFSET);
	const bool have_dump = (hdr->magic == RT1176_COREDUMP_MAGIC) && (hdr->size > 0);

	switch (query_id) {
	case COREDUMP_QUERY_GET_ERROR:
		return backend_ok ? 0 : -EIO;
	case COREDUMP_QUERY_HAS_STORED_DUMP:
		return have_dump ? 1 : 0;
	case COREDUMP_QUERY_GET_STORED_DUMP_SIZE:
		return have_dump ? (int)hdr->size : 0;
	default:
		return -ENOTSUP;
	}
}

static int rt1176_coredump_cmd(enum coredump_cmd_id cmd_id, void *arg)
{
	ARG_UNUSED(cmd_id);
	ARG_UNUSED(arg);

	/* Not implemented: retrieval goes through Zephyr::Util::
	 * last_crash_dump_size()/last_crash_dump_ptr() reading the XIP-mapped
	 * partition directly (Util.cpp), the same pattern Storage.cpp already
	 * uses for its own reads - no ROM call needed, so no need to route
	 * retrieval through this command interface at all. */
	return -ENOTSUP;
}

struct coredump_backend_api coredump_backend_other = {
	.start = rt1176_coredump_start,
	.end = rt1176_coredump_end,
	.buffer_output = rt1176_coredump_buffer_output,
	.query = rt1176_coredump_query,
	.cmd = rt1176_coredump_cmd,
};
