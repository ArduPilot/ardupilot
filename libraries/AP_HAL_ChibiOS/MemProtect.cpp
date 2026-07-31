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
 */

#include "MemProtect.h"

#if AP_BOARDCONFIG_MCU_MEMPROTECT_TRACE_ENABLED

#include <AP_Common/AP_Common.h>
#include <ch.h>
#include "hal.h"

#if defined(PORT_ENABLE_GUARD_PAGES) && PORT_ENABLE_GUARD_PAGES == TRUE
#error "MEMPROTECT_TRACE needs MPU region 7, used by the ChibiOS stack guard"
#endif
#if defined(STM32_NOCACHE_MPU_REGION_1) && STM32_NOCACHE_MPU_REGION_1 == MPU_REGION_7
#error "MEMPROTECT_TRACE needs MPU region 7, used by STM32_NOCACHE_MPU_REGION_1"
#endif
#if defined(STM32_NOCACHE_MPU_REGION_2) && STM32_NOCACHE_MPU_REGION_2 == MPU_REGION_7
#error "MEMPROTECT_TRACE needs MPU region 7, used by STM32_NOCACHE_MPU_REGION_2"
#endif
#if defined(STM32_NOCACHE_MPU_REGION) && STM32_NOCACHE_MPU_REGION == MPU_REGION_7
#error "MEMPROTECT_TRACE needs MPU region 7, used by STM32_NOCACHE_MPU_REGION"
#endif
#if defined(STM32_NOCACHE_MPU_REGION_ETH) && STM32_NOCACHE_MPU_REGION_ETH == MPU_REGION_7
#error "MEMPROTECT_TRACE needs MPU region 7, used by STM32_NOCACHE_MPU_REGION_ETH"
#endif

extern "C" {

struct memprotect_state_t memprotect_state;

// bounds of the image in flash, used to spot plausible return addresses.
// _vectors rather than FLASH_LOAD_ADDRESS as the latter is wrong for the
// external flash link scripts
extern uint32_t _vectors;
extern uint32_t _etext;

extern stkalign_t __main_stack_base__;
extern stkalign_t __main_stack_end__;
extern stkalign_t __main_thread_stack_base__;
extern stkalign_t __main_thread_stack_end__;

// the MemManage status bits we know how to recover from
#define MEMPROTECT_OK_BITS (SCB_CFSR_DACCVIOL_Msk | SCB_CFSR_MMARVALID_Msk)

// every MemManage status bit
#define MEMPROTECT_MM_BITS (SCB_CFSR_IACCVIOL_Msk  | SCB_CFSR_DACCVIOL_Msk | \
                            SCB_CFSR_MUNSTKERR_Msk | SCB_CFSR_MSTKERR_Msk  | \
                            SCB_CFSR_MLSPERR_Msk   | SCB_CFSR_MMARVALID_Msk)

void memprotect_arm(void)
{
    /*
      set armed before enabling the hardware. The handler treats a fault while
      !armed as fatal, so the other order would leave a window where a real
      null write crashes the vehicle
     */
    memprotect_state.armed = true;
    __DMB();
    /*
      NON_CACHEABLE gives this Normal memory rather than the Strongly-ordered
      that a bare AP_NA_NA would. It matters because we resume the faulting
      access: unaligned accesses and ones spanning the end of the region are
      UNPREDICTABLE if the two halves have different memory types
     */
    mpuConfigureRegion(MPU_REGION_7,
                       0x0,
                       MPU_RASR_ATTR_AP_NA_NA |
                       MPU_RASR_ATTR_NON_CACHEABLE |
                       MPU_RASR_SIZE_1K |
                       MPU_RASR_ENABLE);
    __DSB();
    __ISB();
}

void memprotect_init(void)
{
    memprotect_arm();
    /*
      enable MEMFAULTENA before MPU->CTRL. The other order leaves a window
      where the region traps but MemManage is disabled, so a fault in that
      window would escalate to a HardFault
     */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
    __DSB();
    MPU->CTRL = MPU_CTRL_PRIVDEFENA | MPU_CTRL_ENABLE;
    __DSB();
    __ISB();
    memprotect_state.initialised = true;
}

/*
  disable region 7 only, leaving MPU->CTRL and MEMFAULTENA alone. Using
  mpuDisable() here would zero MPU->CTRL and kill the nocache DMA regions
 */
void memprotect_disarm(void)
{
    MPU->RNR = MPU_REGION_7;
    MPU->RASR = 0;
    __DSB();
    __ISB();
    memprotect_state.armed = false;
}

static bool memprotect_is_code(uint32_t v)
{
    // Thumb return addresses have bit 0 set and point into our image
    return (v & 1U) != 0U &&
        v > (uint32_t)&_vectors &&
        v < (uint32_t)&_etext;
}

/*
  scan the interrupted stack for plausible return addresses. We have no
  unwinder on ARM builds, so this is a heuristic: it can miss frames and can
  report stale ones. Every load is bounded to a stack we have proved is mapped,
  as a fault in here would escalate to a HardFault
 */
static void memprotect_backtrace(struct port_extctx *ctx, uint32_t exc_return, thread_t *tp)
{
    // clear rather than just resetting the count, as we always log all of the
    // entries and stale ones from an earlier fault would be misleading
    memset(memprotect_state.backtrace, 0, sizeof(memprotect_state.backtrace));
    memprotect_state.backtrace_count = 0;

    // step over the hardware stacked frame to get the interrupted SP
    uint32_t fsize = 8U * 4U;
    if ((exc_return & 0x10U) == 0U) {
        fsize += 18U * 4U;   // FP context
    }
    if ((ctx->xpsr & (1U << 9)) != 0U) {
        fsize += 4U;         // stack aligner
    }
    const uint32_t sp = (uint32_t)ctx + fsize;

    uint32_t lo, hi;
    if ((exc_return & 4U) != 0U) {
        // thread mode, frame came off PSP
        if (tp == nullptr || tp->wabase == nullptr) {
            return;
        }
        lo = (uint32_t)tp->wabase;
        if (tp->wabase == (void*)&__main_thread_stack_base__) {
            // the main thread's stack is separate from its thread_t
            hi = (uint32_t)&__main_thread_stack_end__;
        } else {
            hi = (uint32_t)tp;
        }
    } else {
        lo = (uint32_t)&__main_stack_base__;
        hi = (uint32_t)&__main_stack_end__;
    }
    if (sp < lo || sp >= hi || (sp & 3U) != 0U) {
        return;
    }

    uint32_t end = sp + AP_MEMPROTECT_SCAN_WORDS * 4U;
    if (end > hi) {
        end = hi;
    }

    uint32_t prev = 0;
    for (uint32_t p = sp; p < end; p += 4U) {
        const uint32_t v = *(const uint32_t *)p;
        if (v != prev && memprotect_is_code(v)) {
            memprotect_state.backtrace[memprotect_state.backtrace_count++] = v;
            if (memprotect_state.backtrace_count >= AP_MEMPROTECT_BT_LEN) {
                break;
            }
        }
        prev = v;
    }
}

/*
  returns non-zero if this was an access (read or write; DACCVIOL covers
  both) to the reserved region that we have recorded and can resume from.
  Anything else is left for the normal fatal fault path
 */
uint32_t memprotect_handle_fault(struct port_extctx *ctx, uint32_t exc_return);
uint32_t memprotect_handle_fault(struct port_extctx *ctx, uint32_t exc_return)
{
    const uint32_t cfsr = SCB->CFSR;

    if ((cfsr & MEMPROTECT_MM_BITS) != MEMPROTECT_OK_BITS) {
        // instruction fetch, stacking or unstacking fault, or no valid address
        return 0;
    }
    const uint32_t addr = SCB->MMFAR;
    if (addr >= AP_MEMPROTECT_SIZE) {
        return 0;
    }
    if (!memprotect_state.armed || memprotect_state.latched_off) {
        // returning would fault again forever
        return 0;
    }

    memprotect_disarm();

    // sticky, write 1 to clear. Leaves the BusFault and UsageFault bits alone
    SCB->CFSR = MEMPROTECT_OK_BITS;

    if (memprotect_state.hit_count < AP_MEMPROTECT_MAX_HITS) {
        if (!memprotect_state.pending) {
            // don't clobber a record the monitor thread hasn't read yet
            memprotect_state.fault_addr = addr;
            memprotect_state.pc = ctx->pc;
            memprotect_state.lr = ctx->lr_thd;
            memprotect_state.exc_return = exc_return;
            // exception number of the context we interrupted, 0 for thread
            // mode. __get_IPSR() here would just tell us we are in MemManage
            memprotect_state.ipsr = (uint8_t)(ctx->xpsr & 0x1FFU);

            thread_t *tp = currcore->rlist.current;
            memset(memprotect_state.thread_name, 0, sizeof(memprotect_state.thread_name));
            if (tp != nullptr && tp->name != nullptr) {
                // leave room for the NUL, the GCS message prints this with %s
                strncpy_noterm(memprotect_state.thread_name, tp->name,
                               sizeof(memprotect_state.thread_name)-1);
            }
            memprotect_backtrace(ctx, exc_return, tp);

            __DMB();
            memprotect_state.pending = true;
        }
        memprotect_state.hit_count++;
    }
    if (memprotect_state.hit_count >= AP_MEMPROTECT_MAX_HITS) {
        memprotect_state.latched_off = true;
    }

    return 1;
}

}  // extern "C"

#endif  // AP_BOARDCONFIG_MCU_MEMPROTECT_TRACE_ENABLED
