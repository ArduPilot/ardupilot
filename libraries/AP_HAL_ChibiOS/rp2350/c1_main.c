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

/*
 * @file rp2350/c1_main.c
 * RP2350 Core1 entry point for ChibiOS Full SMP mode.
 * Initialises the ch1 OS instance so threads created with affinity &ch1
 * (e.g. the rate thread) are scheduled on core1.
 */
#include "ch.h"
#include "hal.h"

#if CH_CFG_SMP_MODE == TRUE
extern volatile uint32_t c1_xip_lock_ready;
extern uint32_t rp2350_c1_vectors[];
#endif  // CH_CFG_SMP_MODE == TRUE

// the boot ROM's core1 launch handshake leaves SPARE_IRQ_1 (IRQ47) enabled in core1's banked NVIC
void __c1_cpu_init(void)
{
    NVIC->ICER[0] = 0xFFFFFFFFU;
    NVIC->ICER[1] = 0xFFFFFFFFU;
    NVIC->ICPR[0] = 0xFFFFFFFFU;
    NVIC->ICPR[1] = 0xFFFFFFFFU;
#if CH_CFG_SMP_MODE == TRUE
    // _crt0_c1_entry has just pointed VTOR back at the flash table
    SCB->VTOR = (uint32_t)rp2350_c1_vectors;
#endif  // CH_CFG_SMP_MODE == TRUE
    __DSB();
    __ISB();
}

void c1_main(void)
{
#if CH_CFG_SMP_MODE == TRUE
    chSysWaitSystemState(ch_sys_running);
    chInstanceObjectInit(&ch1, &ch_core1_cfg);

    // XIP lockout doorbell, handled in board_rp2350.c. Minimum priority so
    // BASEPRI masks it while core1 holds the kernel lock.
    nvicEnableVector(RP_SIO_IRQ_BELL_NUMBER, CORTEX_MINIMUM_PRIORITY);
    c1_xip_lock_ready = 1U;

    ch1.mainthread.name = "c1_main";
    chSysUnlock();

    while (true) {
        chThdSleep(TIME_INFINITE);
    }
#else
    while (true) {}
#endif  // CH_CFG_SMP_MODE == TRUE
}
