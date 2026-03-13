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

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP_Class.h"
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "task.h"

volatile TaskHandle_t stack_overflow_task;
volatile const char *stack_overflow_task_name;

volatile uint32_t hardfault_sp;
volatile uint32_t hardfault_exc_return;
volatile uint32_t hardfault_r0;
volatile uint32_t hardfault_r1;
volatile uint32_t hardfault_r2;
volatile uint32_t hardfault_r3;
volatile uint32_t hardfault_r12;
volatile uint32_t hardfault_lr;
volatile uint32_t hardfault_pc;
volatile uint32_t hardfault_psr;
volatile uint32_t hardfault_cfsr;
volatile uint32_t hardfault_hfsr;
volatile uint32_t hardfault_mmfar;
volatile uint32_t hardfault_bfar;
volatile uint32_t hardfault_shcsr;
volatile uint32_t hardfault_icsr;
volatile uint32_t hardfault_afsr;
volatile uint32_t hardfault_ipsr;
volatile uint32_t hardfault_control;
volatile uint32_t hardfault_msp;
volatile uint32_t hardfault_psp_active;
volatile uint32_t hardfault_hint_pc0;
volatile uint32_t hardfault_hint_pc1;
volatile uint32_t hardfault_hint_pc2;
volatile uint32_t hardfault_hint_idx0;
volatile uint32_t hardfault_hint_idx1;
volatile uint32_t hardfault_hint_idx2;

extern "C" void hardfault_capture(uint32_t *sp, uint32_t exc_return)
{
    hardfault_sp = (uint32_t)sp;
    hardfault_exc_return = exc_return;

    hardfault_r0 = sp[0];
    hardfault_r1 = sp[1];
    hardfault_r2 = sp[2];
    hardfault_r3 = sp[3];
    hardfault_r12 = sp[4];
    hardfault_lr = sp[5];
    hardfault_pc = sp[6];
    hardfault_psr = sp[7];

    hardfault_cfsr = *((volatile uint32_t *)0xE000ED28U);
    hardfault_hfsr = *((volatile uint32_t *)0xE000ED2CU);
    hardfault_mmfar = *((volatile uint32_t *)0xE000ED34U);
    hardfault_bfar = *((volatile uint32_t *)0xE000ED38U);
    hardfault_shcsr = *((volatile uint32_t *)0xE000ED24U);
    hardfault_icsr = *((volatile uint32_t *)0xE000ED04U);
    hardfault_afsr = *((volatile uint32_t *)0xE000ED3CU);

    __asm volatile("mrs %0, ipsr" : "=r"(hardfault_ipsr));
    __asm volatile("mrs %0, control" : "=r"(hardfault_control));
    __asm volatile("mrs %0, msp" : "=r"(hardfault_msp));
    __asm volatile("mrs %0, psp" : "=r"(hardfault_psp_active));

    hardfault_hint_pc0 = 0;
    hardfault_hint_pc1 = 0;
    hardfault_hint_pc2 = 0;
    hardfault_hint_idx0 = 0xFFFFFFFFU;
    hardfault_hint_idx1 = 0xFFFFFFFFU;
    hardfault_hint_idx2 = 0xFFFFFFFFU;

    uint8_t hint_slot = 0;
    for (uint32_t i = 0; i < 32U; i++) {
        const uint32_t candidate = sp[i];
        if ((candidate & 1U) == 0U) {
            continue;
        }

        const uint32_t candidate_addr = candidate & ~1U;
        if ((candidate_addr < 0x10000000U) || (candidate_addr >= 0x11000000U)) {
            continue;
        }

        if (hint_slot == 0U) {
            hardfault_hint_pc0 = candidate_addr;
            hardfault_hint_idx0 = i;
        } else if (hint_slot == 1U) {
            hardfault_hint_pc1 = candidate_addr;
            hardfault_hint_idx1 = i;
        } else {
            hardfault_hint_pc2 = candidate_addr;
            hardfault_hint_idx2 = i;
            break;
        }
        hint_slot++;
    }

    __asm volatile("cpsid i");
    while (true) {
        __asm volatile("bkpt #0");
        __asm volatile("wfi");
    }
}

extern "C" __attribute__((naked)) void isr_hardfault(void)
{
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "b hardfault_capture\n");
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    stack_overflow_task = xTask;
    stack_overflow_task_name = pcTaskName;

    __asm volatile("cpsid i");
    while (true) {
        __asm volatile("wfi");
    }
}

namespace AP_HAL
{

void panic(const char *errormsg, ...)
{
#if 0
    va_list ap;

    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);
#endif
    while (1) {}
}

uint32_t micros()
{
    return micros64();
}

uint32_t millis()
{
    return millis64();
}

uint64_t micros64()
{
    return time_us_64();
}

uint64_t millis64()
{
    return (uint64_t)(time_us_64() / 1000ULL);
}

} // namespace AP_HAL
