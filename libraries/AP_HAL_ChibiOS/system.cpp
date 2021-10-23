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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <stdarg.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#include "hwdef/common/watchdog.h"
#include "hwdef/common/stm32_util.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <CrashCatcher.h>
#include <ch.h>
#include "hal.h"
#include <hrt.h>

#if CH_CFG_ST_RESOLUTION == 16
static_assert(sizeof(systime_t) == 2, "expected 16 bit systime_t");
#elif CH_CFG_ST_RESOLUTION == 32
static_assert(sizeof(systime_t) == 4, "expected 32 bit systime_t");
#endif
static_assert(sizeof(systime_t) == sizeof(sysinterval_t), "expected systime_t same size as sysinterval_t");

#if defined(HAL_EXPECTED_SYSCLOCK)
#ifdef STM32_SYS_CK
static_assert(HAL_EXPECTED_SYSCLOCK == STM32_SYS_CK, "unexpected STM32_SYS_CK value");
#elif defined(STM32_HCLK)
static_assert(HAL_EXPECTED_SYSCLOCK == STM32_HCLK, "unexpected STM32_HCLK value");
#else
#error "unknown system clock"
#endif
#endif

extern const AP_HAL::HAL& hal;
extern "C"
{
#define bkpt() __asm volatile("BKPT #0\n")
typedef enum  {
    Reset = 1,
    NMI = 2,
    HardFault = 3,
    MemManage = 4,
    BusFault = 5,
    UsageFault = 6,
} FaultType;

/*
  save watchdog data for a hard fault
 */
void save_fault_watchdog(uint16_t line, FaultType fault_type, uint32_t fault_addr, uint32_t lr)
{
#ifndef HAL_BOOTLOADER_BUILD
    bool using_watchdog = AP_BoardConfig::watchdog_enabled();
    if (using_watchdog) {
        AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
        if (pd.fault_type == 0) {
            // don't overwrite earlier fault
            pd.fault_line = line;
            pd.fault_type = fault_type;
            pd.fault_addr = fault_addr;
            thread_t *tp = chThdGetSelfX();
            if (tp) {
                pd.fault_thd_prio = tp->hdr.pqueue.prio;
                // get first 4 bytes of the name, but only of first fault
                if (tp->name && pd.thread_name4[0] == 0) {
                    strncpy_noterm(pd.thread_name4, tp->name, 4);
                }
            }
            pd.fault_icsr = SCB->ICSR;
            pd.fault_lr = lr;
        }
        stm32_watchdog_save((uint32_t *)&hal.util->persistent_data, (sizeof(hal.util->persistent_data)+3)/4);
    }
#endif
}

void *__dso_handle;

void __cxa_pure_virtual(void);
void __cxa_pure_virtual() { while (1); } //TODO: Handle properly, maybe generate a traceback


static bool initialised = false;

#ifndef HAL_CRASH_SERIAL_PORT_BAUD
#define HAL_CRASH_SERIAL_PORT_BAUD 921600
#endif

#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE                      USART_ISR_RXNE_RXFNE
#endif

void NMI_Handler(void);
void NMI_Handler(void) { while (1); }

extern void HardFault_Handler(void);

void BusFault_Handler(void);
void BusFault_Handler(void) {
    HardFault_Handler();
}

void UsageFault_Handler(void);
void UsageFault_Handler(void) {
    HardFault_Handler();
}

void MemManage_Handler(void);
void MemManage_Handler(void) {
    HardFault_Handler();
}

/*
  initialise serial ports
 */
static void init_uarts(void)
{
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    IRQ_DISABLE_HAL_CRASH_SERIAL_PORT();
    RCC_RESET_HAL_CRASH_SERIAL_PORT();
    uint32_t fck = (uint32_t)(((HAL_CRASH_SERIAL_PORT_CLOCK + ((HAL_CRASH_SERIAL_PORT_BAUD)/2)) / HAL_CRASH_SERIAL_PORT_BAUD));

    u->BRR = fck;

    /* Resetting eventual pending status flags.*/
    u->ICR = 0xFFFFFFFFU;

    u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    initialised = true;
}

int CrashCatcher_getc(void);
int CrashCatcher_getc(void)
{
    if (!initialised) {
        init_uarts();
    }
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    // wait for a follwing string, only then do we start dumping
    static const char* wait_for_string = "dump_crash_log";
    uint8_t curr_off = 0;
    while (true) {
        while (!(USART_ISR_RXNE & u->ISR)) {}
        uint8_t c = u->RDR;
        if (c == wait_for_string[curr_off]) {
            curr_off++;
            if (curr_off == strlen(wait_for_string)) {
                return 0;
            }
        } else {
            curr_off = 0;
        }
    }
    return -1;
}

void CrashCatcher_putc(int c);
void CrashCatcher_putc(int c)
{
    if (!initialised) {
        init_uarts();
    }
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    u->TDR = c & 0xFF;
    while (!(USART_ISR_TC & u->ISR)) {
        // keep alive while dump is happening
        stm32_watchdog_pat();
    }
}

extern uint32_t __ram0_start__, __ram0_end__;
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void);
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    static const CrashCatcherMemoryRegion regions[] = {
        {(uint32_t)&__ram0_start__, (uint32_t)&__ram0_end__, CRASH_CATCHER_BYTE},
        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
    };
    return regions;
}

}
namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
#if !defined(HAL_BOOTLOADER_BUILD) && !APM_BUILD_TYPE(APM_BUILD_iofirmware)
    INTERNAL_ERROR(AP_InternalError::error_t::panic);
    va_list ap;

    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);

    hal.scheduler->delay_microseconds(10000);
    while (1) {
        va_start(ap, errormsg);
        vprintf(errormsg, ap);
        va_end(ap);
        hal.scheduler->delay(500);
    }
#else
    // we don't support variable args in bootlaoder
    chSysHalt(errormsg);
    // we will never get here, this just to silence a warning
    while (1) {}
#endif
}

uint32_t micros()
{
#if CH_CFG_ST_RESOLUTION == 32 && CH_CFG_ST_FREQUENCY==1000000U
    // special case optimisation for 32 bit timers
    return st_lld_get_counter();
#else
    return hrt_micros32();
#endif
}

uint16_t micros16()
{
#if CH_CFG_ST_RESOLUTION == 32 && CH_CFG_ST_FREQUENCY==1000000U
    return st_lld_get_counter() & 0xFFFF;
#elif CH_CFG_ST_RESOLUTION == 16 && CH_CFG_ST_FREQUENCY==1000000U
    return st_lld_get_counter();
#else
    return hrt_micros32() & 0xFFFF;
#endif
}
    
uint32_t millis()
{
    return hrt_millis32();
}

uint16_t millis16()
{
    return hrt_millis32() & 0xFFFF;
}

uint64_t micros64()
{
    return hrt_micros64();
}

uint64_t millis64()
{
    return hrt_micros64() / 1000U;
}


uint32_t native_micros()
{
    return micros();
}

uint32_t native_millis()
{
    return millis();
}

uint16_t native_millis16()
{
    return millis16();
}

uint64_t native_micros64()
{
    return micros64();
}

uint64_t native_millis64()
{
    return millis64();
}


} // namespace AP_HAL
