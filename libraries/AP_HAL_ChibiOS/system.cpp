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
#include "hwdef/common/watchdog.h"
#include "hwdef/common/stm32_util.h"

#include <ch.h>
#include "hal.h"
#include <hrt.h>

#if CH_CFG_ST_RESOLUTION == 16
static_assert(sizeof(systime_t) == 2, "expected 16 bit systime_t");
#elif CH_CFG_ST_RESOLUTION == 32
static_assert(sizeof(systime_t) == 4, "expected 32 bit systime_t");
#endif

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

void *__dso_handle;

void __cxa_pure_virtual(void);
void __cxa_pure_virtual() { while (1); } //TODO: Handle properly, maybe generate a traceback

void NMI_Handler(void);
void NMI_Handler(void) { while (1); }

/*
  save watchdog data for a hard fault
 */
static void save_fault_watchdog(uint16_t line, FaultType fault_type, uint32_t fault_addr, uint32_t lr)
{
#ifndef HAL_BOOTLOADER_BUILD
    bool using_watchdog = AP_BoardConfig::watchdog_enabled();
    if (using_watchdog) {
        AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
        pd.fault_line = line;
        if (pd.fault_type == 0) {
            // don't overwrite earlier fault
            pd.fault_type = fault_type;
        }
        pd.fault_addr = fault_addr;
        thread_t *tp = chThdGetSelfX();
        if (tp) {
            pd.fault_thd_prio = tp->prio;
            // get first 4 bytes of the name, but only of first fault
            if (tp->name && pd.thread_name4[0] == 0) {
                strncpy_noterm(pd.thread_name4, tp->name, 4);
            }
        }
        pd.fault_icsr = SCB->ICSR;
        pd.fault_lr = lr;
        stm32_watchdog_save((uint32_t *)&hal.util->persistent_data, (sizeof(hal.util->persistent_data)+3)/4);
    }
#endif
}

void HardFault_Handler(void);
void HardFault_Handler(void) {
    //Copy to local variables (not pointers) to allow GDB "i loc" to directly show the info
    //Get thread context. Contains main registers including PC and LR
    struct port_extctx ctx;
    memcpy(&ctx, (void*)__get_PSP(), sizeof(struct port_extctx));
    (void)ctx;
    //Interrupt status register: Which interrupt have we encountered, e.g. HardFault?
    FaultType faultType = (FaultType)__get_IPSR();
    (void)faultType;
    //For HardFault/BusFault this is the address that was accessed causing the error
    uint32_t faultAddress = SCB->BFAR;
    (void)faultAddress;
    //Flags about hardfault / busfault
    //See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Cihdjcfc.html for reference
    bool isFaultPrecise = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 1) ? true : false);
    bool isFaultImprecise = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 2) ? true : false);
    bool isFaultOnUnstacking = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 3) ? true : false);
    bool isFaultOnStacking = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 4) ? true : false);
    bool isFaultAddressValid = ((SCB->CFSR >> SCB_CFSR_BUSFAULTSR_Pos) & (1 << 7) ? true : false);
    (void)isFaultPrecise;
    (void)isFaultImprecise;
    (void)isFaultOnUnstacking;
    (void)isFaultOnStacking;
    (void)isFaultAddressValid;

    save_fault_watchdog(__LINE__, faultType, faultAddress, (uint32_t)ctx.lr_thd);

#ifdef HAL_GPIO_PIN_FAULT
    while (true) {
        fault_printf("HARDFAULT\n");
        fault_printf("CUR=0x%08x\n", ch.rlist.current);
        if (ch.rlist.current) {
            fault_printf("NAME=%s\n", ch.rlist.current->name);
        }
        fault_printf("FA=0x%08x\n", faultAddress);
        fault_printf("PC=0x%08x\n", ctx.pc);
        fault_printf("LR=0x%08x\n", ctx.lr_thd);
        fault_printf("R0=0x%08x\n", ctx.r0);
        fault_printf("R1=0x%08x\n", ctx.r1);
        fault_printf("R2=0x%08x\n", ctx.r2);
        fault_printf("R3=0x%08x\n", ctx.r3);
        fault_printf("R12=0x%08x\n", ctx.r12);
        fault_printf("XPSR=0x%08x\n", ctx.xpsr);
        fault_printf("\n\n");
    }
#endif
    //Cause debugger to stop. Ignored if no debugger is attached
    while(1) {}
}

void BusFault_Handler(void) __attribute__((alias("HardFault_Handler")));

void UsageFault_Handler(void);
void UsageFault_Handler(void) {
    //Copy to local variables (not pointers) to allow GDB "i loc" to directly show the info
    //Get thread context. Contains main registers including PC and LR
    struct port_extctx ctx;
    memcpy(&ctx, (void*)__get_PSP(), sizeof(struct port_extctx));
    (void)ctx;
    //Interrupt status register: Which interrupt have we encountered, e.g. HardFault?
    FaultType faultType = (FaultType)__get_IPSR();
    (void)faultType;
    uint32_t faultAddress = SCB->BFAR;
    //Flags about hardfault / busfault
    //See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Cihdjcfc.html for reference
    bool isUndefinedInstructionFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 0) ? true : false);
    bool isEPSRUsageFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 1) ? true : false);
    bool isInvalidPCFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 2) ? true : false);
    bool isNoCoprocessorFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 3) ? true : false);
    bool isUnalignedAccessFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 8) ? true : false);
    bool isDivideByZeroFault = ((SCB->CFSR >> SCB_CFSR_USGFAULTSR_Pos) & (1 << 9) ? true : false);
    (void)isUndefinedInstructionFault;
    (void)isEPSRUsageFault;
    (void)isInvalidPCFault;
    (void)isNoCoprocessorFault;
    (void)isUnalignedAccessFault;
    (void)isDivideByZeroFault;

    save_fault_watchdog(__LINE__, faultType, faultAddress, (uint32_t)ctx.lr_thd);

    //Cause debugger to stop. Ignored if no debugger is attached
    while(1) {}
}

void MemManage_Handler(void);
void MemManage_Handler(void) {
    //Copy to local variables (not pointers) to allow GDB "i loc" to directly show the info
    //Get thread context. Contains main registers including PC and LR
    struct port_extctx ctx;
    memcpy(&ctx, (void*)__get_PSP(), sizeof(struct port_extctx));
    (void)ctx;
    //Interrupt status register: Which interrupt have we encountered, e.g. HardFault?
    FaultType faultType = (FaultType)__get_IPSR();
    (void)faultType;
    //For HardFault/BusFault this is the address that was accessed causing the error
    uint32_t faultAddress = SCB->MMFAR;
    (void)faultAddress;
    //Flags about hardfault / busfault
    //See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Cihdjcfc.html for reference
    bool isInstructionAccessViolation = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 0) ? true : false);
    bool isDataAccessViolation = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 1) ? true : false);
    bool isExceptionUnstackingFault = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 3) ? true : false);
    bool isExceptionStackingFault = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 4) ? true : false);
    bool isFaultAddressValid = ((SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & (1 << 7) ? true : false);
    (void)isInstructionAccessViolation;
    (void)isDataAccessViolation;
    (void)isExceptionUnstackingFault;
    (void)isExceptionStackingFault;
    (void)isFaultAddressValid;

    save_fault_watchdog(__LINE__, faultType, faultAddress, (uint32_t)ctx.lr_thd);

    while(1) {}
}
}
namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
#ifndef HAL_BOOTLOADER_BUILD
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
    return hrt_micros32();
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
