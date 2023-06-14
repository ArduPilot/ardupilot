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
#if AP_CRASHDUMP_ENABLED
#include <CrashCatcher.h>
#endif
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

// debug variables chew up flash, but are handy if you've got a Fault and need
// easy access to the fault data iun the debugger:
#ifndef AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
#define AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED 1
#endif

#define QUOTE(str) #str
#define EXPAND_AND_QUOTE(str) QUOTE(str)
#define ASSERT_CLOCK(clk) static_assert(HAL_EXPECTED_ ##clk == (clk), "unexpected " #clk " value: '" EXPAND_AND_QUOTE(clk) "'")

#if defined(HAL_EXPECTED_STM32_SYS_CK) && defined(STM32_SYS_CK)
ASSERT_CLOCK(STM32_SYS_CK);
#endif
#if defined(HAL_EXPECTED_STM32_HCLK) && defined(STM32_HCLK)
ASSERT_CLOCK(STM32_HCLK);
#endif
#if defined(HAL_EXPECTED_STM32_SDMMC1CLK) && defined(STM32_SDMMC1CLK)
ASSERT_CLOCK(STM32_SDMMC1CLK);
#endif
#if defined(HAL_EXPECTED_STM32_SPI45CLK) && defined(STM32_SPI45CLK)
ASSERT_CLOCK(STM32_SPI45CLK);
#endif
#if defined(HAL_EXPECTED_STM32_FDCANCLK) && defined(STM32_FDCANCLK)
ASSERT_CLOCK(STM32_FDCANCLK);
#endif

extern const AP_HAL::HAL& hal;
extern "C"
{
#define bkpt() __asm volatile("BKPT #0\n")

#if !AP_CRASHDUMP_ENABLED
// do legacy hardfault handling
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
#if AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
    bool forced = SCB->HFSR & SCB_HFSR_FORCED_Msk;
    (void)forced;
    uint32_t cfsr = SCB->CFSR;
    (void)cfsr;
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
#endif  // #if AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED

#if AP_WATCHDOG_SAVE_FAULT_ENABLED
    save_fault_watchdog(__LINE__, faultType, faultAddress, (uint32_t)ctx.lr_thd);
#endif

#ifdef HAL_GPIO_PIN_FAULT
    while (true) {
        // forced means that another kind of unhandled fault got escalated to a hardfault
        if (faultType == BusFault) {
            fault_printf("BUSFAULT\n");
        } else if (forced) {
            fault_printf("FORCED HARDFAULT\n");
        } else {
            fault_printf("HARDFAULT(%d)\n", int(faultType));
        }
        fault_printf("CSFR=0x%08x\n", cfsr);
        fault_printf("CUR=0x%08x\n", currcore->rlist.current);
        if (currcore->rlist.current) {
            fault_printf("NAME=%s\n", currcore->rlist.current->name);
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

// For the BusFault handler to be active SCB_SHCSR_BUSFAULTENA_Msk should be set in SCB->SHCSR
// ChibiOS does not do this by default
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
    (void)faultAddress;
#if AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
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
#endif  // AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED

#if AP_WATCHDOG_SAVE_FAULT_ENABLED
    save_fault_watchdog(__LINE__, faultType, faultAddress, (uint32_t)ctx.lr_thd);
#endif

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
#if AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
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
#endif  // AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED

#if AP_WATCHDOG_SAVE_FAULT_ENABLED
    save_fault_watchdog(__LINE__, faultType, faultAddress, (uint32_t)ctx.lr_thd);
#endif

    while(1) {}
}
#else
// Handle via Crash Catcher
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
#endif


#if AP_WATCHDOG_SAVE_FAULT_ENABLED
/*
  save watchdog data for a hard fault
 */
void save_fault_watchdog(uint16_t line, FaultType fault_type, uint32_t fault_addr, uint32_t lr)
{
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
}
#endif  // AP_WATCHDOG_SAVE_FAULT_ENABLED

void *__dso_handle;

void __cxa_pure_virtual(void);
void __cxa_pure_virtual() { while (1); } //TODO: Handle properly, maybe generate a traceback

void NMI_Handler(void);
void NMI_Handler(void) { while (1); }

#if defined(HAL_BOOTLOADER_BUILD) && HAL_ENABLE_DFU_BOOT
void __entry_hook(void);
void __entry_hook()
{
    // read the persistent data
    AP_HAL::Util::PersistentData pd;
    stm32_watchdog_load((uint32_t *)&pd, (sizeof(pd)+3)/4);
    if (pd.boot_to_dfu) {
        pd.boot_to_dfu = false;
        stm32_watchdog_save((uint32_t *)&pd, (sizeof(pd)+3)/4);
#if defined(STM32H7)
        const uint32_t *app_base = (const uint32_t *)(0x1FF09800); 
#else
        const uint32_t *app_base = (const uint32_t *)(0x1FFF0000);
#endif
        __set_MSP(*app_base);
        ((void (*)())*(&app_base[1]))();
        while(true);
    }
}
#endif

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

__FASTRAMFUNC__ uint32_t micros()
{
#if CH_CFG_ST_RESOLUTION == 32 && CH_CFG_ST_FREQUENCY==1000000U
    // special case optimisation for 32 bit timers
#ifdef AP_BOARD_START_TIME
    return st_lld_get_counter() + AP_BOARD_START_TIME;
#else
    return st_lld_get_counter();
#endif
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
    
__FASTRAMFUNC__ uint32_t millis()
{
    return hrt_millis32();
}

__FASTRAMFUNC__ uint16_t millis16()
{
    return hrt_millis32() & 0xFFFF;
}

__FASTRAMFUNC__ uint64_t micros64()
{
    return hrt_micros64();
}

__FASTRAMFUNC__ uint64_t millis64()
{
    return hrt_micros64() / 1000U;
}


__FASTRAMFUNC__ uint32_t native_micros()
{
    return micros();
}

__FASTRAMFUNC__ uint32_t native_millis()
{
    return millis();
}

__FASTRAMFUNC__ uint16_t native_millis16()
{
    return millis16();
}

__FASTRAMFUNC__ uint64_t native_micros64()
{
    return micros64();
}

__FASTRAMFUNC__ uint64_t native_millis64()
{
    return millis64();
}


} // namespace AP_HAL
