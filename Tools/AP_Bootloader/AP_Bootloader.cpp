/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ArduPilot bootloader. This implements the same protocol originally
  developed for PX4, but builds on top of the ChibiOS HAL

  It does not use the full AP_HAL API in order to keep the firmware
  size below the maximum of 16kByte required for F4 based
  boards. Instead it uses the ChibiOS APIs directly
 */

#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include "support.h"
#include "bl_protocol.h"
#include "flash_from_sd.h"
#include "can.h"
#include <stdio.h>
#if EXT_FLASH_SIZE_MB
#include <AP_FlashIface/AP_FlashIface_JEDEC.h>
#endif
#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include "network.h"

extern "C" {
    int main(void);
}


struct boardinfo board_info = {
    .board_type = APJ_BOARD_ID,
    .board_rev = 0,
    .fw_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + FLASH_RESERVE_END_KB + APP_START_OFFSET_KB))*1024,
    .extf_size = (EXT_FLASH_SIZE_MB * 1024 * 1024) - (EXT_FLASH_RESERVE_START_KB + EXT_FLASH_RESERVE_END_KB) * 1024
};

#ifndef HAL_BOOTLOADER_TIMEOUT
#define HAL_BOOTLOADER_TIMEOUT 5000
#endif

#ifndef HAL_STAY_IN_BOOTLOADER_VALUE
#define HAL_STAY_IN_BOOTLOADER_VALUE 0
#endif

#if EXT_FLASH_SIZE_MB
AP_FlashIface_JEDEC ext_flash;
#endif

#if AP_BOOTLOADER_NETWORK_ENABLED
static BL_Network network;
#endif

int main(void)
{
#ifdef AP_BOOTLOADER_CUSTOM_HERE4
    custom_startup();
#endif

    flash_init();

#if AP_FLASH_ECC_CHECK_ENABLED
    check_ecc_errors();
#endif
    
#ifdef STM32F427xx
    if (BOARD_FLASH_SIZE > 1024 && check_limit_flash_1M()) {
        board_info.fw_size = (1024 - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    }
#endif

    bool try_boot = false;
    uint32_t timeout = HAL_BOOTLOADER_TIMEOUT;

#ifdef HAL_BOARD_AP_PERIPH_ZUBAXGNSS
    // setup remapping register for ZubaxGNSS
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG;
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2 | AFIO_MAPR_SPI3_REMAP;
#endif

#if HAL_FLASH_PROTECTION
    stm32_flash_unprotect_flash();
#endif

#if AP_BOOTLOADER_NETWORK_ENABLED
    network.restore_comms_ip();
#endif

#if AP_FASTBOOT_ENABLED
    enum rtc_boot_magic m = check_fast_reboot();
    bool was_watchdog = stm32_was_watchdog_reset();
    if (was_watchdog) {
        try_boot = true;
        timeout = 0;
    } else if (m == RTC_BOOT_HOLD) {
        timeout = 0;
    } else if (m == RTC_BOOT_FAST) {
        try_boot = true;
        timeout = 0;
    }
#if HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES
    else if ((m & 0xFFFFFF00) == RTC_BOOT_CANBL) {
        try_boot = false;
        timeout = 10000;
        can_set_node_id(m & 0xFF);
    }
    if (can_check_update()) {
        // trying to update firmware, stay in bootloader
        try_boot = false;
        timeout = 0;
    }
#if AP_CHECK_FIRMWARE_ENABLED
    const auto ok = check_good_firmware();
    if (ok != check_fw_result_t::CHECK_FW_OK) {
        // bad firmware CRC, don't try and boot
        timeout = 0;
        try_boot = false;
        led_set(LED_BAD_FW);
    }
#ifndef BOOTLOADER_DEV_LIST
    else if (timeout == HAL_BOOTLOADER_TIMEOUT) {
        // fast boot for good firmware if we haven't been told to stay
        // in bootloader
        try_boot = true;
        timeout = 1000;
    }
#endif   // ifndef(BOOTLOADER_DEV_LIST)
#if AP_BOOTLOADER_NETWORK_ENABLED
    if (ok == check_fw_result_t::CHECK_FW_OK) {
        const auto *app_descriptor = get_app_descriptor();
        if (app_descriptor != nullptr) {
            network.status_printf("Firmware OK: %ld.%ld.%lx\n", app_descriptor->version_major,
                                                                app_descriptor->version_minor,
                                                                app_descriptor->git_hash);
        }
    } else {
        network.status_printf("Firmware Error: %d\n", (int)ok);
    }
#endif
#endif  // AP_CHECK_FIRMWARE_ENABLED

    if (was_watchdog && m != RTC_BOOT_FWOK) {
        // we've had a watchdog within 30s of booting main CAN
        // firmware. We will stay in bootloader to allow the user to
        // load a fixed firmware
        stm32_watchdog_clear_reason();
        try_boot = false;
        timeout = 0;
    }
#elif AP_CHECK_FIRMWARE_ENABLED
    const auto ok = check_good_firmware();
    if (ok != check_fw_result_t::CHECK_FW_OK) {
        // bad firmware, don't try and boot
        timeout = 0;
        try_boot = false;
        led_set(LED_BAD_FW);
    }
#endif

#if defined(HAL_GPIO_PIN_VBUS) && defined(HAL_ENABLE_VBUS_CHECK)
#if HAL_USE_SERIAL_USB == TRUE
    else if (palReadLine(HAL_GPIO_PIN_VBUS) == 0)  {
        try_boot = true;
        timeout = 0;
    }
#endif
#endif

    // if we fail to boot properly we want to pause in bootloader to give
    // a chance to load new app code
    set_fast_reboot(RTC_BOOT_OFF);
#endif  // AP_FASTBOOT_ENABLED

#ifdef HAL_GPIO_PIN_STAY_IN_BOOTLOADER
    // optional "stay in bootloader" pin
    if (palReadLine(HAL_GPIO_PIN_STAY_IN_BOOTLOADER) == HAL_STAY_IN_BOOTLOADER_VALUE) {
        try_boot = false;
        timeout = 0;
    }
#endif

#if EXT_FLASH_SIZE_MB
    while (!ext_flash.init()) {
        // keep trying until we get it working
        // there's no future without it
        chThdSleep(chTimeMS2I(20));
    }
#endif

    if (try_boot) {
        jump_to_app();
    }

#if defined(BOOTLOADER_DEV_LIST)
    init_uarts();
#endif

#if HAL_ENABLE_DFU_BOOT
    /*
      check if the main firmware requested a reboot into DFU mode
      (STM32 system bootloader for USB firmware upload).

      this check is placed here deliberately — after ChibiOS init and
      init_uarts() — for two reasons:

      1. on Cortex-M7 (F7/H7), D-Cache flush requires caches to be
         enabled. the firmware uses write-back D-Cache and dirty cache
         lines persist through NVIC_SystemReset. if not flushed, the
         system bootloader reads stale data and its USB OTG core soft
         reset (GRSTCTL.CSRST) hangs. SCB_CleanDCache() is a no-op
         when caches are disabled, so we must wait until __cpu_init
         has re-enabled them.

      2. the USB OTG FS peripheral is configured by init_uarts() and
         must be fully de-initialised before the system bootloader can
         re-initialise it. without this, the USB PHY is left in an
         active state that prevents the core reset from completing.

      the firmware signals DFU mode by setting boot_to_dfu in the
      persistent data (RTC backup registers) and rebooting with
      RTC_BOOT_HOLD so the bootloader stays long enough to reach here.
    */
    {
        AP_HAL::Util::PersistentData pd;
        stm32_watchdog_load((uint32_t *)&pd, (sizeof(pd)+3)/4);
        if (pd.boot_to_dfu) {
            pd.boot_to_dfu = false;
            stm32_watchdog_save((uint32_t *)&pd, (sizeof(pd)+3)/4);

            // ensure USB OTG FS clock is enabled so we can
            // access its registers for de-initialisation
            rccEnableOTG_FS(false);

            // fully de-initialise the USB OTG FS peripheral:
            // disconnect from bus, power down the analog PHY,
            // and complete a core soft reset. the register layout
            // (stm32_otg_t) is the same across all STM32 families.
            OTG_FS->DCTL |= DCTL_SDIS;
            OTG_FS->GCCFG = 0;
            OTG_FS->GRSTCTL = GRSTCTL_CSRST;
            while (OTG_FS->GRSTCTL & GRSTCTL_CSRST) {}

            // reset the peripheral via RCC and disable its clock
            rccResetOTG_FS();
            rccDisableOTG_FS();

#if CORTEX_MODEL == 7
            // flush dirty D-Cache lines to RAM and disable caches.
            // on Cortex-M7 (F7/H7) the firmware's cached writes
            // persist through NVIC_SystemReset and must be flushed.
            SCB_CleanDCache();
            SCB_DisableDCache();
            SCB_InvalidateICache();
            SCB_DisableICache();
#endif

            // reset clock tree to HSI — the system bootloader
            // expects HSI as clock source. PLL settings vary by
            // board (crystal frequency) and must be cleared.
            RCC->CFGR = 0;
            while ((RCC->CFGR & RCC_CFGR_SWS) != 0) {}
            RCC->CR &= ~(RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON |
                         RCC_CR_HSEON | RCC_CR_HSI48ON);

            // restore CPU state to power-on reset defaults
            __enable_irq();
            SCB->VTOR = 0;
            SCB->CPACR = 0;
            __set_CONTROL(0);
            __ISB();

            // jump to STM32 system bootloader in system memory
#if defined(STM32H7)
            const uint32_t *app_base = (const uint32_t *)(0x1FF09800);
#elif defined(STM32F7)
            const uint32_t *app_base = (const uint32_t *)(0x1FF00000);
#else
            const uint32_t *app_base = (const uint32_t *)(0x1FFF0000);
#endif
            __set_MSP(*app_base);
            ((void (*)())*(&app_base[1]))();
            while (true);
        }
    }
#endif
#if HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES
    can_start();
#endif

#if AP_BOOTLOADER_NETWORK_ENABLED
    network.init();
#endif

#if AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
    if (flash_from_sd()) {
        jump_to_app();
    }
#endif

#if defined(BOOTLOADER_DEV_LIST)
    while (true) {
        bootloader(timeout);
        jump_to_app();
    }
#else
    // CAN and network only
    while (true) {
        uint32_t t0 = AP_HAL::millis();
        while (timeout == 0 || AP_HAL::millis() - t0 <= timeout) {
            can_update();
            chThdSleep(chTimeMS2I(1));
        }
        jump_to_app();
    }
#endif
}


