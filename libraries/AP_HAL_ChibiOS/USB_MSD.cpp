/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include "USB_MSD.h"

#if AP_REBOOT_MASS_STORAGE_ENABLED && HAL_USB_MSD_BOOT_ENABLED

#include <hal.h>
#include <stdlib.h>

#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"
#include "sdcard.h"
#include "shared_dma.h"

#if !defined(STM32H7) && !defined(STM32F7) && !defined(STM32F4)
#error HAL_USB_MSD_BOOT_ENABLED is only supported on STM32H7, STM32F7 and STM32F4
#endif

#if HAL_USE_USB_MSD != TRUE || (HAL_USE_SDC != TRUE && HAL_USE_MMC_SPI != TRUE) || HAL_USE_SERIAL_USB != TRUE
#error HAL_USB_MSD_BOOT_ENABLED requires USB MSD, a microSD block device and USB support
#endif

extern const AP_HAL::HAL& hal;

namespace ChibiOS {

static constexpr size_t USB_MSD_IO_SIZE = 4096;

static const uint8_t device_descriptor_data[18] = {
    USB_DESC_DEVICE(
        0x0110,
        0x00,
        0x00,
        0x00,
        0x40,
        HAL_USB_VENDOR_ID,
        HAL_USB_PRODUCT_ID,
        0x0200,
        0,
        1,
        0,
        1)
};

static const USBDescriptor device_descriptor = {
    sizeof(device_descriptor_data),
    device_descriptor_data
};

static const uint8_t configuration_descriptor_data[32] = {
    USB_DESC_CONFIGURATION(32, 1, 1, 0, 0xC0, 50),
    USB_DESC_INTERFACE(0, 0, 2, 0x08, 0x06, 0x50, 0),
    USB_DESC_ENDPOINT(0x01, 0x02, 0x0040, 0),
    USB_DESC_ENDPOINT(0x81, 0x02, 0x0040, 0)
};

static const USBDescriptor configuration_descriptor = {
    sizeof(configuration_descriptor_data),
    configuration_descriptor_data
};

static const uint8_t string0[] = {
    USB_DESC_BYTE(4),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    USB_DESC_WORD(0x0409)
};

static const uint8_t product_string[] = {
    USB_DESC_BYTE(36), USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'A', 0, 'r', 0, 'd', 0, 'u', 0, 'P', 0, 'i', 0, 'l', 0, 'o', 0, 't', 0,
    ' ', 0, 'S', 0, 'D', 0, ' ', 0, 'C', 0, 'a', 0, 'r', 0, 'd', 0
};

static const USBDescriptor string_descriptors[] = {
    { sizeof(string0), string0 },
    { sizeof(product_string), product_string }
};

static const USBDescriptor *get_descriptor(USBDriver *usbp, uint8_t type,
                                           uint8_t index, uint16_t language)
{
    (void)usbp;
    (void)language;

    switch (type) {
    case USB_DESCRIPTOR_DEVICE:
        return &device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
        return &configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
        if (index < ARRAY_SIZE(string_descriptors)) {
            return &string_descriptors[index];
        }
        break;
    }
    return nullptr;
}

static USBInEndpointState ep1_in_state;
static USBOutEndpointState ep1_out_state;

static const USBEndpointConfig ep1_config = {
    USB_EP_MODE_TYPE_BULK,
    nullptr,
    nullptr,
    nullptr,
    0x0040,
    0x0040,
    &ep1_in_state,
    &ep1_out_state,
    2,
    nullptr
};

static void usb_event(USBDriver *usbp, usbevent_t event)
{
    if (event != USB_EVENT_CONFIGURED) {
        return;
    }

    chSysLockFromISR();
    usbInitEndpointI(usbp, 1, &ep1_config);
    chSysUnlockFromISR();
}

static const USBConfig usb_config = {
    usb_event,
    get_descriptor,
    msd_request_hook,
    nullptr
};

bool usb_msd_boot_requested()
{
    AP_HAL::Util::PersistentData persistent_data {};
    stm32_watchdog_load(reinterpret_cast<uint32_t *>(&persistent_data),
                        (sizeof(persistent_data) + 3) / 4);
    if (!persistent_data.boot_to_mass_storage) {
        return false;
    }
    persistent_data.boot_to_mass_storage = false;
    stm32_watchdog_save(reinterpret_cast<uint32_t *>(&persistent_data),
                        (sizeof(persistent_data) + 3) / 4);
    return true;
}

void usb_msd_set_boot_request()
{
    hal.util->persistent_data.boot_to_mass_storage = true;
    stm32_watchdog_save(reinterpret_cast<uint32_t *>(&hal.util->persistent_data),
                        (sizeof(hal.util->persistent_data) + 3) / 4);
}

void usb_msd_run()
{
    peripheral_power_enable();

#if AP_HAL_SHARED_DMA_ENABLED
    Shared_DMA::init();
#endif

    // Mass-storage mode is dedicated to bulk transfers, so deliberately use
    // no SD slowdown for maximum speed. Normal flight logging honours
    // BRD_SD_SLOWDOWN for maximum robustness.
    if (!sdcard_init_raw(0, 3)) {
        return;
    }

    auto *msdp = static_cast<USBMassStorageDriver *>(calloc(1, sizeof(USBMassStorageDriver)));
    auto *block_buffer = static_cast<uint8_t *>(
        hal.util->malloc_type(USB_MSD_IO_SIZE * 2, AP_HAL::Util::MEM_FILESYSTEM));
    if (msdp == nullptr || block_buffer == nullptr) {
        free(msdp);
        hal.util->free_type(block_buffer, USB_MSD_IO_SIZE * 2, AP_HAL::Util::MEM_FILESYSTEM);
        return;
    }

#if STM32_OTG2_IS_OTG1
    auto *usbp = &USBD2;
#else
    auto *usbp = &USBD1;
#endif

    usbDisconnectBus(usbp);
    chThdSleep(chTimeUS2I(1500));
    usbStart(usbp, &usb_config);

    msdObjectInit(msdp);
    msdStart(msdp, usbp, sdcard_get_block_device(),
             block_buffer, block_buffer + USB_MSD_IO_SIZE, USB_MSD_IO_SIZE,
             nullptr, nullptr, nullptr, nullptr);

    usbConnectBus(usbp);

#if !defined(DISABLE_WATCHDOG)
    stm32_watchdog_init();
#endif
    // Mass-storage mode deliberately owns the SD card until power is removed;
    // never return to the flight firmware while the USB host may retain access.
    while (true) {
#if !defined(DISABLE_WATCHDOG)
        stm32_watchdog_pat();
#endif
        chThdSleepMilliseconds(50);
    }
}

}

#endif // AP_REBOOT_MASS_STORAGE_ENABLED && HAL_USB_MSD_BOOT_ENABLED
#endif // HAL_BOARD_CHIBIOS
