#pragma once

#ifndef HAL_USB_MSD_BOOT_ENABLED
#define HAL_USB_MSD_BOOT_ENABLED 0
#endif

namespace ChibiOS {

#if AP_REBOOT_MASS_STORAGE_ENABLED && HAL_USB_MSD_BOOT_ENABLED
bool usb_msd_boot_requested();
void usb_msd_set_boot_request();
void usb_msd_run();
#endif

}
