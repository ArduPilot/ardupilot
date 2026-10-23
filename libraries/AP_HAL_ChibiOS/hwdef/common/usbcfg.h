/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
        http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
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
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once
#include "hwdef.h"

#ifndef HAL_HAVE_DUAL_USB_CDC
#define HAL_HAVE_DUAL_USB_CDC 0
#endif

/* Experimental CDC-ECM composite (single ACM + ECM). Default off.
 * When enabling: require distinct USB_PRODUCT (not stock 0x5740/0x5741),
 * drop OTG2 from SERIAL_ORDER (single ACM), and gadget OTG ENDPOINTS >= 8.
 * See usbcfg_cdc_ecm.c compile guards; lab board MatekH743-ECM (PID 0x574E).
 */
#ifndef HAL_WITH_USB_CDC_ECM
#define HAL_WITH_USB_CDC_ECM 0
#endif

/*
 * Phase-1 spike: ACM+ECM is mutually exclusive with dual CDC-ACM.
 * When ECM is enabled, only SDU1 is provided (see usbcfg_cdc_ecm.c).
 */
#if HAL_WITH_USB_CDC_ECM
#undef HAL_HAVE_DUAL_USB_CDC
#define HAL_HAVE_DUAL_USB_CDC 0
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#if HAL_USE_SERIAL_USB
extern const USBConfig usbcfg;
extern const SerialUSBConfig serusbcfg1;
extern SerialUSBDriver SDU1;
#if HAL_HAVE_DUAL_USB_CDC
extern SerialUSBDriver SDU2;
extern const SerialUSBConfig serusbcfg2;
#endif //HAL_HAVE_DUAL_USB_CDC
uint32_t get_usb_baud(uint16_t endpoint_id);
uint8_t get_usb_parity(uint16_t endpoint_id);
#if HAL_WITH_USB_CDC_ECM
uint8_t usb_ecm_get_data_altsetting(void);
bool usb_ecm_link_up(void);
/* Phase 2: Ethernet frame I/O on ECM data bulk EP (max 1514 bytes). */
#ifndef USB_ECM_MAX_FRAME
#define USB_ECM_MAX_FRAME 1514
#endif
bool usb_ecm_send_frame(const uint8_t *data, uint16_t len);
uint16_t usb_ecm_recv_frame(uint8_t *data, uint16_t maxlen);
void usb_ecm_get_stats(uint32_t *rx_pkts, uint32_t *rx_frames,
                       uint32_t *tx_frames, uint32_t *tx_fail);
#endif
#endif
#define USB_DESC_MAX_STRLEN 100
void setup_usb_strings(void);
void string_substitute(const char *str, char *str2);
bool string_contains(const char *haystack, const char *needle);

#if defined(__cplusplus)
}
#endif
    
/** @} */
