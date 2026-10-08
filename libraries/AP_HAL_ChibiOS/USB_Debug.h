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
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if AP_USB_DEBUG_ENABLED
#include <stdint.h>

namespace ChibiOS {
// Attach/detach at a main-loop safe point; the monitor runs in debug exceptions.
void usb_debug_poll();
void usb_debug_startup_wait();
bool usb_debug_active();
bool usb_debug_configured();
uint32_t usb_debug_gcs_read(uint8_t *data, uint32_t size);
uint32_t usb_debug_gcs_write(const uint8_t *data, uint32_t size);
}
#endif // AP_USB_DEBUG_ENABLED
