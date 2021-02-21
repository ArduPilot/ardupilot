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
#include "hal.h"
#include "hwdef.h"

#include <stdlib.h>
#include <string.h>

#include "usbcfg.h"

#if defined(HAL_USB_PRODUCT_ID)
/*
  check if one string contains another
 */
bool string_contains(const char *haystack, const char *needle)
{
    uint8_t needle_len = strlen(needle);
    while (*haystack) {
        if (strncmp(haystack, needle, needle_len) == 0) {
            return true;
        }
        haystack++;
    }
    return false;
}

/*
  handle substitution of variables in strings for USB descriptors
 */
void string_substitute(const char *str, char *str2)
{
    const char *board = "%BOARD%";
    const char *serial = "%SERIAL%";
    uint8_t new_len = strlen(str);
    if (string_contains(str, board)) {
        new_len += strlen(HAL_BOARD_NAME) - strlen(board);
    }
    if (string_contains(str, serial)) {
        new_len += 24 - strlen(serial);
    }
    if (new_len+1 > USB_DESC_MAX_STRLEN) {
        strcpy(str2, str);
        return;
    }
    char *p = str2;
    while (*str) {
        char c = *str;
        if (c == '%') {
            if (strncmp(str, board, strlen(board)) == 0) {
                memcpy(p, HAL_BOARD_NAME, strlen(HAL_BOARD_NAME));
                str += 7;
                p += strlen(HAL_BOARD_NAME);
                continue;
            }
            if (strncmp(str, serial, strlen(serial)) == 0) {
                const char *hex = "0123456789ABCDEF";
                const uint8_t *cpu_id = (const uint8_t *)UDID_START;
                uint8_t i;
                for (i=0; i<12; i++) {
                    *p++ = hex[(cpu_id[i]>>4)&0xF];
                    *p++ = hex[cpu_id[i]&0xF];
                }
                str += 8;
                continue;
            }
        }
        *p++ = *str++;
    }
    *p = 0;
}
#endif
