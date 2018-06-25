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
#include "support.h"
#include "bl_protocol.h"

extern "C" {
    int main(void);
}

struct boardinfo board_info;

#ifndef BOOTLOADER_BAUDRATE
#define BOOTLOADER_BAUDRATE 115200
#endif

int main(void)
{
    init_uarts();

    board_info.board_type = APJ_BOARD_ID;
    board_info.board_rev = 0;
    board_info.fw_size = (BOARD_FLASH_SIZE - FLASH_BOOTLOADER_LOAD_KB)*1024;
    if (BOARD_FLASH_SIZE > 1024 && check_limit_flash_1M()) {
        board_info.fw_size = (1024 - FLASH_BOOTLOADER_LOAD_KB)*1024;        
    }

    flash_init();

    while (true) {
        bootloader(5000);
        jump_to_app();
    }
}


