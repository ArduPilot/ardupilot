/*
   Please contribute your ideas! See https://ardupilot.org/dev for details

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
  SerialManager configuration defines
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>

#ifdef HAL_UART_NUM_SERIAL_PORTS
#if HAL_UART_NUM_SERIAL_PORTS >= 4
#define SERIALMANAGER_NUM_PORTS HAL_UART_NUM_SERIAL_PORTS
#else
// we want a minimum of 4 as the default GPS port is SERIAL3
#define SERIALMANAGER_NUM_PORTS 4
#endif
#else
// assume max 8 ports
#define SERIALMANAGER_NUM_PORTS 8
#endif

#ifndef HAL_NUM_SERIAL_PORTS
#define HAL_NUM_SERIAL_PORTS SERIALMANAGER_NUM_PORTS
#endif

#ifndef AP_SERIALMANAGER_ENABLED
#define AP_SERIALMANAGER_ENABLED 1
#endif

/*
  array size for state[]. This needs to be at least
  SERIALMANAGER_NUM_PORTS, but we want it to be the same length on
  similar boards to get the ccache efficiency up. This wastes a small
  amount of memory, but makes a huge difference to the build times
 */
#if SERIALMANAGER_NUM_PORTS > 10 || SERIALMANAGER_NUM_PORTS < 5
#define SERIALMANAGER_MAX_PORTS SERIALMANAGER_NUM_PORTS
#else
#define SERIALMANAGER_MAX_PORTS 10
#endif

#ifndef AP_SERIALMANAGER_REGISTER_ENABLED
#define AP_SERIALMANAGER_REGISTER_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024 && (AP_NETWORKING_ENABLED || HAL_ENABLE_DRONECAN_DRIVERS || CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SERIALMANAGER_IMUOUT_ENABLED
#define AP_SERIALMANAGER_IMUOUT_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL) && AP_INERTIALSENSOR_ENABLED
#endif

// enable compile-time checks that each serial port's default protocol is compiled in (see AP_SerialManager_default_protocol_check.cpp)
#ifndef AP_SERIALMANAGER_DEFAULTS_CHECKS_ENABLED
#define AP_SERIALMANAGER_DEFAULTS_CHECKS_ENABLED 0
#endif

 // console default baud rates and buffer sizes
#ifdef DEFAULT_SERIAL0_BAUD
#define AP_SERIALMANAGER_CONSOLE_BAUD          DEFAULT_SERIAL0_BAUD
#else
#define AP_SERIALMANAGER_CONSOLE_BAUD          115200
#endif

#ifndef HAL_HAVE_SERIAL0
#define HAL_HAVE_SERIAL0 (HAL_NUM_SERIAL_PORTS > 0)
#endif
#ifndef HAL_HAVE_SERIAL1
#define HAL_HAVE_SERIAL1 (HAL_NUM_SERIAL_PORTS > 1)
#endif
#ifndef HAL_HAVE_SERIAL2
#define HAL_HAVE_SERIAL2 (HAL_NUM_SERIAL_PORTS > 2)
#endif
#ifndef HAL_HAVE_SERIAL3
#define HAL_HAVE_SERIAL3 (HAL_NUM_SERIAL_PORTS > 3)
#endif
#ifndef HAL_HAVE_SERIAL4
#define HAL_HAVE_SERIAL4 (HAL_NUM_SERIAL_PORTS > 4)
#endif
#ifndef HAL_HAVE_SERIAL5
#define HAL_HAVE_SERIAL5 (HAL_NUM_SERIAL_PORTS > 5)
#endif
#ifndef HAL_HAVE_SERIAL6
#define HAL_HAVE_SERIAL6 (HAL_NUM_SERIAL_PORTS > 6)
#endif
#ifndef HAL_HAVE_SERIAL7
#define HAL_HAVE_SERIAL7 (HAL_NUM_SERIAL_PORTS > 7)
#endif
#ifndef HAL_HAVE_SERIAL8
#define HAL_HAVE_SERIAL8 (HAL_NUM_SERIAL_PORTS > 8)
#endif
#ifndef HAL_HAVE_SERIAL9
#define HAL_HAVE_SERIAL9 (HAL_NUM_SERIAL_PORTS > 9)
#endif

#ifndef HAL_HAVE_SERIAL0_PARAMS
#define HAL_HAVE_SERIAL0_PARAMS HAL_HAVE_SERIAL0
#endif
#ifndef HAL_HAVE_SERIAL1_PARAMS
#define HAL_HAVE_SERIAL1_PARAMS HAL_HAVE_SERIAL1
#endif
#ifndef HAL_HAVE_SERIAL2_PARAMS
#define HAL_HAVE_SERIAL2_PARAMS HAL_HAVE_SERIAL2
#endif
#ifndef HAL_HAVE_SERIAL3_PARAMS
#define HAL_HAVE_SERIAL3_PARAMS HAL_HAVE_SERIAL3
#endif
#ifndef HAL_HAVE_SERIAL4_PARAMS
#define HAL_HAVE_SERIAL4_PARAMS HAL_HAVE_SERIAL4
#endif
#ifndef HAL_HAVE_SERIAL5_PARAMS
#define HAL_HAVE_SERIAL5_PARAMS HAL_HAVE_SERIAL5
#endif
#ifndef HAL_HAVE_SERIAL6_PARAMS
#define HAL_HAVE_SERIAL6_PARAMS HAL_HAVE_SERIAL6
#endif
#ifndef HAL_HAVE_SERIAL7_PARAMS
#define HAL_HAVE_SERIAL7_PARAMS HAL_HAVE_SERIAL7
#endif
#ifndef HAL_HAVE_SERIAL8_PARAMS
#define HAL_HAVE_SERIAL8_PARAMS HAL_HAVE_SERIAL8
#endif
#ifndef HAL_HAVE_SERIAL9_PARAMS
#define HAL_HAVE_SERIAL9_PARAMS HAL_HAVE_SERIAL9
#endif
