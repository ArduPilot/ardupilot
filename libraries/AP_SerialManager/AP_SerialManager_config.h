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

#include <AP_HAL/AP_HAL.h>
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
#define AP_SERIALMANAGER_REGISTER_ENABLED BOARD_FLASH_SIZE > 1024 && (AP_NETWORKING_ENABLED || HAL_ENABLE_DRONECAN_DRIVERS)
#endif

#ifndef AP_SERIALMANAGER_IMUOUT_ENABLED
#define AP_SERIALMANAGER_IMUOUT_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL) && AP_INERTIALSENSOR_ENABLED
#endif

// serial ports registered by AP_Networking will use IDs starting at 21 for the first port
#define AP_SERIALMANAGER_NET_PORT_1         21 // NET_P1_*

// serial ports registered by AP_DroneCAN will use IDs starting at 41/51 for the first port
#define AP_SERIALMANAGER_CAN_D1_PORT_1         41 // CAN_D1_UC_S1_*
#define AP_SERIALMANAGER_CAN_D2_PORT_1         51 // CAN_D2_UC_S1_*

 // console default baud rates and buffer sizes
#ifdef DEFAULT_SERIAL0_BAUD
#define AP_SERIALMANAGER_CONSOLE_BAUD          DEFAULT_SERIAL0_BAUD
#else
#define AP_SERIALMANAGER_CONSOLE_BAUD          115200
#endif
#define AP_SERIALMANAGER_CONSOLE_BUFSIZE_RX    128
#define AP_SERIALMANAGER_CONSOLE_BUFSIZE_TX    512

// mavlink default baud rates and buffer sizes
#define AP_SERIALMANAGER_MAVLINK_BAUD           57600
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX     128
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX     256

// LTM buffer sizes
#define AP_SERIALMANAGER_LTM_BUFSIZE_RX         0
#define AP_SERIALMANAGER_LTM_BUFSIZE_TX         32

// FrSky default baud rates, use default buffer sizes
#define AP_SERIALMANAGER_FRSKY_D_BAUD           9600
#define AP_SERIALMANAGER_FRSKY_SPORT_BAUD       57600
#define AP_SERIALMANAGER_FRSKY_BUFSIZE_RX       0
#define AP_SERIALMANAGER_FRSKY_BUFSIZE_TX       0

// GPS default baud rates and buffer sizes
// we need a 256 byte buffer for some GPS types (eg. UBLOX)
#define AP_SERIALMANAGER_GPS_BAUD               230400
#define AP_SERIALMANAGER_GPS_BUFSIZE_RX         256
#define AP_SERIALMANAGER_GPS_BUFSIZE_TX         16

// AlexMos Gimbal protocol default baud rates and buffer sizes
#define AP_SERIALMANAGER_ALEXMOS_BAUD           115200
#define AP_SERIALMANAGER_ALEXMOS_BUFSIZE_RX     128
#define AP_SERIALMANAGER_ALEXMOS_BUFSIZE_TX     128

#define AP_SERIALMANAGER_GIMBAL_BAUD            115200
#define AP_SERIALMANAGER_GIMBAL_BUFSIZE_RX      128
#define AP_SERIALMANAGER_GIMBAL_BUFSIZE_TX      128

#define AP_SERIALMANAGER_VOLZ_BAUD           115
#define AP_SERIALMANAGER_VOLZ_BUFSIZE_RX     128
#define AP_SERIALMANAGER_VOLZ_BUFSIZE_TX     128

#define AP_SERIALMANAGER_ROBOTIS_BUFSIZE_RX  128
#define AP_SERIALMANAGER_ROBOTIS_BUFSIZE_TX  128

// MegaSquirt EFI protocol
#define AP_SERIALMANAGER_EFI_MS_BAUD           115
#define AP_SERIALMANAGER_EFI_MS_BUFSIZE_RX     512
#define AP_SERIALMANAGER_EFI_MS_BUFSIZE_TX     16

// SBUS servo outputs
#define AP_SERIALMANAGER_SBUS1_BAUD           100000
#define AP_SERIALMANAGER_SBUS1_BUFSIZE_RX     16
#define AP_SERIALMANAGER_SBUS1_BUFSIZE_TX     32

#define AP_SERIALMANAGER_SLCAN_BAUD             115200
#define AP_SERIALMANAGER_SLCAN_BUFSIZE_RX       128
#define AP_SERIALMANAGER_SLCAN_BUFSIZE_TX       128

// MSP protocol default buffer sizes
#define AP_SERIALMANAGER_MSP_BUFSIZE_RX     128
#define AP_SERIALMANAGER_MSP_BUFSIZE_TX     256
#define AP_SERIALMANAGER_MSP_BAUD           115200

// IMU OUT protocol
#define AP_SERIALMANAGER_IMUOUT_BAUD           921600
#define AP_SERIALMANAGER_IMUOUT_BUFSIZE_RX     128
#define AP_SERIALMANAGER_IMUOUT_BUFSIZE_TX     2048

// PPP protocol
#define AP_SERIALMANAGER_PPP_BAUD           921600
#define AP_SERIALMANAGER_PPP_BUFSIZE_RX     4096
#define AP_SERIALMANAGER_PPP_BUFSIZE_TX     4096

#ifndef HAL_HAVE_SERIAL0
#define HAL_HAVE_SERIAL0 HAL_NUM_SERIAL_PORTS > 0
#endif
#ifndef HAL_HAVE_SERIAL1
#define HAL_HAVE_SERIAL1 HAL_NUM_SERIAL_PORTS > 1
#endif
#ifndef HAL_HAVE_SERIAL2
#define HAL_HAVE_SERIAL2 HAL_NUM_SERIAL_PORTS > 2
#endif
#ifndef HAL_HAVE_SERIAL3
#define HAL_HAVE_SERIAL3 HAL_NUM_SERIAL_PORTS > 3
#endif
#ifndef HAL_HAVE_SERIAL4
#define HAL_HAVE_SERIAL4 HAL_NUM_SERIAL_PORTS > 4
#endif
#ifndef HAL_HAVE_SERIAL5
#define HAL_HAVE_SERIAL5 HAL_NUM_SERIAL_PORTS > 5
#endif
#ifndef HAL_HAVE_SERIAL6
#define HAL_HAVE_SERIAL6 HAL_NUM_SERIAL_PORTS > 6
#endif
#ifndef HAL_HAVE_SERIAL7
#define HAL_HAVE_SERIAL7 HAL_NUM_SERIAL_PORTS > 7
#endif
#ifndef HAL_HAVE_SERIAL8
#define HAL_HAVE_SERIAL8 HAL_NUM_SERIAL_PORTS > 8
#endif
#ifndef HAL_HAVE_SERIAL9
#define HAL_HAVE_SERIAL9 HAL_NUM_SERIAL_PORTS > 9
#endif
