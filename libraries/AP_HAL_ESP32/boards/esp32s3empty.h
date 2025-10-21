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
 */
#pragma once

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3EMPTY
// make sensor selection clearer

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

#define HAL_ESP32_BOARD_NAME "esp32s3empty"

#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_14

// no sensors
#define HAL_INS_DEFAULT HAL_INS_NONE

#define HAL_BARO_ALLOW_INIT_NO_BARO 1

#define AP_COMPASS_ENABLE_DEFAULT 0
#define ALLOW_ARM_NO_COMPASS
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

// no ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

// 2 use udp, 1 use tcp...  for udp,client needs to connect as UDPCL in missionplanner etc to 192.168.4.1 port 14550
#define HAL_ESP32_WIFI 2

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

#define WIFI_SSID "ardupilot123"
#define WIFI_PWD "ardupilot123"

//RCOUT which pins are used?

#define HAL_ESP32_RCOUT { GPIO_NUM_11,GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_8, GPIO_NUM_7, GPIO_NUM_6 }

// SPI BUS setup, including gpio, dma, etc
// note... we use 'vspi' for the bmp280 and mpu9250
#define HAL_ESP32_SPI_BUSES {}

// SPI per-device setup, including speeds, etc.
#define HAL_ESP32_SPI_DEVICES {}

//I2C bus list
#define HAL_ESP32_I2C_BUSES {.port=I2C_NUM_0, .sda=GPIO_NUM_13, .scl=GPIO_NUM_14, .speed=400*KHZ, .internal=true, .soft=true}

// rcin on what pin?
#define HAL_ESP32_RCIN GPIO_NUM_14


//HARDWARE UARTS
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_44, .tx=GPIO_NUM_43 },{.port=UART_NUM_1, .rx=GPIO_NUM_17, .tx=GPIO_NUM_18 }

#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

#define AP_RCPROTOCOL_ENABLED 0

#define AP_FILESYSTEM_ESP32_ENABLED 0
#define AP_SCRIPTING_ENABLED 0
#define HAL_USE_EMPTY_STORAGE 1

