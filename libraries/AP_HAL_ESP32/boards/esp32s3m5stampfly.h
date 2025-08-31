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

// GROVE EXPANSION
//
// The Stampfly has two Seeed Studio Grove connectors to attach peripherals. By
// default, the left (red) connector is I2C (with no other devices on the bus)
// and the right (black) connector is SERIAL3. However, the two signal pins in
// each can be reassigned by editing this file.
//
// Grove Red (I2C)
// 1 (toward front): GPIO_NUM_15, SCL (has 4.7kΩ pullup to 3.3V)
// 2               : GPIO_NUM_13, SDA (has 4.7kΩ pullup to 3.3V)
// 3               : +V (+5V if plugged into USB, else +Vbat)
// 4 (toward rear) : GND
//
// Grove Black (SERIAL3)
// 1 (toward rear) : GPIO_NUM_1, RX
// 2               : GPIO_NUM_3, TX
// 3               : +V (+5V if plugged into USB, else +Vbat)
// 4 (toward front): GND

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
//------------------------------------


#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3M5STAMPFLY
// make sensor selection clearer

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

#define HAL_ESP32_BOARD_NAME "esp32s3m5stampfly"

#define AP_COMPASS_ENABLE_DEFAULT 0
#define ALLOW_ARM_NO_COMPASS

// no ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

#define AP_BATTERY_INA3221_ENABLED 1
#define HAL_BATTMON_INA3221_SHUNT_OHMS 0.01
#define HAL_BATTMON_INA3221_SHUNT_CONV_TIME_SEL HAL_BATTMON_INA3221_CONV_TIME_4156US
#define HAL_BATTMON_INA3221_BUS_CONV_TIME_SEL HAL_BATTMON_INA3221_CONV_TIME_4156US
#define HAL_BATTMON_INA3221_AVG_MODE_SEL HAL_BATTMON_INA3221_AVG_MODE_16

// 2 use udp, 1 use tcp...  for udp,client needs to connect as UDPCL in missionplanner etc to 192.168.4.1 port 14550
#define HAL_ESP32_WIFI 2

#define WIFI_SSID "ardupilot123"
#define WIFI_PWD "ardupilot123"

// SPI BUS setup, including gpio, dma, etc is in the hwdef.dat
// SPI per-device setup, including speeds, etc. is in the hwdef.dat

// rcin on what pin? enable AP_RCPROTOCOL_ENABLED below if using
// currently on another unmapped pin
//#define HAL_ESP32_RCIN GPIO_NUM_16
#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_16


// log over MAVLink by default
#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 1
#define HAL_LOGGING_BACKENDS_DEFAULT 2

#define AP_RCPROTOCOL_ENABLED 0

#define AP_FILESYSTEM_ESP32_ENABLED 0
#define AP_SCRIPTING_ENABLED 0
