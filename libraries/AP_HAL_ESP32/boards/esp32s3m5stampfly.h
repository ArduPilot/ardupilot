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

// make sensor selection clearer
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))
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

#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(BMI270, "bmi270", ROTATION_ROLL_180_YAW_90)

#define HAL_BARO_PROBE_LIST         PROBE_BARO_I2C(BMP280, 0, 0x76)

#define AP_COMPASS_ENABLE_DEFAULT 0
#define ALLOW_ARM_NO_COMPASS
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

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

//RCOUT which pins are used?
// r-up, l-down, l-up, r-down (quad X order)
#define HAL_ESP32_RCOUT { GPIO_NUM_42, GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_41 }

// SPI BUS setup, including gpio, dma, etc
#define HAL_ESP32_SPI_BUSES \
    {.host=SPI2_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_14, .miso=GPIO_NUM_43, .sclk=GPIO_NUM_44}
// SPI2 was used in original firmware

// SPI per-device setup, including speeds, etc.
// the optical flow sensor (pixartflow) is on the same bus as the main IMU so
// a) we have to list it here so its CS gets deasserted and the IMU can talk, but
// b) we name it something the driver won't find because it's too slow and ties up the bus
#define HAL_ESP32_SPI_DEVICES \
    {.name= "bmi270", .bus=0, .device=0, .cs=GPIO_NUM_46, .mode=3, .lspeed=10*MHZ, .hspeed=10*MHZ}, \
    {.name="pixartflow_disabled", .bus=0, .device=1, .cs=GPIO_NUM_12, .mode=3, .lspeed=2*MHZ, .hspeed=2*MHZ},

//I2C bus list. bus 0 is internal, bus 1 is the red grove connector
#define HAL_ESP32_I2C_BUSES \
  {.port=I2C_NUM_0, .sda=GPIO_NUM_3, .scl=GPIO_NUM_4, .speed=400*KHZ, .internal=true}, \
  {.port=I2C_NUM_1, .sda=GPIO_NUM_13, .scl=GPIO_NUM_15, .speed=400*KHZ, .internal=false}

// rcin on what pin? enable AP_RCPROTOCOL_ENABLED below if using
// currently on another unmapped pin
//#define HAL_ESP32_RCIN GPIO_NUM_16
#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_16


// HARDWARE UARTS. UART 0 apparently goes over USB? so we assign it to pins we
// don't have mapped to anything. might be fixable in the HAL...
// UART 1 (SERIAL3) is the black grove connector
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_18, .tx=GPIO_NUM_17 }, \
  {.port=UART_NUM_1, .rx=GPIO_NUM_1, .tx=GPIO_NUM_2 }

// log over MAVLink by default
#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 1
#define HAL_LOGGING_BACKENDS_DEFAULT 2

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define AP_RCPROTOCOL_ENABLED 0

#define AP_FILESYSTEM_ESP32_ENABLED 0
#define AP_SCRIPTING_ENABLED 0
