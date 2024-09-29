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

#define HAL_ESP32_BOARD_NAME "esp32-wemos"

#define TRUE						1
#define FALSE						0

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

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_WEMOS

//Inertial sensors
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, HAL_INS_MPU9250_NAME, ROTATION_PITCH_180)

//BAROMETER
//#define HAL_BARO_DEFAULT HAL_BARO_MS56XX_SPI
#define HAL_BARO_MS56XX_NAME "ms5611"
#define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, HAL_BARO_MS56XX_NAME)
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

//I2C Buses
#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_21, .scl=GPIO_NUM_22, .speed=400*KHZ, .internal=true}

//SPI Buses
#define HAL_ESP32_SPI_BUSES \
    {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}

//SPI Devices
#define HAL_ESP32_SPI_DEVICES \
    {.name= "ms5611", .bus=0, .device=0, .cs=GPIO_NUM_26, .mode = 3, .lspeed=1*MHZ, .hspeed=1*MHZ}, \
    {.name="mpu9250", .bus=0, .device=1, .cs=GPIO_NUM_5,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}

//RCIN
#define HAL_ESP32_RCIN GPIO_NUM_4

//RCOUT
#define HAL_ESP32_RCOUT \
    {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_14, GPIO_NUM_27}

//AIRSPEED
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

//COMPASS
#define AP_COMPASS_ENABLE_DEFAULT 0
#define ALLOW_ARM_NO_COMPASS

//See boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

//WIFI
#define HAL_ESP32_WIFI 1  //1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_SSID "ardupilot-esp32"
#define WIFI_PWD "ardupilot-esp32"

//UARTs
// UART_NUM_0 and UART_NUM_2 are configured to use defaults
#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1 },\
    {.port=UART_NUM_1, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17}

//ADC
//#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC TRUE
#define HAL_ESP32_ADC_PINS {\
	{ADC1_GPIO35_CHANNEL, 11, 1},\
	{ADC1_GPIO34_CHANNEL, 11, 2}\
}

//LED
#define DEFAULT_NTF_LED_TYPES Notify_LED_None

//RMT pin number
#define HAL_ESP32_RMT_RX_PIN_NUMBER 4

//SD CARD
// Do u want to use mmc or spi mode for the sd card, this is board specific,
// as mmc uses specific pins but is quicker,
// and spi is more flexible pinouts....
// dont forget vspi/hspi should be selected to NOT conflict with HAL_ESP32_SPI_BUSES

//#define HAL_ESP32_SDCARD //after enabled, uncomment one of below
//#define HAL_ESP32_SDMMC
//#define HAL_ESP32_SDSPI {.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_2, .miso=GPIO_NUM_15, .sclk=GPIO_NUM_26, .cs=GPIO_NUM_21}

#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

#define AP_RCPROTOCOL_ENABLED 0
