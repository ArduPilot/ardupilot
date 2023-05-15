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

/*
 * This board is tomte76's version of esp32empty.h to work with an aliexpress WROOM32 devkit clone, mpu9255 spi, bmp280 i2c, micro sd on spi2 and NEO M8 clone on UART.
 */
#pragma once

#define HAL_ESP32_BOARD_NAME "esp32-tomte76"
// CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_TOMTE76

#define TRUE						1
#define FALSE						0

#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))

//Protocols
//list of protocols/enum:	ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
//default protocols:		ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
//ESP32 serials:		AP_HAL_ESP32/HAL_ESP32_Class.cpp

//#define HAL_SERIAL0_PROTOCOL				SerialProtocol_MAVLink2			//A	UART0: Always: Console, MAVLink2
//#define HAL_SERIAL0_BAUD				AP_SERIALMANAGER_CONSOLE_BAUD/1000	//115200

//#define HAL_SERIAL1_PROTOCOL				SerialProtocol_MAVLink2			//C	WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define HAL_SERIAL1_BAUD				AP_SERIALMANAGER_MAVLINK_BAUD/1000	//57600

//#define HAL_SERIAL2_PROTOCOL				SerialProtocol_GPS			//D	UART2: Always: MAVLink2 on ESP32
//#define HAL_SERIAL2_BAUD				AP_SERIALMANAGER_GPS_BAUD/1000		//38400

//#define HAL_SERIAL3_PROTOCOL				SerialProtocol_GPS			//B	UART1: GPS1
//#define HAL_SERIAL4_BAUD				AP_SERIALMANAGER_GPS_BAUD/1000		//38400, Can not define default baudrate here (by config only)

//#define HAL_SERIAL4_PROTOCOL				SerialProtocol_None			//E
//#define HAL_SERIAL4_BAUD				AP_SERIALMANAGER_GPS_BAUD/1000		//38400, Can not define default baudrate here (by config only)

/*#define HAL_SERIAL5_PROTOCOL				SerialProtocol_None			//F
#define HAL_SERIAL5_BAUD				(115200/1000)

#define HAL_SERIAL6_PROTOCOL				SerialProtocol_None			//G
#define HAL_SERIAL6_BAUD				(115200/1000)

#define HAL_SERIAL7_PROTOCOL				SerialProtocol_None			//H
#define HAL_SERIAL7_BAUD				(115200/1000)

#define HAL_SERIAL8_PROTOCOL				SerialProtocol_None			//I
#define HAL_SERIAL8_BAUD				(115200/1000)

#define HAL_SERIAL9_PROTOCOL				SerialProtocol_None			//J
#define HAL_SERIAL9_BAUD				(115200/1000)*/

//Inertial sensors
#define HAL_INS_DEFAULT                              	HAL_INS_MPU9250_SPI
#define HAL_INS_PROBE_LIST 			     	PROBE_IMU_SPI(Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)
#define HAL_MAG_PROBE_LIST 			     	PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
#define HAL_INS_MPU9250_NAME 				"mpu9250"

//Baro
#define HAL_BARO_DEFAULT 				HAL_BARO_BMP280_I2C
#define HAL_BARO_BMP085_NAME 				"bmp280"
#define HAL_BARO_PROBE_LIST 				PROBE_BARO_I2C(BMP280, 0, 0x76)

//I2C Buses
#define HAL_ESP32_I2C_BUSES				{.port=I2C_NUM_0, .sda=GPIO_NUM_21, .scl=GPIO_NUM_22, .speed=400*KHZ, .internal=true}

//SPI Buses
#define HAL_ESP32_SPI_BUSES				{.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}

//SPI Devices
#define HAL_ESP32_SPI_DEVICES				{.name="mpu9250", .bus=0, .device=0, .cs=GPIO_NUM_5, .mode = 0, .lspeed=2*MHZ, .hspeed=2*MHZ}

//RCIN
#define HAL_ESP32_RCIN					GPIO_NUM_36

//RCOUT
#define HAL_ESP32_RCOUT					{ GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32 }

//BAROMETER
#define HAL_BARO_ALLOW_INIT_NO_BARO			1

//COMPASS
#define ALLOW_ARM_NO_COMPASS				1

//WIFI
#define HAL_ESP32_WIFI					1	//1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_SSID					"ardupilot123"
#define WIFI_PWD					"ardupilot123"

//UARTs
#define HAL_ESP32_UART_DEVICES 				{.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1},\
    							{.port=UART_NUM_1, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17}

//ADC
#define HAL_DISABLE_ADC_DRIVER				1
#define HAL_USE_ADC					0

//LED
#define BUILD_DEFAULT_LED_TYPE				Notify_LED_None

//RMT pin number
#define HAL_ESP32_RMT_RX_PIN_NUMBER			4

//SD CARD
// Do u want to use mmc or spi mode for the sd card, this is board specific ,
//  as mmc uses specific pins but is quicker,
// and spi is more flexible pinouts....  dont forget vspi/hspi should be selected to NOT conflict with HAL_ESP32_SPI_BUSES

//#define HAL_ESP32_SDCARD //after enabled, uncomment one of below
//#define HAL_ESP32_SDMMC
#define HAL_ESP32_SDCARD 				1
#define HAL_ESP32_SDSPI 				{.host=HSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_13, .miso=GPIO_NUM_12, .sclk=GPIO_NUM_14, .cs=GPIO_NUM_15}

#define HAL_LOGGING_FILESYSTEM_ENABLED			1
#define HAL_LOGGING_DATAFLASH_ENABLED			0
#define HAL_LOGGING_MAVLINK_ENABLED			0

#define HAL_BOARD_LOG_DIRECTORY				"/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY			"/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY				"/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY			"/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT			1
