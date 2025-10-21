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
 * This board that does not contain any sensors (but pins are active), it is a great help for a novice user,
 * by flashing an empty board, you can connect via Mavlink (Mission Planner - MP) and gradually add sensors.
 * If you had some sensor configured and it doesn't work then the MP connection does not work and then you may not know what to do next.
*/

#pragma once

#define HAL_ESP32_BOARD_NAME "esp32-empty"

#define TRUE  1
#define FALSE 0

//Protocols
// list of protocols/enum:  ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
// default protocols:    ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
// ESP32 serials:    AP_HAL_ESP32/HAL_ESP32_Class.cpp

//#define DEFAULT_SERIAL0_PROTOCOL        SerialProtocol_MAVLink2   //A  UART0: Always: Console, MAVLink2
//#define DEFAULT_SERIAL0_BAUD            AP_SERIALMANAGER_CONSOLE_BAUD/1000  //115200

//#define DEFAULT_SERIAL1_PROTOCOL        SerialProtocol_MAVLink2   //C  WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define DEFAULT_SERIAL1_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_MAVLink2   //D  UART2
#define DEFAULT_SERIAL2_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_GPS        //B  UART1: GPS1
#define DEFAULT_SERIAL3_BAUD            AP_SERIALMANAGER_GPS_BAUD/1000    //38400, Can not define default baudrate here (by config only)
//#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_None       //B
//#define DEFAULT_SERIAL3_BAUD            (115200/1000)

#define DEFAULT_SERIAL4_PROTOCOL        SerialProtocol_None       //E
#define DEFAULT_SERIAL5_BAUD            (115200/1000)

#define DEFAULT_SERIAL5_PROTOCOL        SerialProtocol_None       //F
#define DEFAULT_SERIAL5_BAUD            (115200/1000)

#define DEFAULT_SERIAL6_PROTOCOL        SerialProtocol_None       //G
#define DEFAULT_SERIAL6_BAUD            (115200/1000)

#define DEFAULT_SERIAL7_PROTOCOL        SerialProtocol_None       //H
#define DEFAULT_SERIAL7_BAUD            (115200/1000)

#define DEFAULT_SERIAL8_PROTOCOL        SerialProtocol_None       //I
#define DEFAULT_SERIAL8_BAUD            (115200/1000)

#define DEFAULT_SERIAL9_PROTOCOL        SerialProtocol_None       //J
#define DEFAULT_SERIAL9_BAUD            (115200/1000)

//Inertial sensors
#define HAL_INS_DEFAULT HAL_INS_NONE
//#define HAL_INS_DEFAULT HAL_INS_MPU9250_I2C
//#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
//#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 0, 0x68, ROTATION_NONE)

//I2C Buses
#define HAL_ESP32_I2C_BUSES {.port=I2C_NUM_0, .sda=GPIO_NUM_13, .scl=GPIO_NUM_14, .speed=400*KHZ, .internal=true, .soft=true}

//SPI Buses
#define HAL_ESP32_SPI_BUSES {}

//SPI Devices
#define HAL_ESP32_SPI_DEVICES {}

//RCIN
#define HAL_ESP32_RCIN GPIO_NUM_36

//RCOUT
#define HAL_ESP32_RCOUT {GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_22, GPIO_NUM_21}

//AIRSPEED
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

//BAROMETER
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

//IMU
// #define AP_INERTIALSENSOR_ENABLED 1
// #define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0

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
    {.port=UART_NUM_1, .rx=GPIO_NUM_34, .tx=GPIO_NUM_18},\
    {.port=UART_NUM_2, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17}

//ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

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
