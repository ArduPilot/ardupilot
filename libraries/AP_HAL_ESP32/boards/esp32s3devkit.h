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

#define HAL_ESP32_BOARD_NAME "esp32s3devkit"
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT
// make sensor selection clearer

#define AP_RCPROTOCOL_ENABLED 1

#define AP_RCPROTOCOL_IBUS_ENABLED 1

#define AP_RCPROTOCOL_PPMSUM_ENABLED 0
#define AP_RCPROTOCOL_SBUS_ENABLED 0
#define AP_RCPROTOCOL_FASTSBUS_ENABLED 0
#define AP_RCPROTOCOL_DSM_ENABLED 0
#define AP_RCPROTOCOL_SUMD_ENABLED 0
#define AP_RCPROTOCOL_SRXL_ENABLED 0
#define AP_RCPROTOCOL_SBUS_NI_ENABLED 0
#define AP_RCPROTOCOL_SRXL2_ENABLED 0
#define AP_RCPROTOCOL_CRSF_ENABLED 0
#define AP_RCPROTOCOL_FPORT2_ENABLED 0
#define AP_RCPROTOCOL_ST24_ENABLED 0
#define AP_RCPROTOCOL_FPORT_ENABLED 0
#define AP_RCPROTOCOL_DRONECAN_ENABLED 0
#define AP_RCPROTOCOL_GHST_ENABLED 0
#define AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED 0
#define AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED 0
#define AP_RCPROTOCOL_UDP_ENABLED 0
#define AP_RCPROTOCOL_FDM_ENABLED 0
#define AP_RCPROTOCOL_RADIO_ENABLED 0

#define HAL_LOGGING_ENABLED 1

#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))

//ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(1, ROTATION_YAW_270));

#define AP_INERTIALSENSOR_ENABLED 1

#define HAL_INS_DEFAULT HAL_INS_MPU9250_I2C
#define HAL_INS_MPU9250_NAME "MPU9250"
// IMU probing:
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 0, 0x68, ROTATION_YAW_180)

// BARO choices:
#define AP_BARO_ENABLED 1

#define AP_BARO_BMP280_ENABLED 1
#define HAL_BARO_DEFAULT HAL_BARO_BMP280_I2C
#define HAL_BARO_BMP280_NAME "BMP280"

// BARO probing:
#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x76)

#define HAL_BARO_ALLOW_INIT_NO_BARO 1


#define AP_COMPASS_ENABLE 0
/*
#define AP_COMPASS_AK8963_ENABLED 1
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963_MPU9250

#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(GET_I2C_DEVICE(0,0x0c), ROTATION_YAW_270));
*/

#define ALLOW_ARM_NO_COMPASS
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

// no ADC
//#define HAL_DISABLE_ADC_DRIVER 0
#define HAL_USE_ADC 1

#define HAL_ESP32_ADC_PINS_OPTION1 { \
    {ADC1_GPIO1_CHANNEL, 11, 1}, \
    {ADC1_GPIO5_CHANNEL, 11, 2} \
}

#define HAL_ESP32_ADC_PINS HAL_ESP32_ADC_PINS_OPTION1

// /* -- Disable WiFi
// 2 use udp, 1 use tcp...  for udp,client needs to connect as UDPCL in missionplanner etc to 192.168.4.1 port 14550
#define HAL_ESP32_WIFI 2
#define WIFI_SSID "ardupilot123"
#define WIFI_PWD "ardupilot123"
// */

//RCOUT which pins are used?

#define HAL_ESP32_RCOUT { GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_16 }

// SPI BUS setup, including gpio, dma, etc
// note... we use 'vspi' for the bmp280 and mpu9250
#define HAL_ESP32_SPI_BUSES {}
//    {.host=SPI3_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_35, .miso=GPIO_NUM_37, .sclk=GPIO_NUM_36}

// SPI per-device setup, including speeds, etc.
#define HAL_ESP32_SPI_DEVICES {}
//    {.name= "mpu9250", .bus=0, .device=0, .cs=GPIO_NUM_7,  .mode = 0, .lspeed=2*MHZ, .hspeed=1*MHZ},
//	  {.name= "bmp280",  .bus=0, .device=1, .cs=GPIO_NUM_17, .mode = 3, .lspeed=1*MHZ, .hspeed=1*MHZ}

//I2C bus list
#define HAL_ESP32_I2C_BUSES {.port=I2C_NUM_0, .sda=GPIO_NUM_8, .scl=GPIO_NUM_9, .speed=400*KHZ, .internal=false, .soft = false }

// rcin on what pin?
#define HAL_ESP32_RCIN GPIO_NUM_4


//HARDWARE UARTS
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_44, .tx=GPIO_NUM_43 }, \
  {.port=UART_NUM_1, .rx=GPIO_NUM_17, .tx=GPIO_NUM_18 }

#define HAVE_FILESYSTEM_SUPPORT 1

// Do u want to use mmc or spi mode for the sd card, this is board specific ,
//  as mmc uses specific pins but is quicker,
/*
SDCARD connection
ESP32	SDCARD
D2	D0/PIN7
D14	CLK/PIN5
D15	CMD/PIN2
GND	Vss1/PIN3 and Vss2/PIN6
3.3v	Vcc/PIN4
*/

#define HAL_ESP32_SDMMC 1
// and spi is more flexible pinouts....  dont forget vspi/hspi should be selected to NOT conflict with SPI_BUSES above
//#define HAL_ESP32_SDSPI {.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_2, .miso=GPIO_NUM_15, .sclk=GPIO_NUM_14, .cs=GPIO_NUM_21}

#define HAL_ESP32_SDCARD 1
#define LOGGER_MAVLINK_SUPPORT 1
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_OS_POSIX_IO 1

// this becomes the default value for the ardupilot param LOG_BACKEND_TYPE, which most ppl want to be 1, for log-to-flash
// setting to 2 means log-over-mavlink to a companion computer etc.
#define HAL_LOGGING_BACKENDS_DEFAULT 1
