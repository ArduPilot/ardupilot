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

#define HAL_ESP32_BOARD_NAME "esp32-buzz"

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


//#define CONFIG_HAL_BOARD 12
//#define HAL_BOARD_ESP32 12

//INS choices:
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
//#define HAL_INS_MPU9250_NAME "MPU9250"

// or this:
//#define HAL_INS_DEFAULT HAL_INS_ICM20XXX_I2C
//#define HAL_INS_ICM20XXX_I2C_BUS 0
//#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)

// BARO choices:
#define HAL_BARO_DEFAULT HAL_BARO_BMP280_SPI
#define HAL_BARO_BMP280_NAME "BMP280"
// or one of these:
//#define HAL_BARO_DEFAULT HAL_BARO_MS5837_I2C
// or: GPIO 34
//#define HAL_BARO_ANALOG_PIN (6)

// MAG/COMPASS choices:
// or others:
//#define HAL_COMPASS_ICM20948_I2C_ADDR (0x68)
//#define HAL_COMPASS_AK09916_I2C_BUS 0
//#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)
//#define HAL_COMPASS_MAX_SENSORS 3

// IMU probing:
//#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x68, ROTATION_YAW_270)
// MAG/COMPASS probing:
//#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_NONE));
// BARO probing:
//#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x77)

// allow boot without a baro
#define HAL_BARO_ALLOW_INIT_NO_BARO 1


// ADC is available on lots of pints on the esp32, but adc2 cant co-exist with wifi we choose to allow ADC on :
//#define HAL_DISABLE_ADC_DRIVER 1
#define TRUE 1
#define HAL_USE_ADC TRUE

// the pin number, the gain/multiplier associated with it, the ardupilot name for the pin in parameter/s.
//
// two different pin numbering schemes, both are ok, but only one at a time:
#define HAL_ESP32_ADC_PINS_OPTION1 {\
	{ADC1_GPIO35_CHANNEL, 11, 1},\
	{ADC1_GPIO34_CHANNEL, 11, 2},\
	{ADC1_GPIO39_CHANNEL, 11, 3},\
	{ADC1_GPIO36_CHANNEL, 11, 4}\
}
#define HAL_ESP32_ADC_PINS_OPTION2 {\
	{ADC1_GPIO35_CHANNEL, 11, 35},\
	{ADC1_GPIO34_CHANNEL, 11, 34},\
	{ADC1_GPIO39_CHANNEL, 11, 39},\
	{ADC1_GPIO36_CHANNEL, 11, 36}\
}
// pick one:
//#define HAL_ESP32_ADC_PINS HAL_ESP32_ADC_PINS_OPTION1
#define HAL_ESP32_ADC_PINS HAL_ESP32_ADC_PINS_OPTION2



#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1


#define HAL_INS_MPU9250_NAME "mpu9250"

// uncommenting one or more of these will give more console debug in certain areas..
//#define INSEDEBUG 1
//#define STORAGEDEBUG 1
//#define SCHEDDEBUG 1
//#define FSDEBUG 1
//#define BUSDEBUG 1

#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)
//#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_MPU9250_NAME, ROTATION_ROLL_180)


#define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(BMP280, "bmp280")

// 2 use udp, 1 use tcp...  for udp,client needs to connect as UDPCL in missionplanner etc to 192.168.4.1 port 14550
#define HAL_ESP32_WIFI 1

// tip: if u are ok getting mavlink-over-tcp or mavlink-over-udp and want to disable mavlink-over-serial-usb
//then set ardupilot parameter SERIAL0_PROTOCOL = 0 and reboot.
// u also will/may want..
//LOG_BACKEND_TYPE 1
//LOG_DISARMED 1
//SERIAL0_PROTOCOL 0


// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

#define WIFI_SSID "ardupilot123"
#define WIFI_PWD "ardupilot123"

//RCOUT which pins are used?

#define HAL_ESP32_RCOUT { GPIO_NUM_25,GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_22, GPIO_NUM_21 }

// SPI BUS setup, including gpio, dma, etc
// note... we use 'vspi' for the bmp280 and mpu9250
#define HAL_ESP32_SPI_BUSES \
    {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}
// tip:  VSPI_HOST  is an alternative name for esp's SPI3
//#define HAL_ESP32_SPI_BUSES {}

// SPI per-device setup, including speeds, etc.
#define HAL_ESP32_SPI_DEVICES \
    {.name= "bmp280", .bus=0, .device=0, .cs=GPIO_NUM_26, .mode = 3, .lspeed=1*MHZ, .hspeed=1*MHZ}, \
    {.name="mpu9250", .bus=0, .device=1, .cs=GPIO_NUM_5,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}
//#define HAL_ESP32_SPI_DEVICES {}

//I2C bus list
#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_13, .scl=GPIO_NUM_12, .speed=400*KHZ, .internal=true}
//#define HAL_ESP32_I2C_BUSES {} // using this embty block appears to cause crashes?


// rcin on what pin?
#define HAL_ESP32_RCIN GPIO_NUM_4


//HARDWARE UARTS
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_3, .tx=GPIO_NUM_1 },{.port=UART_NUM_1, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17 }

#define HAVE_FILESYSTEM_SUPPORT 1

// Do u want to use mmc or spi mode for the sd card, this is board specific ,
//  as mmc uses specific pins but is quicker,
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

#define HAL_ESP32_RMT_RX_PIN_NUMBER 4
