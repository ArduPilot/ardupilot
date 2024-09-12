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

#define HAL_ESP32_BOARD_NAME "esp32-diy"

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

#define HAL_INS_DEFAULT HAL_INS_ICM20XXX_I2C
#define HAL_INS_ICM20XXX_I2C_BUS 0
#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)

#define HAL_LOGGING_STACK_SIZE 4096

//#define HAL_BARO_DEFAULT HAL_BARO_MS5837_I2C
//GPIO 34
//#define HAL_BARO_ANALOG_PIN (6)

#define HAL_COMPASS_ICM20948_I2C_ADDR (0x68)
#define HAL_COMPASS_AK09916_I2C_BUS 0
#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)
#define HAL_COMPASS_MAX_SENSORS 3

#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x68, ROTATION_ROLL_180)

#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_ROLL_180_YAW_270));

#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x77)

#define HAL_ESP32_WIFI 1 //To define tcp wifi

//TODO RCOUT config
#define HAL_ESP32_RCOUT {GPIO_NUM_15, GPIO_NUM_2, GPIO_NUM_0, GPIO_NUM_4}

#define HAL_ESP32_SPI_BUSES {}

#define HAL_ESP32_SPI_DEVICES {}

#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_5, .scl=GPIO_NUM_18, .speed=400*KHZ, .internal=true, .soft=true},\
	{.port=I2C_NUM_1, .sda=GPIO_NUM_22, .scl=GPIO_NUM_23, .speed=400*KHZ, .internal=true, .soft=false}

// GPIO36
#define HAL_BATT_VOLT_PIN (0)
#define HAL_BATT_VOLT_SCALE (18.1)
//GPIO 32
#define HAL_BATT_CURR_PIN (4)
#define HAL_BATT_CURR_SCALE (36)

// ADC is available on lots of pints on the esp32, but adc2 cant co-exist with wifi we choose to allow ADC on :
//#define HAL_DISABLE_ADC_DRIVER 1
#define TRUE 1
#define HAL_USE_ADC TRUE

#ifndef ENABLE_HEAP
#define ENABLE_HEAP 0
#endif

// the pin number, the gain/multiplier associated with it, the ardupilot name for the pin in parameter/s.
#define HAL_ESP32_ADC_PINS {\
	{ADC1_GPIO36_CHANNEL, 11, 1},\
	{ADC1_GPIO32_CHANNEL, 11, 2}\
}

#define HAL_ESP32_RCIN GPIO_NUM_17

#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_3, .tx=GPIO_NUM_1 },\
	{.port=UART_NUM_1, .rx=GPIO_NUM_39, .tx=GPIO_NUM_33 },\
	{.port=UART_NUM_2, .rx=GPIO_NUM_34, .tx=GPIO_NUM_25 }

#define AP_FILESYSTEM_ESP32_ENABLED 1
#define HAL_ESP32_SDCARD 1
#define LOGGER_MAVLINK_SUPPORT 1
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"

#define HAL_LOGGING_BACKENDS_DEFAULT 2


#define HAL_ESP32_SDSPI \
   {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_19, .miso=GPIO_NUM_35, .sclk=GPIO_NUM_12, .cs=GPIO_NUM_21}


#define HAL_LOGGING_BACKENDS_DEFAULT 2




#define HAL_ESP32_RMT_RX_PIN_NUMBER 4
