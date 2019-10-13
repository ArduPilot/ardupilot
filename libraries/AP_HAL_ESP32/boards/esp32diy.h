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

//#define HAL_BARO_DEFAULT HAL_BARO_MS5837_I2C
//GPIO 34
//#define HAL_BARO_ANALOG_PIN (6)

#define HAL_COMPASS_ICM20948_I2C_ADDR (0x68)
#define HAL_COMPASS_AK09916_I2C_BUS 0
#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)

#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x68, ROTATION_YAW_270)

#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_NONE));

#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x77)

#define HAL_ESP32_WIFI 1

//TODO RCOUT config
#define HAL_ESP32_RCOUT {GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18}

#define HAL_ESP32_SPI_BUSES {}

#define HAL_ESP32_SPI_DEVICES {}

#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_14, .scl=GPIO_NUM_27, .speed=400*KHZ, .internal=true},\
	{.port=I2C_NUM_1, .sda=GPIO_NUM_25, .scl=GPIO_NUM_26, .speed=400*KHZ, .internal=true}


#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_34, .tx=GPIO_NUM_1 },\
	{.port=UART_NUM_1, .rx=GPIO_NUM_39, .tx=GPIO_NUM_33 },\
	{.port=UART_NUM_2, .rx=GPIO_NUM_36, .tx=GPIO_NUM_32 }



