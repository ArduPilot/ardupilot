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

#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_MPU9250_NAME "MPU9250"

#define HAL_BARO_DEFAULT HAL_BARO_BMP280_SPI
#define HAL_BARO_BMP280_NAME "BMP280"

#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963_MPU9250

#define HAL_ESP32_WIFI 1

#define HAL_ESP32_RCOUT {GPIO_NUM_27, GPIO_NUM_13, GPIO_NUM_22, GPIO_NUM_21}

#define HAL_ESP32_SPI_BUSES \
    {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}

#define HAL_ESP32_SPI_DEVICES \
    {.name="MPU9250", .bus=0, .device=1, .cs=GPIO_NUM_5,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ},\
    {.name= "BMP280", .bus=0, .device=2, .cs=GPIO_NUM_26, .mode = 3, .lspeed=1*MHZ, .hspeed=1*MHZ}

#define HAL_ESP32_I2C_BUSES {}
    
#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1 }
