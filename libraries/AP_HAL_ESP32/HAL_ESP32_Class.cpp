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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_ESP32_Class.h"
#include "Scheduler.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "I2CDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Storage.h"
#include "RCOutput.h"
#include "Storage.h"
#include "RCOutput.h"
#include "AnalogIn.h"
#include "Util.h"



#ifdef HAL_ESP32_NO_MAVLINK_0
static Empty::UARTDriver uartADriver;
static ESP32::UARTDriver cons(0);
#else
static ESP32::UARTDriver uartADriver(0);
#define cons uartADriver
#endif
static ESP32::UARTDriver uartBDriver(2);
#ifdef HAL_ESP32_WIFI
static ESP32::WiFiDriver uartCDriver;
#else
static Empty::UARTDriver uartCDriver;
#endif
static ESP32::UARTDriver uartDDriver(1);
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
static Empty::UARTDriver uartHDriver;

static ESP32::I2CDeviceManager i2cDeviceManager;
static ESP32::SPIDeviceManager spiDeviceManager;
static Empty::AnalogIn analogIn;
static ESP32::Storage storageDriver;
static Empty::GPIO gpioDriver;
static ESP32::RCOutput rcoutDriver;
static ESP32::RCInput rcinDriver;
static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

extern const AP_HAL::HAL& hal;

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &uartADriver, //Console/mavlink
        &uartBDriver, //GPS 1
        &uartCDriver, //Telem 1
        &uartDDriver, //Telem 2
        &uartEDriver, //GPS 2
        &uartFDriver, //Extra 1
        &uartGDriver, //Extra 2
        &uartHDriver, //Extra 3
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        nullptr
    )
{}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
}


void AP_HAL::init()
{
}

