/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include <assert.h>

#include "HAL_FLYMAPLE_Class.h"
#include "AP_HAL_FLYMAPLE_Private.h"

using namespace AP_HAL_FLYMAPLE_NS;
class HardwareSerial;
extern HardwareSerial Serial1; // Serial1 is labelled "COM1" on Flymaple pins 7 and 8
extern HardwareSerial Serial2; // Serial2 is Flymaple pins 0 and 1
extern HardwareSerial Serial3; // Serial3 is labelled "GPS" on Flymaple pins 29 and 30

static FLYMAPLEUARTDriver uartADriver(&Serial1); // AP Console and highspeed mavlink
static FLYMAPLEUARTDriver uartBDriver(&Serial2); // AP GPS connection
static FLYMAPLEUARTDriver uartCDriver(&Serial3); // Optional AP telemetry radio
static FLYMAPLESemaphore  i2cSemaphore;
static FLYMAPLEI2CDriver  i2cDriver(&i2cSemaphore);
static FLYMAPLESPIDeviceManager spiDeviceManager;
static FLYMAPLEAnalogIn analogIn;
static FLYMAPLEStorage storageDriver;
static FLYMAPLEGPIO gpioDriver;
static FLYMAPLERCInput rcinDriver;
static FLYMAPLERCOutput rcoutDriver;
static FLYMAPLEScheduler schedulerInstance;
static FLYMAPLEUtil utilInstance;

HAL_FLYMAPLE::HAL_FLYMAPLE() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        NULL,
        &i2cDriver,
        NULL,   /* only 1 i2c */
        NULL,   /* only 1 i2c */
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        NULL /* no optical flow */
    )
{}

void HAL_FLYMAPLE::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();

    /* uartA is the serial port used for the console, so lets make sure
     * it is initialized at boot */
    uartA->begin(115200);

    rcin->init();
    rcout->init();
    spi->init();
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init();
    storage->init(); // Uses EEPROM.*, flash_stm* copied from AeroQuad_v3.2

    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_FLYMAPLE hal;
    return hal;
}

#endif
