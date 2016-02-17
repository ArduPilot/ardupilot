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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "HAL_QURT_Class.h"
#include "AP_HAL_QURT_Private.h"
#include "Scheduler.h"
#include "Storage.h"
#include "Semaphores.h"
#include "RCInput.h"
#include "RCOutput.h"
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL/utility/getopt_cpp.h>
#include <assert.h>

using namespace QURT;

static UDPDriver uartADriver;
static UARTDriver uartBDriver("/dev/tty-4");
static UARTDriver uartCDriver("/dev/tty-2");
static UARTDriver uartDDriver(nullptr);
static UARTDriver uartEDriver(nullptr);
static Semaphore  i2cSemaphore;
static Empty::I2CDriver  i2cDriver(&i2cSemaphore);

static Empty::SPIDeviceManager spiDeviceManager;
static Empty::AnalogIn analogIn;
static Storage storageDriver;
static Empty::GPIO gpioDriver;
static RCInput rcinDriver("/dev/tty-1");
static RCOutput rcoutDriver("/dev/tty-3");
static Util utilInstance;
static Scheduler schedulerInstance;
static Empty::I2CDeviceManager i2c_mgr_instance;

bool qurt_ran_overtime;

HAL_QURT::HAL_QURT() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &i2c_mgr_instance,
        &i2cDriver,
        NULL, /* only one i2c */
        NULL, /* only one i2c */
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        NULL)
{
}

void HAL_QURT::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    int opt;
    const struct GetOptLong::option options[] = {
        {"uartB",         true,  0, 'B'},
        {"uartC",         true,  0, 'C'},
        {"uartD",         true,  0, 'D'},
        {"uartE",         true,  0, 'E'},
        {"dsm",           true,  0, 'S'},
        {"ESC",           true,  0, 'e'},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "B:C:D:E:e:S",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'B':
            uartBDriver.set_device_path(gopt.optarg);
            break;
        case 'C':
            uartCDriver.set_device_path(gopt.optarg);
            break;
        case 'D':
            uartDDriver.set_device_path(gopt.optarg);
            break;
        case 'E':
            uartEDriver.set_device_path(gopt.optarg);
            break;
        case 'e':
            rcoutDriver.set_device_path(gopt.optarg);
            break;
        case 'S':
            rcinDriver.set_device_path(gopt.optarg);
            break;
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    schedulerInstance.hal_initialized();
    uartA->begin(115200);
    rcinDriver.init();
    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_QURT *hal;
    if (hal == nullptr) {
        hal = new HAL_QURT;
        HAP_PRINTF("allocated HAL_QURT of size %u", sizeof(*hal));
    }
    return *hal;
}

#endif
