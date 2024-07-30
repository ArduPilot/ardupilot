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

#include "HAL_QURT_Class.h"
#include "AP_HAL_QURT_Private.h"
#include "Scheduler.h"
#include "Storage.h"
#include "Semaphores.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "AnalogIn.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL/utility/getopt_cpp.h>
#include <assert.h>
#include "interface.h"
#include "ap_host/src/protocol.h"


extern "C" {
    typedef void (*external_error_handler_t)(void);
};

static void crash_error_handler(void)
{
    HAP_PRINTF("CRASH_ERROR_HANDLER: at %p", &crash_error_handler);
}

using namespace QURT;

static UARTDriver_Console consoleDriver;
static UARTDriver_MAVLinkUDP serial0Driver(0);
static UARTDriver_MAVLinkUDP serial1Driver(1);
static UARTDriver_Local serial3Driver(QURT_UART_GPS);
static UARTDriver_Local serial4Driver(QURT_UART_RCIN);

static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static Empty::GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Util utilInstance;
static Scheduler schedulerInstance;
static I2CDeviceManager i2c_mgr_instance;

bool qurt_ran_overtime;

HAL_QURT::HAL_QURT() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        nullptr,
        &serial3Driver,
        &serial4Driver,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &i2c_mgr_instance,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &consoleDriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        nullptr,
        nullptr,
        nullptr)
{
}

static HAL_QURT::Callbacks *_callbacks;

void HAL_QURT::main_thread(void)
{
    sl_client_register_fatal_error_cb(crash_error_handler);

    // Let SLPI image send out it's initialization response before we
    // try to send anything out.
    qurt_timer_sleep(1000000);

    rcinDriver.init();
    analogIn.init();
    rcoutDriver.init();
    _callbacks->setup();
    scheduler->set_system_initialized();

    HAP_PRINTF("starting loop");

    for (;;) {
        // ensure other threads get some time
        qurt_timer_sleep(200);

        // call main loop
        _callbacks->loop();
    }
}

void HAL_QURT::start_main_thread(Callbacks* callbacks)
{
    _callbacks = callbacks;
    scheduler->thread_create(FUNCTOR_BIND_MEMBER(&HAL_QURT::main_thread, void), "main_thread",
                             (20 * 1024),
                             AP_HAL::Scheduler::PRIORITY_MAIN,
                             0);
}

void HAL_QURT::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    schedulerInstance.hal_initialized();
    serial0Driver.begin(115200);

    HAP_PRINTF("Creating main thread");

    const_cast<HAL_QURT *>(this)->start_main_thread(callbacks);
}

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    static const HAL_QURT *hal;
    if (hal == nullptr) {
        hal = new HAL_QURT;
    }
    return *hal;
}
