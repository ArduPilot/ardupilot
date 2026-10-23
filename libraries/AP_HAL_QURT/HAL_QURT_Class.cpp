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

/*
  SERIAL3 defaults to the DSP-local GPS UART. Boards that run GPS through
  a UART on the apps processor instead (e.g. ModalAI-VOXL3) define
  HAL_NO_DSP_GPS_UART in hwdef and leave SERIAL3 unclaimed; the remote
  GPS is reached via one of the registered tunneled serial ports.
 */
#ifndef HAL_NO_DSP_GPS_UART
static UARTDriver_Local serial3Driver(QURT_UART_GPS);
#endif

static UARTDriver_Local serial4Driver(QURT_UART_RCIN);

/*
  Per-slot device IDs for the 5 tunneled remote serial ports. Each device id
  maps to "/dev/ttyHS<device_id>" on the apps processor. Boards set these
  explicitly in hwdef.dat; values here are only a fallback for boards that
  leave them unset.
 */
#ifndef APPS_REMOTE_UART_0_DEVICE
  #define APPS_REMOTE_UART_0_DEVICE 0
#endif
#ifndef APPS_REMOTE_UART_1_DEVICE
  #define APPS_REMOTE_UART_1_DEVICE 1
#endif
#ifndef APPS_REMOTE_UART_2_DEVICE
  #define APPS_REMOTE_UART_2_DEVICE 2
#endif
#ifndef APPS_REMOTE_UART_3_DEVICE
  #define APPS_REMOTE_UART_3_DEVICE 3
#endif
#ifndef APPS_REMOTE_UART_4_DEVICE
  #define APPS_REMOTE_UART_4_DEVICE 4
#endif

/*
  SERIALn slot each remote port occupies. Chosen to avoid the fixed HAL slots
  (SERIAL0..SERIAL4 carry MAVLink/GPS/RCIN on existing targets).
 */
static const uint8_t remote_uart_serial_idx[] = { 5, 6, 7, 8, 9 };

static UARTDriver_RemoteRegistered remote_uart_ports[] = {
    { 0, APPS_REMOTE_UART_0_DEVICE },
    { 1, APPS_REMOTE_UART_1_DEVICE },
    { 2, APPS_REMOTE_UART_2_DEVICE },
    { 3, APPS_REMOTE_UART_3_DEVICE },
    { 4, APPS_REMOTE_UART_4_DEVICE },
};

// Catch a future mismatch between the protocol cap and the per-board
// configuration arrays at compile time rather than walking off the end
// of them in the registration loop below.
static_assert(ARRAY_SIZE(remote_uart_ports) == MAX_REMOTE_UART_INSTANCES,
              "remote_uart_ports size must match MAX_REMOTE_UART_INSTANCES");
static_assert(ARRAY_SIZE(remote_uart_serial_idx) == MAX_REMOTE_UART_INSTANCES,
              "remote_uart_serial_idx size must match MAX_REMOTE_UART_INSTANCES");

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
#ifndef HAL_NO_DSP_GPS_UART
        &serial3Driver,
#else
        nullptr,
#endif
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

    // register tunneled remote serial ports before hal_initialized() so the
    // uart thread will start ticking them as soon as it starts
    for (uint8_t i = 0; i < MAX_REMOTE_UART_INSTANCES; i++) {
        remote_uart_ports[i].init(remote_uart_serial_idx[i]);
    }

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
