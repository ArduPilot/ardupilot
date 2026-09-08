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
 *
 * Code by @davidbuzz and Claude
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#include <zephyr/kernel.h>

#include "HAL_Zephyr_Class.h"
#include "WiFiDriver.h"

using namespace Zephyr;

static I2CDeviceManager i2cDeviceManager;
static SPIDeviceManager spiDeviceManager;
static WSPIDeviceManager wspiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static UARTDriver serial0Driver(0);
static UARTDriver serial1Driver(1);
static UARTDriver serial2Driver(2);
static UARTDriver serial3Driver(3);
static UARTDriver serial4Driver(4);
static UARTDriver serial5Driver(5);
static UARTDriver serial6Driver(6);
static UARTDriver serial7Driver(7);
static UARTDriver serial8Driver(8);
static UARTDriver serial9Driver(9);

/* WiFi virtual serial ports (hwdef SERIAL_ORDER WIFI_TCP/WIFI_UDP tokens):
   the AP_SERIALn_DRIVER macros below swap these instances into the HAL's
   serial table at the hwdef-assigned indices. */
#if defined(AP_ZEPHYR_WIFI_ENABLED) && AP_ZEPHYR_WIFI_ENABLED
#if defined(HAL_ZEPHYR_WIFI_TCP_SERIAL)
static WiFiDriver wifiTcpDriver;
#endif
#if defined(HAL_ZEPHYR_WIFI_UDP_SERIAL)
static WiFiUdpDriver wifiUdpDriver;
#endif
#endif

#define AP_SERIAL_PTR(n)                                                     \
    (AP_ZEPHYR_WIFI_TCP_IS(n) ? AP_ZEPHYR_WIFI_TCP_PTR                       \
     : AP_ZEPHYR_WIFI_UDP_IS(n) ? AP_ZEPHYR_WIFI_UDP_PTR                     \
     : static_cast<AP_HAL::UARTDriver *>(&serial##n##Driver))
#if defined(HAL_ZEPHYR_WIFI_TCP_SERIAL)
#define AP_ZEPHYR_WIFI_TCP_IS(n) ((n) == HAL_ZEPHYR_WIFI_TCP_SERIAL)
#define AP_ZEPHYR_WIFI_TCP_PTR   static_cast<AP_HAL::UARTDriver *>(&wifiTcpDriver)
#else
#define AP_ZEPHYR_WIFI_TCP_IS(n) 0
#define AP_ZEPHYR_WIFI_TCP_PTR   nullptr
#endif
#if defined(HAL_ZEPHYR_WIFI_UDP_SERIAL)
#define AP_ZEPHYR_WIFI_UDP_IS(n) ((n) == HAL_ZEPHYR_WIFI_UDP_SERIAL)
#define AP_ZEPHYR_WIFI_UDP_PTR   static_cast<AP_HAL::UARTDriver *>(&wifiUdpDriver)
#else
#define AP_ZEPHYR_WIFI_UDP_IS(n) 0
#define AP_ZEPHYR_WIFI_UDP_PTR   nullptr
#endif
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;
static Flash flashDriver;
#if HAL_WITH_DSP
static DSP dspInstance;
#endif

#if HAL_NUM_CAN_IFACES
static Zephyr::CANIface *can_ifaces[HAL_NUM_CAN_IFACES];
#endif

HAL_Zephyr::HAL_Zephyr() :
    AP_HAL::HAL(
        AP_SERIAL_PTR(0),
        AP_SERIAL_PTR(1),
        AP_SERIAL_PTR(2),
        AP_SERIAL_PTR(3),
        AP_SERIAL_PTR(4),
        AP_SERIAL_PTR(5),
        AP_SERIAL_PTR(6),
        AP_SERIAL_PTR(7),
        AP_SERIAL_PTR(8),
        AP_SERIAL_PTR(9),
        &i2cDeviceManager,
        &spiDeviceManager,
        &wspiDeviceManager,
        &analogIn,
        &storageDriver,
        &serial0Driver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if HAL_WITH_DSP
        &dspInstance,
#endif
#if HAL_NUM_CAN_IFACES
        (AP_HAL::CANIface**)can_ifaces
#else
        nullptr
#endif
        )
{}

void HAL_Zephyr::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    printk("AP: run() entered\n");
    scheduler->init();
    printk("AP: scheduler->init() done\n");

    /* Persistent crash/watchdog forensics, ChibiOS parity - same placement
       as HAL_ChibiOS_Class.cpp's own stm32_watchdog_load() call (after
       scheduler->init(), before setup()). No-ops unless
       hal.util->was_watchdog_reset() is true. */
    schedulerInstance.restore_persistent_data();

    storage->init();
    printk("AP_thread: storage->init() done\n");
    serial(0)->begin(115200);  // console default baud
    printk("AP: serial(0)->begin() done\n");

    /* RESTORED 2026-08-10 (rcin only): rcin->init() is safe to call here now that
     * the capture path no longer depends on later init. */
#if HAL_RCIN_THREAD_ENABLED
    rcin->init();
    printk("AP: rcin->init() done\n");

    /* RESTORED 2026-08-11: rcout->init() was deliberately withheld here while the
     * pad arbitration was unresolved. */
    rcout->init();
    printk("AP: rcout->init() done\n");
#endif

    // set_system_initialized before setup() so the IO thread runs and can
    // drain AP_Param::save_queue during init (stats.init, BoardConfig.init etc
    // all call set_and_save which spins on the queue if the IO thread is idle)
    scheduler->set_system_initialized();
    printk("AP: set_system_initialized() done\n");

    printk("AP: about to call callbacks->setup()\n");
    /* Because _initialized is now true, the monitor thread's !_initialized guard no
     * longer suppresses its warnings. */
    scheduler->expect_delay_ms(180000);
    callbacks->setup();
    scheduler->expect_delay_ms(0);
    printk("AP: callbacks->setup() returned\n");

    for (;;) {
        callbacks->loop();
        /* The authoritative software-watchdog pat, matching AP_HAL_ChibiOS: patted only
         * when the main loop is alive, never unconditionally from a timer. */
        schedulerInstance.watchdog_pat();
    }
}

static HAL_Zephyr hal_zephyr;

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_zephyr;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_zephyr;
}

#endif  // CONFIG_HAL_BOARD
