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
#include <AP_Vehicle/AP_Vehicle_Type.h>

/* Same default and same name as AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp, so a
 * hwdef can turn the main-loop yield off the same way on either HAL. */
#ifndef HAL_SCHEDULER_LOOP_DELAY_ENABLED
#define HAL_SCHEDULER_LOOP_DELAY_ENABLED 1
#endif

/* Microseconds the main loop gives up per iteration when the INS did not
 * boost.
 *
 * ChibiOS uses a flat 50 us. That is 2% of its 2.5 ms period at the 400 Hz it
 * normally runs. This HAL's boards run slower loops - CubeOrangeZephyr under
 * Renode is SCHED_LOOP_RATE 125, an 8 ms period - where the same 50 us is only
 * 0.6%, and that was measurably not enough: with io moved below main (2.29),
 * AP_Logger reported "stuck thread ()" with an empty last_io_operation,
 * meaning its callback had not run at ALL.
 *
 * 160 us restores that 2% at 125 Hz - and it HANGS THE BOARD when io sits
 * above main. Measured 2026-09-11 across six flights: io at PREEMPT(5) with a
 * 50 us yield boots and flies; io at PREEMPT(13) with 160 us boots and flies;
 * io at PREEMPT(5) with 160 us never reaches MAVLink at all, three times,
 * with and without SD logging. Giving main's time away to a thread that
 * outranks it starves the loop rather than helping it.
 *
 * So this stays at ChibiOS's 50 us while io is above main. If io is ever moved
 * below main permanently, the larger fraction becomes
 * available again - and is needed there, because at 50 us AP_Logger's io
 * process was never scheduled at all. The two settings are coupled; do not
 * change one alone. */
#ifndef AP_SCHEDULER_LOOP_YIELD_US
#define AP_SCHEDULER_LOOP_YIELD_US 50U   /* ChibiOS's value. 160 HANGS - see below. */
#endif
#include "WiFiDriver.h"
#include "SPIDevice.h"
#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
#endif

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

#if HAL_WITH_IO_MCU
/* The port wired to an IO co-processor. hwdef.h's HAL_UART_IO_DRIVER binds
   uart_io to the driver at HAL_UART_IOMCU_IDX, which sits past the
   user-facing SERIALn ports so AP_SerialManager never offers it. Same shape
   as AP_HAL_ChibiOS. */
HAL_UART_IO_DRIVER;
AP_IOMCU iomcu(uart_io);
#endif

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
    // this is a DEV_PRINTF style output thats only active on --debug
    BOOT_TRACE("AP: run() entered\n");

    /* ChibiOS parity (HAL_ChibiOS_Class.cpp): analogin->init() immediately
       before scheduler->init(). Without this call AnalogIn::_adc_ready and
       _oc_ready stayed false for the whole flight, every read_latest()/
       voltage_average() returned 0, and board voltage and any analog
       battery monitor read as flat zero with no error anywhere. */
    analogin->init();
    BOOT_TRACE("AP: analogin->init() done\n");

    scheduler->init();
    BOOT_TRACE("AP: scheduler->init() done\n");

    /* Persistent crash/watchdog forensics, ChibiOS parity - same placement
       as HAL_ChibiOS_Class.cpp's own stm32_watchdog_load() call (after
       scheduler->init(), before setup()). No-ops unless
       hal.util->was_watchdog_reset() is true. */
    schedulerInstance.restore_persistent_data();

    storage->init();
    BOOT_TRACE("AP_thread: storage->init() done\n");
    serial(0)->begin(115200);  // console default baud
    BOOT_TRACE("AP: serial(0)->begin() done\n");

#ifdef HAL_SPI_CHECK_CLOCK_FREQ
    // optional bring-up measurement of the real SPI clock on each bus
    Zephyr::SPIDevice::test_clock_freq();
#endif

    /* RESTORED 2026-08-10 (rcin only): rcin->init() is safe to call here now that
     * the capture path no longer depends on later init. */
#if HAL_RCIN_THREAD_ENABLED
    rcin->init();
    BOOT_TRACE("AP: rcin->init() done\n");

    /* RESTORED 2026-08-11: rcout->init() was deliberately withheld here while the
     * pad arbitration was unresolved. */
    rcout->init();
    BOOT_TRACE("AP: rcout->init() done\n");
#endif

    BOOT_TRACE("AP: about to call callbacks->setup()\n");
    /* set_system_initialized() is NOT called here. ChibiOS calls it after
       g_callbacks->setup() returns (HAL_ChibiOS_Class.cpp), so
       hal.scheduler->is_system_initialized() means "setup() has finished" -
       and AP code uses it that way. Claiming it early made every such test
       true throughout init.

       It used to be needed: the IO thread gated _run_io() on _initialized, so
       nothing drained AP_Param::save_queue until this was set, and set_and_save
       during init (stats.init, BoardConfig.init) spun forever. The gate is now
       _hal_initialized, matching ChibiOS's own io thread, so the queue drains
       from HAL init onwards and this can wait until setup() is done. */
    scheduler->expect_delay_ms(180000);
    /* setup() runs at APM_STARTUP_PRIORITY, below every service, bus, UART and
       user thread, exactly as ChibiOS's main_loop() (HAL_ChibiOS_Class.cpp:274
       and :326) - until the first INS wait inside it boosts main, after which
       the first expect_delay_ms() drops it to APM_MAIN_PRIORITY, the same
       10 -> 182 -> 180 sequence ChibiOS goes through. */
    Zephyr::Scheduler::set_main_priority(APM_STARTUP_PRIORITY);
    callbacks->setup();
    scheduler->expect_delay_ms(0);
    /* Back to the flight-loop level for the rest of the run. */
    Zephyr::Scheduler::set_main_priority(APM_MAIN_PRIORITY);
    BOOT_TRACE("AP: callbacks->setup() returned\n");

    /* After setup(), as ChibiOS does. This is also what un-suppresses the
       monitor thread's !_initialized guard, so its warnings start now rather
       than during a long init. */
    scheduler->set_system_initialized();
    BOOT_TRACE("AP: set_system_initialized() done\n");

    for (;;) {
        callbacks->loop();

#if HAL_SCHEDULER_LOOP_DELAY_ENABLED && !APM_BUILD_TYPE(APM_BUILD_Replay)
        /*
          Give up 50 microseconds if the INS loop did not already call
          delay_microseconds_boost(), so lower-priority threads get a chance to
          run. Copied from AP_HAL_ChibiOS's main loop, which has always done
          this; check_called_boost() was ported to this HAL at the same time as
          the rest of Scheduler and then never called, so the main thread here
          only ever yielded when something else happened to block it.

          Calling delay_microseconds_boost() already gives up main-loop time,
          which is why that case is excluded rather than delayed twice.

          This matters more on this HAL than the 0.6% of a 125 Hz period it
          costs: the sensor bus threads and the IO thread run below main
          priority, and a main thread that never yields starves them exactly
          when the vehicle is busiest.
         */
        if (!schedulerInstance.check_called_boost()) {
            /* schedulerInstance rather than hal.scheduler: `hal` is not in
               scope in this file - the HAL object is constructed at the bottom
               of it - and this is the same object either way.

               The size is a FRACTION of the loop period, not ChibiOS's literal
               50 us. ChibiOS yields 50 us out of a 2.5 ms period at 400 Hz,
               which is 2% of the loop. This board runs SCHED_LOOP_RATE 125, an
               8 ms period, where the same 50 us is 0.6% - and that was not
               enough to feed the io thread once io was moved below main
               (2.29): AP_Logger reported "stuck thread ()", with an empty
               last_io_operation meaning its callback had not run at all.
               AP_SCHEDULER_LOOP_YIELD_US keeps the 2% and lets a board or a
               loop rate change it. */
            schedulerInstance.delay_microseconds(AP_SCHEDULER_LOOP_YIELD_US);
        }
#endif

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
