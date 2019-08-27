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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <assert.h>

#include "HAL_ChibiOS_Class.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Private.h>
#include "shared_dma.h"
#include "sdcard.h"
#include "hwdef/common/usbcfg.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <AP_Logger/AP_Logger.h>
#endif

#include <hwdef.h>

#ifndef HAL_NO_UARTDRIVER
static HAL_UARTA_DRIVER;
static HAL_UARTB_DRIVER;
static HAL_UARTC_DRIVER;
static HAL_UARTD_DRIVER;
static HAL_UARTE_DRIVER;
static HAL_UARTF_DRIVER;
static HAL_UARTG_DRIVER;
static HAL_UARTH_DRIVER;
#else
static Empty::UARTDriver uartADriver;
static Empty::UARTDriver uartBDriver;
static Empty::UARTDriver uartCDriver;
static Empty::UARTDriver uartDDriver;
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
static Empty::UARTDriver uartHDriver;
#endif

#if HAL_USE_I2C == TRUE && defined(HAL_I2C_DEVICE_LIST)
static ChibiOS::I2CDeviceManager i2cDeviceManager;
#else
static Empty::I2CDeviceManager i2cDeviceManager;
#endif

#if HAL_USE_SPI == TRUE
static ChibiOS::SPIDeviceManager spiDeviceManager;
#else
static Empty::SPIDeviceManager spiDeviceManager;
#endif

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)
static ChibiOS::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif

#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ChibiOS::Storage storageDriver;
#endif
static ChibiOS::GPIO gpioDriver;
static ChibiOS::RCInput rcinDriver;

#if HAL_USE_PWM == TRUE
static ChibiOS::RCOutput rcoutDriver;
#else
static Empty::RCOutput rcoutDriver;
#endif

static ChibiOS::Scheduler schedulerInstance;
static ChibiOS::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;

#ifndef HAL_NO_FLASH_SUPPORT
static ChibiOS::Flash flashDriver;
#else
static Empty::Flash flashDriver;
#endif


#if HAL_WITH_IO_MCU
HAL_UART_IO_DRIVER;
#include <AP_IOMCU/AP_IOMCU.h>
AP_IOMCU iomcu(uart_io);
#endif

HAL_ChibiOS::HAL_ChibiOS() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &uartFDriver,
        &uartGDriver,
        &uartHDriver,
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
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

static bool thread_running = false;        /**< Daemon status flag */
static thread_t* daemon_task;              /**< Handle of daemon task / thread */

extern const AP_HAL::HAL& hal;


/*
  set the priority of the main APM task
 */
void hal_chibios_set_priority(uint8_t priority)
{
    chSysLock();
#if CH_CFG_USE_MUTEXES == TRUE
    if ((daemon_task->prio == daemon_task->realprio) || (priority > daemon_task->prio)) {
      daemon_task->prio = priority;
    }
    daemon_task->realprio = priority;
#endif
    chSchRescheduleS();
    chSysUnlock();
}

thread_t* get_main_thread()
{
    return daemon_task;
}

static AP_HAL::HAL::Callbacks* g_callbacks;

static AP_HAL::Util::PersistentData last_persistent_data;

static void main_loop()
{
    daemon_task = chThdGetSelfX();

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

#ifdef HAL_I2C_CLEAR_BUS
    // Clear all I2C Buses. This can be needed on some boards which
    // can get a stuck I2C peripheral on boot
    ChibiOS::I2CBus::clear_all();
#endif

    ChibiOS::Shared_DMA::init();
    peripheral_power_enable();
        
    hal.uartA->begin(115200);

#ifdef HAL_SPI_CHECK_CLOCK_FREQ
    // optional test of SPI clock frequencies
    ChibiOS::SPIDevice::test_clock_freq();
#endif 

    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.analogin->init();
    hal.scheduler->init();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_chibios_set_priority(APM_STARTUP_PRIORITY);

    if (stm32_was_watchdog_reset()) {
        // load saved watchdog data
        stm32_watchdog_load((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4);
        last_persistent_data = utilInstance.persistent_data;
    }

    schedulerInstance.hal_initialized();

    g_callbacks->setup();

#ifdef IOMCU_FW
    stm32_watchdog_init();
#elif !defined(HAL_BOOTLOADER_BUILD)
    // setup watchdog to reset if main loop stops
    if (AP_BoardConfig::watchdog_enabled()) {
        stm32_watchdog_init();
    }

#ifndef HAL_NO_LOGGING
    if (hal.util->was_watchdog_reset()) {
        AP::internalerror().error(AP_InternalError::error_t::watchdog_reset);
        const AP_HAL::Util::PersistentData &pd = last_persistent_data;
        AP::logger().WriteCritical("WDOG", "TimeUS,Task,IErr,IErrCnt,MavMsg,MavCmd,SemLine,FL,FT,FA,FP,ICSR", "QbIIHHHHHIBI",
                                   AP_HAL::micros64(),
                                   pd.scheduler_task,
                                   pd.internal_errors,
                                   pd.internal_error_count,
                                   pd.last_mavlink_msgid,
                                   pd.last_mavlink_cmd,
                                   pd.semaphore_line,
                                   pd.fault_line,
                                   pd.fault_type,
                                   pd.fault_addr,
                                   pd.fault_thd_prio,
                                   pd.fault_icsr);
    }
#endif // HAL_NO_LOGGING
#endif // IOMCU_FW

    schedulerInstance.watchdog_pat();

    hal.scheduler->system_initialized();

    thread_running = true;
    chRegSetThreadName(SKETCHNAME);
    
    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

    while (true) {
        g_callbacks->loop();

        /*
          give up 50 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
         */
#ifndef HAL_DISABLE_LOOP_DELAY
        if (!schedulerInstance.check_called_boost()) {
            hal.scheduler->delay_microseconds(50);
        }
#endif
        schedulerInstance.watchdog_pat();
    }
    thread_running = false;
}

void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    /*
     * System initializations.
     * - ChibiOS HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */

#ifdef HAL_USB_PRODUCT_ID
  setup_usb_strings();
#endif
    
#ifdef HAL_STDOUT_SERIAL
    //STDOUT Initialistion
    SerialConfig stdoutcfg =
    {
      HAL_STDOUT_BAUDRATE,
      0,
      USART_CR2_STOP1_BITS,
      0
    };
    sdStart((SerialDriver*)&HAL_STDOUT_SERIAL, &stdoutcfg);
#endif

    assert(callbacks);
    g_callbacks = callbacks;

    //Takeover main
    main_loop();
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_ChibiOS hal_chibios;
    return hal_chibios;
}

#endif
