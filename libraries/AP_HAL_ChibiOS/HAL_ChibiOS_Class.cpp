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

#include <hal.h>
#include "HAL_ChibiOS_Class.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Private.h>
#include "shared_dma.h"
#include "sdcard.h"
#include <sysperf.h>
#include "hwdef/common/usbcfg.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <AP_Logger/AP_Logger.h>
#endif
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/SIMState.h>

#include <hwdef.h>

#ifndef DEFAULT_SERIAL0_BAUD
#define SERIAL0_BAUD 115200
#else
#define SERIAL0_BAUD DEFAULT_SERIAL0_BAUD
#endif

#ifndef HAL_SCHEDULER_LOOP_DELAY_ENABLED
#define HAL_SCHEDULER_LOOP_DELAY_ENABLED 1
#endif

#if AP_HAL_UARTDRIVER_ENABLED
static HAL_SERIAL0_DRIVER;
static HAL_SERIAL1_DRIVER;
static HAL_SERIAL2_DRIVER;
static HAL_SERIAL3_DRIVER;
static HAL_SERIAL4_DRIVER;
static HAL_SERIAL5_DRIVER;
static HAL_SERIAL6_DRIVER;
static HAL_SERIAL7_DRIVER;
static HAL_SERIAL8_DRIVER;
static HAL_SERIAL9_DRIVER;
#else
static Empty::UARTDriver serial0Driver;
static Empty::UARTDriver serial1Driver;
static Empty::UARTDriver serial2Driver;
static Empty::UARTDriver serial3Driver;
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;
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

#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif

#if HAL_WITH_DSP
static ChibiOS::DSP dspDriver;
#endif

#ifndef HAL_NO_FLASH_SUPPORT
static ChibiOS::Flash flashDriver;
#else
static Empty::Flash flashDriver;
#endif

#if HAL_NUM_CAN_IFACES > 0
static ChibiOS::CANIface* canDrivers[HAL_NUM_CAN_IFACES];
#endif

#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
static ChibiOS::WSPIDeviceManager wspiDeviceManager;
#endif

#if HAL_WITH_IO_MCU
HAL_UART_IO_DRIVER;
#include <AP_IOMCU/AP_IOMCU.h>
AP_IOMCU iomcu(uart_io);
#endif

HAL_ChibiOS::HAL_ChibiOS() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        &serial2Driver,
        &serial3Driver,
        &serial4Driver,
        &serial5Driver,
        &serial6Driver,
        &serial7Driver,
        &serial8Driver,
        &serial9Driver,
        &i2cDeviceManager,
        &spiDeviceManager,
#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
        &wspiDeviceManager,
#else
        nullptr,
#endif
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
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
#if HAL_NUM_CAN_IFACES
        (AP_HAL::CANIface**)canDrivers
#else
        nullptr
#endif
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
    if ((daemon_task->hdr.pqueue.prio == daemon_task->realprio) || (priority > daemon_task->hdr.pqueue.prio)) {
      daemon_task->hdr.pqueue.prio = priority;
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

#if AP_BOARDCONFIG_MCU_MEMPROTECT_ENABLED
#if !defined(STM32H7)
#error AP_BOARDCONFIG_MCU_MEMPROTECT_ENABLED only available on H7 processors
#endif  // !defined(STM32H7)
static void mem_protect_enable()
{
    /*
      enable this on H7 to make writes to the first 1k of RAM on H7
      produce a hard fault and crash dump
     */
    mpuConfigureRegion(MPU_REGION_7,
                       0x0,
                       MPU_RASR_ATTR_AP_NA_NA |
                       MPU_RASR_SIZE_1K |
                       MPU_RASR_ENABLE);
    mpuEnable(MPU_CTRL_PRIVDEFENA | MPU_CTRL_ENABLE);
}
#endif  // AP_BOARDCONFIG_MCU_MEMPROTECT_ENABLED

static AP_HAL::HAL::Callbacks* g_callbacks;

static void main_loop()
{
    daemon_task = chThdGetSelfX();

#if AP_CPU_IDLE_STATS_ENABLED && HAL_USE_LOAD_MEASURE
    if (AP_BoardConfig::use_idle_stats()) {
        sysInitLoadMeasure();
        sysStartLoadMeasure();
    }
#endif

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

#ifdef HAL_I2C_CLEAR_BUS
    // Clear all I2C Buses. This can be needed on some boards which
    // can get a stuck I2C peripheral on boot
    ChibiOS::I2CBus::clear_all();
#endif

#if AP_HAL_SHARED_DMA_ENABLED
    ChibiOS::Shared_DMA::init();
#endif

    peripheral_power_enable();

    hal.serial(0)->begin(SERIAL0_BAUD);

#if (HAL_USE_SPI == TRUE) && defined(HAL_SPI_CHECK_CLOCK_FREQ)
    // optional test of SPI clock frequencies
    ChibiOS::SPIDevice::test_clock_freq();
#endif

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
        utilInstance.last_persistent_data = utilInstance.persistent_data;
    }

    schedulerInstance.hal_initialized();

    g_callbacks->setup();

#if HAL_ENABLE_SAVE_PERSISTENT_PARAMS
    utilInstance.apply_persistent_params();
#endif

#if HAL_FLASH_PROTECTION
    if (AP_BoardConfig::unlock_flash()) {
        stm32_flash_unprotect_flash();
    } else {
        stm32_flash_protect_flash(false, AP_BoardConfig::protect_flash());
        stm32_flash_protect_flash(true, AP_BoardConfig::protect_bootloader());
    }
#endif

#if !defined(DISABLE_WATCHDOG)
#ifdef IOMCU_FW
    stm32_watchdog_init();
#elif !defined(HAL_BOOTLOADER_BUILD)
#if !defined(HAL_EARLY_WATCHDOG_INIT)
    // setup watchdog to reset if main loop stops
    if (AP_BoardConfig::watchdog_enabled()) {
        stm32_watchdog_init();
    }
#endif

    if (hal.util->was_watchdog_reset()) {
        INTERNAL_ERROR(AP_InternalError::error_t::watchdog_reset);
    }
#endif // IOMCU_FW
#endif // DISABLE_WATCHDOG

    schedulerInstance.watchdog_pat();

    hal.scheduler->set_system_initialized();

    thread_running = true;
    chRegSetThreadName(AP_BUILD_TARGET_NAME);

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

#if AP_BOARDCONFIG_MCU_MEMPROTECT_ENABLED
    mem_protect_enable();
#endif  // AP_BOARDCONFIG_MCU_MEMPROTECT_ENABLED

    while (true) {
        g_callbacks->loop();

#if HAL_SCHEDULER_LOOP_DELAY_ENABLED && !APM_BUILD_TYPE(APM_BUILD_Replay)
        /*
          give up 50 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
         */
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
#if defined(HAL_EARLY_WATCHDOG_INIT) && !defined(DISABLE_WATCHDOG)
    stm32_watchdog_init();
    stm32_watchdog_pat();
#endif
    /*
     * System initializations.
     * - ChibiOS HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */

#if AP_SIM_ENABLED
    AP::sitl()->init();
#endif  // AP_SIM_ENABLED

#if HAL_USE_SERIAL_USB == TRUE
    usb_initialise();
#endif

#ifdef HAL_STDOUT_SERIAL
    //STDOUT Initialisation
    SerialConfig stdoutcfg =
    {
      HAL_STDOUT_BAUDRATE,
      0,
      USART_CR2_STOP1_BITS,
      0
    };
    sdStart((SerialDriver*)&HAL_STDOUT_SERIAL, &stdoutcfg);
#endif

    g_callbacks = callbacks;

    //Takeover main
    main_loop();
}

static HAL_ChibiOS hal_chibios;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_chibios;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_chibios;
}

#endif
