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

static ChibiOS::ChibiUARTDriver uartADriver(0);
static ChibiOS::ChibiUARTDriver uartBDriver(1);
static ChibiOS::ChibiUARTDriver uartCDriver(2);
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
static Empty::UARTDriver uartDDriver;
static Empty::UARTDriver uartEDriver;
#else
static ChibiOS::ChibiUARTDriver uartDDriver(3);
static ChibiOS::ChibiUARTDriver uartEDriver(4);
#endif
static ChibiOS::I2CDeviceManager i2cDeviceManager;
static ChibiOS::SPIDeviceManager spiDeviceManager;
static ChibiOS::ChibiAnalogIn analogIn;
static ChibiOS::ChibiStorage storageDriver;
static ChibiOS::ChibiGPIO gpioDriver;
static ChibiOS::ChibiRCInput rcinDriver;
static ChibiOS::ChibiRCOutput rcoutDriver;
static ChibiOS::ChibiScheduler schedulerInstance;
static ChibiOS::ChibiUtil utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
#ifdef USE_POSIX
static FATFS SDC_FS; // FATFS object
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
ChibiOS::ChibiUARTDriver uart_io(5);
AP_IOMCU iomcu(uart_io);
#endif

HAL_ChibiOS::HAL_ChibiOS() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        nullptr,            /* no uartF */
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
    if ((daemon_task->prio == daemon_task->realprio) || (priority > daemon_task->prio)) {
      daemon_task->prio = priority;
    }
    daemon_task->realprio = priority;
    chSchRescheduleS();
    chSysUnlock();
}

thread_t* get_main_thread()
{
    return daemon_task;
}

static AP_HAL::HAL::Callbacks* g_callbacks;
THD_WORKING_AREA(_main_thread_wa, APM_MAIN_THREAD_STACK_SIZE);
static THD_FUNCTION(main_loop,arg)
{
    daemon_task = chThdGetSelfX();

    Shared_DMA::init();
    
    hal.uartA->begin(115200);
    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.rcin->init();
    hal.rcout->init();
    hal.analogin->init();
    hal.gpio->init();
    hal.scheduler->init();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_chibios_set_priority(APM_STARTUP_PRIORITY);

    schedulerInstance.hal_initialized();

    g_callbacks->setup();
    hal.scheduler->system_initialized();

    thread_running = true;
    daemon_task->name = SKETCHNAME;
    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

    while (true) {
        g_callbacks->loop();

        /*
          give up 250 microseconds of time, to ensure drivers get a
          chance to run.
         */
        hal.scheduler->delay_microseconds(250);
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
    hrt_init();
    //STDOUT Initialistion
    SerialConfig stdoutcfg =
    {
      HAL_STDOUT_BAUDRATE,
      0,
      USART_CR2_STOP1_BITS,
      0
    };
    sdStart((SerialDriver*)&HAL_STDOUT_SERIAL, &stdoutcfg);

    //Setup SD Card and Initialise FATFS bindings
    /*
     * Start SD Driver
     */
#ifdef USE_POSIX
    FRESULT err;
    sdcStart(&SDCD1, NULL);

    if(sdcConnect(&SDCD1) == HAL_FAILED) {
        printf("Err: Failed to initialize SDIO!\n");
    } else {
        err = f_mount(&SDC_FS, "/", 1);
        if (err != FR_OK) {
            printf("Err: Failed to mount SD Card!\n");
            sdcDisconnect(&SDCD1);
        } else {
            printf("Successfully mounted SDCard..\n");
        }
        //Create APM Directory
        mkdir("/APM", 0777);
    }
#endif
    assert(callbacks);
    g_callbacks = callbacks;

    chThdCreateStatic(_main_thread_wa,
                      sizeof(_main_thread_wa),
                      APM_MAIN_PRIORITY,     /* Initial priority.    */
                      main_loop,             /* Thread function.     */
                      nullptr);              /* Thread parameter.    */
    chThdExit(0);
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_ChibiOS hal_chibios;
    return hal_chibios;
}

#endif
