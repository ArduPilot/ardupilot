#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include <AP_HAL/utility/RCOutput_Tap.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "AP_HAL_VRBRAIN.h"
#include "AP_HAL_VRBRAIN_Namespace.h"
#include "HAL_VRBRAIN_Class.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "AnalogIn.h"
#include "Util.h"
#include "GPIO.h"
#include "I2CDevice.h"
#include "SPIDevice.h"

#if HAL_WITH_UAVCAN
#include "CAN.h"
#endif

#include <stdlib.h>
#include <systemlib/systemlib.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <poll.h>
#include <drivers/drv_hrt.h>

using namespace VRBRAIN;
using namespace ap;

//static Empty::GPIO gpioDriver;

static VRBRAINScheduler schedulerInstance;
static VRBRAINStorage storageDriver;
static VRBRAINRCInput rcinDriver;
static VRBRAINRCOutput rcoutDriver;
static VRBRAINAnalogIn analogIn;
static VRBRAINUtil utilInstance;
static VRBRAINGPIO gpioDriver;

static VRBRAIN::I2CDeviceManager i2c_mgr_instance;
static VRBRAIN::SPIDeviceManager spi_mgr_instance;

#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/null"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52E)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRCORE_V10)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#else
#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"
#define UARTD_DEFAULT_DEVICE "/dev/ttyS1"
#define UARTE_DEFAULT_DEVICE "/dev/null"
#define UARTF_DEFAULT_DEVICE "/dev/null"
#endif

// 3 UART drivers, for GPS plus two mavlink-enabled devices
static VRBRAINUARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "APM_uartA");
static VRBRAINUARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "APM_uartB");
static VRBRAINUARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "APM_uartC");
static VRBRAINUARTDriver uartDDriver(UARTD_DEFAULT_DEVICE, "APM_uartD");
static VRBRAINUARTDriver uartEDriver(UARTE_DEFAULT_DEVICE, "APM_uartE");
static VRBRAINUARTDriver uartFDriver(UARTF_DEFAULT_DEVICE, "APM_uartF");

HAL_VRBRAIN::HAL_VRBRAIN() :
    AP_HAL::HAL(
        &uartADriver,  /* uartA */
        &uartBDriver,  /* uartB */
        &uartCDriver,  /* uartC */
        &uartDDriver,  /* uartD */
        &uartEDriver,  /* uartE */
        &uartFDriver,  /* uartF */
        &i2c_mgr_instance,
        &spi_mgr_instance,
        &analogIn, /* analogin */
        &storageDriver, /* storage */
        &uartADriver, /* console */
        &gpioDriver, /* gpio */
        &rcinDriver,  /* rcinput */
        &rcoutDriver, /* rcoutput */
        &schedulerInstance, /* scheduler */
        &utilInstance, /* util */
        nullptr,    /* no onboard optical flow */
        nullptr)   /* CAN */
{}

bool _vrbrain_thread_should_exit = false;        /**< Daemon exit flag */
static bool thread_running = false;        /**< Daemon status flag */
static int daemon_task;                /**< Handle of daemon task / thread */
bool vrbrain_ran_overtime;

extern const AP_HAL::HAL& hal;

/*
  set the priority of the main APM task
 */
void hal_vrbrain_set_priority(uint8_t priority)
{
    struct sched_param param;
    param.sched_priority = priority;
    sched_setscheduler(daemon_task, SCHED_FIFO, &param);    
}

/*
  this is called when loop() takes more than 1 second to run. If that
  happens then something is blocking for a long time in the main
  sketch - probably waiting on a low priority driver. Set the priority
  of the APM task low to let the driver run.
 */
static void loop_overtime(void *)
{
    hal_vrbrain_set_priority(APM_OVERTIME_PRIORITY);
    vrbrain_ran_overtime = true;
}

static AP_HAL::HAL::Callbacks* g_callbacks;

static int main_loop(int argc, char **argv)
{
    hal.uartA->begin(115200);
    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.uartD->begin(57600);
    hal.uartE->begin(57600);
    hal.scheduler->init();

    // init the I2C wrapper class
    VRBRAIN_I2C::init_lock();
    
    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_vrbrain_set_priority(APM_STARTUP_PRIORITY);

    schedulerInstance.hal_initialized();

    g_callbacks->setup();
    hal.scheduler->system_initialized();

    perf_counter_t perf_loop = perf_alloc(PC_ELAPSED, "APM_loop");
    perf_counter_t perf_overrun = perf_alloc(PC_COUNT, "APM_overrun");
    struct hrt_call loop_overtime_call;

    thread_running = true;

    /*
      switch to high priority for main loop
     */
    hal_vrbrain_set_priority(APM_MAIN_PRIORITY);

    while (!_vrbrain_thread_should_exit) {
        perf_begin(perf_loop);
        
        /*
          this ensures a tight loop waiting on a lower priority driver
          will eventually give up some time for the driver to run. It
          will only ever be called if a loop() call runs for more than
          0.1 second
         */
        hrt_call_after(&loop_overtime_call, 100000, (hrt_callout)loop_overtime, nullptr);

        g_callbacks->loop();

        if (vrbrain_ran_overtime) {
            /*
              we ran over 1s in loop(), and our priority was lowered
              to let a driver run. Set it back to high priority now.
             */
            hal_vrbrain_set_priority(APM_MAIN_PRIORITY);
            perf_count(perf_overrun);
            vrbrain_ran_overtime = false;
        }

        perf_end(perf_loop);

        /*
          give up 250 microseconds of time, to ensure drivers get a
          chance to run. This relies on the accurate semaphore wait
          using hrt in semaphore.cpp
         */
        hal.scheduler->delay_microseconds(250);
    }
    thread_running = false;
    return 0;
}

static void usage(void)
{
    printf("Usage: %s [options] {start,stop,status}\n", SKETCHNAME);
    printf("Options:\n");
    printf("\t-d  DEVICE         set terminal device (default %s)\n", UARTA_DEFAULT_DEVICE);
    printf("\t-d2 DEVICE         set second terminal device (default %s)\n", UARTC_DEFAULT_DEVICE);
    printf("\t-d3 DEVICE         set 3rd terminal device (default %s)\n", UARTD_DEFAULT_DEVICE);
    printf("\t-d4 DEVICE         set 2nd GPS device (default %s)\n", UARTE_DEFAULT_DEVICE);
    printf("\t-d5 DEVICE         set uartF (default %s)\n", UARTF_DEFAULT_DEVICE);
    printf("\n");
}


void HAL_VRBRAIN::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    int i;
    const char *deviceA = UARTA_DEFAULT_DEVICE;
    const char *deviceC = UARTC_DEFAULT_DEVICE;
    const char *deviceD = UARTD_DEFAULT_DEVICE;
    const char *deviceE = UARTE_DEFAULT_DEVICE;
    const char *deviceF = UARTF_DEFAULT_DEVICE;

    if (argc < 1) {
        printf("%s: missing command (try '%s start')", 
               SKETCHNAME, SKETCHNAME);
        usage();
        exit(1);
    }

    assert(callbacks);
    g_callbacks = callbacks;

    for (i=0; i<argc; i++) {
        if (strcmp(argv[i], "start") == 0) {
            if (thread_running) {
                printf("%s already running\n", SKETCHNAME);
                /* this is not an error */
                exit(0);
            }

            uartADriver.set_device_path(deviceA);
            uartCDriver.set_device_path(deviceC);
            uartDDriver.set_device_path(deviceD);
            uartEDriver.set_device_path(deviceE);
            uartFDriver.set_device_path(deviceF);

            printf("Starting %s uartA=%s uartC=%s uartD=%s uartE=%s uartF=%s\n",
                   SKETCHNAME, deviceA, deviceC, deviceD, deviceE, deviceF);

            _vrbrain_thread_should_exit = false;
            daemon_task = px4_task_spawn_cmd(SKETCHNAME,
                                             SCHED_FIFO,
                                             APM_MAIN_PRIORITY,
                                             APM_MAIN_THREAD_STACK_SIZE,
                                             main_loop,
                                             nullptr);
            exit(0);
        }

        if (strcmp(argv[i], "stop") == 0) {
            _vrbrain_thread_should_exit = true;
            exit(0);
        }
 
        if (strcmp(argv[i], "status") == 0) {
            if (_vrbrain_thread_should_exit && thread_running) {
                printf("\t%s is exiting\n", SKETCHNAME);
            } else if (thread_running) {
                printf("\t%s is running\n", SKETCHNAME);
            } else {
                printf("\t%s is not started\n", SKETCHNAME);
            }
            exit(0);
        }

        if (strcmp(argv[i], "-d") == 0) {
            // set terminal device
            if (argc > i + 1) {
                deviceA = strdup(argv[i+1]);
            } else {
                printf("missing parameter to -d DEVICE\n");
                usage();
                exit(1);
            }
        }

        if (strcmp(argv[i], "-d2") == 0) {
            // set uartC terminal device
            if (argc > i + 1) {
                deviceC = strdup(argv[i+1]);
            } else {
                printf("missing parameter to -d2 DEVICE\n");
                usage();
                exit(1);
            }
        }

        if (strcmp(argv[i], "-d3") == 0) {
            // set uartD terminal device
            if (argc > i + 1) {
                deviceD = strdup(argv[i+1]);
            } else {
                printf("missing parameter to -d3 DEVICE\n");
                usage();
                exit(1);
            }
        }

        if (strcmp(argv[i], "-d4") == 0) {
            // set uartE 2nd GPS device
            if (argc > i + 1) {
                deviceE = strdup(argv[i+1]);
            } else {
                printf("missing parameter to -d4 DEVICE\n");
                usage();
                exit(1);
            }
        }

        if (strcmp(argv[i], "-d5") == 0) {
            // set uartF
            if (argc > i + 1) {
                deviceF = strdup(argv[i+1]);
            } else {
                printf("missing parameter to -d5 DEVICE\n");
                usage();
                exit(1);
            }
        }
    }
 
    usage();
    exit(1);
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_VRBRAIN hal_vrbrain;
    return hal_vrbrain;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

