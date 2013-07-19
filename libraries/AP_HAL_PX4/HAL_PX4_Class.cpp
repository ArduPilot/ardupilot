/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_HAL_PX4.h>
#include "AP_HAL_PX4_Namespace.h"
#include "HAL_PX4_Class.h"
#include "Console.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "AnalogIn.h"
#include "Util.h"
#include "GPIO.h"

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

#include <stdlib.h>
#include <systemlib/systemlib.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <poll.h>
#include <drivers/drv_hrt.h>

using namespace PX4;

static Empty::EmptySemaphore  i2cSemaphore;
static Empty::EmptyI2CDriver  i2cDriver(&i2cSemaphore);
static Empty::EmptySPIDeviceManager spiDeviceManager;
//static Empty::EmptyGPIO gpioDriver;

static PX4ConsoleDriver consoleDriver;
static PX4Scheduler schedulerInstance;
static PX4Storage storageDriver;
static PX4RCInput rcinDriver;
static PX4RCOutput rcoutDriver;
static PX4AnalogIn analogIn;
static PX4Util utilInstance;
static PX4GPIO gpioDriver;

#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS3"
#define UARTC_DEFAULT_DEVICE "/dev/ttyS1"

// 3 UART drivers, for GPS plus two mavlink-enabled devices
static PX4UARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "APM_uartA");
static PX4UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "APM_uartB");
static PX4UARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "APM_uartC");

HAL_PX4::HAL_PX4() :
    AP_HAL::HAL(
        &uartADriver,  /* uartA */
        &uartBDriver,  /* uartB */
        &uartCDriver,  /* uartC */
        &i2cDriver, /* i2c */
        &spiDeviceManager, /* spi */
        &analogIn, /* analogin */
        &storageDriver, /* storage */
        &consoleDriver, /* console */
        &gpioDriver, /* gpio */
        &rcinDriver,  /* rcinput */
        &rcoutDriver, /* rcoutput */
        &schedulerInstance, /* scheduler */
        &utilInstance) /* util */
{}

bool _px4_thread_should_exit = false;        /**< Daemon exit flag */
static bool thread_running = false;        /**< Daemon status flag */
static int daemon_task;                /**< Handle of daemon task / thread */
static bool ran_overtime;

extern const AP_HAL::HAL& hal;

static void semaphore_yield(void *sem)
{
    sem_post((sem_t *)sem);
}

/*
  set the priority of the main APM task
 */
static void set_priority(uint8_t priority)
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
    set_priority(APM_OVERTIME_PRIORITY);
    ran_overtime = true;
}

static int main_loop(int argc, char **argv)
{
    extern void setup(void);
    extern void loop(void);


    hal.uartA->begin(115200);
    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.console->init((void*) hal.uartA);
    hal.scheduler->init(NULL);
    hal.rcin->init(NULL);
    hal.rcout->init(NULL);
    hal.analogin->init(NULL);
    hal.gpio->init();


    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    set_priority(APM_STARTUP_PRIORITY);

    setup();
    hal.scheduler->system_initialized();

    perf_counter_t perf_loop = perf_alloc(PC_ELAPSED, "APM_loop");
    perf_counter_t perf_overrun = perf_alloc(PC_COUNT, "APM_overrun");
    sem_t loop_semaphore;
    struct hrt_call loop_call;
    struct hrt_call loop_overtime_call;

    sem_init(&loop_semaphore, 0, 0);
             
    thread_running = true;

    /*
      switch to high priority for main loop
     */
    set_priority(APM_MAIN_PRIORITY);

    while (!_px4_thread_should_exit) {
        perf_begin(perf_loop);
        
        /*
          this ensures a tight loop waiting on a lower priority driver
          will eventually give up some time for the driver to run. It
          will only ever be called if a loop() call runs for more than
          0.1 second
         */
        hrt_call_after(&loop_overtime_call, 100000, (hrt_callout)loop_overtime, NULL);

        loop();

        if (ran_overtime) {
            /*
              we ran over 1s in loop(), and our priority was lowered
              to let a driver run. Set it back to high priority now.
             */
            set_priority(APM_MAIN_PRIORITY);
            perf_count(perf_overrun);
            ran_overtime = false;
        }

        perf_end(perf_loop);

        if (hal.scheduler->in_timerprocess()) {
            // we are running when a timer process is running! This is
            // a scheduling error, and breaks the assumptions made in
            // our locking system
            ::printf("ERROR: timer processing running in loop()\n");
        }

        /*
          give up 500 microseconds of time, to ensure drivers get a
          chance to run. This gives us better timing performance than
          a poll(NULL, 0, 1)
         */
        hrt_call_after(&loop_call, 500, (hrt_callout)semaphore_yield, &loop_semaphore);
        sem_wait(&loop_semaphore);
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
    printf("\n");
}


void HAL_PX4::init(int argc, char * const argv[]) const 
{
    int i;
    const char *deviceA = UARTA_DEFAULT_DEVICE;
    const char *deviceC = UARTC_DEFAULT_DEVICE;

    if (argc < 1) {
        printf("%s: missing command (try '%s start')", 
               SKETCHNAME, SKETCHNAME);
        usage();
        exit(1);
    }

    for (i=0; i<argc; i++) {
        if (strcmp(argv[i], "start") == 0) {
            if (thread_running) {
                printf("%s already running\n", SKETCHNAME);
                /* this is not an error */
                exit(0);
            }

            uartADriver.set_device_path(deviceA);
            uartCDriver.set_device_path(deviceC);
            printf("Starting %s on %s and %s\n", 
                   SKETCHNAME, deviceA, deviceC);

            _px4_thread_should_exit = false;
            daemon_task = task_spawn_cmd(SKETCHNAME,
                                     SCHED_FIFO,
                                     APM_MAIN_PRIORITY,
                                     8192,
                                     main_loop,
                                     NULL);
            exit(0);
        }

        if (strcmp(argv[i], "stop") == 0) {
            _px4_thread_should_exit = true;
            exit(0);
        }
 
        if (strcmp(argv[i], "status") == 0) {
            if (_px4_thread_should_exit && thread_running) {
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
    }
 
    usage();
    exit(1);
}

const HAL_PX4 AP_HAL_PX4;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4

