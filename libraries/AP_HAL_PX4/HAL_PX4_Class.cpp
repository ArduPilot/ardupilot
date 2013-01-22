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

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

#include <stdlib.h>
#include <systemlib/systemlib.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

using namespace PX4;

static Empty::EmptySemaphore  i2cSemaphore;
static Empty::EmptyI2CDriver  i2cDriver(&i2cSemaphore);
static Empty::EmptySPIDeviceManager spiDeviceManager;
static Empty::EmptyGPIO gpioDriver;

static PX4ConsoleDriver consoleDriver;
static PX4Scheduler schedulerInstance;
static PX4EEPROMStorage storageDriver;
static PX4RCInput rcinDriver;
static PX4RCOutput rcoutDriver;
static PX4AnalogIn analogIn;
static PX4Util utilInstance;

#define UARTA_DEFAULT_DEVICE "/dev/ttyS0"
#define UARTB_DEFAULT_DEVICE "/dev/ttyS3"

// only two real UART drivers for now
static PX4UARTDriver uartADriver(UARTA_DEFAULT_DEVICE);
static PX4UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE);
static Empty::EmptyUARTDriver uartCDriver;

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

bool _px4_thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

extern const AP_HAL::HAL& hal;

static int main_loop(int argc, char **argv)
{
    extern void setup(void);
    extern void loop(void);

    hal.uartA->begin(57600);
    hal.console->init((void*) hal.uartA);
    hal.scheduler->init(NULL);
    hal.rcin->init(NULL);
    hal.rcout->init(NULL);
    hal.analogin->init(NULL);

    setup();
    hal.scheduler->system_initialized();

    thread_running = true;
    while (!_px4_thread_should_exit) {
		loop();
        // yield the CPU between loops to let other apps 
        // get some CPU time
		pthread_yield();
	}
    thread_running = false;
    return 0;
}

static void usage(void)
{
    printf("Usage: %s [options] {start,stop,status}\n", SKETCHNAME);
    printf("Options:\n");
    printf("\t-d DEVICE         set terminal device (default %s)\n", UARTA_DEFAULT_DEVICE);
    printf("\n");
}


void HAL_PX4::init(int argc, char * const argv[]) const 
{
    int i;
    const char *device = UARTA_DEFAULT_DEVICE;

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

            uartADriver.set_device_path(device);
            printf("Starting %s on %s\n", SKETCHNAME, device);

            _px4_thread_should_exit = false;
            daemon_task = task_spawn(SKETCHNAME,
                                     SCHED_RR,
                                     SCHED_PRIORITY_DEFAULT,
                                     4096,
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
                device = strdup(argv[i+1]);
			} else {
				printf("missing parameter to -d DEVICE\n");
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

