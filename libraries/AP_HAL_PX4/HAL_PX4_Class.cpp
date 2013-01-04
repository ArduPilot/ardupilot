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

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

#include <stdlib.h>
#include <poll.h>
#include <systemlib/systemlib.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>

using namespace PX4;

static Empty::EmptyI2CDriver  i2cDriver;
static Empty::EmptySPIDeviceManager spiDeviceManager;
static Empty::EmptyAnalogIn analogIn;
static Empty::EmptyGPIO gpioDriver;
static Empty::EmptyRCOutput rcoutDriver;
static Empty::EmptyUtil utilInstance;

static PX4ConsoleDriver consoleDriver;
static PX4Scheduler schedulerInstance;
static PX4EEPROMStorage storageDriver;
static PX4RCInput rcinDriver;

#define UARTA_DEVICE "/dev/ttyS2"
#define UARTB_DEVICE "/dev/ttyS3"

// only two real UART drivers for now
static PX4UARTDriver uartADriver(UARTA_DEVICE);
static PX4UARTDriver uartBDriver(UARTB_DEVICE);
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

static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

extern const AP_HAL::HAL& hal;

static int main_loop(int argc, char **argv)
{
    extern void setup(void);
    extern void loop(void);
    hal.uartA->begin(115200);
    hal.console->init((void*) hal.uartA);
    hal.scheduler->init(NULL);
    hal.rcin->init(NULL);

    setup();

    while (true) {
		loop();

        // yield the CPU for 1ms between loops to let other apps 
        // get some CPU time
		poll(NULL, 0, 1);
	}
    return 0;
}


void HAL_PX4::init(int argc, char * const argv[]) const 
{
    if (argc < 1) {
		printf("%s: missing command (try '%s start')", 
               SKETCHNAME, SKETCHNAME);
        exit(1);
    }

    if (strcmp(argv[1], "start") == 0) {
		if (thread_running) {
			printf("%s already running\n", SKETCHNAME);
			/* this is not an error */
			exit(0);
		}

        printf("Starting %s on %s\n", SKETCHNAME, UARTA_DEVICE);

		thread_should_exit = false;
		daemon_task = task_spawn(SKETCHNAME,
                                 SCHED_RR,
                                 SCHED_PRIORITY_DEFAULT,
                                 4096,
                                 main_loop,
                                 NULL);
		exit(0);
	}

    if (strcmp(argv[1], "stop") == 0) {
		thread_should_exit = true;
		exit(0);
	}
 
	if (strcmp(argv[1], "status") == 0) {
		if (thread_running) {
			printf("\t%s is running\n", SKETCHNAME);
		} else {
			printf("\t%s is not started\n", SKETCHNAME);
		}
		exit(0);
	}
 
	printf("%s: unrecognized command ('start', 'stop' or 'status')", SKETCHNAME);
	exit(1);
}

const HAL_PX4 AP_HAL_PX4;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4

