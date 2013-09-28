#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_Linux_Class.h"
#include "AP_HAL_Linux_Private.h"

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

using namespace Linux;

// only using 2 serial ports on Linux for now
static LinuxUARTDriver uartADriver;
static LinuxUARTDriver uartBDriver;
static Empty::EmptyUARTDriver uartCDriver;

static LinuxSemaphore  i2cSemaphore;
static LinuxI2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static Empty::EmptySPIDeviceManager spiDeviceManager;
static LinuxAnalogIn analogIn;
static LinuxStorage storageDriver;
static LinuxConsoleDriver consoleDriver(&uartADriver);
static LinuxGPIO gpioDriver;
static LinuxRCInput rcinDriver;
static LinuxRCOutput rcoutDriver;
static LinuxScheduler schedulerInstance;
static LinuxUtil utilInstance;

HAL_Linux::HAL_Linux() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &consoleDriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void HAL_Linux::init(int argc,char* const argv[]) const 
{
    int opt;
    /*
      parse command line options
     */
    while ((opt = getopt(argc, argv, "A:B:h")) != -1) {
        switch (opt) {
        case 'A':
            uartADriver.set_device_path(optarg);
            break;
        case 'B':
            uartBDriver.set_device_path(optarg);
            break;
        case 'h':
            printf("Usage: -A uartAPath -B uartAPath\n");
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);
    uartA->begin(115200);
    i2c->begin();
}

const HAL_Linux AP_HAL_Linux;

#endif
