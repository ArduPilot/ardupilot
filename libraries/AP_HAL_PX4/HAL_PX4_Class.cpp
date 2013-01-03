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

// only one real UART driver for now
static PX4UARTDriver uartADriver("/dev/ttyS0");
static Empty::EmptyUARTDriver uartBDriver;
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

void HAL_PX4::init(int argc, char * const argv[]) const 
{
    scheduler->init(NULL);
    uartA->begin(115200);
    console->init((void*) uartA);
}

const HAL_PX4 AP_HAL_PX4;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4

