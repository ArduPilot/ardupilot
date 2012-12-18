/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_private.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "Console.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "SITL_State.h"

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

using namespace AVR_SITL;

static SITLScheduler sitlScheduler;
static SITLEEPROMStorage sitlEEPROMStorage;
static SITLConsoleDriver consoleDriver;
static SITL_State sitlState;
static SITLRCInput  sitlRCInput(&sitlState);
static SITLRCOutput sitlRCOutput(&sitlState);
static SITLAnalogIn sitlAnalogIn(&sitlState);

static Empty::EmptyGPIO emptyGPIO;

static SITLUARTDriver sitlUart0Driver(0, &sitlState);
static SITLUARTDriver sitlUart1Driver(1, &sitlState);
static SITLUARTDriver sitlUart2Driver(2, &sitlState);

HAL_AVR_SITL::HAL_AVR_SITL() :
    AP_HAL::HAL(
	    &sitlUart0Driver,  /* uartA */
        &sitlUart1Driver, /* uartB */
        &sitlUart2Driver,  /* uartC */
        NULL, /* i2c */
        NULL, /* spi */
        &sitlAnalogIn, /* analogin */
        &sitlEEPROMStorage, /* storage */
        &consoleDriver, /* console */
        &emptyGPIO, /* gpio */
        &sitlRCInput,  /* rcinput */
        &sitlRCOutput, /* rcoutput */
        &sitlScheduler), /* scheduler */
    _sitl_state(&sitlState)
{}

void HAL_AVR_SITL::init(int argc, char * const argv[]) const 
{
    _sitl_state->init(argc, argv);
    scheduler->init(NULL);
    uartA->begin(115200);
    console->init((void*) uartA);

    rcin->init(NULL);
    rcout->init(NULL);

    //spi->init(NULL);
    //i2c->begin();
    //i2c->setTimeout(100);
    analogin->init(NULL);
}

const HAL_AVR_SITL AP_HAL_AVR_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
