/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_private.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "Console.h"

static SITLScheduler sitlScheduler;
static SITLEEPROMStorage sitlEEPROMStorage;
static SITLConsoleDriver consoleDriver;

static AVR_SITL::SITLUARTDriver sitlUart0Driver(0);
static AVR_SITL::SITLUARTDriver sitlUart1Driver(1);
static AVR_SITL::SITLUARTDriver sitlUart2Driver(2);

HAL_AVR_SITL::HAL_AVR_SITL() :
    AP_HAL::HAL(
	    &sitlUart0Driver,  /* uartA */
        &sitlUart1Driver, /* uartB */
        &sitlUart2Driver,  /* uartC */
        NULL, /* i2c */
        NULL, /* spi */
        NULL, /* analogin */
        &sitlEEPROMStorage, /* storage */
        &consoleDriver, /* console */
        NULL, /* gpio */
        NULL, /* rcinput */
        NULL, /* rcoutput */
        &sitlScheduler) /* scheduler */
{}

void HAL_AVR_SITL::init(void* machtnichts) const {

    scheduler->init(NULL);
    uartA->begin(115200);
    console->init((void*) uartA);

    rcin->init(NULL);
    rcout->init(NULL);
    spi->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init(NULL);
}

const HAL_AVR_SITL AP_HAL_AVR_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
