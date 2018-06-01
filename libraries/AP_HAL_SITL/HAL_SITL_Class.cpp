/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <assert.h>

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "SITL_State.h"
#include "Util.h"

#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

using namespace HALSITL;

static SITLEEPROMStorage sitlEEPROMStorage;
static SITL_State sitlState;
static SITLScheduler sitlScheduler(&sitlState);
static SITLRCInput  sitlRCInput(&sitlState);
static SITLRCOutput sitlRCOutput(&sitlState);
static SITLAnalogIn sitlAnalogIn(&sitlState);

// use the Empty HAL for hardware we don't emulate
static Empty::EmptyGPIO emptyGPIO;
static Empty::EmptySemaphore emptyI2Csemaphore;
static Empty::EmptyI2CDriver emptyI2C(&emptyI2Csemaphore);
static Empty::EmptySPIDeviceManager emptySPI;

static SITLUARTDriver sitlUart0Driver(0, &sitlState);
static SITLUARTDriver sitlUart1Driver(1, &sitlState);
static SITLUARTDriver sitlUart2Driver(2, &sitlState);
static SITLUARTDriver sitlUart3Driver(3, &sitlState);
static SITLUARTDriver sitlUart4Driver(4, &sitlState);

static SITLUtil utilInstance;

HAL_SITL::HAL_SITL() :
    AP_HAL::HAL(
        &sitlUart0Driver,  /* uartA */
        &sitlUart1Driver,  /* uartB */
        &sitlUart2Driver,  /* uartC */
        &sitlUart3Driver,  /* uartD */
        &sitlUart4Driver,  /* uartE */
        &emptyI2C, /* i2c */
        &emptyI2C, /* i2c */
        &emptyI2C, /* i2c */
        &emptySPI, /* spi */
        &sitlAnalogIn, /* analogin */
        &sitlEEPROMStorage, /* storage */
        &sitlUart0Driver, /* console */
        &emptyGPIO, /* gpio */
        &sitlRCInput,  /* rcinput */
        &sitlRCOutput, /* rcoutput */
        &sitlScheduler, /* scheduler */
        &utilInstance), /* util */
    _sitl_state(&sitlState)
{}

void HAL_SITL::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    _sitl_state->init(argc, argv);
    scheduler->init(NULL);
    uartA->begin(115200);

    rcin->init(NULL);
    rcout->init(NULL);

    //spi->init(NULL);
    //i2c->begin();
    //i2c->setTimeout(100);
    analogin->init(NULL);

    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_SITL hal;
    return hal;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
