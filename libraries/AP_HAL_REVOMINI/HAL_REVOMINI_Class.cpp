
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include "AP_HAL_REVOMINI_Namespace.h"
#include "HAL_REVOMINI_Class.h"
#include "AP_HAL_REVOMINI_Private.h"
#include "Util.h"
#include <assert.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <pwm_in.h>
#include <usart.h>
#include <i2c.h>

using namespace AP_HAL;
using namespace REVOMINI;

//_USART1 PIN 3 AND 4 OF THE INPUT RAIL
//_USART2 INTERNAL SERIAL PORT
//_USART3 PIN 1 AND 2 OF THE INPUT RAIL


// XXX make sure these are assigned correctly
static REVOMINIUARTDriver uartADriver(_USART1,1);
static REVOMINIUARTDriver uartBDriver(_USART6,0);

static REVOMINISemaphore  i2cSemaphore;
static REVOMINISemaphore  i2c2Semaphore;
static REVOMINII2CDriver  i2cDriver(_I2C1,&i2cSemaphore);
static REVOMINII2CDriver  i2c2Driver(_I2C1,&i2c2Semaphore);
static REVOMINISPIDeviceManager spiDeviceManager;
static REVOMINIAnalogIn analogIn;
static REVOMINIStorage storageDriver;
static REVOMINIGPIO gpioDriver;
static REVOMINIRCInput rcinDriver;
static REVOMINIRCOutput rcoutDriver;
static REVOMINIScheduler schedulerInstance;
static REVOMINIUtil utilInstance;



HAL_REVOMINI::HAL_REVOMINI() :
    AP_HAL::HAL(
        &uartADriver,  /* uartA */
        &uartBDriver,  /* uartB */
        NULL,  /* no uartC */
        NULL,  /* no uartD */
        NULL,  /* no uartE */
        NULL,
        &i2cDriver, /* i2c */
        &i2c2Driver,  /* 2 i2c */
        NULL,   /* only 2 i2c */
        &spiDeviceManager, /* spi */
        &analogIn, /* analogin */
        &storageDriver, /* storage */
        &uartADriver, /* console */
        &gpioDriver, /* gpio */
        &rcinDriver,  /* rcinput */
        &rcoutDriver, /* rcoutput */
        &schedulerInstance, /* scheduler */
        &utilInstance,		/* util */
        NULL /* no optical flow */
    ) 

/* 
	&uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            // no uartD 
        NULL,            // no uartE 
        NULL,
        &i2cDriver,
        NULL,   // only 1 i2c 
        NULL,   // only 1 i2c 
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        NULL // no optical flow 
*/

{}


/*
AP_HAL::HAL(
*/


void HAL_REVOMINI::run(int argc,char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();

    /* uartA is the serial port used for the console, so lets make sure
     * it is initialized at boot */
    uartA->begin(57600);
    uartB->begin(57600);

    rcin->init();
    rcout->init();
    spi->init();
    i2c->begin();
    i2c->setTimeout(100);
    i2c2->begin();
    i2c2->setTimeout(100);
    analogin->init();
    storage->init(); // Uses EEPROM.*, flash_stm* copied from AeroQuad_v3.2

    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

//const HAL_REVOMINI AP_HAL_REVOMINI;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_REVOMINI hal;
    return hal;
}


#endif
