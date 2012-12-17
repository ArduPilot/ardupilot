
#include <AP_HAL.h>

#include "HAL_Empty_Class.h"
#include "AP_HAL_Empty_Private.h"

using namespace Empty;

static EmptyUARTDriver uartADriver;
static EmptyUARTDriver uartBDriver;
static EmptyUARTDriver uartCDriver;
static EmptyI2CDriver  i2cDriver;
static EmptySPIDeviceManager spiDeviceManager;
static EmptyAnalogIn analogIn;
static EmptyStorage storageDriver;
static EmptyGPIO gpioDriver;
static EmptyRCInput rcinDriver;
static EmptyRCOutput rcoutDriver;
static EmptyScheduler scheduler;

HAL_Empty::HAL_Empty() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &scheduler ),
    _priv(new EmptyPrivateMember(123))
{}

void HAL_Empty::init(int argc, const char * argv[]) const {
    uartA->init();    
    _priv->init();
}

const HAL_Empty AP_HAL_Empty;

