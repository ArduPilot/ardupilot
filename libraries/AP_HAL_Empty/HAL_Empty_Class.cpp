
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_EMPTY

#include <assert.h>

#include "HAL_Empty_Class.h"
#include "AP_HAL_Empty_Private.h"

using namespace Empty;

static UARTDriver uartADriver;
static UARTDriver uartBDriver;
static UARTDriver uartCDriver;
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;

HAL_Empty::HAL_Empty() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        nullptr,            /* no uartD */
        nullptr,            /* no uartE */
        nullptr,            /* no uartF */
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver),
    _member(new EmptyPrivateMember(123))
{}

void HAL_Empty::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    uartA->begin(115200);
    _member->init();

    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_Empty hal;
    return hal;
}

#endif
