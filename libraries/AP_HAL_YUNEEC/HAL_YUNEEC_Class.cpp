
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "HAL_YUNEEC_Class.h"
#include "AP_HAL_YUNEEC_Private.h"

using namespace YUNEEC;

static YUNEECUARTDriver uartADriver;
static YUNEECUARTDriver uartBDriver;
static YUNEECUARTDriver uartCDriver;
static YUNEECSemaphore  i2cSemaphore;
static YUNEECI2CDriver  i2cDriver(&i2cSemaphore);
static YUNEECSPIDeviceManager spiDeviceManager;
static YUNEECAnalogIn analogIn;
static YUNEECStorage storageDriver;
static YUNEECGPIO gpioDriver;
static YUNEECRCInput rcinDriver;
static YUNEECRCOutput rcoutDriver;
static YUNEECScheduler schedulerInstance;
static YUNEECUtil utilInstance;

HAL_YUNEEC::HAL_YUNEEC() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void HAL_YUNEEC::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);
    uartA->begin(115200);
}

const HAL_YUNEEC AP_HAL_YUNEEC;

#endif
