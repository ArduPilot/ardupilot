
#include <AP_HAL/AP_HAL.h>

#include <assert.h>

#include "HAL_RP_Class.h"
#include "AP_HAL_RP_Private.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
//#include <AP_Filesystem/AP_Filesystem_RP2350.h>

using namespace RP;

static UARTDriver serial0Driver(uart0, UART0_TX, UART0_RX, 0);
static UARTDriver serial1Driver(uart1, UART1_TX, UART1_RX, 1);
//static UARTDriver serial2Driver;
//static UARTDriver serial3Driver;
static SPIDeviceManager spiDeviceManager;
static I2CDeviceManager i2cDeviceManager;
static AnalogIn analogIn;

#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static Storage storageDriver;
#endif

static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Flash flashDriver;
static NAND_PIO_Driver nandPioDriver;

#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
static WSPIDeviceManager wspiDeviceManager;
#endif

HAL_RP::HAL_RP() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        nullptr,
        nullptr,
        //&serial2Driver,
        //&serial3Driver,
        nullptr,            /* no SERIAL4 */
        nullptr,            /* no SERIAL5 */
        nullptr,            /* no SERIAL6 */
        nullptr,            /* no SERIAL7 */
        nullptr,            /* no SERIAL8 */
        nullptr,            /* no SERIAL9 */
        &i2cDeviceManager,
        &spiDeviceManager,
#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
        &wspiDeviceManager,
#else
        nullptr,
#endif
        &analogIn,
        &storageDriver,
        &serial1Driver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
        nullptr)
{}

void HAL_RP::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    serial(1)->begin(115200);
    this->get_nand_pio()->init(NAND_FLASH_IO_BASE, NAND_FLASH_SCLK, NAND_FLASH_CS);

    callbacks->setup();
    scheduler->set_system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

NAND_PIO_Driver* HAL_RP::get_nand_pio() const {
    return &nandPioDriver;
}

static HAL_RP hal_rp;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_rp;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_rp;
}

void AP_HAL::init()
{
}
