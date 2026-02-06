
#include <AP_HAL/AP_HAL.h>

#include <assert.h>

#include "HAL_RP_Class.h"
#include "AP_HAL_RP_Private.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
//#include <AP_Filesystem/AP_Filesystem_RP2350.h>

using namespace RP;

static UARTDriver serial0Driver(uart0, UART0_TX, UART0_RX, 0);
static UARTDriver serial1Driver(uart1, UART1_TX, UART1_RX, 1);

#if defined(HAL_SERIAL2_DRIVER_ENABLED) && HAL_SERIAL2_DRIVER_ENABLED == 1
static UARTDriver serial2Driver;
#endif

#if defined(HAL_SERIAL3_DRIVER_ENABLED) && HAL_SERIAL3_DRIVER_ENABLED == 1
static UARTDriver serial3Driver;
#endif

#if defined(HAL_SPI_DEVICE_DRIVER_ENABLED) && HAL_SPI_DEVICE_DRIVER_ENABLED == 1
static SPIDeviceManager spiDeviceManager;
#endif

#if defined(HAL_I2C_DEVICE_DRIVER_ENABLED) && HAL_I2C_DEVICE_DRIVER_ENABLED == 1
static I2CDeviceManager i2cDeviceManager;
#endif

#if defined(HAL_ANALOGIN_DRIVER_ENABLED) && HAL_ANALOGIN_DRIVER_ENABLED == 1
static AnalogIn analogIn;
#endif

#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#elif defined(HAL_STORAGE_DRIVER_ENABLED) && HAL_STORAGE_DRIVER_ENABLED == 1
static Storage storageDriver;
#endif

static GPIO gpioDriver;

#if defined(HAL_RCIN_DRIVER_ENABLED) && HAL_RCIN_DRIVER_ENABLED == 1
static RCInput rcinDriver;
#endif

#if defined(HAL_RCOUT_DRIVER_ENABLED) && HAL_RCOUT_DRIVER_ENABLED == 1
static RCOutput rcoutDriver;
#endif

static Scheduler schedulerInstance;
static Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;

#if defined(HAL_FLASH_DRIVER_ENABLED) && HAL_FLASH_DRIVER_ENABLED == 1
static Flash flashDriver;
#endif

#if defined(HAL_NAND_FLASH_ENABLED) && HAL_NAND_FLASH_ENABLED == 1
static NAND_PIO_Driver nandPioDriver;
#endif

#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
static WSPIDeviceManager wspiDeviceManager;
#endif

HAL_RP::HAL_RP() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
#if defined(HAL_SERIAL2_DRIVER_ENABLED) && HAL_SERIAL2_DRIVER_ENABLED == 1
        &serial2Driver,
#else
        nullptr,
#endif
#if defined(HAL_SERIAL3_DRIVER_ENABLED) && HAL_SERIAL3_DRIVER_ENABLED == 1
        &serial3Driver,
#else
        nullptr,
#endif
        nullptr,            /* no SERIAL4 */
        nullptr,            /* no SERIAL5 */
        nullptr,            /* no SERIAL6 */
        nullptr,            /* no SERIAL7 */
        nullptr,            /* no SERIAL8 */
        nullptr,            /* no SERIAL9 */
#if defined(HAL_I2C_DEVICE_DRIVER_ENABLED) && HAL_I2C_DEVICE_DRIVER_ENABLED == 1
        &i2cDeviceManager,
#else
        nullptr,
#endif
#if defined(HAL_SPI_DEVICE_DRIVER_ENABLED) && HAL_SPI_DEVICE_DRIVER_ENABLED == 1
        &spiDeviceManager,
#else
        nullptr,
#endif
#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
        &wspiDeviceManager,
#else
        nullptr,
#endif
#if defined(HAL_ANALOGIN_DRIVER_ENABLED) && HAL_ANALOGIN_DRIVER_ENABLED == 1
        &analogIn,
#else
        nullptr,
#endif
#if defined(HAL_STORAGE_DRIVER_ENABLED) && HAL_STORAGE_DRIVER_ENABLED == 1
        &storageDriver,
#else
        nullptr,
#endif
        &serial1Driver, // console
        &gpioDriver,
#if defined(HAL_RCIN_DRIVER_ENABLED) && HAL_RCIN_DRIVER_ENABLED == 1
        &rcinDriver,
#else
        nullptr,
#endif
#if defined(HAL_RCOUT_DRIVER_ENABLED) && HAL_RCOUT_DRIVER_ENABLED == 1
        &rcoutDriver,
#else
        nullptr,
#endif
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
#if defined(HAL_FLASH_DRIVER_ENABLED) && HAL_FLASH_DRIVER_ENABLED == 1
        &flashDriver,
#else
        nullptr,
#endif
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
#if defined(HAL_NAND_FLASH_ENABLED) && HAL_NAND_FLASH_ENABLED == 1
    this->get_nand_pio()->init(NAND_FLASH_IO_BASE, NAND_FLASH_SCLK, NAND_FLASH_CS);
#endif

    callbacks->setup();
    scheduler->set_system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

NAND_PIO_Driver* HAL_RP::get_nand_pio() const {
#if defined(HAL_NAND_FLASH_ENABLED) && HAL_NAND_FLASH_ENABLED == 1
    return &nandPioDriver;
#else
    return nullptr;
#endif
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
