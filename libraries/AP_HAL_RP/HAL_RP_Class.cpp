
#include <AP_HAL/AP_HAL.h>

#include <assert.h>

#include "HAL_RP_Class.h"
#include "LED.h"
#include "USBSerialDriver.h"
#include "AP_HAL_RP_Private.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
//#include <AP_Filesystem/AP_Filesystem_RP2350.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

using namespace RP;

#ifndef HAL_RP_USB_CDC_CONNECT_TIMEOUT_MS
#define HAL_RP_USB_CDC_CONNECT_TIMEOUT_MS 1500U
#endif

#ifndef HAL_RP_USB_CDC_FALLBACK_TO_UART0
#define HAL_RP_USB_CDC_FALLBACK_TO_UART0 0
#endif

static UARTDriver serial0Driver(uart0, UART0_TX, UART0_RX, 0, true);
static UARTDriver serial1Driver(uart1, UART1_TX, UART1_RX, 1);
static USBSerialDriver usbConsoleDriver;

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

#if defined(AP_NOTIFY_GPIO_LED_1_ENABLED) && AP_NOTIFY_GPIO_LED_1_ENABLED == 1
static LED greenLed;
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
#if defined(HAL_RP_CONSOLE_USB_CDC) && HAL_RP_CONSOLE_USB_CDC == 1
        &usbConsoleDriver,
#else
        &serial0Driver,
#endif
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

extern const AP_HAL::HAL& hal;

void HAL_RP::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    (void)argc;
    (void)argv;

    console->begin(115200);

#if defined(HAL_RP_CONSOLE_USB_CDC) && HAL_RP_CONSOLE_USB_CDC == 1
    const uint32_t usb_wait_start_ms = AP_HAL::millis();
    while (!stdio_usb_connected() && (AP_HAL::millis() - usb_wait_start_ms) < HAL_RP_USB_CDC_CONNECT_TIMEOUT_MS) {
        sleep_ms(10);
    }
#if HAL_RP_USB_CDC_FALLBACK_TO_UART0
    if (!stdio_usb_connected()) {
        serial0Driver.begin(115200);
        serial0Driver.printf("USB CDC not detected after %lu ms, falling back to UART0 console\n",
                             (unsigned long)HAL_RP_USB_CDC_CONNECT_TIMEOUT_MS);
        const_cast<HAL_RP*>(this)->console = &serial0Driver;
    }
#endif
#endif
#if defined(HAL_NAND_FLASH_ENABLED) && HAL_NAND_FLASH_ENABLED == 1
    this->get_nand_pio()->init(NAND_FLASH_IO_BASE, NAND_FLASH_SCLK, NAND_FLASH_CS);
#endif
#if defined(AP_NOTIFY_GPIO_LED_1_ENABLED) && AP_NOTIFY_GPIO_LED_1_ENABLED == 1
    this->get_led_driver()->init();
#endif
    RP::Scheduler * s = ((RP::Scheduler *)hal.scheduler);
    s->set_callbacks(callbacks);
    s->init();
    s->start();
}

NAND_PIO_Driver* HAL_RP::get_nand_pio() const {
#if defined(HAL_NAND_FLASH_ENABLED) && HAL_NAND_FLASH_ENABLED == 1
    return &nandPioDriver;
#else
    return nullptr;
#endif
}

LED* HAL_RP::get_led_driver() const {
#if defined(AP_NOTIFY_GPIO_LED_1_ENABLED) && AP_NOTIFY_GPIO_LED_1_ENABLED == 1
    return &greenLed;
#else
    return nullptr;
#endif
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static HAL_RP hal_rp;
    return hal_rp;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return const_cast<AP_HAL::HAL&>(get_HAL());
}

void AP_HAL::init()
{
}
