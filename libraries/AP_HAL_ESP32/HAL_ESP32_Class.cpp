#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "HAL_ESP32_Class.h"
#include "Scheduler.h"
#include "SPIDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"

static ESP32::UARTDriver cons(0);
static Empty::UARTDriver uartADriver;
static Empty::UARTDriver uartBDriver;
static ESP32::WiFiDriver uartCDriver;
static Empty::UARTDriver uartDDriver;
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
static Empty::I2CDeviceManager i2cDeviceManager;
static ESP32::SPIDeviceManager spiDeviceManager;
static Empty::AnalogIn analogIn;
static Empty::Storage storageDriver;
static Empty::GPIO gpioDriver;
static Empty::RCInput rcinDriver;
static Empty::RCOutput rcoutDriver;
static ESP32::Scheduler schedulerInstance;
static Empty::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;

extern const AP_HAL::HAL& hal;

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &uartFDriver,
        &uartGDriver,
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        nullptr
    )
{}

void print_stats()
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char buffer[1024];
        vTaskGetRunTimeStats(buffer);
        printf("\n\n%s\n", buffer);
        heap_caps_print_heap_info(0);
        last_run = AP_HAL::millis64();
    }
}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    hal.uartA->begin(115200);
    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.analogin->init();
    hal.scheduler->init();

    callbacks->setup();
    hal.scheduler->system_initialized();

    while (true) {
        callbacks->loop();
        hal.scheduler->delay_microseconds(1);
        print_stats();
    }

}

void AP_HAL::init()
{
}

#endif
