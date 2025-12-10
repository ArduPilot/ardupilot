

#include "AP_HAL_Embox/RCInput.h"
#include "AP_HAL_Embox/RCOutput.h"
#include "AP_HAL_Embox/Scheduler.h"
#include "AP_HAL_Embox/Storage.h"
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_EMBOX

#include <assert.h>

#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "AP_HAL_Embox/I2CDevice.h"
#include "AP_HAL_Embox/SPIDevice.h"
#include "AP_HAL_Embox/UARTDriver.h"
#include "AP_HAL_Embox/Util.h"
#include "HAL_Embox_Class.h"

using namespace Embox;

static Util utilInstance;

static UARTDriver serial0Driver(true);
static UARTDriver serial1Driver(false);

static I2CDeviceManager i2c_mgr_instance;
static SPIDeviceManager spi_mgr_instance;

// static UARTDriver* serialDrivers[] = {
//     &serial0Driver,
//     &serial1Driver,
// };

static Empty::AnalogIn analogIn;

static Storage storageDriver;

static Empty::GPIO gpioDriver;

static RCInput rcinDriver;
static RCOutput rcoutDriver;

static Scheduler schedulerInstance;

static Empty::OpticalFlow opticalFlowDriver;
#if HAL_WITH_DSP
static Empty::DSP dspDriver;
#endif
static Empty::Flash flashDriver;
static Empty::WSPIDeviceManager wspi_mgr_instance;

HAL_Embox::HAL_Embox() : AP_HAL::HAL(
                             &serial0Driver,
                             &serial1Driver,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr,
                             &i2c_mgr_instance,
                             &spi_mgr_instance,
                             &wspi_mgr_instance,
                             &analogIn,
                             &storageDriver,
                             &serial0Driver,
                             &gpioDriver,
                             &rcinDriver,
                             &rcoutDriver,
                             &schedulerInstance,
                             &utilInstance,
                             &opticalFlowDriver,
                             &flashDriver,
#if HAL_WITH_DSP
                             &dspDriver,
#endif
                             nullptr

                         ) {
}

void HAL_Embox::run(int argc, char* const argv[], Callbacks* callbacks) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    gpio->init();
    rcout->init();
    rcin->init();
    serial(0)->begin(115200);

    callbacks->setup();
    scheduler->set_system_initialized();

    for (;;) {
        callbacks->loop();
    }
}
void HAL_Embox::setup_signal_handlers() const {
    struct sigaction sa = {};

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = HAL_Embox::exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
}

static HAL_Embox hal_embox;

void HAL_Embox::exit_signal_handler(int signum) {
    hal_embox._should_exit = true;
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_embox;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_embox;
}

#endif
