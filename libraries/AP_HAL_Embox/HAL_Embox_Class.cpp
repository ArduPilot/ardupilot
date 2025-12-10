#include "HAL_Embox_Class.h"

#include <assert.h>
#include <cstdio>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/getopt_cpp.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "GPIO.h"
#include "I2CDevice.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "SPIDevice.h"
#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

using namespace Embox;

static Util utilInstance;

static UARTDriver serial0Driver(true);
static UARTDriver serial1Driver(false);

static I2CDeviceManager i2c_mgr_instance;
static SPIDeviceManager spi_mgr_instance;

static UARTDriver* serialDrivers[] = {
    &serial0Driver,
    &serial1Driver,
};

static Empty::AnalogIn analogIn;

static Storage storageDriver;

static GPIO gpioDriver;

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
    enum long_options {
        CMDLINE_SERIAL0 = 1, // must be in 0-9 order and numbered consecutively
        CMDLINE_SERIAL1,

    };

    int opt;
    const struct GetOptLong::option options[] = {
        {"serial0", true, 0, CMDLINE_SERIAL0},
        {"serial1", true, 0, CMDLINE_SERIAL1},
        {"log-directory", true, 0, 'l'},
        {"terrain-directory", true, 0, 't'},
        {"storage-directory", true, 0, 's'},
        {"module-directory", true, 0, 'M'},
        {"defaults", true, 0, 'd'},
        {"help", false, 0, 'h'},
        {0, false, 0, 0}};

    GetOptLong gopt(argc, argv, "l:t:s:he:SM:",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case CMDLINE_SERIAL0:
        case CMDLINE_SERIAL1:
            serialDrivers[opt - CMDLINE_SERIAL0]->set_device_path(gopt.optarg);
            break;
        case 'l':
            utilInstance.set_custom_log_directory(gopt.optarg);
            break;
        case 't':
            utilInstance.set_custom_terrain_directory(gopt.optarg);
            break;
        case 's':
            utilInstance.set_custom_storage_directory(gopt.optarg);
            break;
        case 'd':
            utilInstance.set_custom_defaults_path(gopt.optarg);
            break;
        case 'h':
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    // NOTE: signal handlers are only set before the main loop, so
    // that if anything before the loops hangs, the default signals
    // can still stop the process proprely, although without proper
    // teardown.
    // This isn't perfect, but still prevents an unkillable process.

    scheduler->init();
    gpio->init();
    rcout->init();
    rcin->init();
    serial(0)->begin(115200);
    analogin->init();
    utilInstance.init(argc + gopt.optind - 1, &argv[gopt.optind - 1]);

    // NOTE: See commit 9f5b4ffca ("AP_HAL_Linux_Class: Correct
    // deadlock, and infinite loop in setup()") for details about the
    // order of scheduler initialize and setup on Linux.
    scheduler->set_system_initialized();

    callbacks->setup();

    setup_signal_handlers();

    while (!_should_exit) {
        callbacks->loop();
    }

    // At least try to stop all PWM before shutting down
    rcout->force_safety_on();
    rcin->teardown();
    Scheduler::from(scheduler)->teardown();
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
