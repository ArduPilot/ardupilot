#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <assert.h>
#include <errno.h>

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "SITL_State.h"
#include "Util.h"

#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

using namespace HALSITL;

static Storage sitlStorage;
static SITL_State sitlState;
static Scheduler sitlScheduler(&sitlState);
static RCInput  sitlRCInput(&sitlState);
static RCOutput sitlRCOutput(&sitlState);
static AnalogIn sitlAnalogIn(&sitlState);
static GPIO sitlGPIO(&sitlState);

// use the Empty HAL for hardware we don't emulate
static Empty::I2CDeviceManager i2c_mgr_instance;
static Empty::SPIDeviceManager emptySPI;
static Empty::OpticalFlow emptyOpticalFlow;

static UARTDriver sitlUart0Driver(0, &sitlState);
static UARTDriver sitlUart1Driver(1, &sitlState);
static UARTDriver sitlUart2Driver(2, &sitlState);
static UARTDriver sitlUart3Driver(3, &sitlState);
static UARTDriver sitlUart4Driver(4, &sitlState);
static UARTDriver sitlUart5Driver(5, &sitlState);
static UARTDriver sitlUart6Driver(6, &sitlState);

static Util utilInstance(&sitlState);

HAL_SITL::HAL_SITL() :
    AP_HAL::HAL(
        &sitlUart0Driver,   /* uartA */
        &sitlUart1Driver,   /* uartB */
        &sitlUart2Driver,   /* uartC */
        &sitlUart3Driver,   /* uartD */
        &sitlUart4Driver,   /* uartE */
        &sitlUart5Driver,   /* uartF */
        &sitlUart6Driver,   /* uartG */
        &i2c_mgr_instance,
        &emptySPI,          /* spi */
        &sitlAnalogIn,      /* analogin */
        &sitlStorage, /* storage */
        &sitlUart0Driver,   /* console */
        &sitlGPIO,          /* gpio */
        &sitlRCInput,       /* rcinput */
        &sitlRCOutput,      /* rcoutput */
        &sitlScheduler,     /* scheduler */
        &utilInstance,      /* util */
        &emptyOpticalFlow, /* onboard optical flow */
        nullptr),           /* CAN */
    _sitl_state(&sitlState)
{}

static char *new_argv[100];

void HAL_SITL::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    _sitl_state->init(argc, argv);
    scheduler->init();
    uartA->begin(115200);

    rcin->init();
    rcout->init();

    // spi->init();
    analogin->init();

    callbacks->setup();
    scheduler->system_initialized();

    while (!HALSITL::Scheduler::_should_reboot) {
        callbacks->loop();
        HALSITL::Scheduler::_run_io_procs();
    }

    // form a new argv, removing problem parameters
    uint8_t new_argv_offset = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(new_argv) && i<argc; i++) {
        if (!strcmp(argv[i], "-w")) {
            // don't wipe params on reboot
            continue;
        }
        new_argv[new_argv_offset++] = argv[i];
    }
    execv(new_argv[0], new_argv);
    AP_HAL::panic("PANIC: REBOOT FAILED: %s", strerror(errno));
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_SITL hal;
    return hal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
