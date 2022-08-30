#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
#include "I2CDevice.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "SITL_State.h"
#include "Util.h"
#include "DSP.h"
#include "CANSocketIface.h"
#include "SPIDevice.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>

using namespace HALSITL;

HAL_SITL& hal_sitl = (HAL_SITL&)AP_HAL::get_HAL();

static Storage sitlStorage;
static SITL_State sitlState;
static Scheduler sitlScheduler(&sitlState);
#if !defined(HAL_BUILD_AP_PERIPH)
static RCInput  sitlRCInput(&sitlState);
static RCOutput sitlRCOutput(&sitlState);
static GPIO sitlGPIO(&sitlState);
#else
static Empty::RCInput  sitlRCInput;
static Empty::RCOutput sitlRCOutput;
static Empty::GPIO sitlGPIO;
#endif
static AnalogIn sitlAnalogIn(&sitlState);
static DSP dspDriver;


// use the Empty HAL for hardware we don't emulate
static Empty::OpticalFlow emptyOpticalFlow;
static Empty::Flash emptyFlash;

static UARTDriver sitlUart0Driver(0, &sitlState);
static UARTDriver sitlUart1Driver(1, &sitlState);
static UARTDriver sitlUart2Driver(2, &sitlState);
static UARTDriver sitlUart3Driver(3, &sitlState);
static UARTDriver sitlUart4Driver(4, &sitlState);
static UARTDriver sitlUart5Driver(5, &sitlState);
static UARTDriver sitlUart6Driver(6, &sitlState);
static UARTDriver sitlUart7Driver(7, &sitlState);
static UARTDriver sitlUart8Driver(8, &sitlState);
static UARTDriver sitlUart9Driver(9, &sitlState);

#if defined(HAL_BUILD_AP_PERIPH)
static Empty::I2CDeviceManager i2c_mgr_instance;
static Empty::SPIDeviceManager spi_mgr_instance;
#else
static I2CDeviceManager i2c_mgr_instance;
static SPIDeviceManager spi_mgr_instance;
#endif
static Util utilInstance(&sitlState);

#if HAL_NUM_CAN_IFACES
static HALSITL::CANIface* canDrivers[HAL_NUM_CAN_IFACES];
#endif

static Empty::QSPIDeviceManager qspi_mgr_instance;

HAL_SITL::HAL_SITL() :
    AP_HAL::HAL(
        &sitlUart0Driver,   /* uartA */
        &sitlUart1Driver,   /* uartB */
        &sitlUart2Driver,   /* uartC */
        &sitlUart3Driver,   /* uartD */
        &sitlUart4Driver,   /* uartE */
        &sitlUart5Driver,   /* uartF */
        &sitlUart6Driver,   /* uartG */
        &sitlUart7Driver,   /* uartH */
        &sitlUart8Driver,   /* uartI */
        &sitlUart9Driver,   /* uartJ */
        &i2c_mgr_instance,
        &spi_mgr_instance,  /* spi */
        &qspi_mgr_instance,
        &sitlAnalogIn,      /* analogin */
        &sitlStorage, /* storage */
        &sitlUart0Driver,   /* console */
        &sitlGPIO,          /* gpio */
        &sitlRCInput,       /* rcinput */
        &sitlRCOutput,      /* rcoutput */
        &sitlScheduler,     /* scheduler */
        &utilInstance,      /* util */
        &emptyOpticalFlow,  /* onboard optical flow */
        &emptyFlash,        /* flash driver */
        &dspDriver,         /* dsp driver */
#if HAL_NUM_CAN_IFACES
        (AP_HAL::CANIface**)canDrivers
#else
        nullptr
#endif
        ),           /* CAN */
    _sitl_state(&sitlState)
{}

static char *new_argv[100];

/*
  save watchdog data
 */
static bool watchdog_save(const uint32_t *data, uint32_t nwords)
{
    int fd = ::open("persistent.dat", O_WRONLY|O_CREAT|O_TRUNC, 0644);
    bool ret = false;
    if (fd != -1) {
        if (::write(fd, data, nwords*4) == (ssize_t)(nwords*4)) {
            ret = true;
        }
        ::close(fd);
    }
    return ret;
}

/*
  load watchdog data
 */
static bool watchdog_load(uint32_t *data, uint32_t nwords)
{
    int fd = ::open("persistent.dat", O_RDONLY, 0644);
    bool ret = false;
    if (fd != -1) {
        ret = (::read(fd, data, nwords*4) == (ssize_t)(nwords*4));
        ::close(fd);
    }
    return ret;
}

/*
  implement watchdoh reset via SIGALRM
 */
static void sig_alrm(int signum)
{
    static char env[] = "SITL_WATCHDOG_RESET=1";
    putenv(env);
    printf("GOT SIGALRM\n");
    execv(new_argv[0], new_argv);
}

void HAL_SITL::exit_signal_handler(int signum)
{
    HALSITL::Scheduler::_should_exit = true;
}

void HAL_SITL::setup_signal_handlers() const
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = HAL_SITL::exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
#if defined(HAL_COVERAGE_BUILD) && HAL_COVERAGE_BUILD == 1
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
#endif

}

/*
  fill 8k of stack with NaN. This allows us to find uses of
  uninitialised memory without valgrind
 */
static void fill_stack_nan(void)
{
    float stk[2048];
    fill_nanf(stk, ARRAY_SIZE(stk));
}

uint8_t HAL_SITL::get_instance() const
{
    return _sitl_state->get_instance();
}

#if defined(HAL_BUILD_AP_PERIPH)
bool HAL_SITL::run_in_maintenance_mode() const
{
    return _sitl_state->run_in_maintenance_mode();
}
#endif

void HAL_SITL::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    utilInstance.init(argc, argv);
    _sitl_state->init(argc, argv);

    scheduler->init();
    serial(0)->begin(115200);

    rcin->init();
    rcout->init();

    // spi->init();
    analogin->init();

    if (getenv("SITL_WATCHDOG_RESET")) {
        INTERNAL_ERROR(AP_InternalError::error_t::watchdog_reset);
        if (watchdog_load((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4)) {
            serial(0)->printf("Loaded watchdog data");
            utilInstance.last_persistent_data = utilInstance.persistent_data;
        }
    }

    // form a new argv, removing problem parameters. This is used for reboot
    uint8_t new_argv_offset = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(new_argv) && i<argc; i++) {
        if (!strcmp(argv[i], "-w")) {
            // don't wipe params on reboot
            continue;
        }
        new_argv[new_argv_offset++] = argv[i];
    }

    fill_stack_nan();

    callbacks->setup();
    scheduler->set_system_initialized();

#if HAL_LOGGING_ENABLED
    if (getenv("SITL_WATCHDOG_RESET")) {
        const AP_HAL::Util::PersistentData &pd = util->persistent_data;
        AP::logger().WriteCritical("WDOG", "TimeUS,Task,IErr,IErrCnt,IErrLn,MavMsg,MavCmd,SemLine", "QbIHHHHH",
                                   AP_HAL::micros64(),
                                   pd.scheduler_task,
                                   pd.internal_errors,
                                   pd.internal_error_count,
                                   pd.internal_error_last_line,
                                   pd.last_mavlink_msgid,
                                   pd.last_mavlink_cmd,
                                   pd.semaphore_line);
    }
#endif

    bool using_watchdog = AP_BoardConfig::watchdog_enabled();
    if (using_watchdog) {
        signal(SIGALRM, sig_alrm);
        alarm(2);
    }
    setup_signal_handlers();

    uint32_t last_watchdog_save = AP_HAL::millis();
    uint8_t fill_count = 0;

    while (!HALSITL::Scheduler::_should_reboot) {
        if (HALSITL::Scheduler::_should_exit) {
            ::fprintf(stderr, "Exitting\n");
            exit(0);
        }
        if (fill_count++ % 10 == 0) {
            // only fill every 10 loops. This still gives us a lot of
            // protection, but saves a lot of CPU
            fill_stack_nan();
        }
        callbacks->loop();
        HALSITL::Scheduler::_run_io_procs();

        uint32_t now = AP_HAL::millis();
        if (now - last_watchdog_save >= 100 && using_watchdog) {
            // save persistent data every 100ms
            last_watchdog_save = now;
            watchdog_save((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4);
        }

        if (using_watchdog) {
            // note that this only works for a speedup of 1
            alarm(2);
        }
    }

    actually_reboot();
}

void HAL_SITL::actually_reboot()
{
    execv(new_argv[0], new_argv);
    AP_HAL::panic("PANIC: REBOOT FAILED: %s", strerror(errno));
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_SITL hal;
    return hal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
