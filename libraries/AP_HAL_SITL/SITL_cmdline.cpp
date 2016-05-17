/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <AP_HAL/utility/getopt_cpp.h>

#include <SITL/SIM_Multicopter.h>
#include <SITL/SIM_Helicopter.h>
#include <SITL/SIM_Plane.h>
#include <SITL/SIM_QuadPlane.h>
#include <SITL/SIM_Rover.h>
#include <SITL/SIM_CRRCSim.h>
#include <SITL/SIM_Gazebo.h>
#include <SITL/SIM_last_letter.h>
#include <SITL/SIM_JSBSim.h>
#include <SITL/SIM_Tracker.h>
#include <SITL/SIM_Balloon.h>
#include <SITL/SIM_FlightAxis.h>
#include <SITL/SIM_Calibration.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;
using namespace SITL;

// catch floating point exceptions
static void _sig_fpe(int signum)
{
    fprintf(stderr, "ERROR: Floating point exception - aborting\n");
    abort();
}

void SITL_State::_usage(void)
{
    printf("Options:\n"
           "\t--home HOME        set home location (lat,lng,alt,yaw)\n"
           "\t--model MODEL      set simulation model\n"
           "\t--wipe             wipe eeprom and dataflash\n"
           "\t--rate RATE        set SITL framerate\n"
           "\t--console          use console instead of TCP ports\n"
           "\t--instance N       set instance of SITL (adds 10*instance to all port numbers)\n"
           "\t--speedup SPEEDUP  set simulation speedup\n"
           "\t--gimbal           enable simulated MAVLink gimbal\n"
           "\t--adsb             enable simulated ADSB peripheral\n"
           "\t--autotest-dir DIR set directory for additional files\n"
           "\t--uartA device     set device string for UARTA\n"
           "\t--uartB device     set device string for UARTB\n"
           "\t--uartC device     set device string for UARTC\n"
           "\t--uartD device     set device string for UARTD\n"
           "\t--uartE device     set device string for UARTE\n"
           "\t--defaults path    set path to defaults file\n"
        );
}

static const struct {
    const char *name;
    Aircraft *(*constructor)(const char *home_str, const char *frame_str);
} model_constructors[] = {
    { "quadplane",          QuadPlane::create },
    { "firefly",            QuadPlane::create },
    { "+",                  MultiCopter::create },
    { "quad",               MultiCopter::create },
    { "copter",             MultiCopter::create },
    { "x",                  MultiCopter::create },
    { "hexa",               MultiCopter::create },
    { "octa",               MultiCopter::create },
    { "tri",                MultiCopter::create },
    { "y6",                 MultiCopter::create },
    { "heli",               Helicopter::create },
    { "heli-dual",          Helicopter::create },
    { "heli-compound",      Helicopter::create },
    { "rover",              SimRover::create },
    { "crrcsim",            CRRCSim::create },
    { "jsbsim",             JSBSim::create },
    { "flightaxis",         FlightAxis::create },
    { "gazebo",             Gazebo::create },
    { "last_letter",        last_letter::create },
    { "tracker",            Tracker::create },
    { "balloon",            Balloon::create },
    { "plane",              Plane::create },
    { "calibration",        Calibration::create },
};

void SITL_State::_parse_command_line(int argc, char * const argv[])
{
    int opt;
    // default to CMAC
    const char *home_str = "-35.363261,149.165230,584,353";
    const char *model_str = NULL;
    char *autotest_dir = NULL;
    float speedup = 1.0f;

    if (asprintf(&autotest_dir, SKETCHBOOK "/Tools/autotest") <= 0) {
        AP_HAL::panic("out of memory");
    }

    signal(SIGFPE, _sig_fpe);
    // No-op SIGPIPE handler
    signal(SIGPIPE, SIG_IGN);

    setvbuf(stdout, (char *)0, _IONBF, 0);
    setvbuf(stderr, (char *)0, _IONBF, 0);

    _synthetic_clock_mode = false;
    _base_port = 5760;
    _rcout_port = 5502;
    _simin_port = 5501;
    _fdm_address = "127.0.0.1";
    _client_address = NULL;
    _instance = 0;

    enum long_options {
        CMDLINE_CLIENT=0,
        CMDLINE_GIMBAL,
        CMDLINE_AUTOTESTDIR,
        CMDLINE_UARTA,
        CMDLINE_UARTB,
        CMDLINE_UARTC,
        CMDLINE_UARTD,
        CMDLINE_UARTE,
        CMDLINE_UARTF,
        CMDLINE_ADSB,
        CMDLINE_RTSCTS,
        CMDLINE_DEFAULTS
    };

    const struct GetOptLong::option options[] = {
        {"help",            false,  0, 'h'},
        {"wipe",            false,  0, 'w'},
        {"speedup",         true,   0, 's'},
        {"rate",            true,   0, 'r'},
        {"console",         false,  0, 'C'},
        {"instance",        true,   0, 'I'},
        {"param",           true,   0, 'P'},
        {"synthetic-clock", false,  0, 'S'},
        {"home",            true,   0, 'O'},
        {"model",           true,   0, 'M'},
        {"uartA",           true,   0, CMDLINE_UARTA},
        {"uartB",           true,   0, CMDLINE_UARTB},
        {"uartC",           true,   0, CMDLINE_UARTC},
        {"uartD",           true,   0, CMDLINE_UARTD},
        {"uartE",           true,   0, CMDLINE_UARTE},
        {"client",          true,   0, CMDLINE_CLIENT},
        {"gimbal",          false,  0, CMDLINE_GIMBAL},
        {"adsb",            false,  0, CMDLINE_ADSB},
        {"autotest-dir",    true,   0, CMDLINE_AUTOTESTDIR},
        {"defaults",        true,   0, CMDLINE_DEFAULTS},
        {"rtscts",          false,  0, CMDLINE_RTSCTS},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "hws:r:CI:P:SO:M:F:",
                    options);

    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'w':
            AP_Param::erase_all();
            unlink("dataflash.bin");
            break;
        case 'r':
            _framerate = (unsigned)atoi(gopt.optarg);
            break;
        case 'C':
            HALSITL::UARTDriver::_console = true;
            break;
        case 'I': {
            _instance = atoi(gopt.optarg);
            _base_port  += _instance * 10;
            _rcout_port += _instance * 10;
            _simin_port += _instance * 10;
        }
        break;
        case 'P':
            _set_param_default(gopt.optarg);
            break;
        case 'S':
            _synthetic_clock_mode = true;
            break;
        case 'O':
            home_str = gopt.optarg;
            break;
        case 'M':
            model_str = gopt.optarg;
            break;
        case 's':
            speedup = strtof(gopt.optarg, NULL);
            break;
        case 'F':
            _fdm_address = gopt.optarg;
            break;
        case CMDLINE_CLIENT:
            _client_address = gopt.optarg;
            break;
        case CMDLINE_GIMBAL:
            enable_gimbal = true;
            break;
        case CMDLINE_ADSB:
            enable_ADSB = true;
            break;
        case CMDLINE_RTSCTS:
            _use_rtscts = true;
            break;
        case CMDLINE_AUTOTESTDIR:
            autotest_dir = strdup(gopt.optarg);
            break;
        case CMDLINE_DEFAULTS:
            defaults_path = strdup(gopt.optarg);
            break;

        case CMDLINE_UARTA:
        case CMDLINE_UARTB:
        case CMDLINE_UARTC:
        case CMDLINE_UARTD:
        case CMDLINE_UARTE:
        case CMDLINE_UARTF:
            _uart_path[opt - CMDLINE_UARTA] = gopt.optarg;
            break;
            
        default:
            _usage();
            exit(1);
        }
    }

    if (!model_str) {
        printf("You must specify a vehicle model\n");
        exit(1);
    }

    for (uint8_t i=0; i < ARRAY_SIZE(model_constructors); i++) {
        if (strncasecmp(model_constructors[i].name, model_str, strlen(model_constructors[i].name)) == 0) {
            sitl_model = model_constructors[i].constructor(home_str, model_str);
            sitl_model->set_speedup(speedup);
            sitl_model->set_instance(_instance);
            sitl_model->set_autotest_dir(autotest_dir);
            _synthetic_clock_mode = true;
            printf("Started model %s at %s at speed %.1f\n", model_str, home_str, speedup);
            break;
        }
    }
    if (sitl_model == nullptr) {
        printf("Vehicle model (%s) not found\n", model_str);
        exit(1);
    }

    fprintf(stdout, "Starting sketch '%s'\n", SKETCH);

    if (strcmp(SKETCH, "ArduCopter") == 0) {
        _vehicle = ArduCopter;
        if (_framerate == 0) {
            _framerate = 200;
        }
    } else if (strcmp(SKETCH, "APMrover2") == 0) {
        _vehicle = APMrover2;
        if (_framerate == 0) {
            _framerate = 50;
        }
        // set right default throttle for rover (allowing for reverse)
        pwm_input[2] = 1500;
    } else {
        _vehicle = ArduPlane;
        if (_framerate == 0) {
            _framerate = 50;
        }
    }

    _sitl_setup(home_str);
}

#endif
