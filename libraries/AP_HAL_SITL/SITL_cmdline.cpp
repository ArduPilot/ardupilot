#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include <AP_HAL/utility/getopt_cpp.h>
#include <AP_HAL_SITL/Storage.h>
#include <AP_Param/AP_Param.h>

#include <SITL/SIM_Multicopter.h>
#include <SITL/SIM_Helicopter.h>
#include <SITL/SIM_SingleCopter.h>
#include <SITL/SIM_Plane.h>
#include <SITL/SIM_Glider.h>
#include <SITL/SIM_QuadPlane.h>
#include <SITL/SIM_Rover.h>
#include <SITL/SIM_BalanceBot.h>
#include <SITL/SIM_Sailboat.h>
#include <SITL/SIM_MotorBoat.h>
#include <SITL/SIM_CRRCSim.h>
#include <SITL/SIM_Gazebo.h>
#include <SITL/SIM_last_letter.h>
#include <SITL/SIM_JSBSim.h>
#include <SITL/SIM_Tracker.h>
#include <SITL/SIM_Balloon.h>
#include <SITL/SIM_FlightAxis.h>
#include <SITL/SIM_Calibration.h>
#include <SITL/SIM_XPlane.h>
#include <SITL/SIM_Submarine.h>
#include <SITL/SIM_SilentWings.h>
#include <SITL/SIM_Morse.h>
#include <SITL/SIM_AirSim.h>
#include <SITL/SIM_Scrimmage.h>
#include <SITL/SIM_Webots.h>
#include <SITL/SIM_Webots_Python.h>
#include <SITL/SIM_JSON.h>
#include <SITL/SIM_Blimp.h>
#include <SITL/SIM_NoVehicle.h>
#include <SITL/SIM_StratoBlimp.h>

#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"

extern HAL_SITL& hal;

using namespace HALSITL;
using namespace SITL;

// catch floating point exceptions
static void _sig_fpe(int signum)
{
    fprintf(stderr, "ERROR: Floating point exception - aborting\n");
    AP_HAL::dump_stack_trace();
    AP_HAL::dump_core_file();
    abort();
}

// catch segfault
static void _sig_segv(int signum)
{
    fprintf(stderr, "ERROR: segmentation fault - aborting\n");
    AP_HAL::dump_stack_trace();
    AP_HAL::dump_core_file();
    abort();
}

void SITL_State::_usage(void)
{
    printf("Options:\n"
           "\t--help|-h                display this help information\n"
           "\t--wipe|-w                wipe eeprom\n"
           "\t--unhide-groups|-u       parameter enumeration ignores AP_PARAM_FLAG_ENABLE\n"
           "\t--speedup|-s SPEEDUP     set simulation speedup\n"
           "\t--rate|-r RATE           set SITL framerate\n"
           "\t--console|-C             use console instead of TCP ports\n"
           "\t--instance|-I N          set instance of SITL (adds 10*instance to all port numbers)\n"
           // "\t--param|-P NAME=VALUE    set some param\n"  CURRENTLY BROKEN!
           "\t--synthetic-clock|-S     set synthetic clock mode\n"
           "\t--home|-O HOME           set start location (lat,lng,alt,yaw) or location name\n"
           "\t--model|-M MODEL         set simulation model\n"
           "\t--config string          set additional simulation config string\n"
           "\t--fg|-F ADDRESS          set Flight Gear view address, defaults to 127.0.0.1\n"
           "\t--enable-fgview          enable Flight Gear view\n"
           "\t--gimbal                 enable simulated MAVLink gimbal\n"
           "\t--autotest-dir DIR       set directory for additional files\n"
           "\t--defaults path          set path to defaults file\n"
           "\t--serial0 device         set device string for SERIAL0\n"
           "\t--serial1 device         set device string for SERIAL1\n"
           "\t--serial2 device         set device string for SERIAL2\n"
           "\t--serial3 device         set device string for SERIAL3\n"
           "\t--serial4 device         set device string for SERIAL4\n"
           "\t--serial5 device         set device string for SERIAL5\n"
           "\t--serial6 device         set device string for SERIAL6\n"
           "\t--serial7 device         set device string for SERIAL7\n"
           "\t--serial8 device         set device string for SERIAL8\n"
           "\t--serial9 device         set device string for SERIAL9\n"
           "\t--uartA device           alias for --serial0 (do not use)\n"
           "\t--rtscts                 enable rtscts on serial ports (default false)\n"
           "\t--base-port PORT         set port num for base port(default 5670) must be before -I option\n"
           "\t--rc-in-port PORT        set port num for rc in\n"
           "\t--sim-address ADDR       set address string for simulator\n"
           "\t--sim-port-in PORT       set port num for simulator in\n"
           "\t--sim-port-out PORT      set port num for simulator out\n"
           "\t--irlock-port PORT       set port num for irlock\n"
           "\t--start-time TIMESTR     set simulation start time in UNIX timestamp\n"
           "\t--sysid ID               set MAV_SYSID\n"
           "\t--slave number           set the number of JSON slaves\n"
        );
}

static const struct {
    const char *name;
    Aircraft *(*constructor)(const char *frame_str);
} model_constructors[] = {
    { "quadplane",          QuadPlane::create },
#if AP_SIM_XPLANE_ENABLED
    { "xplane",             XPlane::create },
#endif  // AP_SIM_XPLANE_ENABLED
    { "firefly",            QuadPlane::create },
    { "+",                  MultiCopter::create },
    { "quad",               MultiCopter::create },
    { "copter",             MultiCopter::create },
    { "x",                  MultiCopter::create },
    { "bfxrev",             MultiCopter::create },
    { "bfx",                MultiCopter::create },
    { "djix",               MultiCopter::create },
    { "cwx",                MultiCopter::create },
    { "hexa",               MultiCopter::create },
    { "hexax",              MultiCopter::create },
    { "hexa-cwx",           MultiCopter::create },
    { "hexa-dji",           MultiCopter::create },
    { "octa",               MultiCopter::create },
    { "octa-cwx",           MultiCopter::create },
    { "octa-dji",           MultiCopter::create },
    { "octa-quad-cwx",      MultiCopter::create },
    { "dotriaconta_octaquad_x", MultiCopter::create },
    { "dodeca-hexa",        MultiCopter::create },
    { "hexadeca-octa",      MultiCopter::create },
    { "hexadeca-octa-cwx",  MultiCopter::create },
    { "tri",                MultiCopter::create },
    { "y6",                 MultiCopter::create },
    { "deca",               MultiCopter::create },
    { "deca-cwx",           MultiCopter::create },
    { "heli",               Helicopter::create },
    { "heli-dual",          Helicopter::create },
    { "heli-compound",      Helicopter::create },
    { "heli-blade360",         Helicopter::create },
    { "singlecopter",       SingleCopter::create },
    { "coaxcopter",         SingleCopter::create },
    { "rover",              SimRover::create },
    { "balancebot",         BalanceBot::create },
    { "sailboat",           Sailboat::create },
    { "motorboat",          MotorBoat::create },
#if AP_SIM_CRRCSIM_ENABLED
    { "crrcsim",            CRRCSim::create },
#endif  // AP_SIM_CRRCSIM_ENABLED
#if AP_SIM_JSBSIM_ENABLED
    { "jsbsim",             JSBSim::create },
#endif  // AP_SIM_JSBSIM_ENABLED
#if AP_SIM_FLIGHTAXIS_ENABLED
    { "flightaxis",         FlightAxis::create },
#endif  // AP_SIM_FLIGHTAXIS_ENABLED
#if AP_SIM_GAZEBO_ENABLED
    { "gazebo",             Gazebo::create },
#endif  // AP_SIM_GAZEBO_ENABLED
#if AP_SIM_LAST_LETTER_ENABLED
    { "last_letter",        last_letter::create },
#endif  // AP_SIM_LAST_LETTER_ENABLED
    { "tracker",            Tracker::create },
    { "balloon",            Balloon::create },
    { "glider",             Glider::create },
    { "plane",              Plane::create },
    { "calibration",        Calibration::create },
    { "vectored",           Submarine::create },
    { "vectored_6dof",      Submarine::create },
#if AP_SIM_SILENTWINGS_ENABLED
    { "silentwings",        SilentWings::create },
#endif  // AP_SIM_SILENTWINGS_ENABLED
#if AP_SIM_MORSE_ENABLED
    { "morse",              Morse::create },
#endif  // AP_SIM_MORSE_ENABLED
#if AP_SIM_AIRSIM_ENABLED
    { "airsim",             AirSim::create},
#endif  // AP_SIM_AIRSIM_ENABLED
#if AP_SIM_SCRIMMAGE_ENABLED
    { "scrimmage",          Scrimmage::create },
#endif  // AP_SIM_SCRIMMAGE_ENABLED
    { "webots-python",      WebotsPython::create },
#if AP_SIM_WEBOTS_ENABLED
    { "webots",             Webots::create },
#endif  // AP_SIM_WEBOTS_ENABLED
#if AP_SIM_JSON_ENABLED
    { "JSON",               JSON::create },
#endif  // AP_SIM_JSON_ENABLED
    { "blimp",              Blimp::create },
    { "novehicle",          NoVehicle::create },
#if AP_SIM_STRATOBLIMP_ENABLED
    { "stratoblimp",        StratoBlimp::create },
#endif
};

void SITL_State::_set_signal_handlers(void) const
{
    struct sigaction sa_fpe = {};
    sigemptyset(&sa_fpe.sa_mask);
    sa_fpe.sa_handler = _sig_fpe;
    sigaction(SIGFPE, &sa_fpe, nullptr);

    struct sigaction sa_pipe = {};
    sigemptyset(&sa_pipe.sa_mask);
    sa_pipe.sa_handler = SIG_IGN; /* No-op SIGPIPE handler */
    sigaction(SIGPIPE, &sa_pipe, nullptr);

    struct sigaction sa_segv = {};
    sigemptyset(&sa_segv.sa_mask);
    sa_segv.sa_handler = _sig_segv;
    sigaction(SIGSEGV, &sa_segv, nullptr);

}

void SITL_State::_parse_command_line(int argc, char * const argv[])
{
    int opt;
    float speedup = 1.0f;
    float sim_rate_hz = 0;
    _instance = 0;
    // default to CMAC
    const char *home_str = nullptr;
    const char *model_str = nullptr;
    const char *vehicle_str = AP_BUILD_TARGET_NAME;
    _use_fg_view = false;
    char *autotest_dir = nullptr;
    _fg_address = "127.0.0.1";
    const char* config = "";

    const int BASE_PORT = 5760;
    const int RCIN_PORT = 5501;
    const int FG_VIEW_PORT = 5503;
    _base_port = BASE_PORT;
    _rcin_port = RCIN_PORT;
    _fg_view_port = FG_VIEW_PORT;

    const int SIM_IN_PORT = 9003;
    const int SIM_OUT_PORT = 9002;
    const int IRLOCK_PORT = 9005;
    const char * simulator_address = "127.0.0.1";
    uint16_t simulator_port_in = SIM_IN_PORT;
    uint16_t simulator_port_out = SIM_OUT_PORT;
    _irlock_port = IRLOCK_PORT;
    struct AP_Param::defaults_table_struct temp_cmdline_param{};

    // Set default start time to the real system time.
    // This will be overwritten if argument provided.
    static struct timeval first_tv;
    gettimeofday(&first_tv, nullptr);
    time_t start_time_UTC = first_tv.tv_sec;
    const bool is_example = APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN);

    enum long_options {
        CMDLINE_GIMBAL = 1,
        CMDLINE_FGVIEW,
        CMDLINE_AUTOTESTDIR,
        CMDLINE_DEFAULTS,
        CMDLINE_UARTA,  // must be in A-J order and numbered consecutively
        CMDLINE_UARTB,
        CMDLINE_UARTC,
        CMDLINE_UARTD,
        CMDLINE_UARTE,
        CMDLINE_UARTF,
        CMDLINE_UARTG,
        CMDLINE_UARTH,
        CMDLINE_UARTI,
        CMDLINE_UARTJ,
        CMDLINE_SERIAL0, // must be in 0-9 order and numbered consecutively
        CMDLINE_SERIAL1,
        CMDLINE_SERIAL2,
        CMDLINE_SERIAL3,
        CMDLINE_SERIAL4,
        CMDLINE_SERIAL5,
        CMDLINE_SERIAL6,
        CMDLINE_SERIAL7,
        CMDLINE_SERIAL8,
        CMDLINE_SERIAL9,
        CMDLINE_RTSCTS,
        CMDLINE_BASE_PORT,
        CMDLINE_RCIN_PORT,
        CMDLINE_SIM_ADDRESS,
        CMDLINE_SIM_PORT_IN,
        CMDLINE_SIM_PORT_OUT,
        CMDLINE_IRLOCK_PORT,
        CMDLINE_START_TIME,
        CMDLINE_SYSID,
        CMDLINE_SLAVE,
#if STORAGE_USE_FLASH
        CMDLINE_SET_STORAGE_FLASH_ENABLED,
#endif
#if STORAGE_USE_POSIX
        CMDLINE_SET_STORAGE_POSIX_ENABLED,
#endif
#if STORAGE_USE_FRAM
        CMDLINE_SET_STORAGE_FRAM_ENABLED,
#endif
    };

    const struct GetOptLong::option options[] = {
        {"help",            false,  0, 'h'},
        {"wipe",            false,  0, 'w'},
        {"unhide-groups",   false,  0, 'u'},
        {"speedup",         true,   0, 's'},
        {"rate",            true,   0, 'r'},
        {"console",         false,  0, 'C'},
        {"instance",        true,   0, 'I'},
        {"param",           true,   0, 'P'},
        {"synthetic-clock", false,  0, 'S'},
        {"home",            true,   0, 'O'},
        {"model",           true,   0, 'M'},
        {"config",          true,   0, 'c'},
        {"fg",              true,   0, 'F'},
        {"gimbal",          false,  0, CMDLINE_GIMBAL},
        {"enable-fgview",   false,  0, CMDLINE_FGVIEW},
        {"autotest-dir",    true,   0, CMDLINE_AUTOTESTDIR},
        {"defaults",        true,   0, CMDLINE_DEFAULTS},
        // {"uartA",           true,   0, CMDLINE_UARTA}, // alias for serial0
        {"uartB",           true,   0, CMDLINE_UARTB},
        {"uartC",           true,   0, CMDLINE_UARTC},
        {"uartD",           true,   0, CMDLINE_UARTD},
        {"uartE",           true,   0, CMDLINE_UARTE},
        {"uartF",           true,   0, CMDLINE_UARTF},
        {"uartG",           true,   0, CMDLINE_UARTG},
        {"uartH",           true,   0, CMDLINE_UARTH},
        {"uartI",           true,   0, CMDLINE_UARTI},
        {"uartJ",           true,   0, CMDLINE_UARTJ},
        {"serial0",         true,   0, CMDLINE_SERIAL0},
        {"uartA",           true,   0, CMDLINE_SERIAL0}, // for MissionPlanner compatibility
        {"serial1",         true,   0, CMDLINE_SERIAL1},
        {"serial2",         true,   0, CMDLINE_SERIAL2},
        {"serial3",         true,   0, CMDLINE_SERIAL3},
        {"serial4",         true,   0, CMDLINE_SERIAL4},
        {"serial5",         true,   0, CMDLINE_SERIAL5},
        {"serial6",         true,   0, CMDLINE_SERIAL6},
        {"serial7",         true,   0, CMDLINE_SERIAL7},
        {"serial8",         true,   0, CMDLINE_SERIAL8},
        {"serial9",         true,   0, CMDLINE_SERIAL9},
        {"rtscts",          false,  0, CMDLINE_RTSCTS},
        {"base-port",       true,   0, CMDLINE_BASE_PORT},
        {"rc-in-port",      true,   0, CMDLINE_RCIN_PORT},
        {"sim-address",     true,   0, CMDLINE_SIM_ADDRESS},
        {"sim-port-in",     true,   0, CMDLINE_SIM_PORT_IN},
        {"sim-port-out",    true,   0, CMDLINE_SIM_PORT_OUT},
        {"irlock-port",     true,   0, CMDLINE_IRLOCK_PORT},
        {"start-time",      true,   0, CMDLINE_START_TIME},
        {"sysid",           true,   0, CMDLINE_SYSID},
        {"slave",           true,   0, CMDLINE_SLAVE},
#if STORAGE_USE_FLASH
        {"set-storage-flash-enabled", true,   0, CMDLINE_SET_STORAGE_FLASH_ENABLED},
#endif
#if STORAGE_USE_POSIX
        {"set-storage-posix-enabled", true,   0, CMDLINE_SET_STORAGE_POSIX_ENABLED},
#endif
#if STORAGE_USE_FRAM
        {"set-storage-fram-enabled", true,   0, CMDLINE_SET_STORAGE_FRAM_ENABLED},
#endif
        {"vehicle",           true,   0, 'v'},
        {0, false, 0, 0}
    };

    if (is_example) {
        model_str = "novehicle";
        HALSITL::UARTDriver::_console = true;
    }

    // storage defaults are set here:
    bool storage_posix_enabled = true;
    bool storage_flash_enabled = false;
    bool storage_fram_enabled = false;
    bool erase_all_storage = false;

    if (asprintf(&autotest_dir, AP_BUILD_ROOT "/Tools/autotest") <= 0) {
        AP_HAL::panic("out of memory");
    }
    _set_signal_handlers();

    setvbuf(stdout, (char *)0, _IONBF, 0);
    setvbuf(stderr, (char *)0, _IONBF, 0);

    bool wiping_storage = false;

    GetOptLong gopt(argc, argv, "hwus:r:CI:P:SO:M:F:c:v:",
                    options);
    while (!is_example && (opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'w':
            erase_all_storage = true;
            break;
        case 'u':
            AP_Param::set_hide_disabled_groups(false);
            break;
        case 's':
            speedup = strtof(gopt.optarg, nullptr);
            temp_cmdline_param = {"SIM_SPEEDUP", speedup};
            cmdline_param.push(temp_cmdline_param);
            printf("Setting SIM_SPEEDUP=%f\n", speedup);
            break;
        case 'r':
            sim_rate_hz = strtof(gopt.optarg, nullptr);
            temp_cmdline_param = {"SIM_RATE_HZ", sim_rate_hz};
            cmdline_param.push(temp_cmdline_param);
            printf("Setting SIM_RATE_HZ=%f\n", sim_rate_hz);
            break;
        case 'C':
            HALSITL::UARTDriver::_console = true;
            break;
        case 'I': {
            _instance = atoi(gopt.optarg);
            if (_base_port == BASE_PORT) {
                _base_port += _instance * 10;
            }
            if (_rcin_port == RCIN_PORT) {
                _rcin_port += _instance * 10;
            }
            if (_fg_view_port == FG_VIEW_PORT) {
                _fg_view_port += _instance * 10;
            }
            if (simulator_port_in == SIM_IN_PORT) {
                simulator_port_in += _instance * 10;
            }
            if (simulator_port_out == SIM_OUT_PORT) {
                simulator_port_out += _instance * 10;
            }
            if (_irlock_port == IRLOCK_PORT) {
                _irlock_port += _instance * 10;
            }
        }
        break;
        case 'P':
            _set_param_default(gopt.optarg);
            break;
        case 'S':
            printf("Ignoring stale command-line parameter '-S'");
            break;
        case 'O':
            home_str = gopt.optarg;
            break;
        case 'M':
            model_str = gopt.optarg;
            break;
        case 'c':
            config = gopt.optarg;
            break;
        case 'F':
            _fg_address = gopt.optarg;
            break;
        case 'v':
            vehicle_str = gopt.optarg;
            break;
#if AP_SIM_SOLOGIMBAL_ENABLED
        case CMDLINE_GIMBAL:
            enable_gimbal = true;
            break;
#endif
        case CMDLINE_FGVIEW:
            _use_fg_view = true;
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
        case CMDLINE_UARTG:
        case CMDLINE_UARTH:
        case CMDLINE_UARTI:
        case CMDLINE_UARTJ: {
            int uart_idx = opt - CMDLINE_UARTA;
            // ordering captures the historical use of uartB as SERIAL3
            static const uint8_t mapping[] = { 0, 3, 1, 2, 4, 5, 6, 7, 8, 9 };
            int serial_idx = mapping[uart_idx];
            char uart_letter = (char)(uart_idx)+'A';
            printf("ERROR: Removed option --uart%c supplied. "
                "Use --serial%d instead.\n", uart_letter, serial_idx);
            exit(1);
        }
        case CMDLINE_SERIAL0:
        case CMDLINE_SERIAL1:
        case CMDLINE_SERIAL2:
        case CMDLINE_SERIAL3:
        case CMDLINE_SERIAL4:
        case CMDLINE_SERIAL5:
        case CMDLINE_SERIAL6:
        case CMDLINE_SERIAL7:
        case CMDLINE_SERIAL8:
        case CMDLINE_SERIAL9:
            _serial_path[opt - CMDLINE_SERIAL0] = gopt.optarg;
            break;
        case CMDLINE_RTSCTS:
            _use_rtscts = true;
            break;
        case CMDLINE_BASE_PORT:
            _base_port = atoi(gopt.optarg);
            break;
        case CMDLINE_RCIN_PORT:
            _rcin_port = atoi(gopt.optarg);
            break;
        case CMDLINE_SIM_ADDRESS:
            simulator_address = gopt.optarg;
            break;
        case CMDLINE_SIM_PORT_IN:
            simulator_port_in = atoi(gopt.optarg);
            break;
        case CMDLINE_SIM_PORT_OUT:
            simulator_port_out = atoi(gopt.optarg);
            break;
        case CMDLINE_IRLOCK_PORT:
            _irlock_port = atoi(gopt.optarg);
            break;
        case CMDLINE_START_TIME:
            start_time_UTC = atoi(gopt.optarg);
            break;
        case CMDLINE_SYSID: {
            const int32_t sysid = atoi(gopt.optarg);
            if (sysid < 1 || sysid > 255) {
                fprintf(stderr, "You must specify a SYSID greater than 0 and less than 256\n");
                exit(1);
            }
            temp_cmdline_param = {"MAV_SYSID", static_cast<float>(sysid)};
            cmdline_param.push(temp_cmdline_param);
            printf("Setting MAV_SYSID=%d\n", sysid);
            break;
        }
#if STORAGE_USE_POSIX
        case CMDLINE_SET_STORAGE_POSIX_ENABLED:
            storage_posix_enabled = atoi(gopt.optarg);
            break;
#endif
#if STORAGE_USE_FLASH
        case CMDLINE_SET_STORAGE_FLASH_ENABLED:
            storage_flash_enabled = atoi(gopt.optarg);
            break;
#endif
#if STORAGE_USE_FRAM
        case CMDLINE_SET_STORAGE_FRAM_ENABLED:
            storage_fram_enabled = atoi(gopt.optarg);
            break;
#endif
        case 'h':
            _usage();
            exit(0);
        case CMDLINE_SLAVE: {
#if AP_SIM_JSON_MASTER_ENABLED
            const int32_t slaves = atoi(gopt.optarg);
            if (slaves > 0) {
                ride_along.init(slaves);
            }
#endif  // AP_SIM_JSON_MASTER_ENABLED
            break;
        }
        default:
            _usage();
            exit(1);
        }
    }

    if (!model_str) {
        printf("You must specify a vehicle model.  Options are:\n");
        for (uint8_t i=0; i < ARRAY_SIZE(model_constructors); i++) {
            printf("  %s\n", model_constructors[i].name);
        }
        // spit this out again as the original message probably just
        // scrolled off the screen:
        printf("You must specify a vehicle model.\n");
        exit(1);
    }

    if (AP::sitl() != nullptr) {  // some examples don't instantiate this object
        AP::sitl()->init();
    }

    for (uint8_t i=0; i < ARRAY_SIZE(model_constructors); i++) {
        if (strncasecmp(model_constructors[i].name, model_str, strlen(model_constructors[i].name)) == 0) {
            // printf("Creating model %f,%f,%f,%f at speed %.1f\n", opos.lat, opos.lng, opos.alt, opos.hdg, speedup);
            sitl_model = model_constructors[i].constructor(model_str);
            if (home_str != nullptr) {
                Location home;
                float home_yaw;
                if (strchr(home_str,',') == nullptr) {
                    if (!lookup_location(home_str, home, home_yaw)) {
                        ::printf("Failed to find location (%s).  Should be in locations.txt or LAT,LON,ALT,HDG e.g. 37.4003371,-122.0800351,0,353\n", home_str);
                        exit(1);
                    }
                } else if (!parse_home(home_str, home, home_yaw)) {
                    ::printf("Failed to parse home string (%s).  Should be LAT,LON,ALT,HDG e.g. 37.4003371,-122.0800351,0,353\n", home_str);
                    exit(1);
                }
                sitl_model->set_start_location(home, home_yaw);
            }
            sitl_model->set_interface_ports(simulator_address, simulator_port_in, simulator_port_out);
            sitl_model->set_speedup(speedup);
            sitl_model->set_instance(_instance);
            sitl_model->set_autotest_dir(autotest_dir);
            sitl_model->set_config(config);
            break;
        }
    }
    if (sitl_model == nullptr) {
        printf("Vehicle model (%s) not found\n", model_str);
        exit(1);
    }

    if (storage_posix_enabled && storage_flash_enabled) {
        // this will change in the future!
        printf("Only one of flash or posix storage may be selected");
        exit(1);
    }

    if (AP::sitl()) {
        // Set SITL start time.
        AP::sitl()->start_time_UTC = start_time_UTC;
    }

    hal.set_storage_posix_enabled(storage_posix_enabled);
    hal.set_storage_flash_enabled(storage_flash_enabled);
    hal.set_storage_fram_enabled(storage_fram_enabled);

    if (erase_all_storage) {
        AP_Param::erase_all();
        unlink("flash.dat");
        hal.set_wipe_storage(wiping_storage);
    }

    fprintf(stdout, "Starting sketch '%s'\n", vehicle_str);

    if (strcmp(vehicle_str, "ArduCopter") == 0) {
        _vehicle = ArduCopter;
    } else if (strcmp(vehicle_str, "Rover") == 0) {
        _vehicle = Rover;
    } else if (strcmp(vehicle_str, "ArduSub") == 0) {
        _vehicle = ArduSub;
    } else if (strcmp(vehicle_str, "Blimp") == 0) {
        _vehicle = Blimp;
    } else {
        _vehicle = ArduPlane;
    }

    _sitl_setup();
}

/*
  parse a home string into a location and yaw
 */
bool SITL_State::parse_home(const char *home_str, Location &loc, float &yaw_degrees)
{
    char *saveptr = nullptr;
    char *s = strdup(home_str);
    if (!s) {
        free(s);
        ::printf("No home string supplied\n");
        return false;
    }
    char *lat_s = strtok_r(s, ",", &saveptr);
    if (!lat_s) {
        free(s);
        ::printf("Failed to parse latitude\n");
        return false;
    }
    char *lon_s = strtok_r(nullptr, ",", &saveptr);
    if (!lon_s) {
        free(s);
        ::printf("Failed to parse longitude\n");
        return false;
    }
    char *alt_s = strtok_r(nullptr, ",", &saveptr);
    if (!alt_s) {
        free(s);
        ::printf("Failed to parse altitude\n");
        return false;
    }
    char *yaw_s = strtok_r(nullptr, ",", &saveptr);
    if (!yaw_s) {
        free(s);
        ::printf("Failed to parse yaw\n");
        return false;
    }

    loc = {};
    loc.lat = static_cast<int32_t>(strtod(lat_s, nullptr) * 1.0e7);
    loc.lng = static_cast<int32_t>(strtod(lon_s, nullptr) * 1.0e7);
    loc.alt = static_cast<int32_t>(strtod(alt_s, nullptr) * 1.0e2);

    if (loc.lat == 0 && loc.lng == 0) {
        // default to CMAC instead of middle of the ocean. This makes
        // SITL in MissionPlanner a bit more useful
        loc.lat = -35.363261*1e7;
        loc.lng = 149.165230*1e7;
        loc.alt = 584*100;
    }

    yaw_degrees = strtof(yaw_s, nullptr);
    free(s);

    return true;
}

/*
  lookup a location in locations.txt in ROMFS
 */
bool SITL_State::lookup_location(const char *home_str, Location &loc, float &yaw_degrees)
{
    const char *locations = "@ROMFS/locations.txt";
    FileData *fd = AP::FS().load_file(locations);
    if (fd == nullptr) {
        ::printf("Missing %s\n", locations);
        return false;
    }
    char *str = strndup((const char *)fd->data, fd->length);
    if (!str) {
        delete fd;
        return false;
    }
    size_t len = strlen(home_str);
    char *saveptr = nullptr;
    for (char *s = strtok_r(str, "\r\n", &saveptr);
         s;
         s=strtok_r(nullptr, "\r\n", &saveptr)) {
        if (strncasecmp(s, home_str, len) == 0 && s[len]=='=') {
            bool ok = parse_home(&s[len+1], loc, yaw_degrees);
            free(str);
            delete fd;
            return ok;
        }
    }
    free(str);
    delete fd;
    ::printf("Failed to find location '%s'\n", home_str);
    return false;
}
    
#endif
