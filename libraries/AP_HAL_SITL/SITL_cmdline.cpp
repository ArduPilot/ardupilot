#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include <AP_HAL/utility/getopt_cpp.h>
#include <AP_Logger/AP_Logger_SITL.h>
#include <AP_Param/AP_Param.h>

#include <SITL/SIM_Multicopter.h>
#include <SITL/SIM_Helicopter.h>
#include <SITL/SIM_SingleCopter.h>
#include <SITL/SIM_Plane.h>
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
#include <SITL/SIM_JSON.h>
#include <SITL/SIM_Blimp.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;
using namespace SITL;

// catch floating point exceptions
static void _sig_fpe(int signum)
{
    fprintf(stderr, "ERROR: Floating point exception - aborting\n");
    AP_HAL::dump_stack_trace();
    abort();
}

// catch segfault
static void _sig_segv(int signum)
{
    fprintf(stderr, "ERROR: segmentation fault - aborting\n");
    AP_HAL::dump_stack_trace();
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
           "\t--disable-fgview         disable Flight Gear view\n"
           "\t--gimbal                 enable simulated MAVLink gimbal\n"
           "\t--autotest-dir DIR       set directory for additional files\n"
           "\t--defaults path          set path to defaults file\n"
           "\t--uartA device           set device string for UARTA\n"
           "\t--uartB device           set device string for UARTB\n"
           "\t--uartC device           set device string for UARTC\n"
           "\t--uartD device           set device string for UARTD\n"
           "\t--uartE device           set device string for UARTE\n"
           "\t--uartF device           set device string for UARTF\n"
           "\t--uartG device           set device string for UARTG\n"
           "\t--uartH device           set device string for UARTH\n"
           "\t--uartI device           set device string for UARTI\n"
           "\t--serial0 device         set device string for SERIAL0\n"
           "\t--serial1 device         set device string for SERIAL1\n"
           "\t--serial2 device         set device string for SERIAL2\n"
           "\t--serial3 device         set device string for SERIAL3\n"
           "\t--serial4 device         set device string for SERIAL4\n"
           "\t--serial5 device         set device string for SERIAL5\n"
           "\t--serial6 device         set device string for SERIAL6\n"
           "\t--serial7 device         set device string for SERIAL7\n"
           "\t--serial8 device         set device string for SERIAL8\n"
           "\t--rtscts                 enable rtscts on serial ports (default false)\n"
           "\t--base-port PORT         set port num for base port(default 5670) must be before -I option\n"
           "\t--rc-in-port PORT        set port num for rc in\n"
           "\t--sim-address ADDR       set address string for simulator\n"
           "\t--sim-port-in PORT       set port num for simulator in\n"
           "\t--sim-port-out PORT      set port num for simulator out\n"
           "\t--irlock-port PORT       set port num for irlock\n"
           "\t--start-time TIMESTR     set simulation start time in UNIX timestamp\n"
           "\t--sysid ID               set SYSID_THISMAV\n"
        );
}

static const struct {
    const char *name;
    Aircraft *(*constructor)(const char *frame_str);
} model_constructors[] = {
    { "quadplane",          QuadPlane::create },
    { "xplane",             XPlane::create },
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
    { "hexa-cwx",           MultiCopter::create },
    { "hexa-dji",           MultiCopter::create },
    { "octa",               MultiCopter::create },
    { "octa-cwx",           MultiCopter::create },
    { "octa-dji",           MultiCopter::create },
    { "octa-quad-cwx",      MultiCopter::create },
    { "dodeca-hexa",        MultiCopter::create },
    { "tri",                MultiCopter::create },
    { "y6",                 MultiCopter::create },
    { "deca",               MultiCopter::create },
    { "deca-cwx",           MultiCopter::create },
    { "heli",               Helicopter::create },
    { "heli-dual",          Helicopter::create },
    { "heli-compound",      Helicopter::create },
    { "singlecopter",       SingleCopter::create },
    { "coaxcopter",         SingleCopter::create },
    { "rover",              SimRover::create },
    { "balancebot",         BalanceBot::create },
    { "sailboat",           Sailboat::create },
    { "motorboat",          MotorBoat::create },
    { "crrcsim",            CRRCSim::create },
    { "jsbsim",             JSBSim::create },
    { "flightaxis",         FlightAxis::create },
    { "gazebo",             Gazebo::create },
    { "last_letter",        last_letter::create },
    { "tracker",            Tracker::create },
    { "balloon",            Balloon::create },
    { "plane",              Plane::create },
    { "calibration",        Calibration::create },
    { "vectored",           Submarine::create },
    { "vectored_6dof",      Submarine::create },
    { "silentwings",        SilentWings::create },
    { "morse",              Morse::create },
    { "airsim",             AirSim::create},
    { "scrimmage",          Scrimmage::create },
    { "webots",             Webots::create },
    { "JSON",               JSON::create },
    { "blimp",              Blimp::create },
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
    _instance = 0;
    _synthetic_clock_mode = false;
    // default to CMAC
    const char *home_str = nullptr;
    const char *model_str = nullptr;
    _use_fg_view = true;
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
    const bool is_replay = APM_BUILD_TYPE(APM_BUILD_Replay);

    enum long_options {
        CMDLINE_GIMBAL = 1,
        CMDLINE_FGVIEW,
        CMDLINE_AUTOTESTDIR,
        CMDLINE_DEFAULTS,
        CMDLINE_UARTA,
        CMDLINE_UARTB,
        CMDLINE_UARTC,
        CMDLINE_UARTD,
        CMDLINE_UARTE,
        CMDLINE_UARTF,
        CMDLINE_UARTG,
        CMDLINE_UARTH,
        CMDLINE_UARTI,
        CMDLINE_SERIAL0,
        CMDLINE_SERIAL1,
        CMDLINE_SERIAL2,
        CMDLINE_SERIAL3,
        CMDLINE_SERIAL4,
        CMDLINE_SERIAL5,
        CMDLINE_SERIAL6,
        CMDLINE_SERIAL7,
        CMDLINE_SERIAL8,
        CMDLINE_RTSCTS,
        CMDLINE_BASE_PORT,
        CMDLINE_RCIN_PORT,
        CMDLINE_SIM_ADDRESS,
        CMDLINE_SIM_PORT_IN,
        CMDLINE_SIM_PORT_OUT,
        CMDLINE_IRLOCK_PORT,
        CMDLINE_START_TIME,
        CMDLINE_SYSID,
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
        {"disable-fgview",  false,  0, CMDLINE_FGVIEW},
        {"autotest-dir",    true,   0, CMDLINE_AUTOTESTDIR},
        {"defaults",        true,   0, CMDLINE_DEFAULTS},
        {"uartA",           true,   0, CMDLINE_UARTA},
        {"uartB",           true,   0, CMDLINE_UARTB},
        {"uartC",           true,   0, CMDLINE_UARTC},
        {"uartD",           true,   0, CMDLINE_UARTD},
        {"uartE",           true,   0, CMDLINE_UARTE},
        {"uartF",           true,   0, CMDLINE_UARTF},
        {"uartG",           true,   0, CMDLINE_UARTG},
        {"uartH",           true,   0, CMDLINE_UARTH},
        {"uartI",           true,   0, CMDLINE_UARTI},
        {"serial0",         true,   0, CMDLINE_SERIAL0},
        {"serial1",         true,   0, CMDLINE_SERIAL1},
        {"serial2",         true,   0, CMDLINE_SERIAL2},
        {"serial3",         true,   0, CMDLINE_SERIAL3},
        {"serial4",         true,   0, CMDLINE_SERIAL4},
        {"serial5",         true,   0, CMDLINE_SERIAL5},
        {"serial6",         true,   0, CMDLINE_SERIAL6},
        {"serial7",         true,   0, CMDLINE_SERIAL7},
        {"serial8",         true,   0, CMDLINE_SERIAL8},
        {"rtscts",          false,  0, CMDLINE_RTSCTS},
        {"base-port",       true,   0, CMDLINE_BASE_PORT},
        {"rc-in-port",      true,   0, CMDLINE_RCIN_PORT},
        {"sim-address",     true,   0, CMDLINE_SIM_ADDRESS},
        {"sim-port-in",     true,   0, CMDLINE_SIM_PORT_IN},
        {"sim-port-out",    true,   0, CMDLINE_SIM_PORT_OUT},
        {"irlock-port",     true,   0, CMDLINE_IRLOCK_PORT},
        {"start-time",      true,   0, CMDLINE_START_TIME},
        {"sysid",           true,   0, CMDLINE_SYSID},
        {0, false, 0, 0}
    };

    if (is_replay) {
        model_str = "quad";
        HALSITL::UARTDriver::_console = true;
    }

    if (asprintf(&autotest_dir, SKETCHBOOK "/Tools/autotest") <= 0) {
        AP_HAL::panic("out of memory");
    }
    _set_signal_handlers();

    setvbuf(stdout, (char *)0, _IONBF, 0);
    setvbuf(stderr, (char *)0, _IONBF, 0);

    GetOptLong gopt(argc, argv, "hwus:r:CI:P:SO:M:F:c:",
                    options);
    while (!is_replay && (opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'w':
#if HAL_LOGGING_FILESYSTEM_ENABLED
            AP_Param::erase_all();
#endif
#if HAL_LOGGING_SITL_ENABLED
            unlink(AP_Logger_SITL::filename);
#endif
            break;
        case 'u':
            AP_Param::set_hide_disabled_groups(false);
            break;
        case 's':
            speedup = strtof(gopt.optarg, nullptr);
            temp_cmdline_param = {"SIM_SPEEDUP", speedup};
            cmdline_param.push_back(temp_cmdline_param);
            printf("Setting SIM_SPEEDUP=%f\n", speedup);
            break;
        case 'r':
            _framerate = (unsigned)atoi(gopt.optarg);
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
            _synthetic_clock_mode = true;
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
        case CMDLINE_GIMBAL:
            enable_gimbal = true;
            break;
        case CMDLINE_FGVIEW:
            _use_fg_view = false;
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
            _uart_path[opt - CMDLINE_UARTA] = gopt.optarg;
            break;
        case CMDLINE_SERIAL0:
        case CMDLINE_SERIAL1:
        case CMDLINE_SERIAL2:
        case CMDLINE_SERIAL3:
        case CMDLINE_SERIAL4:
        case CMDLINE_SERIAL5:
        case CMDLINE_SERIAL6:
        case CMDLINE_SERIAL7:
        case CMDLINE_SERIAL8:
            _uart_path[opt - CMDLINE_SERIAL0] = gopt.optarg;
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
            temp_cmdline_param = {"SYSID_THISMAV", static_cast<float>(sysid)};
            cmdline_param.push_back(temp_cmdline_param);
            printf("Setting SYSID_THISMAV=%d\n", sysid);
            break;
        }
        case 'h':
            _usage();
            exit(0);
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
            _synthetic_clock_mode = true;
            break;
        }
    }
    if (sitl_model == nullptr) {
        printf("Vehicle model (%s) not found\n", model_str);
        exit(1);
    }

    if (AP::sitl()) {
        // Set SITL start time.
        AP::sitl()->start_time_UTC = start_time_UTC;
    }

    fprintf(stdout, "Starting sketch '%s'\n", SKETCH);

    if (strcmp(SKETCH, "ArduCopter") == 0) {
        _vehicle = ArduCopter;
        if (_framerate == 0) {
            _framerate = 200;
        }
    } else if (strcmp(SKETCH, "Rover") == 0) {
        _vehicle = Rover;
        if (_framerate == 0) {
            _framerate = 50;
        }
        // set right default throttle for rover (allowing for reverse)
        pwm_input[2] = 1500;
    } else if (strcmp(SKETCH, "ArduSub") == 0) {
        _vehicle = ArduSub;
        for(uint8_t i = 0; i < 8; i++) {
            pwm_input[i] = 1500;
        }
    } else {
        _vehicle = ArduPlane;
        if (_framerate == 0) {
            _framerate = 50;
        }
    }

    _sitl_setup(home_str);
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
