/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_HAL_SITL.h>
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <utility/getopt_cpp.h>

#include <SIM_Multicopter.h>
#include <SIM_Helicopter.h>
#include <SIM_Rover.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// catch floating point exceptions
static void _sig_fpe(int signum)
{
	fprintf(stderr, "ERROR: Floating point exception - aborting\n");
    abort();
}

void SITL_State::_usage(void)
{
	fprintf(stdout, "Options:\n");
	fprintf(stdout, "\t-w          wipe eeprom and dataflash\n");
	fprintf(stdout, "\t-r RATE     set SITL framerate\n");
	fprintf(stdout, "\t-H HEIGHT   initial barometric height\n");
	fprintf(stdout, "\t-C          use console instead of TCP ports\n");
	fprintf(stdout, "\t-I          set instance of SITL (adds 10*instance to all port numbers)\n");
	fprintf(stdout, "\t-s SPEEDUP  simulation speedup\n");
	fprintf(stdout, "\t-O ORIGIN   set home location (lat,lng,alt,yaw)\n");
	fprintf(stdout, "\t-M MODEL    set simulation model\n");
	fprintf(stdout, "\t-F FDMADDR  set FDM UDP address (IPv4)\n");
}

static const struct {
    const char *name;
    Aircraft *(*constructor)(const char *home_str, const char *frame_str);
} model_constructors[] = {
    { "+",         MultiCopter::create },
    { "x",         MultiCopter::create },
    { "hexa",      MultiCopter::create },
    { "hexax",     MultiCopter::create },
    { "octa",      MultiCopter::create },
    { "octa-quad", MultiCopter::create },
    { "heli",      Helicopter::create },
    { "rover",     Rover::create },
    { "rover-skid",Rover::create }
};

void SITL_State::_parse_command_line(int argc, char * const argv[])
{
	int opt;
    const char *home_str = NULL;
    const char *model_str = NULL;
    float speedup = 1.0f;

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

	const struct GetOptLong::option options[] = {
        {"help",            false,  0, 'h'},
        {"wipe",            false,  0, 'w'},
        {"speedup",         true,   0, 's'},
        {"rate",            true,   0, 'r'},
        {"height",          true,   0, 'H'},
        {"console",         false,  0, 'C'},
        {"instance",        true,   0, 'I'},
        {"param",           true,   0, 'P'},
        {"synthetic-clock", false,  0, 'S'},
        {"home",            true,   0, 'O'},
        {"model",           true,   0, 'M'},
        {"frame",           true,   0, 'F'},
		{0, false, 0, 0}
	};

    GetOptLong gopt(argc, argv, "hws:r:H:CI:P:SO:M:F:",
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
		case 'H':
			_initial_height = atof(gopt.optarg);
			break;
		case 'C':
			HALSITL::SITLUARTDriver::_console = true;
			break;
		case 'I': {
            uint8_t instance = atoi(gopt.optarg);
            _base_port  += instance * 10;
            _rcout_port += instance * 10;
            _simin_port += instance * 10;
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
            speedup = atof(gopt.optarg);
            break;
        case 'F':
            _fdm_address = gopt.optarg;
            break;
		default:
			_usage();
			exit(1);
		}
	}

    if (model_str && home_str) {
        for (uint8_t i=0; i<sizeof(model_constructors)/sizeof(model_constructors[0]); i++) {
            if (strcmp(model_constructors[i].name, model_str) == 0) {
                sitl_model = model_constructors[i].constructor(home_str, model_str);
                sitl_model->set_speedup(speedup);
                _synthetic_clock_mode = true;
                printf("Started model %s at %s at speed %.1f\n", model_str, home_str, speedup);
                break;
            }
        }
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

	_sitl_setup();
}

#endif
