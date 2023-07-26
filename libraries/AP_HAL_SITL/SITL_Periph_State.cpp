#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include "Scheduler.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>

#include <AP_Param/AP_Param.h>
#include <SITL/SIM_JSBSim.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/utility/getopt_cpp.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

    enum long_options {
        CMDLINE_SERIAL0=1,
        CMDLINE_SERIAL1,
        CMDLINE_SERIAL2,
        CMDLINE_SERIAL3,
        CMDLINE_SERIAL4,
        CMDLINE_SERIAL5,
        CMDLINE_SERIAL6,
        CMDLINE_SERIAL7,
        CMDLINE_SERIAL8,
        CMDLINE_SERIAL9,
    };

void SITL_State::init(int argc, char * const argv[]) {
    int opt;
    const struct GetOptLong::option options[] = {
        {"help",            false,  0, 'h'},
        {"instance",        true,   0, 'I'},
        {"maintenance",     false,  0, 'M'},
        {"serial0",         true,   0, CMDLINE_SERIAL0},
        {"serial1",         true,   0, CMDLINE_SERIAL1},
        {"serial2",         true,   0, CMDLINE_SERIAL2},
        {"serial3",         true,   0, CMDLINE_SERIAL3},
        {"serial4",         true,   0, CMDLINE_SERIAL4},
        {"serial5",         true,   0, CMDLINE_SERIAL5},
        {"serial6",         true,   0, CMDLINE_SERIAL6},
        {"serial7",         true,   0, CMDLINE_SERIAL7},
        {"serial8",         true,   0, CMDLINE_SERIAL8},
        {"serial9",         true,   0, CMDLINE_SERIAL9},
        {0, false, 0, 0}
    };

    setvbuf(stdout, (char *)0, _IONBF, 0);
    setvbuf(stderr, (char *)0, _IONBF, 0);

    GetOptLong gopt(argc, argv, "hI:M",
                    options);

    while((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'I':
            _instance = atoi(gopt.optarg);
            break;
        case 'M':
            printf("Running in Maintenance Mode\n");
            _maintenance = true;
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
        case CMDLINE_SERIAL9: {
            static const uint8_t mapping[] = { 0, 2, 3, 1, 4, 5, 6, 7, 8, 9 };
            _uart_path[mapping[opt - CMDLINE_SERIAL0]] = gopt.optarg;
            break;
        }
        default:
            printf("Options:\n"
                   "\t--help|-h                display this help information\n"
                   "\t--instance|-I N          set instance of SITL Periph\n"
                   "\t--maintenance|-M         run in maintenance mode\n"
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
                );
            exit(1);
        }
    }

    printf("Running Instance: %d\n", _instance);
}

void SITL_State::wait_clock(uint64_t wait_time_usec) {
    while (AP_HAL::native_micros64() < wait_time_usec) {
        usleep(1000);
    }
}

// when Periph can use SITL simulated devices we should remove these
// stubs:
ssize_t SITL::SerialDevice::read_from_device(char*, size_t) const { return -1; }

ssize_t SITL::SerialDevice::write_to_device(char const*, size_t) const { return -1; }

#endif //CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(HAL_BUILD_AP_PERIPH)
