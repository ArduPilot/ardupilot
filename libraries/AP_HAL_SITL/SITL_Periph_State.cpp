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
#include <SITL/SITL.h>

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
        CMDLINE_DEFAULTS,
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
        {"defaults",        true,   0, CMDLINE_DEFAULTS},
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
        case CMDLINE_DEFAULTS:
            defaults_path = strdup(gopt.optarg);
            break;
        default:
            printf("Options:\n"
                   "\t--help|-h                display this help information\n"
                   "\t--instance|-I N          set instance of SITL Periph\n"
                   "\t--maintenance|-M         run in maintenance mode\n"
                   "\t--defaults path          set param defaults file\n"
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

    sitl_model = new SimMCast("");

    _sitl = AP::sitl();
}

void SITL_State::wait_clock(uint64_t wait_time_usec)
{
    while (AP_HAL::micros64() < wait_time_usec) {
        struct sitl_input input {};
        sitl_model->update(input);
        sim_update();
        update_voltage_current(input, 0);
        usleep(100);
    }
}

/*
  open multicast input from main simulator
 */
void SimMCast::multicast_open(void)
{
    struct sockaddr_in sockaddr {};
    int ret;

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(SITL_MCAST_PORT);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(SITL_MCAST_IP);

    mc_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (mc_fd == -1) {
        fprintf(stderr, "socket failed - %s\n", strerror(errno));
        exit(1);
    }
    ret = fcntl(mc_fd, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        fprintf(stderr, "fcntl failed on setting FD_CLOEXEC - %s\n", strerror(errno));
        exit(1);
    }
    int one = 1;
    if (setsockopt(mc_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == -1) {
        fprintf(stderr, "setsockopt failed: %s\n", strerror(errno));
        exit(1);
    }

    // close on exec, to allow reboot
    fcntl(mc_fd, F_SETFD, FD_CLOEXEC);

#if defined(__CYGWIN__) || defined(__CYGWIN64__) || defined(CYGWIN_BUILD)
    /*
      on cygwin you need to bind to INADDR_ANY then use the multicast
      IP_ADD_MEMBERSHIP to get on the right address
     */
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
    
    ret = bind(mc_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "multicast bind failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }

    struct ip_mreq mreq {};
    mreq.imr_multiaddr.s_addr = inet_addr(SITL_MCAST_IP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    ret = setsockopt(mc_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    if (ret == -1) {
        fprintf(stderr, "multicast membership add failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }
}

/*
  read state from multicast
 */
void SimMCast::multicast_read(void)
{
    auto *_sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }
    struct SITL::sitl_fdm state;
    if (recv(mc_fd, (void*)&state, sizeof(state), MSG_WAITALL) == sizeof(state)) {
        if (state.timestamp_us < _sitl->state.timestamp_us) {
            // main process has rebooted
            base_time_us += (_sitl->state.timestamp_us - state.timestamp_us);
        }
        _sitl->state = state;
        hal.scheduler->stop_clock(_sitl->state.timestamp_us + base_time_us);
        HALSITL::Scheduler::timer_event();
    }
}

SimMCast::SimMCast(const char *frame_str) :
    Aircraft(frame_str)
{
    multicast_open();
}

void SimMCast::update(const struct sitl_input &input)
{
    multicast_read();
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(HAL_BUILD_AP_PERIPH)
