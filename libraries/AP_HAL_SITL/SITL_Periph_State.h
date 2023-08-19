#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(HAL_BUILD_AP_PERIPH)

#include "SITL_State_common.h"

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>

#include <SITL/SITL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_HAL/utility/Socket.h>

class SimMCast : public SITL::Aircraft {
public:
    SimMCast(const char *frame_str);
    void update(const struct sitl_input &input) override;

private:
    int mc_fd = -1;
    int servo_fd = -1;
    struct sockaddr_in in_addr;

    // offset between multicast timestamp and local timestamp
    uint64_t base_time_us;

    void multicast_open();
    void multicast_read();

    void servo_send(void);
    void servo_fd_open(void);
};

class HAL_SITL;

class HALSITL::SITL_State : public SITL_State_Common {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    void init(int argc, char * const argv[]);

    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    uint16_t base_port(void) const {
        return _base_port;
    }

    // paths for UART devices
    const char *_uart_path[9] {
        "none:0",
        "GPS1",
        "none:1",
        "none:2",
        "none:3",
        "none:4",
        "none:5",
        "none:6",
        "none:7",
    };

    uint8_t get_instance() const { return _instance; }

    bool run_in_maintenance_mode() const { return _maintenance; }

private:

    void wait_clock(uint64_t wait_time_usec);
    bool _use_rtscts;
    uint16_t _base_port;

    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    uint8_t _instance;
    bool _maintenance;

    // simulated GPS devices
    SITL::GPS *gps[1];  // constrained by # of parameter sets
};

#endif
