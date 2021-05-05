#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_HAL/utility/Socket.h>

class HAL_SITL;

class HALSITL::SITL_State {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    void init(int argc, char * const argv[]);
    int gps_pipe(uint8_t index);

    // create a file descriptor attached to a virtual device; type of
    // device is given by name parameter
    int sim_fd(const char *name, const char *arg);
    // returns a write file descriptor for a created virtual device
    int sim_fd_write(const char *name);

    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    uint16_t base_port(void) const {
        return _base_port;
    }

    // simulated airspeed, sonar and battery monitor
    uint16_t sonar_pin_value;    // pin 0
    uint16_t airspeed_pin_value; // pin 1
    uint16_t airspeed_2_pin_value; // pin 2
    uint16_t voltage_pin_value;  // pin 13
    uint16_t current_pin_value;  // pin 12
    uint16_t voltage2_pin_value;  // pin 15
    uint16_t current2_pin_value;  // pin 14
    // paths for UART devices
    const char *_uart_path[9] {
        "none:0",
        "fifo:gps",
        "none:1",
        "none:2",
        "none:3",
        "none:4",
        "none:5",
        "none:6",
        "none:7",
    };

    uint8_t get_instance() const { return _instance; }

private:

    void wait_clock(uint64_t wait_time_usec);
    bool _use_rtscts;
    uint16_t _base_port;

    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    uint8_t _instance;
};

#endif
