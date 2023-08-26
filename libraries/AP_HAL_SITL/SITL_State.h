#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SITL_State_common.h"

#if defined(HAL_BUILD_AP_PERIPH)
#include "SITL_Periph_State.h"
#else

class HAL_SITL;

class HALSITL::SITL_State : public SITL_State_Common {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    void init(int argc, char * const argv[]);

    void loop_hook(void);
    uint16_t base_port(void) const {
        return _base_port;
    }

    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    // paths for UART devices
    const char *_uart_path[9] {
        "tcp:0:wait",
        "GPS1",
        "tcp:2",
        "tcp:3",
        "GPS2",
        "tcp:5",
        "tcp:6",
        "tcp:7",
        "tcp:8",
    };
    std::vector<struct AP_Param::defaults_table_struct> cmdline_param;

    /* parse a home location string */
    static bool parse_home(const char *home_str,
                           Location &loc,
                           float &yaw_degrees);

    /* lookup a location in locations.txt */
    static bool lookup_location(const char *home_str,
                                Location &loc,
                                float &yaw_degrees);
    
    uint8_t get_instance() const { return _instance; }

private:
    void _parse_command_line(int argc, char * const argv[]);
    void _set_param_default(const char *parm);
    void _usage(void);
    void _sitl_setup();
    void _setup_fdm(void);
    void _setup_timer(void);
    void _setup_adc(void);

    void set_height_agl(void);
    void _update_rangefinder();
    void _set_signal_handlers(void) const;

    void _update_airspeed(float airspeed);
    void _check_rc_input(void);
    bool _read_rc_sitl_input();
    void _fdm_input_local(void);
    void _output_to_flightgear(void);
    void _simulator_servos(struct sitl_input &input);
    void _fdm_input_step(void);

    void wait_clock(uint64_t wait_time_usec);

    // internal state
    uint8_t _instance;
    uint16_t _base_port;
    pid_t _parent_pid;
    uint32_t _update_count;

    Scheduler *_scheduler;

    SocketAPM _sitl_rc_in{true};
    uint16_t _rcin_port;
    uint16_t _fg_view_port;
    uint16_t _irlock_port;

    bool _synthetic_clock_mode;

    bool _use_rtscts;
    bool _use_fg_view;
    
    const char *_fg_address;

    // delay buffer variables
    static const uint8_t wind_buffer_length = 50;

    // airspeed sensor delay buffer variables
    struct readings_wind {
        uint32_t time;
        float data;
    };
    uint8_t store_index_wind;
    uint32_t last_store_time_wind;
    VectorN<readings_wind,wind_buffer_length> buffer_wind;
    uint32_t time_delta_wind;
    uint32_t delayed_time_wind;
    uint32_t wind_start_delay_micros;

    // simulated GPS devices
    SITL::GPS *gps[2];  // constrained by # of parameter sets

    // returns a voltage between 0V to 5V which should appear as the
    // voltage from the sensor
    float _sonar_pin_voltage() const;

    // multicast state
    int mc_out_fd = -1;
    int servo_in_fd = -1;

    // send out SITL state as UDP multicast
    void multicast_state_open(void);
    void multicast_state_send(void);
    void multicast_servo_update(struct sitl_input &input);

    uint16_t mc_servo[SITL_NUM_CHANNELS];
    void check_servo_input(void);
};

#endif // defined(HAL_BUILD_AP_PERIPH)
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
