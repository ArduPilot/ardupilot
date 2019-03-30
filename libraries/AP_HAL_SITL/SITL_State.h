#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

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
#include <SITL/SITL.h>
#include <SITL/SITL_Input.h>
#include <SITL/SIM_Gimbal.h>
#include <SITL/SIM_ADSB.h>
#include <SITL/SIM_Vicon.h>
#include <AP_HAL/utility/Socket.h>

class HAL_SITL;

class HALSITL::SITL_State {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    void init(int argc, char * const argv[]);

    enum vehicle_type {
        ArduCopter,
        APMrover2,
        ArduPlane,
        ArduSub
    };

    int gps_pipe(void);
    int gps2_pipe(void);
    ssize_t gps_read(int fd, void *buf, size_t count);
    uint16_t pwm_output[SITL_NUM_CHANNELS];
    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    bool output_ready = false;
    bool new_rc_input;
    void loop_hook(void);
    uint16_t base_port(void) const {
        return _base_port;
    }

    // create a file desciptor attached to a virtual device; type of
    // device is given by name parameter
    int sim_fd(const char *name, const char *arg);

    bool use_rtscts(void) const {
        return _use_rtscts;
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
    const char *_uart_path[7] {
        "tcp:0:wait",
        "GPS1",
        "tcp:2",
        "tcp:3",
        "GPS2",
        "tcp:5",
        "tcp:6",
    };
    
private:
    void _parse_command_line(int argc, char * const argv[]);
    void _set_param_default(const char *parm);
    void _usage(void);
    void _sitl_setup(const char *home_str);
    void _setup_fdm(void);
    void _setup_timer(void);
    void _setup_adc(void);

    void set_height_agl(void);
    void _update_rangefinder(float range_value);
    void _set_signal_handlers(void) const;

    struct gps_data {
        double latitude;
        double longitude;
        float altitude;
        double speedN;
        double speedE;
        double speedD;
        bool have_lock;
    };

#define MAX_GPS_DELAY 100
    gps_data _gps_data[MAX_GPS_DELAY];

    bool _gps_has_basestation_position;
    gps_data _gps_basestation_data;
    void _gps_write(const uint8_t *p, uint16_t size, uint8_t instance);
    void _gps_send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size, uint8_t instance);
    void _update_gps_ubx(const struct gps_data *d, uint8_t instance);
    void _update_gps_mtk(const struct gps_data *d, uint8_t instance);
    void _update_gps_mtk16(const struct gps_data *d, uint8_t instance);
    void _update_gps_mtk19(const struct gps_data *d, uint8_t instance);
    uint16_t _gps_nmea_checksum(const char *s);
    void _gps_nmea_printf(uint8_t instance, const char *fmt, ...);
    void _update_gps_nmea(const struct gps_data *d, uint8_t instance);
    void _sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload, uint8_t instance);
    void _update_gps_sbp(const struct gps_data *d, uint8_t instance);
    void _update_gps_sbp2(const struct gps_data *d, uint8_t instance);
    void _update_gps_file(uint8_t instance);
    void _update_gps_nova(const struct gps_data *d, uint8_t instance);
    void _nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen, uint8_t instance);
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);

    void _update_gps(double latitude, double longitude, float altitude,
                     double speedN, double speedE, double speedD, bool have_lock);
    void _update_airspeed(float airspeed);
    void _update_gps_instance(SITL::SITL::GPSType gps_type, const struct gps_data *d, uint8_t instance);
    void _check_rc_input(void);
    bool _read_rc_sitl_input();
    void _fdm_input_local(void);
    void _output_to_flightgear(void);
    void _simulator_servos(struct sitl_input &input);
    void _simulator_output(bool synthetic_clock_mode);
    uint16_t _airspeed_sensor(float airspeed);
    uint16_t _ground_sonar();
    void _fdm_input_step(void);

    void wait_clock(uint64_t wait_time_usec);

    // internal state
    enum vehicle_type _vehicle;
    uint16_t _framerate;
    uint8_t _instance;
    uint16_t _base_port;
    pid_t _parent_pid;
    uint32_t _update_count;

    AP_Baro *_barometer;
    AP_InertialSensor *_ins;
    Scheduler *_scheduler;
    Compass *_compass;
#if AP_TERRAIN_AVAILABLE
    AP_Terrain *_terrain;
#endif

    SocketAPM _sitl_rc_in{true};
    SITL::SITL *_sitl;
    uint16_t _rcin_port;
    uint16_t _fg_view_port;
    uint16_t _irlock_port;
    float _current;

    bool _synthetic_clock_mode;

    bool _use_rtscts;
    bool _use_fg_view;
    
    const char *_fg_address;

    // delay buffer variables
    static const uint8_t mag_buffer_length = 250;
    static const uint8_t wind_buffer_length = 50;

    // magnetometer delay buffer variables
    struct readings_mag {
        uint32_t time;
        Vector3f data;
    };
    uint8_t store_index_mag;
    uint32_t last_store_time_mag;
    VectorN<readings_mag,mag_buffer_length> buffer_mag;
    uint32_t time_delta_mag;
    uint32_t delayed_time_mag;

    // airspeed sensor delay buffer variables
    struct readings_wind {
        uint32_t time;
        float data;
    };
    uint8_t store_index_wind;
    uint32_t last_store_time_wind;
    VectorN<readings_wind,wind_buffer_length> buffer_wind;
    VectorN<readings_wind,wind_buffer_length> buffer_wind_2;
    uint32_t time_delta_wind;
    uint32_t delayed_time_wind;
    uint32_t wind_start_delay_micros;

    // internal SITL model
    SITL::Aircraft *sitl_model;

    // simulated gimbal
    bool enable_gimbal;
    SITL::Gimbal *gimbal;

    // simulated ADSb
    SITL::ADSB *adsb;

    // simulated vicon system:
    SITL::Vicon *vicon;

    // output socket for flightgear viewing
    SocketAPM fg_socket{true};
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    const char *_home_str;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
