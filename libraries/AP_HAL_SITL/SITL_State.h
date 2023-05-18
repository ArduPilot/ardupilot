#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#if defined(HAL_BUILD_AP_PERIPH)
#include "SITL_Periph_State.h"
#else

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <vector>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <SITL/SITL.h>
#include <SITL/SITL_Input.h>
#include <SITL/SIM_Gimbal.h>
#include <SITL/SIM_ADSB.h>
#include <SITL/SIM_Vicon.h>
#include <SITL/SIM_RF_Benewake_TF02.h>
#include <SITL/SIM_RF_Benewake_TF03.h>
#include <SITL/SIM_RF_Benewake_TFmini.h>
#include <SITL/SIM_RF_TeraRanger_Serial.h>
#include <SITL/SIM_RF_LightWareSerial.h>
#include <SITL/SIM_RF_LightWareSerialBinary.h>
#include <SITL/SIM_RF_Lanbao.h>
#include <SITL/SIM_RF_BLping.h>
#include <SITL/SIM_RF_LeddarOne.h>
#include <SITL/SIM_RF_USD1_v0.h>
#include <SITL/SIM_RF_USD1_v1.h>
#include <SITL/SIM_RF_MaxsonarSerialLV.h>
#include <SITL/SIM_RF_Wasp.h>
#include <SITL/SIM_RF_NMEA.h>
#include <SITL/SIM_RF_MAVLink.h>
#include <SITL/SIM_RF_GYUS42v2.h>
#include <SITL/SIM_VectorNav.h>
#include <SITL/SIM_LORD.h>
#include <SITL/SIM_AIS.h>
#include <SITL/SIM_GPS.h>

#include <SITL/SIM_Frsky_D.h>
#include <SITL/SIM_CRSF.h>
// #include <SITL/SIM_Frsky_SPort.h>
// #include <SITL/SIM_Frsky_SPortPassthrough.h>
#include <SITL/SIM_PS_RPLidarA2.h>
#include <SITL/SIM_PS_TeraRangerTower.h>
#include <SITL/SIM_PS_LightWare_SF45B.h>

#include <SITL/SIM_RichenPower.h>
#include <SITL/SIM_FETtecOneWireESC.h>
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
        Rover,
        ArduPlane,
        ArduSub,
        Blimp
    };

    uint16_t pwm_output[SITL_NUM_CHANNELS];
    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    bool output_ready = false;
    bool new_rc_input;
    void loop_hook(void);
    uint16_t base_port(void) const {
        return _base_port;
    }

    // create a simulated serial device; type of device is given by
    // name parameter
    SITL::SerialDevice *create_serial_sim(const char *name, const char *arg);

    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    // simulated airspeed, sonar and battery monitor
    uint16_t sonar_pin_value;    // pin 0
    uint16_t airspeed_pin_value[AIRSPEED_MAX_SENSORS]; // pin 1
    uint16_t voltage_pin_value;  // pin 13
    uint16_t current_pin_value;  // pin 12
    uint16_t voltage2_pin_value;  // pin 15
    uint16_t current2_pin_value;  // pin 14

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
    enum vehicle_type _vehicle;
    uint8_t _instance;
    uint16_t _base_port;
    pid_t _parent_pid;
    uint32_t _update_count;

    Scheduler *_scheduler;

    SocketAPM _sitl_rc_in{true};
    SITL::SIM *_sitl;
    uint16_t _rcin_port;
    uint16_t _fg_view_port;
    uint16_t _irlock_port;
    float _current;

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

    // internal SITL model
    SITL::Aircraft *sitl_model;

#if HAL_SIM_GIMBAL_ENABLED
    // simulated gimbal
    bool enable_gimbal;
    SITL::Gimbal *gimbal;
#endif

#if HAL_SIM_ADSB_ENABLED
    // simulated ADSb
    SITL::ADSB *adsb;
#endif

    // simulated vicon system:
    SITL::Vicon *vicon;

    // simulated Benewake tf02 rangefinder:
    SITL::RF_Benewake_TF02 *benewake_tf02;
    // simulated Benewake tf03 rangefinder:
    SITL::RF_Benewake_TF03 *benewake_tf03;
    // simulated Benewake tfmini rangefinder:
    SITL::RF_Benewake_TFmini *benewake_tfmini;
    // simulated TeraRanger Serial:
    SITL::RF_TeraRanger_Serial *teraranger_serial;

    // simulated LightWareSerial rangefinder - legacy protocol::
    SITL::RF_LightWareSerial *lightwareserial;
    // simulated LightWareSerial rangefinder - binary protocol:
    SITL::RF_LightWareSerialBinary *lightwareserial_binary;
    // simulated Lanbao rangefinder:
    SITL::RF_Lanbao *lanbao;
    // simulated BLping rangefinder:
    SITL::RF_BLping *blping;
    // simulated LeddarOne rangefinder:
    SITL::RF_LeddarOne *leddarone;
    // simulated USD1 v0 rangefinder:
    SITL::RF_USD1_v0 *USD1_v0;
    // simulated USD1 v1 rangefinder:
    SITL::RF_USD1_v1 *USD1_v1;
    // simulated MaxsonarSerialLV rangefinder:
    SITL::RF_MaxsonarSerialLV *maxsonarseriallv;
    // simulated Wasp rangefinder:
    SITL::RF_Wasp *wasp;
    // simulated NMEA rangefinder:
    SITL::RF_NMEA *nmea;
    // simulated MAVLink rangefinder:
    SITL::RF_MAVLink *rf_mavlink;
    // simulated GYUS42v2 rangefinder:
    SITL::RF_GYUS42v2 *gyus42v2;

    // simulated Frsky devices
    SITL::Frsky_D *frsky_d;
    // SITL::Frsky_SPort *frsky_sport;
    // SITL::Frsky_SPortPassthrough *frsky_sportpassthrough;

#if HAL_SIM_PS_RPLIDARA2_ENABLED
    // simulated RPLidarA2:
    SITL::PS_RPLidarA2 *rplidara2;
#endif

    // simulated FETtec OneWire ESCs:
    SITL::FETtecOneWireESC *fetteconewireesc;

#if HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED
    // simulated SF45B proximity sensor:
    SITL::PS_LightWare_SF45B *sf45b;
#endif

#if HAL_SIM_PS_TERARANGERTOWER_ENABLED
    SITL::PS_TeraRangerTower *terarangertower;
#endif

#if AP_SIM_CRSF_ENABLED
    // simulated CRSF devices
    SITL::CRSF *crsf;
#endif

    // simulated VectorNav system:
    SITL::VectorNav *vectornav;

    // simulated LORD Microstrain system
    SITL::LORD *lord;

#if HAL_SIM_JSON_MASTER_ENABLED
    // Ride along instances via JSON SITL backend
    SITL::JSON_Master ride_along;
#endif

#if HAL_SIM_AIS_ENABLED
    // simulated AIS stream
    SITL::AIS *ais;
#endif

    // simulated EFI MegaSquirt device:
    SITL::EFI_MegaSquirt *efi_ms;

    // output socket for flightgear viewing
    SocketAPM fg_socket{true};
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    char *_gps_fifo[2];

    // simulated GPS devices
    SITL::GPS *gps[2];  // constrained by # of parameter sets

    // returns a voltage between 0V to 5V which should appear as the
    // voltage from the sensor
    float _sonar_pin_voltage() const;

};

#endif // defined(HAL_BUILD_AP_PERIPH)
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
