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
#include <list>
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
#include <SITL/SIM_RF_LightWareSerial.h>
#include <SITL/SIM_RF_Lanbao.h>
#include <SITL/SIM_RF_BLping.h>
#include <SITL/SIM_RF_LeddarOne.h>
#include <SITL/SIM_RF_uLanding_v0.h>
#include <SITL/SIM_RF_uLanding_v1.h>
#include <SITL/SIM_RF_MaxsonarSerialLV.h>
#include <SITL/SIM_RF_Wasp.h>
#include <SITL/SIM_RF_NMEA.h>
#include <SITL/SIM_RF_MAVLink.h>

#include <SITL/SIM_Frsky_D.h>
// #include <SITL/SIM_Frsky_SPort.h>
// #include <SITL/SIM_Frsky_SPortPassthrough.h>

#include <AP_HAL/utility/Socket.h>
#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>
#include <uavcan/uavcan.hpp>

class HAL_SITL;

class HALSITL::SITL_State {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;

public:
    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
    public:
        SystemClock() = default;

        void adjustUtc(uavcan::UtcDuration adjustment) override {
            utc_adjustment_usec = adjustment.toUSec();
        }

        uavcan::MonotonicTime getMonotonic() const override {
            return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
        }

        uavcan::UtcTime getUtc() const override {
            return uavcan::UtcTime::fromUSec(AP_HAL::micros64() + utc_adjustment_usec);
        }

        static SystemClock& instance() {
            static SystemClock inst;
            return inst;
        }

    private:
        int64_t utc_adjustment_usec;
    };
    
    class CANDriver: public uavcan::ICanDriver {
        friend HALSITL::SITL_State;
        bool initialized_;
        SLCAN::CAN driver_;
        uint8_t _ifaces_num = 1;

        virtual int16_t select(uavcan::CanSelectMasks& inout_masks,
                            const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline) override;

        uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces]);
        pthread_t _irq_handler_ctx;
        pthread_mutex_t _irq_handler_mtx;
        pthread_cond_t _irq_handler_cond;

        HALSITL::SITL_State* _sitlState;
        uavcan::Node<0> *_node;
        static HAL_Semaphore spin_sem;
        static bool reader_thread_initialised;
    public:
        CANDriver(HALSITL::SITL_State* sitlState)
            :  _sitlState(sitlState), initialized_(false), driver_(SLCAN_DRIVER_INDEX, SLCAN_RX_QUEUE_SIZE)
        { }


        bool begin(uint32_t bitrate, uint8_t can_number);
        bool is_initialized();
        void initialized(bool val);

        virtual SLCAN::CAN* getIface(uint8_t iface_index) override
        {
            return &driver_;
        }

        virtual uint8_t getNumIfaces() const override
        {
            return _ifaces_num;
        }
        static int self_read_fd[2];
        static int client_write_fd[2];
        static int self_write_fd[2];
        static int client_read_fd[2];
        uint8_t _can_number;
        static HAL_Semaphore& get_sem() { return spin_sem; } 
        uavcan::Node<0>* get_node() const { return _node; }
    };

    void can_reader(void);

    class RaiiSynchronizer {};

    std::list<uavcan::PoolAllocator<UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_BLOCK_SIZE, SITL_State::RaiiSynchronizer>*> _node_allocator;

    void init(int argc, char * const argv[]);
    SITL_State::CANDriver* initNode(uint8_t driver_index, uint8_t node_id);

    enum vehicle_type {
        ArduCopter,
        Rover,
        ArduPlane,
        ArduSub
    };

    int gps_pipe(uint8_t index);
    int can_ap2sitl_pipe(uint8_t index);
    int can_sitl2ap_pipe(uint8_t index);
    ssize_t gps_read(int fd, void *buf, size_t count);
    uint16_t pwm_output[SITL_NUM_CHANNELS];
    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    bool output_ready = false;
    bool new_rc_input;
    void loop_hook(void);
    uint16_t base_port(void) const {
        return _base_port;
    }

    // create a file descriptor attached to a virtual device; type of
    // device is given by name parameter
    int sim_fd(const char *name, const char *arg);
    // returns a write file descriptor for a created virtual device
    int sim_fd_write(const char *name);

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
    const char *_uart_path[8] {
        "tcp:0:wait",
        "GPS1",
        "tcp:2",
        "tcp:3",
        "GPS2",
        "tcp:5",
        "CAN1",
        "CAN2"
    };

    /* parse a home location string */
    static bool parse_home(const char *home_str,
                           Location &loc,
                           float &yaw_degrees);

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
    void uavcan_loop(void);

    struct gps_data {
        double latitude;
        double longitude;
        float altitude;
        double speedN;
        double speedE;
        double speedD;
        double yaw;
        bool have_lock;
    };

#define MAX_GPS_DELAY 100
    gps_data _gps_data[2][MAX_GPS_DELAY];

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
                     double speedN, double speedE, double speedD,
                     double yaw, bool have_lock);
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

    // simulated Benewake tf02 rangefinder:
    SITL::RF_Benewake_TF02 *benewake_tf02;
    // simulated Benewake tf03 rangefinder:
    SITL::RF_Benewake_TF03 *benewake_tf03;
    // simulated Benewake tfmini rangefinder:
    SITL::RF_Benewake_TFmini *benewake_tfmini;

    // simulated LightWareSerial rangefinder:
    SITL::RF_LightWareSerial *lightwareserial;
    // simulated Lanbao rangefinder:
    SITL::RF_Lanbao *lanbao;
    // simulated BLping rangefinder:
    SITL::RF_BLping *blping;
    // simulated LeddarOne rangefinder:
    SITL::RF_LeddarOne *leddarone;
    // simulated uLanding v0 rangefinder:
    SITL::RF_uLanding_v0 *ulanding_v0;
    // simulated uLanding v1 rangefinder:
    SITL::RF_uLanding_v1 *ulanding_v1;
    // simulated MaxsonarSerialLV rangefinder:
    SITL::RF_MaxsonarSerialLV *maxsonarseriallv;
    // simulated Wasp rangefinder:
    SITL::RF_Wasp *wasp;
    // simulated NMEA rangefinder:
    SITL::RF_NMEA *nmea;
    // simulated MAVLink rangefinder:
    SITL::RF_MAVLink *rf_mavlink;

    // simulated Frsky devices
    SITL::Frsky_D *frsky_d;
    // SITL::Frsky_SPort *frsky_sport;
    // SITL::Frsky_SPortPassthrough *frsky_sportpassthrough;

    // output socket for flightgear viewing
    SocketAPM fg_socket{true};
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    const char *_home_str;

    std::list<CANDriver*> _can_mgr;
    std::list<uavcan::Node<0>*> _node;
    uint32_t _num_nodes;
    bool _uc_periph_thread_created;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
