#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#if AP_AIRSPEED_MSP_ENABLED
#include <AP_MSP/msp.h>
#endif
#if AP_AIRSPEED_EXTERNAL_ENABLED
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#endif

class AP_Airspeed_Backend;

class AP_Airspeed_Params {
public:
    // Constructor
    AP_Airspeed_Params(void);

    // parameters for each instance
    AP_Int32 bus_id;
#ifndef HAL_BUILD_AP_PERIPH
    AP_Float offset;
    AP_Float ratio;
#endif
    AP_Float psi_range;
#ifndef HAL_BUILD_AP_PERIPH
    AP_Int8  use;
    AP_Int8  pin;

    enum class SkipCalType : int8_t {
        // Do not skip boot calibration, this is the default
        None = 0,

        // Skip boot calibration, use saved offset, no calibration is required (but can be performed manually)
        NoCalRequired = 1,

        // Skip boot calibration, require manual calibration once per boot
        SkipBootCal = 2,
    };
    AP_Enum<SkipCalType> skip_cal;

    AP_Int8  tube_order;
#endif
    AP_Int8  type;
    AP_Int8  bus;
#if AP_AIRSPEED_AUTOCAL_ENABLE
    AP_Int8  autocal;
#endif

    static const struct AP_Param::GroupInfo var_info[];
};


class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    // constructor
    Airspeed_Calibration();

    // initialise the calibration
    void init(float initial_ratio);

    // take current airspeed in m/s and ground speed vector and return
    // new scaling factor
    float update(float airspeed, const Vector3f &vg, int16_t max_airspeed_allowed_during_cal);

private:
    // state of kalman filter for airspeed ratio estimation
    Matrix3f P; // covariance matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
};

class AP_Airspeed
{
public:
    friend class AP_Airspeed_Backend;
    
    // constructor
    AP_Airspeed();

    void set_fixedwing_parameters(const class AP_FixedWing *_fixed_wing_parameters);

    void init(void);
    void allocate();


    // indicate which bit in LOG_BITMASK indicates we should log airspeed readings
    void set_log_bit(uint32_t log_bit) { _log_bit = log_bit; }

#if AP_AIRSPEED_AUTOCAL_ENABLE
    // inflight ratio calibration
    void set_calibration_enabled(bool enable) {calibration_enabled = enable;}
#endif //AP_AIRSPEED_AUTOCAL_ENABLE

    // read the analog source and update airspeed
    void update(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool in_startup);

    // return the current airspeed in m/s
    float get_airspeed(uint8_t i) const;
    float get_airspeed(void) const { return get_airspeed(primary); }

    // return the unfiltered airspeed in m/s
    float get_raw_airspeed(uint8_t i) const;
    float get_raw_airspeed(void) const { return get_raw_airspeed(primary); }

    // return the current airspeed ratio (dimensionless)
    float get_airspeed_ratio(uint8_t i) const {
#ifndef HAL_BUILD_AP_PERIPH
        return param[i].ratio;
#else
        return 0.0;
#endif
    }
    float get_airspeed_ratio(void) const { return get_airspeed_ratio(primary); }

    // get temperature if available
    bool get_temperature(uint8_t i, float &temperature) const;
    bool get_temperature(float &temperature) const { return get_temperature(primary, temperature); }

    // set the airspeed ratio (dimensionless)
#ifndef HAL_BUILD_AP_PERIPH
    void set_airspeed_ratio(uint8_t i, float ratio) {
        param[i].ratio.set(ratio);
    }
    void set_airspeed_ratio(float ratio) { set_airspeed_ratio(primary, ratio); }
#endif

    // return true if airspeed is enabled, and airspeed use is set
    bool use(uint8_t i) const;
    bool use(void) const { return use(primary); }

    // force disabling of all airspeed sensors
    void force_disable_use(bool value) {
        _force_disable_use = value;
    }

    // return true if airspeed is enabled
    bool enabled(uint8_t i) const;
    bool enabled(void) const { return enabled(primary); }

    // return the differential pressure in Pascal for the last airspeed reading
    float get_differential_pressure(uint8_t i) const;
    float get_differential_pressure(void) const { return get_differential_pressure(primary); }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

    // return health status of sensor
    bool healthy(uint8_t i) const;
    bool healthy(void) const { return healthy(primary); }

    // return true if all enabled sensors are healthy
    bool all_healthy(void) const;
    
    // return time in ms of last update
    uint32_t last_update_ms(uint8_t i) const { return state[i].last_update_ms; }
    uint32_t last_update_ms(void) const { return last_update_ms(primary); }

#if AP_AIRSPEED_HYGROMETER_ENABLE
    bool get_hygrometer(uint8_t i, uint32_t &last_sample_ms, float &temperature, float &humidity) const;
#endif

    static const struct AP_Param::GroupInfo var_info[];

    enum pitot_tube_order { PITOT_TUBE_ORDER_POSITIVE = 0,
                            PITOT_TUBE_ORDER_NEGATIVE = 1,
                            PITOT_TUBE_ORDER_AUTO     = 2 };

    enum OptionsMask {
        ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE                   = (1<<0),   // If set then use airspeed failure check
        ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE         = (1<<1),   // If set then automatically enable the airspeed sensor use when healthy again.
        DISABLE_VOLTAGE_CORRECTION                            = (1<<2),
        USE_EKF_CONSISTENCY                                   = (1<<3),
        REPORT_OFFSET                                         = (1<<4),   // report offset cal to GCS
    };

    enum airspeed_type {
        TYPE_NONE=0,
#if AP_AIRSPEED_MS4525_ENABLED
        TYPE_I2C_MS4525=1,
#endif  // AP_AIRSPEED_MSP_ENABLED
#if AP_AIRSPEED_ANALOG_ENABLED
        TYPE_ANALOG=2,
#endif  // AP_AIRSPEED_ANALOG_ENABLED
#if AP_AIRSPEED_MS5525_ENABLED
        TYPE_I2C_MS5525=3,
        TYPE_I2C_MS5525_ADDRESS_1=4,
        TYPE_I2C_MS5525_ADDRESS_2=5,
#endif  // AP_AIRSPEED_MS5525_ENABLED
#if AP_AIRSPEED_SDP3X_ENABLED
        TYPE_I2C_SDP3X=6,
#endif  // AP_AIRSPEED_SDP3X_ENABLED
#if AP_AIRSPEED_DLVR_ENABLED
        TYPE_I2C_DLVR_5IN=7,
#endif  // AP_AIRSPEED_DLVR_ENABLED
#if AP_AIRSPEED_DRONECAN_ENABLED
        TYPE_UAVCAN=8,
#endif  // AP_AIRSPEED_DRONECAN_ENABLED
#if AP_AIRSPEED_DLVR_ENABLED
        TYPE_I2C_DLVR_10IN=9,
        TYPE_I2C_DLVR_20IN=10,
        TYPE_I2C_DLVR_30IN=11,
        TYPE_I2C_DLVR_60IN=12,
#endif  // AP_AIRSPEED_DLVR_ENABLED
#if AP_AIRSPEED_NMEA_ENABLED
        TYPE_NMEA_WATER=13,
#endif  // AP_AIRSPEED_NMEA_ENABLED
#if AP_AIRSPEED_MSP_ENABLED
        TYPE_MSP=14,
#endif  // AP_AIRSPEED_MSP_ENABLED
#if AP_AIRSPEED_ASP5033_ENABLED
        TYPE_I2C_ASP5033=15,
#endif  // AP_AIRSPEED_ASP5033_ENABLED
#if AP_AIRSPEED_EXTERNAL_ENABLED
        TYPE_EXTERNAL=16,
#endif  // AP_AIRSPEED_EXTERNAL_ENABLED
#if AP_AIRSPEED_AUAV_ENABLED
        TYPE_AUAV_10IN=17,
        TYPE_AUAV_5IN=18,
        TYPE_AUAV_30IN=19,
#endif  // AP_AIRSPEED_AUAV_ENABLED
#if AP_AIRSPEED_SITL_ENABLED
        TYPE_SITL=100,
#endif  // AP_AIRSPEED_SITL_ENABLED
    };

    // get current primary sensor
    uint8_t get_primary(void) const { return primary; }

    // get number of sensors
    uint8_t get_num_sensors(void) const { return num_sensors; }
    
    static AP_Airspeed *get_singleton() { return _singleton; }

    // return the current corrected pressure, public for AP_Periph
    float get_corrected_pressure(uint8_t i) const;
    float get_corrected_pressure(void) const {
        return get_corrected_pressure(primary);
    }

#if AP_AIRSPEED_MSP_ENABLED
    void handle_msp(const MSP::msp_airspeed_data_message_t &pkt);
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED
    void handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt);
#endif

    enum class CalibrationState {
        NOT_STARTED,
        NOT_REQUIRED_ZERO_OFFSET,
        IN_PROGRESS,
        SUCCESS,
        FAILED
    };

    // get aggregate calibration state for the Airspeed library:
    CalibrationState get_calibration_state() const;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(size_t buflen, char *buffer) const;

private:
    static AP_Airspeed *_singleton;

    AP_Int8 _enable;
    bool lib_enabled() const;

    AP_Int8 primary_sensor;
    AP_Int8 max_speed_pcnt;
    AP_Int32 _options;    // bitmask options for airspeed
    AP_Float _wind_max;
    AP_Float _wind_warn;
    AP_Float _wind_gate;

    AP_Airspeed_Params param[AIRSPEED_MAX_SENSORS];

    struct airspeed_state {
        float   raw_airspeed;
        float   airspeed;
        float	last_pressure;
        float   filtered_pressure;
        float	corrected_pressure;
        uint32_t last_update_ms;
        bool	healthy;

        // Pre-flight offset calibration
        struct {
            uint32_t start_ms;
            float    sum;
            uint16_t count;
            uint16_t read_count;
            CalibrationState state;
        } cal;

#if AP_AIRSPEED_AUTOCAL_ENABLE
        // In flight ratio calibration
        Airspeed_Calibration calibration;
        float last_saved_ratio;
        uint8_t counter;
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

        struct {
            uint32_t last_check_ms;
            float health_probability;
            float test_ratio;
            int8_t param_use_backup;
            uint32_t last_warn_ms;
        } failures;

#if AP_AIRSPEED_HYGROMETER_ENABLE
        uint32_t last_hygrometer_log_ms;
#endif
    } state[AIRSPEED_MAX_SENSORS];

    bool calibration_enabled;

    // can be set to true to disable the use of the airspeed sensor
    bool _force_disable_use;

    // current primary sensor
    uint8_t primary;
    uint8_t num_sensors;

    uint32_t _log_bit = -1;     // stores which bit in LOG_BITMASK is used to indicate we should log airspeed readings

    void read(uint8_t i);

    // get the health probability
    float get_health_probability(uint8_t i) const {
        return state[i].failures.health_probability;
    }
    float get_health_probability(void) const {
        return get_health_probability(primary);
    }

    // get the consistency test ratio
    float get_test_ratio(uint8_t i) const {
        return state[i].failures.test_ratio;
    }
    float get_test_ratio(void) const {
        return get_test_ratio(primary);
    }

    void update_calibration(uint8_t i, float raw_pressure);
    void update_calibration(uint8_t i, const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);
    void send_airspeed_calibration(const Vector3f &vg);
    // return the current calibration offset
    float get_offset(uint8_t i) const;
    float get_offset(void) const { return get_offset(primary); }

    void check_sensor_failures();
    void check_sensor_ahrs_wind_max_failures(uint8_t i);

    AP_Airspeed_Backend *sensor[AIRSPEED_MAX_SENSORS];

    void Log_Airspeed();

    bool add_backend(AP_Airspeed_Backend *backend);
    
    const AP_FixedWing *fixed_wing_parameters;

    void convert_per_instance();

};

namespace AP {
    AP_Airspeed *airspeed();
};

#endif  // AP_AIRSPEED_ENABLED
