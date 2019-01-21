#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>

class AP_Airspeed_Backend;

#define AIRSPEED_MAX_SENSORS 2

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
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
};

class AP_Airspeed
{
public:
    friend class AP_Airspeed_Backend;
    
    // constructor
    AP_Airspeed();

    void init(void);

    // read the analog source and update airspeed
    void update(bool log);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool in_startup);

    // return the current airspeed in m/s
    float get_airspeed(uint8_t i) const {
        return state[i].airspeed;
    }
    float get_airspeed(void) const { return get_airspeed(primary); }

    // return the unfiltered airspeed in m/s
    float get_raw_airspeed(uint8_t i) const {
        return state[i].raw_airspeed;
    }
    float get_raw_airspeed(void) const { return get_raw_airspeed(primary); }

    // return the current airspeed ratio (dimensionless)
    float get_airspeed_ratio(uint8_t i) const {
        return param[i].ratio;
    }
    float get_airspeed_ratio(void) const { return get_airspeed_ratio(primary); }

    // get temperature if available
    bool get_temperature(uint8_t i, float &temperature);
    bool get_temperature(float &temperature) { return get_temperature(primary, temperature); }

    // set the airspeed ratio (dimensionless)
    void set_airspeed_ratio(uint8_t i, float ratio) {
        param[i].ratio.set(ratio);
    }
    void set_airspeed_ratio(float ratio) { set_airspeed_ratio(primary, ratio); }

    // return true if airspeed is enabled, and airspeed use is set
    bool use(uint8_t i) const;
    bool use(void) const { return use(primary); }

    // return true if airspeed is enabled
    bool enabled(uint8_t i) const {
        if (i < AIRSPEED_MAX_SENSORS) {
            return param[i].type.get() != TYPE_NONE;
        }
        return false;
    }
    bool enabled(void) const { return enabled(primary); }

    // used by HIL to set the airspeed
    void set_HIL(float airspeed) {
        state[primary].airspeed = airspeed;
    }

    // return the differential pressure in Pascal for the last airspeed reading
    float get_differential_pressure(uint8_t i) const {
        return state[i].last_pressure;
    }
    float get_differential_pressure(void) const { return get_differential_pressure(primary); }

    // return the current calibration offset
    float get_offset(uint8_t i) const {
        return param[i].offset;
    }
    float get_offset(void) const { return get_offset(primary); }

    // return the current corrected pressure
    float get_corrected_pressure(uint8_t i) const {
        return state[i].corrected_pressure;
    }
    float get_corrected_pressure(void) const { return get_corrected_pressure(primary); }

    // set the apparent to true airspeed ratio
    void set_EAS2TAS(uint8_t i, float v) {
        state[i].EAS2TAS = v;
    }
    void set_EAS2TAS(float v) { set_EAS2TAS(primary, v); }

    // get the apparent to true airspeed ratio
    float get_EAS2TAS(uint8_t i) const {
        return state[i].EAS2TAS;
    }
    float get_EAS2TAS(void) const { return get_EAS2TAS(primary); }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

	// log data to MAVLink
	void log_mavlink_send(mavlink_channel_t chan, const Vector3f &vground);

    // return health status of sensor
    bool healthy(uint8_t i) const {
        return state[i].healthy && (fabsf(param[i].offset) > 0 || state[i].use_zero_offset) && enabled(i);
    }
    bool healthy(void) const { return healthy(primary); }

    // return true if all enabled sensors are healthy
    bool all_healthy(void) const;
    
    void setHIL(float pressure) { state[0].healthy=state[0].hil_set=true; state[0].hil_pressure=pressure; }

    // return time in ms of last update
    uint32_t last_update_ms(uint8_t i) const { return state[i].last_update_ms; }
    uint32_t last_update_ms(void) const { return last_update_ms(primary); }

    void setHIL(float airspeed, float diff_pressure, float temperature);

    static const struct AP_Param::GroupInfo var_info[];

    enum pitot_tube_order { PITOT_TUBE_ORDER_POSITIVE = 0,
                            PITOT_TUBE_ORDER_NEGATIVE = 1,
                            PITOT_TUBE_ORDER_AUTO     = 2 };

    enum airspeed_type {
        TYPE_NONE=0,
        TYPE_I2C_MS4525=1,
        TYPE_ANALOG=2,
        TYPE_I2C_MS5525=3,
        TYPE_I2C_MS5525_ADDRESS_1=4,
        TYPE_I2C_MS5525_ADDRESS_2=5,
        TYPE_I2C_SDP3X=6,
        TYPE_I2C_DLVR=7,
        TYPE_UAVCAN=8,
    };

    // get current primary sensor
    uint8_t get_primary(void) const { return primary; }

    static AP_Airspeed *get_singleton() { return _singleton; }
    
private:
    static AP_Airspeed *_singleton;

    AP_Int8 primary_sensor;
    
    struct {
        AP_Float offset;
        AP_Float ratio;
        AP_Float psi_range;
        AP_Int8  use;
        AP_Int8  type;
        AP_Int8  pin;
        AP_Int8  bus;
        AP_Int8  autocal;
        AP_Int8  tube_order;
        AP_Int8  skip_cal;
    } param[AIRSPEED_MAX_SENSORS];

    struct airspeed_state {
        float   raw_airspeed;
        float   airspeed;
        float	last_pressure;
        float   filtered_pressure;
        float	corrected_pressure;
        float   EAS2TAS;
        bool	healthy:1;
        bool	hil_set:1;
        float   hil_pressure;
        uint32_t last_update_ms;
        bool use_zero_offset;
        
        // state of runtime calibration
        struct {
            uint32_t start_ms;
            uint16_t count;
            float    sum;
            uint16_t read_count;
        } cal;

        Airspeed_Calibration calibration;
        float last_saved_ratio;
        uint8_t counter;
    } state[AIRSPEED_MAX_SENSORS];

    // current primary sensor
    uint8_t primary;
    
    void read(uint8_t i);
    // return the differential pressure in Pascal for the last airspeed reading for the requested instance
    // returns 0 if the sensor is not enabled
    float get_pressure(uint8_t i);
    void update_calibration(uint8_t i, float raw_pressure);
    void update_calibration(uint8_t i, const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

    AP_Airspeed_Backend *sensor[AIRSPEED_MAX_SENSORS];
};
