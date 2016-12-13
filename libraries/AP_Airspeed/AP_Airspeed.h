#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "AP_Airspeed_Backend.h"

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

    // read the analog source and update _airspeed
    void        read(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void            calibrate(bool in_startup);

    // return the current airspeed in m/s
    float           get_airspeed(void) const {
        return _airspeed;
    }

    // return the unfiltered airspeed in m/s
    float           get_raw_airspeed(void) const {
        return _raw_airspeed;
    }

    // return the current airspeed in cm/s
    float        get_airspeed_cm(void) const {
        return _airspeed*100;
    }

    // return the current airspeed ratio (dimensionless)
    float        get_airspeed_ratio(void) const {
        return _ratio;
    }

    // get temperature if available
    bool get_temperature(float &temperature);

    // set the airspeed ratio (dimensionless)
    void        set_airspeed_ratio(float ratio) {
        _ratio.set(ratio);
    }

    // return true if airspeed is enabled, and airspeed use is set
    bool        use(void) const {
        return enabled() && _use;
    }

    // return true if airspeed is enabled
    bool        enabled(void) const {
        return _type.get() != TYPE_NONE;
    }

    // force disable the airspeed sensor
    void        disable(void) {
        _type.set(TYPE_NONE);
    }

    // used by HIL to set the airspeed
    void        set_HIL(float airspeed) {
        _airspeed = airspeed;
    }

    // return the differential pressure in Pascal for the last
    // airspeed reading. Used by the calibration code
    float get_differential_pressure(void) const {
        return _last_pressure;
    }

    // return the current offset
    float get_offset(void) const {
        return _offset;
    }

    // return the current corrected pressure
    float get_corrected_pressure(void) const {
        return _corrected_pressure;
    }

    // set the apparent to true airspeed ratio
    void set_EAS2TAS(float v) {
        _EAS2TAS = v;
    }

    // get the apparent to true airspeed ratio
    float get_EAS2TAS(void) const {
        return _EAS2TAS;
    }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

	// log data to MAVLink
	void log_mavlink_send(mavlink_channel_t chan, const Vector3f &vground);

    // return health status of sensor
    bool healthy(void) const { return _healthy && fabsf(_offset) > 0 && enabled(); }

    void setHIL(float pressure) { _healthy=_hil_set=true; _hil_pressure=pressure; }

    // return time in ms of last update
    uint32_t last_update_ms(void) const { return _last_update_ms; }

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
    };
    
private:
    AP_Float        _offset;
    AP_Float        _ratio;
    AP_Float        _psi_range;
    AP_Int8         _use;
    AP_Int8         _type;
    AP_Int8         _pin;
    AP_Int8         _bus;
    AP_Int8         _autocal;
    AP_Int8         _tube_order;
    AP_Int8         _skip_cal;
    float           _raw_airspeed;
    float           _airspeed;
    float			_last_pressure;
    float			_corrected_pressure;
    float           _EAS2TAS;
    bool		    _healthy:1;
    bool		    _hil_set:1;
    float           _hil_pressure;
    uint32_t        _last_update_ms;

    // state of runtime calibration
    struct {
        uint32_t        start_ms;
        uint16_t        count;
        float           sum;
        uint16_t        read_count;
    } _cal;

    Airspeed_Calibration _calibration;
    float _last_saved_ratio;
    uint8_t _counter;

    float get_pressure(void);
    void update_calibration(float raw_pressure);

    AP_Airspeed_Backend *sensor;
};
