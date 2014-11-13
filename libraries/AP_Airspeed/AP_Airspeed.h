/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AIRSPEED_H__
#define __AP_AIRSPEED_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <AP_Airspeed_Backend.h>
#include <AP_Airspeed_analog.h>
#include <AP_Airspeed_PX4.h>
#include <AP_Airspeed_I2C.h>

class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    // constructor
    Airspeed_Calibration(const AP_Vehicle::FixedWing &parms);

    // initialise the calibration
    void init(float initial_ratio);

    // take current airspeed in m/s and ground speed vector and return
    // new scaling factor
    float update(float airspeed, const Vector3f &vg);

private:
    // state of kalman filter for airspeed ratio estimation
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
    const AP_Vehicle::FixedWing &aparm;
};

class AP_Airspeed
{
public:
    // constructor
    AP_Airspeed(const AP_Vehicle::FixedWing &parms) :
        _raw_airspeed(0.0f),
        _airspeed(0.0f),
        _last_pressure(0.0f),
        _raw_pressure(0.0f),
        _EAS2TAS(1.0f),
        _healthy(false),
        _hil_set(false),
        _last_update_ms(0),
        _calibration(parms),
        _last_saved_ratio(0.0f),
        _counter(0),
        analog(_pin)
    {
		AP_Param::setup_object_defaults(this, var_info);
    };

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
        return _enable && _use && fabsf(_offset) > 0 && _healthy;
    }

    // return true if airspeed is enabled
    bool        enabled(void) const {
        return _enable;
    }

    // force disable the airspeed sensor
    void        disable(void) {
        _enable.set(0);
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

    // return the current raw pressure
    float get_raw_pressure(void) const {
        return _raw_pressure;
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
    void update_calibration(const Vector3f &vground);

	// log data to MAVLink
	void log_mavlink_send(mavlink_channel_t chan, const Vector3f &vground);

    // return health status of sensor
    bool healthy(void) const { return _healthy; }

    void setHIL(float pressure) { _healthy=_hil_set=true; _hil_pressure=pressure; };

    // return time in ms of last update
    uint32_t last_update_ms(void) const { return _last_update_ms; }

    void setHIL(float airspeed, float diff_pressure, float temperature);

    static const struct AP_Param::GroupInfo var_info[];

    enum pitot_tube_order { PITOT_TUBE_ORDER_POSITIVE =0, 
                            PITOT_TUBE_ORDER_NEGATIVE =1, 
                            PITOT_TUBE_ORDER_AUTO     =2};

private:
    AP_Float        _offset;
    AP_Float        _ratio;
    AP_Int8         _use;
    AP_Int8         _enable;
    AP_Int8         _pin;
    AP_Int8         _autocal;
    AP_Int8         _tube_order;
    AP_Int8         _skip_cal;
    float           _raw_airspeed;
    float           _airspeed;
    float			_last_pressure;
    float			_raw_pressure;
    float           _EAS2TAS;
    bool		    _healthy:1;
    bool		    _hil_set:1;
    float           _hil_pressure;
    uint32_t        _last_update_ms;

    Airspeed_Calibration _calibration;
    float _last_saved_ratio;
    uint8_t _counter;

    float get_pressure(void);

    AP_Airspeed_Analog analog;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_Airspeed_PX4    digital;
#else
    AP_Airspeed_I2C    digital;
#endif
};

// the virtual pin for digital airspeed sensors
#define AP_AIRSPEED_I2C_PIN 65

#endif // __AP_AIRSPEED_H__

