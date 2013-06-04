/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AIRSPEED_H__
#define __AP_AIRSPEED_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Param.h>

class AP_Airspeed
{
public:
    // constructor
    AP_Airspeed() : _ets_fd(-1) {
		AP_Param::setup_object_defaults(this, var_info);
    };

    void init(void);

    // read the analog source and update _airspeed
    void        read(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void            calibrate();

    // return the current airspeed in m/s
    float           get_airspeed(void) {
        return _airspeed;
    }

    // return the unfiltered airspeed in m/s
    float           get_raw_airspeed(void) {
        return _raw_airspeed;
    }

    // return the current airspeed in cm/s
    float        get_airspeed_cm(void) {
        return _airspeed*100;
    }

    // return true if airspeed is enabled, and airspeed use is set
    bool        use(void) {
        return _enable && _use && _offset != 0;
    }

    // return true if airspeed is enabled
    bool        enabled(void) {
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

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::AnalogSource *_source;
    AP_Float        _offset;
    AP_Float        _ratio;
    AP_Int8         _use;
    AP_Int8         _enable;
    AP_Int8         _pin;
    float           _raw_airspeed;
    float           _airspeed;
    int			    _ets_fd;
    float			_last_pressure;

    // return raw differential pressure in Pascal
    float get_pressure(void);
};


#endif // __AP_AIRSPEED_H__
