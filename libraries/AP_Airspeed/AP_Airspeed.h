/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AIRSPEED_H__
#define __AP_AIRSPEED_H__

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AnalogSource.h>
#include <Filter.h>
#include <AverageFilter.h>

class AP_Airspeed
{
public:
    // constructor
    AP_Airspeed(AP_AnalogSource *source) {
        _source = source;
    }

    // read the analog source and update _airspeed
    void        read(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    // the callback is a delay() like routine
    void            calibrate(void (*callback)(unsigned long t));

    // return the current airspeed in m/s
    float           get_airspeed(void) {
        return _airspeed;
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

    // used by HIL to set the airspeed
    void        set_HIL(float airspeed) {
        _airspeed = airspeed;
    }

    static const struct AP_Param::GroupInfo         var_info[];

private:
    AP_AnalogSource *                               _source;
    AP_Float        _offset;
    AP_Float        _ratio;
    AP_Int8         _use;
    AP_Int8         _enable;
    float           _airspeed;
    float           _airspeed_raw;
};


#endif // __AP_AIRSPEED_H__
