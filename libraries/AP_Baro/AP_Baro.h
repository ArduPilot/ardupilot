/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_H__
#define __AP_BARO_H__

#include <AP_Param.h>
#include <Filter.h>
#include <DerivativeFilter.h>

class AP_Baro
{
public:
    bool                    healthy;

    AP_Baro() {
		AP_Param::setup_object_defaults(this, var_info);
    }

    virtual bool            init()= 0;
    virtual uint8_t         read() = 0;
    virtual float           get_pressure() const = 0;
    virtual float           get_temperature() const = 0;

    virtual int32_t         get_raw_pressure() const = 0;
    virtual int32_t         get_raw_temp() const = 0;

    // accumulate a reading - overridden in some drivers
    virtual void            accumulate(void) {}

    // calibrate the barometer. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    // the callback is a delay() like routine
    void        calibrate();

    // get current altitude in meters relative to altitude at the time
    // of the last calibrate() call
    float        get_altitude(void) const;

    // get scale factor required to convert equivalent to true airspeed
    float        get_EAS2TAS(void) const;

    // return how many pressure samples were used to obtain
    // the last pressure reading
    uint8_t        get_pressure_samples(void) const {
        return _pressure_samples;
    }

    // get current climb rate in meters/s. A positive number means
    // going up
    float           get_climb_rate(void) const;

    // the ground values are only valid after calibration
    float           get_ground_temperature(void) const {
        return _ground_temperature.get();
    }
    float           get_ground_pressure(void) const {
        return _ground_pressure.get();
    }

    // get last time sample was taken (in ms)
    uint32_t        get_last_update() const { return _last_update; };

    static const struct AP_Param::GroupInfo        var_info[];

protected:
    uint32_t                            _last_update; // in ms
    uint8_t                             _pressure_samples;

private:
    AP_Float                            _ground_temperature;
    AP_Float                            _ground_pressure;
    AP_Int8                             _alt_offset;
    mutable float                       _altitude;              // These values are used for caching results and internal
    mutable float                       _last_altitude_EAS2TAS; // calculations that don't change the object's visible state.
    mutable float                       _EAS2TAS;               // Therefore they are declared mutable to allow get_EAS2TAS()
    mutable uint32_t                    _last_altitude_t;       // and get_altitude() to have a const specifier, as one would
    mutable DerivativeFilterFloat_Size7 _climb_rate_filter;     // expect from getters.
};

#include "AP_Baro_MS5611.h"
#include "AP_Baro_BMP085.h"
#include "AP_Baro_HIL.h"
#include "AP_Baro_PX4.h"

#endif // __AP_BARO_H__
