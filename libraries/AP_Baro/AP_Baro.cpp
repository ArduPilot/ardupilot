/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       APM_Baro.cpp - barometer driver
 *
 */

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Baro::var_info[] PROGMEM = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

    // @Param: ABS_PRESS
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Increment: 1
    AP_GROUPINFO("ABS_PRESS", 2, AP_Baro, _ground_pressure, 0),

    // @Param: TEMP
    // @DisplayName: ground temperature
    // @Description: calibrated ground temperature in degrees Celsius
    // @Increment: 1
    AP_GROUPINFO("TEMP", 3, AP_Baro, _ground_temperature, 0),

    // @Param: ALT_OFFSET
    // @DisplayName: altitude offset
    // @Description: altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
    // @Units: meters
    // @Range: -128 127
    // @Increment: 1
    AP_GROUPINFO("ALT_OFFSET", 4, AP_Baro, _alt_offset, 0),

    AP_GROUPEND
};

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Baro::calibrate()
{
    float ground_pressure = 0;
    float ground_temperature = 0;

    // reset the altitude offset when we calibrate. The altitude
    // offset is supposed to be for within a flight
    _alt_offset.set_and_save(0);

    {
        uint32_t tstart = hal.scheduler->millis();
        while (ground_pressure == 0 || !healthy) {
            read();         // Get initial data from absolute pressure sensor
            if (hal.scheduler->millis() - tstart > 500) {
                hal.scheduler->panic(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [1]\r\n"));
            }
            ground_pressure         = get_pressure();
            ground_temperature      = get_temperature();
            hal.scheduler->delay(20);
        }
    }
    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint8_t i = 0; i < 10; i++) {
        uint32_t tstart = hal.scheduler->millis();
        do {
            read();
            if (hal.scheduler->millis() - tstart > 500) {
                hal.scheduler->panic(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [2]\r\n"));
            }
        } while (!healthy);
        ground_pressure     = get_pressure();
        ground_temperature  = get_temperature();

        hal.scheduler->delay(100);
    }

    // now average over 5 values for the ground pressure and
    // temperature settings
    for (uint16_t i = 0; i < 5; i++) {
        uint32_t tstart = hal.scheduler->millis();
        do {
            read();
            if (hal.scheduler->millis() - tstart > 500) {
                hal.scheduler->panic(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [3]\r\n"));
            }
        } while (!healthy);
        ground_pressure = (ground_pressure * 0.8f) + (get_pressure() * 0.2f);
        ground_temperature = (ground_temperature * 0.8f) + 
            (get_temperature() * 0.2f);

        hal.scheduler->delay(100);
    }

    _ground_pressure.set_and_save(ground_pressure);
    _ground_temperature.set_and_save(ground_temperature);
}

/**
   update the barometer calibration
   this updates the baro ground calibration to the current values. It
   can be used before arming to keep the baro well calibrated
*/
void AP_Baro::update_calibration()
{
    _ground_pressure.set(get_pressure());
    _ground_temperature.set(get_temperature());
}

// return current altitude estimate relative to time that calibrate()
// was called. Returns altitude in meters
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_altitude(void)
{
    float scaling, temp;

    if (_last_altitude_t == _last_update) {
        // no new information
        return _altitude + _alt_offset;
    }


#if HAL_CPU_CLASS <= HAL_CPU_CLASS_16
    // on slower CPUs use a less exact, but faster, calculation
    scaling                                 = (float)_ground_pressure / (float)get_pressure();
    temp                                    = ((float)_ground_temperature) + 273.15f;
    _altitude = logf(scaling) * temp * 29.271267f;
#else
    // on faster CPUs use a more exact calculation
    scaling                                 = (float)get_pressure() / (float)_ground_pressure;
    temp                                    = ((float)_ground_temperature) + 273.15f;

    // This is an exact calculation that is within +-2.5m of the standard atmosphere tables
    // in the troposphere (up to 11,000 m amsl).
	_altitude = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));
#endif

    _last_altitude_t = _last_update;

    // ensure the climb rate filter is updated
    _climb_rate_filter.update(_altitude, _last_update);

    return _altitude + _alt_offset;
}

// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
float AP_Baro::get_EAS2TAS(void)
{
    if ((fabs(_altitude - _last_altitude_EAS2TAS) < 100.0f) && (_EAS2TAS != 0.0f)) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }

    float tempK = ((float)_ground_temperature) + 273.15f - 0.0065f * _altitude;
    _EAS2TAS = safe_sqrt(1.225f / ((float)get_pressure() / (287.26f * tempK)));
    _last_altitude_EAS2TAS = _altitude;
    return _EAS2TAS;
}

// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
    return _climb_rate_filter.slope() * 1.0e3f;
}

