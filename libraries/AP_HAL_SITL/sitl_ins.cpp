/*
  SITL handling

  This emulates the ADS7844 ADC

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"

#include <AP_Math/AP_Math.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <SITL/SITL.h>
#include "Scheduler.h"
#include <AP_Math/AP_Math.h>
#include <AP_ADC/AP_ADC.h>
#include "SITL_State.h"
#include <fenv.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

/*
  convert airspeed in m/s to an airspeed sensor value
 */
uint16_t SITL_State::_airspeed_sensor(float airspeed)
{
    const float airspeed_ratio = 1.9936f;
    const float airspeed_offset = 2013;
    float airspeed_pressure, airspeed_raw;

    airspeed_pressure = (airspeed*airspeed) / airspeed_ratio;
    airspeed_raw = airspeed_pressure + airspeed_offset;
    if (airspeed_raw/4 > 0xFFFF) {
        return 0xFFFF;
    }
    // add delay
    uint32_t now = AP_HAL::millis();
    uint32_t best_time_delta_wind = 200; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index_wind = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time_wind >= 10) { // store data every 10 ms.
        last_store_time_wind = now;
        if (store_index_wind > wind_buffer_length-1) { // reset buffer index if index greater than size of buffer
            store_index_wind = 0;
        }
        buffer_wind[store_index_wind].data = airspeed_raw; // add data to current index
        buffer_wind[store_index_wind].time = last_store_time_wind; // add time to current index
        store_index_wind = store_index_wind + 1; // increment index
    }

    // return delayed measurement
    delayed_time_wind = now - _sitl->wind_delay; // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=wind_buffer_length-1; i++) {
        // find difference between delayed time and time stamp in buffer
        time_delta_wind = abs(
                (int32_t)(delayed_time_wind - buffer_wind[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta_wind < best_time_delta_wind) {
            best_index_wind = i;
            best_time_delta_wind = time_delta_wind;
        }
    }
    if (best_time_delta_wind < 200) { // only output stored state if < 200 msec retrieval error
        airspeed_raw = buffer_wind[best_index_wind].data;
    }

    return airspeed_raw/4;
}


/*
  emulate an analog rangefinder
 */
uint16_t SITL_State::_ground_sonar(void)
{
    float altitude = _sitl->height_agl;

    // sensor position offset in body frame
    Vector3f relPosSensorBF = _sitl->rngfnd_pos_offset;

    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    if (!relPosSensorBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        _sitl->state.quaternion.rotation_matrix(rotmat);

        // rotate the offset into earth frame
        Vector3f relPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= relPosSensorEF.z;
    }

    float voltage = 5.0f;
    if (fabs(_sitl->state.rollDeg) < 90 &&
        fabs(_sitl->state.pitchDeg) < 90) {
        // adjust for apparent altitude with roll
        altitude /= cosf(radians(_sitl->state.rollDeg)) * cosf(radians(_sitl->state.pitchDeg));

        altitude += _sitl->sonar_noise * _rand_float();

        // Altitude in in m, scaler in meters/volt
        voltage = altitude / _sitl->sonar_scale;
        voltage = constrain_float(voltage, 0, 5.0f);

        if (_sitl->sonar_glitch >= (_rand_float() + 1.0f)/2.0f) {
            voltage = 5.0f;
        }
    }

    return 1023*(voltage / 5.0f);
}

/*
  setup the INS input channels with new input
 */
void SITL_State::_update_ins(float airspeed)
{
    if (_ins == nullptr) {
        // no inertial sensor in this sketch
        return;
    }

    sonar_pin_value    = _ground_sonar();
    float airspeed_simulated = (fabsf(_sitl->arspd_fail) > 1.0e-6f) ? _sitl->arspd_fail : airspeed;
    airspeed_pin_value = _airspeed_sensor(airspeed_simulated + (_sitl->arspd_noise * _rand_float()));
}

#endif
