/*
  SITL handling

  This simulates an analog airspeed sensor

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "SITL_State.h"
#include <SITL/SITL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
static float get_EAS2TAS(float altitude)
{
    float pressure = AP::baro().get_pressure();
    if (is_zero(pressure)) {
        return 1.0f;
    }

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(altitude * 0.001, sigma, delta, theta);

    float tempK = C_TO_KELVIN(25) - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / (pressure / (ISA_GAS_CONSTANT * tempK));
    if (!is_positive(eas2tas_squared)) {
        return 1.0;
    }
    return sqrtf(eas2tas_squared);
}

/*
  convert airspeed in m/s to an airspeed sensor value
 */
void SITL_State::_update_airspeed(float true_airspeed)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        const auto &arspd = _sitl->airspeed[i];
        float airspeed = true_airspeed / get_EAS2TAS(_sitl->state.altitude);
        const float diff_pressure = sq(airspeed) / arspd.ratio;
        float airspeed_raw;
    
        // apply noise to the differential pressure. This emulates the way
        // airspeed noise reduces with speed
        airspeed = sqrtf(fabsf(arspd.ratio*(diff_pressure + arspd.noise * rand_float())));

        // check sensor failure
        if (is_positive(arspd.fail)) {
            airspeed = arspd.fail;
        }

        if (!is_zero(arspd.fail_pressure)) {
            // compute a realistic pressure report given some level of trapper air pressure in the tube and our current altitude
            // algorithm taken from https://en.wikipedia.org/wiki/Calibrated_airspeed#Calculation_from_impact_pressure
            float tube_pressure = fabsf(arspd.fail_pressure - AP::baro().get_pressure() + arspd.fail_pitot_pressure);
            airspeed = 340.29409348 * sqrt(5 * (pow((tube_pressure / SSL_AIR_PRESSURE + 1), 2.0/7.0) - 1.0));
        }
        float airspeed_pressure = (airspeed * airspeed) / arspd.ratio;

        // flip sign here for simulating reversed pitot/static connections
        if (arspd.signflip) {
            airspeed_pressure *= -1;
        }

        // apply airspeed sensor offset in m/s
        airspeed_raw = airspeed_pressure + arspd.offset;

        _sitl->state.airspeed_raw_pressure[i] = airspeed_pressure;

        airspeed_pin_value[i] = MIN(0xFFFF, airspeed_raw / 4);
    }
}

#endif
