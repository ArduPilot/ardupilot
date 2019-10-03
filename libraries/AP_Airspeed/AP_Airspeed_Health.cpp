#include "AP_Airspeed.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

void AP_Airspeed::check_sensor_failures()
{
#ifndef HAL_BUILD_AP_PERIPH
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        check_sensor_ahrs_wind_max_failures(i);
    }
#endif
}

void AP_Airspeed::check_sensor_ahrs_wind_max_failures(uint8_t i)
{
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - state[i].failures.last_check_ms) <= 200) {
        // slow the checking rate
        return;
    }

    const float aspeed = get_airspeed();
    const float wind_max = AP::ahrs().get_max_wind();

    if (aspeed <= 0 || wind_max <= 0) {
        // invalid estimates
        return;
    }

    state[i].failures.last_check_ms = now_ms;

    // update state[i].failures.health_probability via LowPassFilter
    float speed_accuracy;
    const AP_GPS &gps = AP::gps();
    if (gps.speed_accuracy(speed_accuracy)) {
        const float gnd_speed = gps.ground_speed();

        if (aspeed > (gnd_speed + wind_max) || aspeed < (gnd_speed - wind_max)) {
            // bad, decay fast
            const float probability_coeff = 0.90f;
            //state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability + (1.0f-probability_coeff)*0.0f;
            state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability; // equivalent

        } else if (aspeed < (gnd_speed + wind_max) && aspeed > (gnd_speed - wind_max)) {
            // good, grow slow
            const float probability_coeff = 0.98f;
            state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability + (1.0f-probability_coeff)*1.0f;
        }
    }


    // Now check if we need to disable or enable the sensor

    // here are some probability thresholds
    static const float DISABLE_PROB_THRESH_CRIT = 0.1f;
    static const float DISABLE_PROB_THRESH_WARN = 0.5f;
    static const float RE_ENABLE_PROB_THRESH_OK = 0.95f;

    // if "disable" option is allowed and sensor is enabled
    if (param[i].use > 0 && (AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE & _options)) {
        // and is probably not healthy
        if (state[i].failures.health_probability < DISABLE_PROB_THRESH_CRIT) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Airspeed sensor %d failure. Disabling", i+1);
            state[i].failures.param_use_backup = param[i].use;
            param[i].use.set_and_notify(0);
            state[i].healthy = false;

        // and is probably getting close to not healthy
        } else if ((state[i].failures.health_probability < DISABLE_PROB_THRESH_WARN) && !state[i].failures.has_warned) {
            state[i].failures.has_warned = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "Airspeed sensor %d warning", i+1);

        } else if (state[i].failures.health_probability > RE_ENABLE_PROB_THRESH_OK) {
            state[i].failures.has_warned = false;
        }

    // if Re-Enable options is allowed, and sensor is disabled but was previously enabled, and is probably healthy
    } else if (param[i].use == 0 &&
        (AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE & _options) &&
        state[i].failures.param_use_backup > 0 &&
        state[i].failures.health_probability > RE_ENABLE_PROB_THRESH_OK) {

        gcs().send_text(MAV_SEVERITY_NOTICE, "Airspeed sensor %d now OK. Re-enabled", i+1);
        param[i].use.set_and_notify(state[i].failures.param_use_backup); // resume
        state[i].failures.param_use_backup = -1; // set to invalid so we don't use it
        state[i].failures.has_warned = false;
    }
}
