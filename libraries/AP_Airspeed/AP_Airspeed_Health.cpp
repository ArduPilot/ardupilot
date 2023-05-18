#include "AP_Airspeed.h"

#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

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
#ifndef HAL_BUILD_AP_PERIPH
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - state[i].failures.last_check_ms) <= 200) {
        // slow the checking rate
        return;
    }
    state[i].failures.last_check_ms = now_ms;

    if (!is_positive(_wind_max) && !is_positive(_wind_gate)) {
        // nothing to do
        return;
    }

    if (state[i].airspeed <= 0) {
        // invalid estimate
        return;
    }

    const AP_GPS &gps = AP::gps();
    if (gps.status() < AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
        // GPS speed can't be trusted, re-enable airspeed as a fallback
        if ((param[i].use == 0) && (state[i].failures.param_use_backup == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Airspeed sensor %d, Re-enabled as GPS fall-back", i+1);
            param[i].use.set_and_notify(state[i].failures.param_use_backup); 
            state[i].failures.param_use_backup = -1;
        }
        return;
    }

    // check for airspeed consistent with wind and vehicle velocity using the EKF
    uint32_t age_ms;
    float innovation, innovationVariance;
    if (AP::ahrs().airspeed_health_data(innovation, innovationVariance, age_ms) && age_ms < 1000 && is_positive(innovationVariance)) {
        state[i].failures.test_ratio = fabsf(innovation) / safe_sqrt(innovationVariance);
    } else {
        state[i].failures.test_ratio = 0.0f;
    }
    bool data_is_inconsistent = false;
    if (is_positive(_wind_gate) && (AP_Airspeed::OptionsMask::USE_EKF_CONSISTENCY & _options) != 0) {
        float gate_size = MAX(_wind_gate, 0.0f);
        if (param[i].use == 0) {
            // require a smaller inconsistency for a disabled sensor to be declared consistent
            gate_size *= 0.7f;
        }
        data_is_inconsistent = state[i].failures.test_ratio > gate_size;
    }

    const float speed_diff = fabsf(state[i].airspeed-gps.ground_speed());
    const bool data_is_implausible = is_positive(_wind_max) && speed_diff > _wind_max;
    // update health_probability with LowPassFilter
    if (data_is_implausible || data_is_inconsistent) {
        // bad, decay fast
        const float probability_coeff = 0.90f;
        state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability;

    } else if (!data_is_implausible && !data_is_inconsistent) {
        // good, grow slow
        const float probability_coeff = 0.98f;
        state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability + (1.0f-probability_coeff)*1.0f;
    }

    // Now check if we need to disable or enable the sensor

    // here are some probability thresholds
    static const float DISABLE_PROB_THRESH_CRIT = 0.1f;
    static const float RE_ENABLE_PROB_THRESH_OK = 0.95f;

    if (param[i].use > 0) {
        if (((AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE & _options) != 0) &&
                (state[i].failures.health_probability < DISABLE_PROB_THRESH_CRIT)) {
            // if "disable" option is allowed and sensor is enabled and is probably not healthy
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed sensor %d failure. Disabling", i+1);
            state[i].failures.param_use_backup = param[i].use;
            param[i].use.set_and_notify(0);
            state[i].healthy = false;
        }

        // Warn if necessary

        // set warn to max if not set or larger than max
        float wind_warn = _wind_warn;
        if ((!is_positive(wind_warn) || (wind_warn > _wind_max)) && is_positive(_wind_max)) {
            wind_warn = _wind_max;
        }

        if (is_positive(wind_warn) && (speed_diff > wind_warn) && ((now_ms - state[i].failures.last_warn_ms) > 15000)) {
            state[i].failures.last_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Airspeed %d warning %0.1fm/s air to gnd speed diff", i+1, speed_diff);
        }

    // if Re-Enable options is allowed, and sensor is disabled but was previously enabled, and is probably healthy
    } else if (((AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE & _options) != 0) &&
                (state[i].failures.param_use_backup > 0) && 
                (state[i].failures.health_probability > RE_ENABLE_PROB_THRESH_OK)) {

        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Airspeed sensor %d now OK. Re-enabled", i+1);
        param[i].use.set_and_notify(state[i].failures.param_use_backup); // resume
        state[i].failures.param_use_backup = -1; // set to invalid so we don't use it
    }
#endif // HAL_BUILD_AP_PERIPH
}
