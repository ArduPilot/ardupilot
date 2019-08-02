#include "AP_Airspeed.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

void AP_Airspeed::check_all_sensor_failures()
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        update_error_estimate(i);
    }
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        check_sensor_failures(i);
    }

}
void AP_Airspeed::update_error_estimate(uint8_t i)
{
    //very crude function for how error bars scale with speed
    if(state[i].airspeed<10.0f){
        state[i].error_pos =2.5f;
        state[i].error_neg =2.5f;
    }
    else{
        state[i].error_pos = 1.0f;
        state[i].error_neg = 1.0f;
    }

}

void AP_Airspeed::check_sensor_failures(uint8_t i)
{
const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - state[i].failures.last_check_ms) <= 100) {
        // slow the checking rate
        return;
    }


    state[i].failures.last_check_ms = now_ms;


    check_sensor_ahrs_wind_max_failures(i);
    check_sensor_values_consistent(i);


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

void AP_Airspeed::check_sensor_values_consistent(uint8_t i)
{
    //counter for number of other sensors that this sensor is consistent with, within the sensor error
     int8_t good_bad_count=0;

    // if this sensor is enabled
    if(state[i].failures.param_use_backup <= 0 && param[i].use <=0)
    {
        return;
    }
    // check against all the other sensors
    for (uint8_t j=0; j<AIRSPEED_MAX_SENSORS; j++) {
        if(j != i){
            //if the other sensor is enabled
            if(state[j].failures.param_use_backup > 0 ||param[j].use >0)
            {
                //check if error bars overlap
                if(state[i].airspeed +state[i].error_pos <= state[j].airspeed -state[j].error_neg || state[j].airspeed +state[j].error_pos <= state[i].airspeed -state[i].error_neg)
                {
                    good_bad_count--;
                }
                else
                {
                    good_bad_count++;
                }
            }
        
        }
    }
    if(good_bad_count<0){
         decay_health(i);
    }
    else{
        grow_health(i);
    }
    
}


void AP_Airspeed::decay_health(uint8_t i)
{
    // bad, decay fast
    const float probability_coeff = 0.90f;
    state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability; 
}

void AP_Airspeed::grow_health(uint8_t i)
{
    // good, grow slow
    const float probability_coeff = 0.98f;
    state[i].failures.health_probability = probability_coeff*state[i].failures.health_probability + (1.0f-probability_coeff)*1.0f;
    
}

void AP_Airspeed::check_sensor_ahrs_wind_max_failures(uint8_t i)
{

     const float wind_max = AP::ahrs().get_max_wind();
    if (state[i].airspeed <= 0 || wind_max <= 0) {
        // invalid estimates
        return;
    }
    // update state[i].failures.health_probability via LowPassFilter
    float speed_accuracy;
    const AP_GPS &gps = AP::gps();
    if (gps.speed_accuracy(speed_accuracy)) {
        const float gnd_speed = gps.ground_speed();

        if (state[i].airspeed > (gnd_speed + wind_max) || state[i].airspeed < (gnd_speed - wind_max)) {
            decay_health(i);

        } else if (state[i].airspeed < (gnd_speed + wind_max) && state[i].airspeed > (gnd_speed - wind_max)) {
            grow_health(i);
        }
    }
}
