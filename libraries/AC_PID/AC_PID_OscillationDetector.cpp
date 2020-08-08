
#include "AC_PID_OscillationDetector.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>

/*
 Oscillation Detector using simple filters
*/

// Oscillation params
#define PIDff_ratio_threshold 0.5f // has to larger than 0.5, otherwise more than one value could be above the threshold
#define PIDff_min_scale 0.5f // the minmum scale factor
#define ratio_reduce 0.9985f // 461 calls to get to min scale
#define ratio_reduce_all 0.999f // 692 calls
#define ratio_increase 1.0003f // 2311 calls to return from min scale to 1
#define overshoot_ratio 1.2f // Constrain to this ratio above the oscillation_threshold to stop wind up, must be > 1

// Quick Tune params
#define check_count 4000
#define tune_max_scale 10.0f

// Constructor
AC_PID_OscillationDetector::AC_PID_OscillationDetector(AP_Logger::PID_Info &_pid, AP_Float &_kP, AP_Float &_kI, AP_Float &_kD, AP_Float &_kff, float mag_threshold, float threshold, float filter, float tune_PI_rato) :
    pid(_pid),
    kP(_kP),
    kI(_kI),
    kD(_kD),
    kFF(_kff),
    PI_ratio(tune_PI_rato)
{
    // must be larger than 0 to avoid FPE
    magnitude_threshold = fmaxf(mag_threshold, FLT_EPSILON);

    // must be larger than one to avoid constant triggering
    oscillation_threshold = fmaxf(threshold,1.0f);

    // main filter and envelope filter values
    w = constrain_float(filter,0.0f,1.0f);
    w_env = w*0.1f;
}

void AC_PID_OscillationDetector::update()
{
    // apply filter to P, I, D, actual and target
    P_filt = P_filt * (1-w) + pid.P * w;
    I_filt = I_filt * (1-w) + pid.I * w;
    D_filt = D_filt * (1-w) + pid.D * w;
    ff_filt = ff_filt * (1-w) + pid.FF * w;
    target_filt = target_filt * (1-w) + pid.target * w;
    actual_filt = actual_filt * (1-w) + pid.actual * w;

    // Caulate the filtered envelope of the difference between the filter vale and actual
    P_env = P_env * (1-w_env) + abs(pid.P - P_filt) * w_env;
    I_env = I_env * (1-w_env) + abs(pid.I - I_filt) * w_env;
    D_env = D_env * (1-w_env) + abs(pid.D - D_filt) * w_env;
    ff_env = ff_env * (1-w_env) + abs(pid.FF - ff_filt) * w_env;
    target_env = target_env * (1-w_env) + abs(pid.target - target_filt) * w_env;
    actual_env = actual_env * (1-w_env) + abs(pid.actual - actual_filt) * w_env;

    // Constrain actual to be the threshold level multiplied by the overshoot ratio at most, this reduces the 'reset' time
    actual_env = fminf(actual_env,target_env*oscillation_threshold*overshoot_ratio);

    // is there is a large enough magnitude to reliably calculate the ratio of oscillations
    float amplitude_ratio = 0.0f;
    if (!is_zero(target_env) && ((target_env > magnitude_threshold) || (actual_env > magnitude_threshold))) {
        amplitude_ratio = actual_env/target_env;
    }
    pid.amplitude_ratio = amplitude_ratio;

    const AP_Vehicle *vehicle = AP::vehicle();
    if (!scaling_enable || !vehicle->get_likely_flying()) {
        // report only
        if (action != NONE) {
            action = NONE;
            reset_scale_factors();
        }
        return;
    }

    bool oscillating = amplitude_ratio > oscillation_threshold;

    if (tune_state == TUNE_STATE::NONE) {
        float total_env = P_env + I_env + D_env + ff_env;
        if (oscillating & is_positive(total_env)) {
            // oscillation detected
            action = DECREASE;
            total_env = 1/total_env;
            // see if we can tell which term is causing it
            if ((P_env*total_env > PIDff_ratio_threshold) && (pid.Pmod > PIDff_min_scale)) {
                pid.Pmod *= ratio_reduce;

            } else if ((I_env*total_env > PIDff_ratio_threshold) && (pid.Imod > PIDff_min_scale)) {
                pid.Imod *= ratio_reduce;

            } else if ((D_env*total_env > PIDff_ratio_threshold) && (pid.Dmod > PIDff_min_scale)) {
                pid.Dmod *= ratio_reduce;

            } else if ((ff_env*total_env > PIDff_ratio_threshold) && (pid.FFmod > PIDff_min_scale)) {
                pid.FFmod *= ratio_reduce;

            } else {
                // If we cant tell or already reduced a single gain by 50% then reduce all
                pid.Pmod *= ratio_reduce_all;
                pid.Imod *= ratio_reduce_all;
                pid.Dmod *= ratio_reduce_all;
                pid.FFmod *= ratio_reduce_all;
            }

            // Dont lower too far
            pid.Pmod = fmaxf(pid.Pmod, PIDff_min_scale);
            pid.Imod = fmaxf(pid.Imod, PIDff_min_scale);
            pid.Dmod = fmaxf(pid.Dmod, PIDff_min_scale);
            pid.FFmod = fmaxf(pid.FFmod, PIDff_min_scale);

        } else if (action != NONE) {
            // Decay gains back up to 1
            pid.Pmod *= ratio_increase;
            pid.Imod *= ratio_increase;
            pid.Dmod *= ratio_increase;
            pid.FFmod *= ratio_increase;

            if ((pid.Pmod >= 1.0f) &&
                (pid.Imod >= 1.0f) &&
                (pid.Dmod >= 1.0f) &&
                (pid.FFmod >= 1.0f)) {
                // all back to 1
                action = NONE;
            } else {
                action = INCREASE;
            }

            // max of 1
            pid.Pmod = fminf(pid.Pmod, 1.0f);
            pid.Imod = fminf(pid.Imod, 1.0f);
            pid.Dmod = fminf(pid.Dmod, 1.0f);
            pid.FFmod = fminf(pid.FFmod, 1.0f);
        }
    }

    if (tune_state == TUNE_STATE::NONE || is_zero(amplitude_ratio)) {
        // not good enough reading to autotune
        return;
    }

    // auto tune, target a 1:1 amplitude ratio
    oscillating = amplitude_ratio > 1.0f;

    pid.Imod = 0.0f; // I is not tunned
    switch (tune_state) {
        case TUNE_STATE::FF: {
            // Target amplitude ratio of 1 using feedforward only, use filter to stop gain rising too fast
            pid.Pmod = 0.0f;
            pid.Dmod = 0.0f;
            pid.FFmod = pid.FFmod * (1-w_env) + pid.FFmod * (1.0f/amplitude_ratio) * w_env;
            tune_count++;
            if (pid.FFmod < PIDff_min_scale) {
                pid.FFmod = PIDff_min_scale;
                tune_state = TUNE_STATE::FAILED;
                gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale FF less than %0.0f%%",PIDff_min_scale*100);
                break;
            } else if (pid.FFmod > tune_max_scale) {
                pid.FFmod = tune_max_scale;
                tune_state = TUNE_STATE::FAILED;
                gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale FF more than %0.0fx",tune_max_scale);
                break;
            }
            if (tune_count > check_count) {
                // FF done, move on to P
                tune_count = 0;
                tune_state = TUNE_STATE::P1;
                pid.Pmod = 0.8f;
                gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: FF done");
            }
            break;
        }
        case TUNE_STATE::P1: {
            // Increase P as much as possible with no D
            pid.Dmod = 0.0f;
            if (!oscillating && !has_ossialated) {
                // slow increase to prevent sudden bad oscillation
                pid.Pmod *= 1.00005;
            } else {
                if (!has_ossialated) {
                    // initially back of to 80%
                    has_ossialated = true;
                    pid.Pmod *= 0.8f;
                } else {
                    // back of more if oscillations continue
                    pid.Pmod *= 0.90f;
                }
                if (pid.Pmod < PIDff_min_scale) {
                    pid.Pmod = PIDff_min_scale;
                    tune_state = TUNE_STATE::FAILED;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale P less than %0.0f%%",PIDff_min_scale*100);
                    break;
                } else if (pid.Pmod > tune_max_scale) {
                    pid.Pmod = tune_max_scale;
                    tune_state = TUNE_STATE::FAILED;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale P more than %0.0fx",tune_max_scale);
                    break;
                }
                last_tune_state = tune_state;
                tune_state = TUNE_STATE::CHECK;
                // reset the actual_env this has the effect of waiting to see if the oscillation decays
                actual_env = fminf(actual_env,target_env*0.8f);
            }
            break;
        }
        case TUNE_STATE::D: {
            // Increase D
            if (!oscillating && !has_ossialated) {
                pid.Dmod *= 1.0001;
            } else {
                if (!has_ossialated) {
                    has_ossialated = true;
                    pid.Dmod *= 0.8f;
                } else {
                    pid.Dmod *= 0.95f;
                }
                if (pid.Dmod < 0) {
                    pid.Dmod = 0;
                    tune_state = TUNE_STATE::FAILED;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale D less than 0");
                    break;
                } else if (pid.Dmod > tune_max_scale) {
                    pid.Dmod = tune_max_scale;
                    tune_state = TUNE_STATE::FAILED;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale D more than %0.0fx",tune_max_scale);
                    break;
                }
                last_tune_state = tune_state;
                tune_state = TUNE_STATE::CHECK;
                actual_env = fminf(actual_env,target_env*0.8f);
            }
            break;
        }
        case TUNE_STATE::P2: {
            // re-do P with none-zero D
            if (!oscillating && !has_ossialated) {
                pid.Pmod *= 1.00005;
            } else {
                if (!has_ossialated) {
                    has_ossialated = true;
                    pid.Pmod *= 0.8f;
                } else {
                    pid.Pmod *= 0.95f;
                }
                if (pid.Pmod < PIDff_min_scale) {
                    pid.Pmod = PIDff_min_scale;
                    tune_state = TUNE_STATE::FAILED;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale P less than 50%%");
                    break;
                } else if (pid.Pmod > tune_max_scale) {
                    pid.Pmod = tune_max_scale;
                    tune_state = TUNE_STATE::FAILED;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Failed, cannot scale P more than %0.0fx",tune_max_scale);
                    break;
                }
                if (pid.Pmod < P_only) {
                    tune_state = TUNE_STATE::P1;
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Re-doing D tune");
                    break;
                }
                last_tune_state = tune_state;
                tune_state = TUNE_STATE::CHECK;
                actual_env = fminf(actual_env,target_env*0.8f);
            }
            break;
        }
        case TUNE_STATE::CHECK: {
            if (!oscillating) {
                tune_count++;
            } else {
                tune_count = 0;
                tune_state = last_tune_state;
                break;
            }
            if (tune_count > check_count) {
                // no oscillation for set count, move onto next stage
                if (last_tune_state == TUNE_STATE::P1) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: P done");
                    P_only = pid.Pmod;
                    pid.Dmod = 0.8f;
                    tune_state = TUNE_STATE::D;
                } else if (last_tune_state == TUNE_STATE::D) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: D done");
                    tune_state = TUNE_STATE::P2;
                } else if (last_tune_state == TUNE_STATE::P2) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: Complete");
                    tune_state = TUNE_STATE::COMPLETE;
                }
                tune_count = 0;
                has_ossialated = false;
            }
            break;
        }
        case TUNE_STATE::COMPLETE:
        case TUNE_STATE::NONE:
        case TUNE_STATE::FAILED:
            break;
    }
}

bool AC_PID_OscillationDetector::start_quick_tune()
{
    if (!scaling_enable) {
        return false;
    }
    tune_count = 0;
    has_ossialated = false;
    if (!is_zero(kFF.get())) {
        // some feed forward so do a feed forward tune
        tune_state = TUNE_STATE::FF;
    } else {
        // no feed forward tune
        tune_state = TUNE_STATE::P1;
    }
    return true;
}

void AC_PID_OscillationDetector::stop_quick_tune()
{
    tune_state = TUNE_STATE::NONE;
    reset_scale_factors();
}

// reset gain scale factors to 1
void AC_PID_OscillationDetector::reset_scale_factors()
{
    pid.Pmod = 1.0f;
    pid.Imod = 1.0f;
    pid.Dmod = 1.0f;
    pid.FFmod = 1.0f;
}

// save gains to params, return values
bool AC_PID_OscillationDetector::save_PDFF_scaled(float &P, float &I, float &D, float &FF)
{
    if (tune_state != TUNE_STATE::COMPLETE) {
        return false;
    }
    kP.set_and_save(kP.get() * pid.Pmod);
    kI.set_and_save(kP.get() * PI_ratio); // use pre-set P to I ratio
    kD.set_and_save(kD.get() * pid.Dmod);
    kFF.set_and_save(kFF.get() * pid.FFmod);
    reset_scale_factors();
    get_scaled_gains(P,I,D,FF);
    return true;
}

// return the current gain multiplied by scale factor
void AC_PID_OscillationDetector::get_scaled_gains(float &P, float &I, float &D, float &FF)
{
    P = kP.get() * pid.Pmod;
    I = kI.get() * pid.Imod;
    D = kD.get() * pid.Dmod;
    FF = kFF.get() * pid.FFmod;
}

// return true if a tune is currently running, return current gains
bool AC_PID_OscillationDetector::quick_tune_active(float &P, float &I, float &D, float &FF)
{
    if (!quick_tune_active()) {
        return false;
    }
    get_scaled_gains(P,I,D,FF);
    return true;
}
