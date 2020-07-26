
#include "AC_PID_OscillationDetector.h"

/*
 Oscillation Detector using simple filters
*/

#define PIDff_ratio_threshold 0.5f // has to larger than 0.5, otherwise more than one value could above the threshold
#define PIDff_min_scale 0.5f
#define ratio_reduce 0.9985f // 461 calls to get to min scale
#define ratio_reduce_all 0.999f // 692 calls
#define ratio_increase 1.0003f // 2311 calls to return from min scale to 1
#define overshoot_ratio 1.2f // Constrain to this ratio above the oscillation_threshold to stop wind up, must be > 1

// Constructor
AC_PID_OscillationDetector::AC_PID_OscillationDetector(float mag_threshold, float threshold, float filter)
{
    // must be larger than 0 to avoid FPE
    magnitude_threshold = fmaxf(mag_threshold, FLT_EPSILON);

    // must be larger than one to avoid constant triggering
    oscillation_threshold = fmaxf(threshold,1.0f);

    // main filter and envelope filter values
    w = constrain_float(filter,0.0f,1.0f);
    w_env = w*0.1f;
};

void AC_PID_OscillationDetector::update(AP_Logger::PID_Info& pid)
{
    // apply filter to P, I, D, actual and target
    P_filt = P_filt * (1-w) + pid.P * w;
    I_filt = I_filt * (1-w) + pid.I * w;
    D_filt = D_filt * (1-w) + pid.D * w;
    ff_filt = ff_filt * (1-w) + pid.FF * w;
    target_filt = target_filt * (1-w) + pid.target * w;
    actual_filt = actual_filt * (1-w) + pid.actual * w;

    // Caulate the filtered envelope
    P_env = P_env * (1-w_env) + abs(pid.P - P_filt) * w_env;
    I_env = I_env * (1-w_env) + abs(pid.I - I_filt) * w_env;
    D_env = D_env * (1-w_env) + abs(pid.D - D_filt) * w_env;
    ff_env = ff_env * (1-w_env) + abs(pid.FF - ff_filt) * w_env;
    target_env = target_env * (1-w_env) + abs(pid.target - target_filt) * w_env;
    actual_env = actual_env * (1-w_env) + abs(pid.actual - actual_filt) * w_env;

    // Constrain actual to be the threshold level multiplied by the overshoot ratio at most, this reduces the 'reset' time
    actual_env = fminf(actual_env,target_env*oscillation_threshold*overshoot_ratio);

    // if there is a large enough magnitude to reliably calculate the ratio of oscillations
    float amplitude_ratio = 0.0f;
    if ((target_env > magnitude_threshold) & (actual_env > magnitude_threshold)) {
        amplitude_ratio = actual_env/target_env;
    }
    pid.amplitude_ratio = amplitude_ratio;

    if (!scaling_enable) {
        // report only
        if (action != NONE) {
            action = NONE;
            pid.Pmod = 1.0f;
            pid.Imod = 1.0f;
            pid.Dmod = 1.0f;
            pid.FFmod = 1.0f;
        }
        return;
    }

    float total_env = P_env + I_env + D_env + ff_env;
    if ((amplitude_ratio > oscillation_threshold) & is_positive(total_env)) {
        // oscillation detected
        action = DECREASE;
        total_env  = 1/total_env;
        // see if we can tell which term is causing it
        if ((P_env*total_env > PIDff_ratio_threshold) & (pid.Pmod > PIDff_min_scale)) {
            pid.Pmod *= ratio_reduce;

        } else if ((I_env*total_env > PIDff_ratio_threshold) & (pid.Imod > PIDff_min_scale)) {
            pid.Imod *= ratio_reduce;

        } else if ((D_env*total_env > PIDff_ratio_threshold) & (pid.Dmod > PIDff_min_scale)) {
            pid.Dmod *= ratio_reduce;

        } else if ((ff_env*total_env > PIDff_ratio_threshold) & (pid.FFmod > PIDff_min_scale)) {
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

        if ((pid.Pmod >= 1.0f) &
            (pid.Imod >= 1.0f) &
            (pid.Dmod >= 1.0f) &
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
