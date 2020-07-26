/*
 Oscillation Detector using simple filters
*/
#pragma once
#include <AP_Logger/AP_Logger.h>

class AC_PID_OscillationDetector
{
public:
    // Constructor
    AC_PID_OscillationDetector(float mag_threshold, float threshold, float filter);

    // update detector, retrun true if any action is being taken
    void update(AP_Logger::PID_Info& pid);

    // enable or disable auto scaling
    void set_enable(bool b) {scaling_enable = b;};

    // enum definning the value of the AP_Vehicle bitmask params
    // 'OSCILLATION_DET' and 'QUICK_TUNE_AXIS'
    enum {
        PID_ROLL   = 1 << 0,
        PID_PITCH  = 1 << 1,
        PID_YAW    = 1 << 2,
        PID_QROll  = 1 << 3,
        PID_QPITCH = 1 << 4,
        PID_QYAW   = 1 << 5,
    };

private:

    // the PIDFF scaling action being taken
    enum ACTION { 
        NONE,
        DECREASE, // decresing to stop oscillation
        INCREASE, // increasing to return to oroginal values
    } action;

    // filtered values of gains, target and actual
    float P_filt, I_filt, D_filt, ff_filt, target_filt, actual_filt;

    // filtered evelope of gains, target and actual relative to the unfiltered values
    float P_env, I_env, D_env, ff_env, target_env, actual_env;

    // minimum magnitude of target and actual to enable detection
    float magnitude_threshold;

    // actual amplitude to target ratio above this triggers gain reduction
    float oscillation_threshold;

    // main filter and envelope filter values
    float w, w_env; 

    // true if the detector is allowed to auto reduce gains
    bool scaling_enable;

};
