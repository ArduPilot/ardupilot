/*
 Oscillation Detector using simple filters
*/
#pragma once
#include <AP_Logger/AP_Logger.h>

class AC_PID_OscillationDetector
{
public:
    // Constructor
    AC_PID_OscillationDetector(AP_Logger::PID_Info &_pid, AP_Float &_kP, AP_Float &_kI, AP_Float &_kD, AP_Float &_Kff, float mag_threshold, float threshold, float filter, float tune_PI_rato);

    // update detector
    void update();

    // enable or disable auto scaling
    void set_enable(bool b) {scaling_enable = b;}
    bool enabled() {return scaling_enable;}

    // start or stop a tune
    bool start_quick_tune();
    void stop_quick_tune();

    // save gains to params, return values
    bool save_PDFF_scaled(float &P, float &I, float &D, float &FF);

    // reset gain scale factors to 1
    void reset_scale_factors();

    // return true if a tune is currently running, return current gains
    bool quick_tune_active(float &P, float &I, float &D, float &FF);

    // return true if a tune has been started
    bool quick_tune_has_run() {return (tune_state != TUNE_STATE::NONE);}

private:

    // return the current gain multiplied by scale factor
    void get_scaled_gains(float &P, float &I, float &D, float &FF);

    // return true if tune is currently running
    bool quick_tune_active() {return (tune_state != TUNE_STATE::NONE) && (tune_state < TUNE_STATE::COMPLETE);}

    // the PIDFF scaling action being taken
    enum ACTION { 
        NONE,
        DECREASE, // decresing to stop oscillation
        INCREASE, // increasing to return to oroginal values
    } action;

    // filtered values of gains, target and actual
    float target_filt, actual_filt;

    // filtered evelope of gains, target and actual relative to the unfiltered values
    float target_env, actual_env;

    // minimum magnitude of target and actual to enable detection
    float magnitude_threshold;

    // actual amplitude to target ratio above this triggers gain reduction
    float oscillation_threshold;

    // main filter and envelope filter values
    float w, w_env;

    // true if the detector is allowed to auto reduce gains
    bool scaling_enable;

    // counter to track progress of tune
    uint16_t tune_count;

    // current and last states of the quick tune
    enum class TUNE_STATE : uint8_t {
        NONE,
        FF,
        P1,
        D,
        P2,
        CHECK,
        COMPLETE,
        FAILED,
    } tune_state, last_tune_state;

    // True if a oscillation has been detected at any stage in this tune section
    bool has_ossialated;

    // P gain reached with no D, if P with D is lower we got the D tune wrong
    float P_only;

    // ratio of I to P to use when saveing gains
    float PI_ratio;

    // refences to the main gains use by the parent controller
    AP_Logger::PID_Info &pid;
    AP_Float &kP;
    AP_Float &kI;
    AP_Float &kD;
    AP_Float &kFF;

};
