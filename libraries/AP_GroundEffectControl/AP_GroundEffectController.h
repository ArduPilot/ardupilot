#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <PID/PID.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <Filter/ComplementaryFilter.h>

class GroundEffectController {
public:
    GroundEffectController(AP_AHRS& ahrs, RangeFinder& rangefinder)
        : _ahrs{ahrs}
        , _rangefinder{rangefinder}
        , _enabled{false}
        {
            AP_Param::setup_object_defaults(this, var_info);
        };

    /* Do not allow copies */
    GroundEffectController(const GroundEffectController &other) = delete;
    GroundEffectController &operator=(const GroundEffectController&) = delete;

    bool user_request_enable(bool enable);

    bool enabled_by_user() { return _enabled; }

    void update();

	void reset();

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

    int32_t get_auto_lim_roll_cd();

    int32_t get_pitch() { return _pitch; }

    int16_t get_throttle() { return _throttle; }

private:
    PID _pitch_pid{120.0, 0.0, 0.0, 1000};
    PID _throttle_pid{64, 0.0, 20.0, 1000};

	AP_Float _THR_REF;
    AP_Float _THR_MIN;
    AP_Float _THR_MAX;
    AP_Float _ALT_REF;
    AP_Float _CUTOFF_FREQ;
    AP_Float _LIM_ROLL;

    AP_Logger::PID_Info _pid_info;

    uint32_t _last_time_called;

    AP_AHRS& _ahrs;
    RangeFinder& _rangefinder;
    ComplementaryFilter _altFilter;

    float _last_good_rangefinder_reading;

    bool _enabled;
    int32_t _pitch;
    int16_t _throttle;
};
