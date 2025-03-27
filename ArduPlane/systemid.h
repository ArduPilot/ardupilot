#pragma once

#include "quadplane.h"

#ifndef AP_PLANE_SYSTEMID_ENABLED
#define AP_PLANE_SYSTEMID_ENABLED BOARD_FLASH_SIZE > 1024 && HAL_QUADPLANE_ENABLED
#endif

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/chirp.h>

class AP_SystemID {

public:
    AP_SystemID(void);
    void start(void);
    void stop(void);
    void update();

    static const struct AP_Param::GroupInfo var_info[];

    const Vector3f &get_attitude_offset_deg(void) const {
        return attitude_offset_deg;
    }
    float get_throttle_offset(void) const {
        return throttle_offset;
    }

    bool is_running() const {
        return running;
    }

private:
    Chirp chirp_input;
    bool running;

    enum class AxisType {
        NONE = 0,               // none
        INPUT_ROLL = 1,         // angle input roll axis is being excited
        INPUT_PITCH = 2,        // angle pitch axis is being excited
        INPUT_YAW = 3,          // angle yaw axis is being excited
        RECOVER_ROLL = 4,       // angle roll axis is being excited
        RECOVER_PITCH = 5,      // angle pitch axis is being excited
        RECOVER_YAW = 6,        // angle yaw axis is being excited
        RATE_ROLL = 7,          // rate roll axis is being excited
        RATE_PITCH = 8,         // rate pitch axis is being excited
        RATE_YAW = 9,           // rate yaw axis is being excited
        MIX_ROLL = 10,          // mixer roll axis is being excited
        MIX_PITCH = 11,         // mixer pitch axis is being excited
        MIX_YAW = 12,           // mixer pitch axis is being excited
        MIX_THROTTLE = 13,      // mixer throttle axis is being excited
    };

    void set_bf_feedforward(bool value);
    void log_data() const;

    AP_Enum<AxisType> axis;               // Controls which axis are being excited. Set to non-zero to display other parameters
    AP_Float waveform_magnitude;// Magnitude of chirp waveform
    AP_Float frequency_start;   // Frequency at the start of the chirp
    AP_Float frequency_stop;    // Frequency at the end of the chirp
    AP_Float time_fade_in;      // Time to reach maximum amplitude of chirp
    AP_Float time_record;       // Time taken to complete the chirp waveform
    AP_Float time_fade_out;     // Time to reach zero amplitude after chirp finishes
    AP_Float xy_control_mul;    // multipler for VTOL XY control

    struct {
        bool att_bf_feedforward;    // Setting of attitude_control->get_bf_feedforward
    } restore;

    float waveform_time;        // Time reference for waveform
    float waveform_sample;      // Current waveform sample
    float waveform_freq_rads;   // Instantaneous waveform frequency
    float time_const_freq;      // Time at constant frequency before chirp starts
    uint32_t last_loop_time_ms;   // time in milliseconds of last loop

    Vector2f target_vel;        // target velocity for position controller modes
    Vector2f target_pos;       // target positon
    Vector2f input_vel_last;    // last cycle input velocity

    // current attitude offset
    Vector3f attitude_offset_deg;
    float throttle_offset;

    AxisType start_axis;

};

#endif // AP_PLANE_SYSTEMID_ENABLED
