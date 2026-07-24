#pragma once

/// @file    AP_CustomControl.h
/// @brief   ArduPlane custom control library

#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Bitmask.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <Filter/AP_Filter.h>

#ifndef CUSTOMCONTROL_MAX_TYPES
#define CUSTOMCONTROL_MAX_TYPES 2
#endif

#ifndef FUNCTIONS_MAX
#define FUNCTIONS_MAX 10
#endif

class AP_CustomControl_Backend;

class AP_CustomControl {
public:
    AP_CustomControl(const AP_FixedWing &parms);

    CLASS_NO_COPY(AP_CustomControl);  /* Do not allow copies */

    // These are assumed to be numerically valid at each cycle.
    // The controller backend can choose to access and/or serve them.
    float roll_target_deg;
    float pitch_target_deg;
    float pitch_trim_deg;
    bool is_flying;
    bool ground_mode; // Whether we are disarmed on the ground.
    uint8_t flight_mode; // The flight mode we are currently in.

    // One-off object initialization.
    void init(void);

    // Queried by Plane to determine if controllers need reset.
    bool needs_reset_roll() { return _reset_roll; }
    bool needs_reset_pitch() { return _reset_pitch; }
    bool needs_reset_yaw() {  return _reset_yaw; }
    bool needs_reset_steer() { return _reset_steer; }
    bool needs_reset_throttle() { return _reset_throttle; }

    // Write a scaled value to all channels with a function.
    void set_output_scaled(SRV_Channel::Function function, float value);
    // Write a pwm value to all channels with a function. Not min/max constrained. servos.cpp may overwrite it.
    void set_output_pwm(SRV_Channel::Function function, uint16_t value);
    // Write pwm values on a channel (1-indexed). Not min/max constrained. servos.cpp may overwrite it.
    void set_output_pwm_chan(uint8_t chan, uint16_t value);
    // Override pwm values on a channel (1-indexed) for one loop. servos.cpp will not overwrite it.
    void set_output_pwm_chan_override(uint8_t chan, uint16_t value);
    // set the PID notch sample rates
#if AP_FILTER_ENABLED
    void set_notch_sample_rate(float sample_rate);
#endif // AP_FILTER_ENABLED

    // Main loop update. Return true if the custom controller was active this loop.
    bool update(void);
    // Set controller bank or disable.
    void set_custom_controller(bool enabled);
    // Accessor of CC_MASK for the backends.
    AP_Int32 get_mask() { return _custom_controller_mask; }

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

private:
    // add custom controller here
    enum class CustomControlType : uint8_t {
        NONE            = 0,
        EMPTY           = 1,
        PID             = 2,
    };            // controller that should be used

    struct FunctionInfo {
        SRV_Channel::Function name;
        float scaled; // NaN means don't set.
        uint16_t pwm; // MAXINT16 means don't set.
        bool used; // This slot is in use.
    };

    // Intersampling period in seconds
    float _dt;
    bool _custom_controller_active;

    AP_Enum<CustomControlType> _controller_type;
    AP_Int32 _custom_controller_mask; // Enable up to 31 custom controller axes/outputs.

    AP_CustomControl_Backend *_backend;
    const AP_FixedWing &aparm;

    FunctionInfo functions[FUNCTIONS_MAX];
    uint16_t channels_pwm[NUM_SERVO_CHANNELS]; // MAXINT16 means don't set.
    Bitmask<NUM_SERVO_CHANNELS> ch_override_bitmask; // Holds the servo channels that should be overriden each loop.

    bool _reset_roll;
    bool _reset_pitch;
    bool _reset_yaw;
    bool _reset_steer;
    bool _reset_throttle;
    bool _max_allocations_warned;
 
    void set_controls();
    void reset_main_controller(void);
    bool is_safe_to_run(void);
    void log_switch(void);
    // zero index controller type param, only use it to access _backend or _backend_var_info array
    uint8_t get_type() { return _controller_type > 0 ? (_controller_type - 1) : 0; };
    // Whether a function or channel of this function has been set in this cycle.
    bool function_is_set(SRV_Channel::Function);
    // Return the index in fuctions where a function is found, or -1 if not found.
    int8_t get_function_index(SRV_Channel::Function);
    int8_t allocate_function(); // Find an empty slot. Return FUCTIONS_MAX-1 if all slots are full.
    void clear_outputs(); // Clear all controller outputs.
};

#endif  // AP_PLANE_CUSTOMCONTROL_ENABLED
