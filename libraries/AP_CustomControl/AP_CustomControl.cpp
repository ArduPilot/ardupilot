#include <AP_HAL/AP_HAL.h>
#include <cstdint>

#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_ENABLED

#include "AP_CustomControl.h"
#include "AP_CustomControl_Backend.h"
#include "AP_CustomControl_PID.h"
#include "SRV_Channel/SRV_Channel.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Custom control type
    // @Description: Custom control type to be used
    // @Values: 0:None, 1:Empty, 2:PID
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_CustomControl, _controller_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MASK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which controllers to run or outputs to write.
    // @Bitmask: 0:Output 1,1:Output 2,2:Output 3,3:Output 4,4:Output 5,5:Output 6,6:Output 7,7:Output 8,8:Output 9,9:Output 10,10:Output 11,11:Output 12,12:Output 13,13:Output 14,14:Output 15,15:Output 16, 16:Output 17, 17: Output 18, 18: Output 19, 19: Output 20, 20: Output 21, 21: Output 22, 22: Output 23, 23: Output 24, 24: Output 25, 25: Output 26, 26: Output 27, 27: Output 28, 28: Output 29, 29: Output 30, 30: Output 31, 31: Output 32
    // @User: Advanced
    AP_GROUPINFO("_MASK", 2, AP_CustomControl, _custom_controller_mask, 65535),

    // parameters for empty controller. only used as a template, no need for param table
    // AP_SUBGROUPVARPTR(_backend, "1_", 6, AP_CustomControl, _backend_var_info[0]),

    // parameters for PID controller
    // @Group: 2_
    // @Path: ./AP_CustomControl_PID.cpp
    AP_SUBGROUPVARPTR(_backend, "2_", 7, AP_CustomControl, _backend_var_info[1]),

    AP_GROUPEND
};

const struct AP_Param::GroupInfo *AP_CustomControl::_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

AP_CustomControl::AP_CustomControl(const AP_FixedWing &parms)
    : aparm(parms)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_CustomControl::init(void)
{
    _dt = AP::scheduler().get_loop_period_s();
    clear_outputs();

    switch (CustomControlType(_controller_type)) {
    case CustomControlType::NONE:
        break;
    case CustomControlType::EMPTY:
        // This is template backend. Don't initialize it.
        // _backend = NEW_NOTHROW AP_CustomControl_Empty(*this, _dt);
        // _backend_var_info[get_type()] = AP_CustomControl_Empty::var_info;
        break;
    case CustomControlType::PID:
        _backend = NEW_NOTHROW AP_CustomControl_PID(*this, _dt);
        _backend_var_info[get_type()] = AP_CustomControl_PID::var_info;
        break;
    default:
        break;
    }

    if (_backend && _backend_var_info[get_type()]) {
        AP_Param::load_object_from_eeprom(_backend, _backend_var_info[get_type()]);
    }
}

// Write a scaled value to all channels with a function.
void AP_CustomControl::set_output_scaled(SRV_Channel::Function function, float value) {
    int8_t idx = get_function_index(function);
    if (idx == -1) {
        idx = allocate_function();
        functions[idx].name = function;
    }
    functions[idx].scaled = value;
}

// Write a pwm value to all channels with a function.
void AP_CustomControl::set_output_pwm(SRV_Channel::Function function, uint16_t value) {
    int8_t idx = get_function_index(function);
    if (idx == -1) {
        idx = allocate_function();
        functions[idx].name = function;
    }
    functions[idx].pwm = value;
}

// Write pwm values on a channel.
void AP_CustomControl::set_output_pwm_chan(uint8_t chan, uint16_t value) {
    if ((chan < 1 || chan > NUM_SERVO_CHANNELS)) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Custom controller is not enabled");
        return;
    }
    channels_pwm[chan-1] = value;
}

// Override pwm values on a channel for one loop.
void AP_CustomControl::set_output_pwm_chan_override(uint8_t chan, uint16_t value) {
    if ((chan < 1 || chan > NUM_SERVO_CHANNELS)) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Custom controller is not enabled");
        return;
    }
    set_output_pwm_chan(chan, value);
    ch_override_bitmask.set(chan-1);
}

// run custom controller if it is activated by RC switch and appropriate type is selected
bool AP_CustomControl::update(void)
{
    // This method will always run. Don't reset main controller unless specified later.
    _reset_roll = false;
    _reset_pitch = false;
    _reset_yaw = false;
    _reset_steer = false;
    _reset_throttle = false;

    bool ret{false};

    if (is_safe_to_run()) {
        _backend->update();
        set_controls();
        ret = true;
    }

    clear_outputs();

    return ret;

}

// choose which axis to apply custom controller output
void AP_CustomControl::set_controls()
{
    // If you set the same function via set_output_scaled() and set_output_pwm(), the pwm takes priority.
    for (uint8_t idx = 0; idx < FUNCTIONS_MAX; idx++) {
        if (!functions[idx].used) {
            continue;
        }
        if (functions[idx].pwm != UINT16_MAX) {
            SRV_Channels::set_output_pwm(functions[idx].name, functions[idx].pwm);
            continue;
        }

        if (!isnan(functions[idx].scaled)) {
            SRV_Channels::set_output_scaled(functions[idx].name, functions[idx].scaled);
            continue;
        }
    }

    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels_pwm[i] != UINT16_MAX) {
            if (ch_override_bitmask.get(i)) {
                SRV_Channels::set_output_pwm_chan_timeout(i, channels_pwm[i], 1U);
            } else {
                SRV_Channels::set_output_pwm_chan(i, channels_pwm[i]);
            }
        }
    }

    // Handle special cases which warrant stock controller resets.
    if (function_is_set(SRV_Channel::k_aileron)) {
        _reset_roll = true;
    }
    if (function_is_set(SRV_Channel::k_elevator)) {
        _reset_pitch = true;
    }
    if (function_is_set(SRV_Channel::k_throttle)) {
        _reset_throttle = true;
    }
    if (function_is_set(SRV_Channel::k_rudder)) {
        _reset_yaw = true;
        _reset_steer = true;
    }

}

// Indicate which of the main controllers should be reset.
// The actual resetting will be handled by Plane.
void AP_CustomControl::reset_main_controller(void)
{
    // Reset integrators.
    _reset_roll = true;
    _reset_pitch = true;
    _reset_yaw = true;
    _reset_steer = true;
    _reset_throttle = true;
}

// Select which custom controller to run, or none at all.
// This is called upon AUX_FUNC event.
void AP_CustomControl::set_custom_controller(bool enabled)
{
    // double logging switch makes the state change very clear in the log
    log_switch();

    _custom_controller_active = false;

    // don't allow accidental main controller reset without active custom controller
    if (_controller_type == CustomControlType::NONE) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is not enabled");
        return;
    }

    // controller type is out of range
    if (_controller_type > CUSTOMCONTROL_MAX_TYPES) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller type is out of range");
        return;
    }

    // backend is not created
    if (_backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reboot to enable selected custom controller");
        return;
    }

    if (_custom_controller_mask == 0 && enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Axis mask is not set");
        return;
    }

    // reset main controller
    if (!enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is OFF");
        // We are just switching to the stock controllers. Reset them to avoid bumps.
        if (_controller_type > CustomControlType::EMPTY) {
            reset_main_controller();
        }
    }

    if (enabled && _controller_type > CustomControlType::NONE) {
        // reset custom controller filter, integrator etc.
        _backend->reset();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is ON");
    }

    _custom_controller_active = enabled;
#if AP_FILTER_ENABLED
    const float loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
    set_notch_sample_rate(loop_rate);
#endif // AP_FILTER_ENABLED

    // log successful switch
    log_switch();
}

// Verify the conditions which are safe to run the custom controller.
bool AP_CustomControl::is_safe_to_run(void)
{
    if (!_custom_controller_active) {
        return false;
    }
    if (_controller_type == CustomControlType::NONE) {
        return false;
    }
    if (_controller_type > CUSTOMCONTROL_MAX_TYPES) {
        return false;
    }
    if (_backend == nullptr) {
        return false;
    }
    if (!_backend->can_run()) {
        return false;
    }

    return true;
}

// log when the custom controller is switch into
void AP_CustomControl::log_switch(void) {
    // @LoggerMessage: CP
    // @Description: Custom Controller data
    // @Field: TimeUS: Time since system startup
    // @Field: Type: controller type
    // @FieldValueEnum: Type: AP_CustomControl::CustomControlType
    // @Field: Act: true if controller is active
    AP::logger().Write("CP", "TimeUS,Type,Act","QBB",
                       AP_HAL::micros64(),
                       _controller_type,
                       _custom_controller_active);
}

#if AP_FILTER_ENABLED
void AP_CustomControl::set_notch_sample_rate(float sample_rate)
{
    if (_backend != nullptr) {
        _backend->set_notch_sample_rate(sample_rate);
    }
}
#endif // AP_FILTER_ENABLED

bool AP_CustomControl::function_is_set(SRV_Channel::Function function) {
    const int8_t idx = get_function_index(function);
    if (idx >= 0 && ( !isnan(functions[idx].scaled) || (functions[idx].pwm != UINT16_MAX) )) {
        return true;
    };

    uint8_t chan;
    if (SRV_Channels::find_channel(function, chan) && (channels_pwm[chan] != UINT16_MAX)) {
        return true;
    }

    return false;
}

int8_t AP_CustomControl::get_function_index(SRV_Channel::Function function) {
    for ( uint8_t i = 0; i < FUNCTIONS_MAX; i++) {
        if (functions[i].name == function) {
            return i;
        }
    }
    return -1;
}

int8_t AP_CustomControl::allocate_function() {
    uint8_t i;
    for ( i = 0; i < FUNCTIONS_MAX; i++) {
        if (!functions[i].used) {
            break;
        }
    }
    if (i == FUNCTIONS_MAX) {
        if (!_max_allocations_warned) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Max # of custom controller functions allocated.");
            _max_allocations_warned = true;
        }
        i = FUNCTIONS_MAX-1;
    }
    functions[i].used = true;
    return i;
}

void AP_CustomControl::clear_outputs() {
    // Clear the commanded values.
    for (uint8_t i = 0; i < FUNCTIONS_MAX; i++) {
        functions[i].scaled = nanf("");
        functions[i].pwm = UINT16_MAX;
    }
    memset(channels_pwm, UINT16_MAX, sizeof(channels_pwm));
    ch_override_bitmask.clearall();
}

#endif  // AP_PLANE_CUSTOMCONTROL_ENABLED
