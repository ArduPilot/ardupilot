#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_PID_ENABLED

#include "AP_CustomControl_PID.h"
#include "AP_CustomControl/AP_CustomControl.h"

#include "RC_Channel/RC_Channel.h"
#include "SRV_Channel/SRV_Channel.h"
#include <AP_AHRS/AP_AHRS.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl_PID::var_info[] = {
    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 1, AP_CustomControl_PID, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 2, AP_CustomControl_PID, AC_P),


    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain. Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_RLL_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: RAT_RLL_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_RLL_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_RLL_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 3, AP_CustomControl_PID, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_PIT_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: RAT_PIT_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_PIT_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_PIT_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 4, AP_CustomControl_PID, AC_PID),

    AP_GROUPEND
};

AP_CustomControl_PID::AP_CustomControl_PID(AP_CustomControl& frontend, float dt) :
    AP_CustomControl_Backend(frontend, dt),
    // Initialize gains at slightly more relax than the defaults.
    _p_angle_roll(1.8f),
    _p_angle_pitch(1.8f),
    _pid_rate_roll( AC_PID::Defaults{
                        .p         = 0.08,
                        .i         = 0.15,
                        .d         = 0.0,
                        .ff        = 0.345,
                        .imax      = 0.666,
                        .filt_T_hz = 3.0,
                        .filt_E_hz = 0.0,
                        .filt_D_hz = 12.0,
                        .srmax     = 150.0,
                        .srtau     = 1.0
                    }),
    _pid_rate_pitch( AC_PID::Defaults{
                         .p         = 0.04,
                         .i         = 0.15,
                         .d         = 0.0,
                         .ff        = 0.345,
                         .imax      = 0.666,
                         .filt_T_hz = 3.0,
                         .filt_E_hz = 0.0,
                         .filt_D_hz = 12.0,
                         .srmax     = 150.0,
                         .srtau     = 1.0
                     })
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_CustomControl_PID::can_run()
{
    // Do not run the custom controller during RC failsafe.
    if (!rc().has_valid_input()) {
        return false;
    }

    // Do not run in modes where target angles are not available.
    switch (_frontend.flight_mode) {
    case 0: // MANUAL
    case 2: // STABILIZE
    case 4: // ACRO
    case 16: // INITIALISING
        return false;
    }

    return true;
}

void AP_CustomControl_PID::update()
{
    // Reset controllers when not flying, e.g. to avoid windup.
    if (!_frontend.is_flying) {
        reset();
    }

    // Example accessor for normalized pilot inputs.
    const RC_Channel* channel_roll = &rc().get_roll_channel(); // Guaranteed to be nonnull.
    const float roll_input = channel_roll->norm_input();
    // Example of direct RC channel access.
    const RC_Channel *channel_custom = rc().find_channel_for_option(RC_Channel::AUX_FUNC::LANDING_GEAR);
    float custom_input{0};
    if (channel_custom != nullptr) {
        custom_input = channel_custom->norm_input_dz();
    }
    const RC_Channel *channel_7 = rc().channel(6); // The channels are 0-indexed.
    const float ch_7_input = channel_7->get_radio_in();

    // Run attitude controller.
    const float target_roll_rate = _p_angle_roll.kP() * (get_roll_target_deg() - AP::ahrs().get_roll_deg() );
    const float target_pitch_rate = _p_angle_pitch.kP() * (get_pitch_target_deg() - AP::ahrs().get_pitch_deg() );

    // Run rate controllers.
    // We can gate every output write with the custom controller mask.
    Vector3f gyro_latest = AP::ahrs().get_gyro_latest();
    float aileron{0};
    if (_frontend.get_mask() & 1 << 0) {
        aileron = _pid_rate_roll.update_all(target_roll_rate, gyro_latest[0], _dt, false);
        aileron += _pid_rate_roll.get_ff();
    } else {
        _pid_rate_roll.reset_I();
        _pid_rate_roll.reset_filter();
    }
    float elevator{0};
    if (_frontend.get_mask() & 1 << 1) {
        elevator = _pid_rate_pitch.update_all(target_pitch_rate, gyro_latest[1], _dt, false);
        elevator += _pid_rate_pitch.get_ff();
    } else {
        _pid_rate_pitch.reset_I();
        _pid_rate_pitch.reset_filter();
    }
    float rudder{0};
    if (_frontend.get_mask() & 1 << 2) {
        rudder = aileron*0.5f; // Add default value of rudder mix.
    }

    // Write to the control surfaces.
    if (_frontend.get_mask() & 1 << 0) {
        _frontend.set_output_scaled(SRV_Channel::k_aileron,constrain_float(aileron*100, -4500, 4500));
    }
    if (_frontend.get_mask() & 1 << 1) {
        _frontend.set_output_scaled(SRV_Channel::k_elevator,constrain_float(elevator*100, -4500, 4500));
    }
    if (_frontend.get_mask() & 1 << 2) {
        _frontend.set_output_scaled(SRV_Channel::k_rudder,constrain_float(rudder*100, -4500, 4500));
    }

    // Example of writing to outputs directly.
    if (_frontend.get_mask() & 1 << 3) {
        _frontend.set_output_scaled(SRV_Channel::Function::k_steering, custom_input*4500); // Write scaled value on a function.
    }
    if (_frontend.get_mask() & 1 << 4) {
        _frontend.set_output_pwm_chan(6, 1500+roll_input*500); // To verify that we can read the main inputs and write pwm to a channel.
    }
    if (_frontend.get_mask() & 1 << 5) {
        _frontend.set_output_pwm(SRV_Channel::Function::k_airbrake, ch_7_input); // Write pwm on a function.
    }
    if (_frontend.get_mask() & 1 << 6) {
        _frontend.set_output_pwm(SRV_Channel::Function::k_parachute_release, ch_7_input); // Write pwm on a function.
    }
    if (_frontend.get_mask() & 1 << 7) {
        _frontend.set_output_pwm_chan_override(10, ch_7_input); // Override pwm to a channel.
    }

}

// Reset our own custom controllers.
void AP_CustomControl_PID::reset(void)
{
    _pid_rate_roll.reset_I();
    _pid_rate_roll.reset_filter();
    _pid_rate_pitch.reset_I();
    _pid_rate_pitch.reset_filter();
}

#if AP_FILTER_ENABLED
void AP_CustomControl_PID::set_notch_sample_rate(float sample_rate)
{
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
}
#endif

#endif  // AP_PLANE_CUSTOMCONTROL_PID_ENABLED
