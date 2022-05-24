#include <AP_Winch/AP_Winch_Backend.h>

#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

// setup rc input and output
void AP_Winch_Backend::init()
{
    // set servo output range
    SRV_Channels::set_angle(SRV_Channel::k_winch,  1000);

    RC_Channel *rc_input = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WINCH_CONTROL);
    if (rc_input != nullptr) {
        // rc input deadzone is required or pilot input will always override autonomous control of winch
        rc_input->set_default_dead_zone(30);
    }
}

// calculate the pilot desired rate (+ve deploys line, -ve retracts line, 0 stops) from rc input
// may change the state to RATE and update config.rate_desired
void AP_Winch_Backend::read_pilot_desired_rate()
{
    // fail if no input channel defined
    const RC_Channel *rc_input = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WINCH_CONTROL);
    if (rc_input == nullptr) {
        return;
    }

    // if pilot is controlling winch, set rate to zero during RC failsafe
    if (!rc().has_valid_input() && (config.control_mode == AP_Winch::ControlMode::RATE_FROM_RC)) {
        config.rate_desired = 0.0f;
        return;
    }

    // initialise previous_radio_in
    if (previous_radio_in == -1) {
        previous_radio_in = rc_input->get_radio_in();
    }

    // if significant change in rc input switch to rate mode
    const int16_t radio_in = rc_input->get_radio_in();
    if (config.control_mode != AP_Winch::ControlMode::RATE_FROM_RC) {
        if (abs(radio_in - previous_radio_in) > rc_input->get_dead_zone()) {
            config.control_mode = AP_Winch::ControlMode::RATE_FROM_RC;
        }
    }

    // update desired rate
    if (config.control_mode == AP_Winch::ControlMode::RATE_FROM_RC) {
        config.rate_desired = rc_input->norm_input_dz() * config.rate_max;
        previous_radio_in = radio_in;
    }
}

// returns the rate limited by the maximum acceleration
// this also updates the previous rate so should not be called more than once per loop
float AP_Winch_Backend::get_rate_limited_by_accel(float rate, float dt)
{
    // use maximum rate x 2 for maximum acceleration
    const float rate_change_max = MAX(config.rate_max * 2.0f, 0.1f) * dt;

    // apply acceleration limit
    const float rate_limited = constrain_float(rate, previous_rate - rate_change_max, previous_rate + rate_change_max);

    // update previous rate for next time
    set_previous_rate(rate_limited);

    return rate_limited;
}
