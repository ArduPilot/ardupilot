#include "Rover.h"

/*
  allow for runtime change of control channel ordering
 */
void Rover::set_control_channels(void)
{
    // check change on RCMAP
    channel_steer    = RC_Channels::rc_channel(rcmap.roll()-1);
    channel_throttle = RC_Channels::rc_channel(rcmap.throttle()-1);
    channel_aux      = RC_Channels::rc_channel(g.aux_channel-1);

    // set rc channel ranges
    channel_steer->set_angle(SERVO_MAX);
    channel_throttle->set_angle(100);

    // Allow to reconfigure ouput when not armed
    if (!arming.is_armed()) {
        g2.motors.setup_servo_output();
        // For a rover safety is TRIM throttle
        g2.motors.setup_safety_output();
    }
    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed. Default to 1000 to 2000 for systems without
    // a k_throttle output
    hal.rcout->set_esc_scaling(1000, 2000);
    g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
}

void Rover::init_rc_in()
{
    // set rc dead zones
    channel_steer->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
}

void Rover::init_rc_out()
{
    // set auxiliary ranges
    update_aux();
}

/*
  check for driver input on rudder/steering stick for arming/disarming
*/
void Rover::rudder_arm_disarm_check()
{
    // In Rover we need to check that its set to the throttle trim and within the DZ
    // if throttle is not within trim dz, then pilot cannot rudder arm/disarm
    if (!channel_throttle->in_trim_dz()) {
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual throttle mode then disallow rudder arming/disarming
    if (control_mode->auto_throttle()) {
        rudder_arm_timer = 0;
        return;
    }

    if (!arming.is_armed()) {
        // when not armed, full right rudder starts arming counter
        if (channel_steer->get_control_in() > 4000) {
            const uint32_t now = millis();

            if (rudder_arm_timer == 0 ||
                now - rudder_arm_timer < ARM_DELAY_MS) {
                if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
            } else {
                // time to arm!
                arm_motors(AP_Arming::RUDDER);
                rudder_arm_timer = 0;
            }
        } else {
            // not at full right rudder
            rudder_arm_timer = 0;
        }
    } else if (!motor_active()) {
        // when armed and motor not active (not moving), full left rudder starts disarming counter
        if (channel_steer->get_control_in() < -4000) {
            const uint32_t now = millis();

            if (rudder_arm_timer == 0 ||
                now - rudder_arm_timer < ARM_DELAY_MS) {
                if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
            } else {
                // time to disarm!
                disarm_motors();
                rudder_arm_timer = 0;
            }
        } else {
            // not at full left rudder
            rudder_arm_timer = 0;
        }
    }
}

void Rover::read_radio()
{
    if (!hal.rcin->new_input()) {
        // check if we lost RC link
        control_failsafe(channel_throttle->get_radio_in());
        return;
    }

    failsafe.last_valid_rc_ms = AP_HAL::millis();
    // read the RC value
    RC_Channels::set_pwm_all();
    // check that RC value are valid
    control_failsafe(channel_throttle->get_radio_in());

    // check if we try to do RC arm/disarm
    rudder_arm_disarm_check();
}

void Rover::control_failsafe(uint16_t pwm)
{
    if (!g.fs_throttle_enabled) {
        // no throttle failsafe
        return;
    }

    // Check for failsafe condition based on loss of GCS control
    if (rc_override_active) {
        failsafe_trigger(FAILSAFE_EVENT_RC, (millis() - failsafe.rc_override_timer) > 1500);
    } else if (g.fs_throttle_enabled) {
        bool failed = pwm < static_cast<uint16_t>(g.fs_throttle_value);
        if (AP_HAL::millis() - failsafe.last_valid_rc_ms > 2000) {
            failed = true;
        }
        failsafe_trigger(FAILSAFE_EVENT_THROTTLE, failed);
    }
}

void Rover::trim_control_surfaces()
{
    read_radio();
    // Store control surface trim values
    // ---------------------------------
    if (channel_steer->get_radio_in() > 1400) {
        channel_steer->set_radio_trim(channel_steer->get_radio_in());
        // save to eeprom
        channel_steer->save_eeprom();
    }
}

void Rover::trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }
    trim_control_surfaces();
}
