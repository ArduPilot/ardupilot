#include "Rover.h"

/*
  allow for runtime change of control channel ordering
 */
void Rover::set_control_channels(void)
{
    // check change on RCMAP
    channel_steer    = rc().channel(rcmap.roll()-1);
    channel_throttle = rc().channel(rcmap.throttle()-1);
    channel_lateral  = rc().channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_steer->set_angle(SERVO_MAX);
    channel_throttle->set_angle(100);
    channel_lateral->set_angle(100);

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
    channel_lateral->set_default_dead_zone(30);
}

/*
  check for driver input on rudder/steering stick for arming/disarming
*/
void Rover::rudder_arm_disarm_check()
{
    // check if arming/disarm using rudder is allowed
    const AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        return;
    }

    // In Rover we need to check that its set to the throttle trim and within the DZ
    // if throttle is not within trim dz, then pilot cannot rudder arm/disarm
    if (!channel_throttle->in_trim_dz()) {
        rudder_arm_timer = 0;
        return;
    }

    // check if arming/disarming allowed from this mode
    if (!control_mode->allows_arming_from_transmitter()) {
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
                arm_motors(AP_Arming::Method::RUDDER);
                rudder_arm_timer = 0;
            }
        } else {
            // not at full right rudder
            rudder_arm_timer = 0;
        }
    } else if ((arming_rudder == AP_Arming::RudderArming::ARMDISARM) && !g2.motors.active()) {
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
    if (!rc().read_input()) {
        // check if we lost RC link
        radio_failsafe_check(channel_throttle->get_radio_in());
        return;
    }

    failsafe.last_valid_rc_ms = AP_HAL::millis();
    // check that RC value are valid
    radio_failsafe_check(channel_throttle->get_radio_in());

    // check if we try to do RC arm/disarm
    rudder_arm_disarm_check();
}

void Rover::radio_failsafe_check(uint16_t pwm)
{
    if (!g.fs_throttle_enabled) {
        // radio failsafe disabled
        return;
    }

    bool failed = pwm < static_cast<uint16_t>(g.fs_throttle_value);
    if (AP_HAL::millis() - failsafe.last_valid_rc_ms > 200) {
        failed = true;
    }
    failsafe_trigger(FAILSAFE_EVENT_THROTTLE, failed);
}

bool Rover::trim_radio()
{
    if (!rc().has_valid_input()) {
        // can't trim without valid input
        return false;
    }

    // Store control surface trim values
    // ---------------------------------
    if ((channel_steer->get_radio_in() > 1400) && (channel_steer->get_radio_in() < 1600)) {
        channel_steer->set_and_save_radio_trim(channel_steer->get_radio_in());
    } else {
        return false;
    }

    return true;
}
