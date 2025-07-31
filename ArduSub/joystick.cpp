#include "Sub.h"
#include "mode.h"

// Functions that will handle joystick/gamepad input
// ----------------------------------------------------------------------------

// Anonymous namespace to hold variables used only in this file
namespace {
#if HAL_MOUNT_ENABLED
float cam_tilt = 1500.0;
float cam_pan = 1500.0;
#endif  // HAL_MOUNT_ENABLED
float lights1 = 0;
float lights2 = 0;
int16_t rollTrim = 0;
int16_t pitchTrim = 0;
int16_t zTrim = 0;
int16_t xTrim = 0;
int16_t yTrim = 0;
int16_t x_last, y_last, z_last;
uint32_t buttons_prev;

bool controls_reset_since_input_hold = true;
}

void Sub::init_joystick()
{
    default_js_buttons();

    set_mode(Mode::Number::MANUAL, ModeReason::RC_COMMAND); // Initialize flight mode

    if (g.numGainSettings < 1) {
        g.numGainSettings.set_and_save(1);
    }

    if (g.numGainSettings == 1 || (g.gain_default < g.maxGain + 0.01 && g.gain_default > g.minGain - 0.01)) {
        gain = constrain_float(g.gain_default, g.minGain, g.maxGain); // Use default gain parameter
    } else {
        // Use setting closest to average of minGain and maxGain
        gain = g.minGain + (g.numGainSettings/2 - 1) * (g.maxGain - g.minGain) / (g.numGainSettings - 1);
    }

    gain = constrain_float(gain, 0.1, 1.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_video_switch, 0.0);
}

void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions,
            int16_t s,
            int16_t t,
            int16_t aux1,
            int16_t aux2,
            int16_t aux3,
            int16_t aux4,
            int16_t aux5,
            int16_t aux6)
{

    float rpyScale = 0.4*gain; // Scale -1000-1000 to -400-400 with gain
    float throttleScale = 0.8*gain*g.throttle_gain; // Scale 0-1000 to 0-800 times gain
    int16_t rpyCenter = 1500;
    int16_t throttleBase = 1500-500*throttleScale;

    bool shift = false;

#if HAL_MOUNT_ENABLED
    // Neutralize camera tilt and pan speed setpoint
    cam_tilt = 1500;
    cam_pan = 1500;
#endif  // HAL_MOUNT_ENABLED

    uint32_t all_buttons = buttons | (buttons2 << 16);
    // Detect if any shift button is pressed
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((all_buttons & (1 << i)) && get_button(i)->function() == JSButton::button_function_t::k_shift) {
            shift = true;
        }
    }

    // Act if button is pressed
    // Only act upon pressing button and ignore holding. This provides compatibility with Taranis as joystick.
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((all_buttons & (1 << i))) {
            handle_jsbutton_press(i,shift,(buttons_prev & (1 << i)));
            // buttonDebounce = tnow_ms;
        } else if (buttons_prev & (1 << i)) {
            handle_jsbutton_release(i, shift);
        }
    }

    buttons_prev = all_buttons;

    // attitude mode:
    if (roll_pitch_flag == 1) {
    // adjust roll/pitch trim with joystick input instead of forward/lateral
        pitchTrim = -x * rpyScale;
        rollTrim  =  y * rpyScale;
    }

    uint32_t tnow = AP_HAL::millis();

    int16_t zTot;
    int16_t yTot;
    int16_t xTot;

    if (!controls_reset_since_input_hold) {
        zTot = zTrim + 500; // 500 is neutral for throttle
        yTot = yTrim;
        xTot = xTrim;
        // if all 3 axes return to neutral, than we're ready to accept input again
        controls_reset_since_input_hold = (abs(z - 500) < 50) && (abs(y) < 50) && (abs(x) < 50);
    } else {
        zTot = z + zTrim;
        yTot = y + yTrim;
        xTot = x + xTrim;
    }

    channel_pitch->set_override(constrain_int16(s + pitchTrim + rpyCenter,1100,1900), tnow);
    channel_roll->set_override(constrain_int16(t + rollTrim  + rpyCenter,1100,1900), tnow);

    channel_throttle->set_override(constrain_int16((zTot)*throttleScale+throttleBase,1100,1900), tnow);
    channel_yaw->set_override(constrain_int16(r*rpyScale+rpyCenter,1100,1900), tnow);

    // maneuver mode:
    if (roll_pitch_flag == 0) {
        // adjust forward and lateral with joystick input instead of roll and pitch
        channel_forward->set_override(constrain_int16((xTot)*rpyScale+rpyCenter,1100,1900), tnow);
        channel_lateral->set_override(constrain_int16((yTot)*rpyScale+rpyCenter,1100,1900), tnow);
    } else {
        // neutralize forward and lateral input while we are adjusting roll and pitch
        channel_forward->set_override(constrain_int16(xTrim*rpyScale+rpyCenter,1100,1900), tnow);
        channel_lateral->set_override(constrain_int16(yTrim*rpyScale+rpyCenter,1100,1900), tnow);
    }

#if HAL_MOUNT_ENABLED
    RC_Channel *cam_pan_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOUNT1_YAW);
    if (cam_pan_chan != nullptr) {
        cam_pan_chan->set_override(cam_pan, tnow);
    }
    RC_Channel *cam_tilt_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOUNT1_PITCH);
    if (cam_tilt_chan != nullptr) {
        cam_tilt_chan->set_override(cam_tilt, tnow);
    }
#endif  // HAL_MOUNT_ENABLED

    // Store old x, y, z values for use in input hold logic
    x_last = x;
    y_last = y;
    z_last = z;
}

void Sub::handle_jsbutton_press(uint8_t _button, bool shift, bool held)
{
    // Act based on the function assigned to this button
    switch (get_button(_button)->function(shift)) {
    case JSButton::button_function_t::k_arm_toggle:
        if (motors.armed()) {
            arming.disarm(AP_Arming::Method::MAVLINK);
        } else {
            arming.arm(AP_Arming::Method::MAVLINK);
        }
        break;
    case JSButton::button_function_t::k_arm:
        arming.arm(AP_Arming::Method::MAVLINK);
        break;
    case JSButton::button_function_t::k_disarm:
        arming.disarm(AP_Arming::Method::MAVLINK);
        break;

    case JSButton::button_function_t::k_mode_manual:
        set_mode(Mode::Number::MANUAL, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_stabilize:
        set_mode(Mode::Number::STABILIZE, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_depth_hold:
        set_mode(Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_auto:
        set_mode(Mode::Number::AUTO, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_guided:
        set_mode(Mode::Number::GUIDED, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_circle:
        set_mode(Mode::Number::CIRCLE, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_acro:
        set_mode(Mode::Number::ACRO, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_poshold:
        set_mode(Mode::Number::POSHOLD, ModeReason::RC_COMMAND);
        break;
#if AP_RANGEFINDER_ENABLED
    case JSButton::button_function_t::k_mode_surftrak:
        set_mode(Mode::Number::SURFTRAK, ModeReason::RC_COMMAND);
        break;
#endif
#if HAL_MOUNT_ENABLED
    case JSButton::button_function_t::k_mount_center:
        camera_mount.set_angle_target(0, 0, 0, false);
        // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
        camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
        break;
    case JSButton::button_function_t::k_mount_tilt_up:
        cam_tilt = 1900;
        break;
    case JSButton::button_function_t::k_mount_tilt_down:
        cam_tilt = 1100;
        break;
#endif  // HAL_MOUNT_ENABLED
    case JSButton::button_function_t::k_camera_trigger:
        break;
    case JSButton::button_function_t::k_camera_source_toggle:
        if (!held) {
            static bool video_toggle = false;
            video_toggle = !video_toggle;
            if (video_toggle) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_video_switch, 1000);
                gcs().send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 2");
            } else {
                SRV_Channels::set_output_scaled(SRV_Channel::k_video_switch, 0.0);
                gcs().send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 1");
            }
        }
        break;
#if HAL_MOUNT_ENABLED
    case JSButton::button_function_t::k_mount_pan_right:
        cam_pan = 1900;
        break;
    case JSButton::button_function_t::k_mount_pan_left:
        cam_pan = 1100;
        break;
#endif  // HAL_MOUNT_ENABLED
    case JSButton::button_function_t::k_lights1_cycle:
        if (!held) {
            static bool increasing = true;
            uint16_t step = 1000.0 / g.lights_steps;
            if (increasing) {
                lights1 = constrain_float(lights1 + step, 0.0, 1000.0);
            } else {
                lights1 = constrain_float(lights1 - step, 0.0, 1000.0);
            }
            if (lights1 >= 1000.0 || lights1 <= 0.0) {
                increasing = !increasing;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, lights1);
        }
        break;
    case JSButton::button_function_t::k_lights1_brighter:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights1 = constrain_float(lights1 + step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, lights1);
        }
        break;
    case JSButton::button_function_t::k_lights1_dimmer:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights1 = constrain_float(lights1 - step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, lights1);
        }
        break;
    case JSButton::button_function_t::k_lights2_cycle:
       if (!held) {
            static bool increasing = true;
            uint16_t step = 1000.0 / g.lights_steps;
            if (increasing) {
                lights2 = constrain_float(lights2 + step, 0.0, 1000.0);
            } else {
                lights2 = constrain_float(lights2 - step, 0.0, 1000.0);
            }
            if (lights2 >= 1000.0 || lights2 <= 0.0) {
                increasing = !increasing;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, lights2);
        }
        break;
    case JSButton::button_function_t::k_lights2_brighter:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights2 = constrain_float(lights2 + step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, lights2);
        }
        break;
    case JSButton::button_function_t::k_lights2_dimmer:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights2 = constrain_float(lights2 - step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, lights2);
        }
        break;
    case JSButton::button_function_t::k_gain_toggle:
        if (!held) {
            static bool lowGain = false;
            lowGain = !lowGain;
            if (lowGain) {
                gain = 0.5f;
            } else {
                gain = 1.0f;
            }
            gcs().send_text(MAV_SEVERITY_INFO,"#Gain: %2.0f%%",(double)gain*100);
        }
        break;
    case JSButton::button_function_t::k_gain_inc:
        if (!held) {
            // check that our gain parameters are in correct range, update in eeprom and notify gcs if needed
            g.minGain.set_and_save(constrain_float(g.minGain, 0.10, 0.80));
            g.maxGain.set_and_save(constrain_float(g.maxGain, g.minGain, 1.0));
            g.numGainSettings.set_and_save(constrain_int16(g.numGainSettings, 1, 10));

            if (g.numGainSettings == 1) {
                gain = constrain_float(g.gain_default, g.minGain, g.maxGain);
            } else {
                gain = constrain_float(gain + (g.maxGain-g.minGain)/(g.numGainSettings-1), g.minGain, g.maxGain);
            }

            gcs().send_text(MAV_SEVERITY_INFO,"#Gain is %2.0f%%",(double)gain*100);
        }
        break;
    case JSButton::button_function_t::k_gain_dec:
        if (!held) {
            // check that our gain parameters are in correct range, update in eeprom and notify gcs if needed
            g.minGain.set_and_save(constrain_float(g.minGain, 0.10, 0.80));
            g.maxGain.set_and_save(constrain_float(g.maxGain, g.minGain, 1.0));
            g.numGainSettings.set_and_save(constrain_int16(g.numGainSettings, 1, 10));

            if (g.numGainSettings == 1) {
                gain = constrain_float(g.gain_default, g.minGain, g.maxGain);
            } else {
                gain = constrain_float(gain - (g.maxGain-g.minGain)/(g.numGainSettings-1), g.minGain, g.maxGain);
            }

            gcs().send_text(MAV_SEVERITY_INFO,"#Gain is %2.0f%%",(double)gain*100);
        }
        break;
    case JSButton::button_function_t::k_trim_roll_inc:
        rollTrim = constrain_float(rollTrim+10,-200,200);
        break;
    case JSButton::button_function_t::k_trim_roll_dec:
        rollTrim = constrain_float(rollTrim-10,-200,200);
        break;
    case JSButton::button_function_t::k_trim_pitch_inc:
        pitchTrim = constrain_float(pitchTrim+10,-200,200);
        break;
    case JSButton::button_function_t::k_trim_pitch_dec:
        pitchTrim = constrain_float(pitchTrim-10,-200,200);
        break;
    case JSButton::button_function_t::k_input_hold_set:
        if(!motors.armed()) {
            break;
        }
        if (!held) {
            zTrim = abs(z_last-500) > 50 ? z_last-500 : 0;
            xTrim = abs(x_last) > 50 ? x_last : 0;
            yTrim = abs(y_last) > 50 ? y_last : 0;
            bool input_hold_engaged_last = input_hold_engaged;
            input_hold_engaged = zTrim || xTrim || yTrim;
            if (input_hold_engaged) {
                gcs().send_text(MAV_SEVERITY_INFO,"#Input Hold Set");
            } else if (input_hold_engaged_last) {
                gcs().send_text(MAV_SEVERITY_INFO,"#Input Hold Disabled");
            }
            controls_reset_since_input_hold = !input_hold_engaged;
        }
        break;
#if AP_RELAY_ENABLED
    case JSButton::button_function_t::k_relay_1_on:
        relay.on(0);
        break;
    case JSButton::button_function_t::k_relay_1_off:
        relay.off(0);
        break;
    case JSButton::button_function_t::k_relay_1_toggle:
        if (!held) {
            relay.toggle(0);
        }
        break;
    case JSButton::button_function_t::k_relay_1_momentary:
        if (!held) {
            relay.on(0);
        }
        break;
    case JSButton::button_function_t::k_relay_2_on:
        relay.on(1);
        break;
    case JSButton::button_function_t::k_relay_2_off:
        relay.off(1);
        break;
    case JSButton::button_function_t::k_relay_2_toggle:
        if (!held) {
            relay.toggle(1);
        }
        break;
    case JSButton::button_function_t::k_relay_2_momentary:
        if (!held) {
            relay.on(1);
        }
        break;
    case JSButton::button_function_t::k_relay_3_on:
        relay.on(2);
        break;
    case JSButton::button_function_t::k_relay_3_off:
        relay.off(2);
        break;
    case JSButton::button_function_t::k_relay_3_toggle:
        if (!held) {
            relay.toggle(2);
        }
        break;
    case JSButton::button_function_t::k_relay_3_momentary:
        if (!held) {
            relay.on(2);
        }
        break;
    case JSButton::button_function_t::k_relay_4_on:
        relay.on(3);
        break;
    case JSButton::button_function_t::k_relay_4_off:
        relay.off(3);
        break;
    case JSButton::button_function_t::k_relay_4_toggle:
        if (!held) {
            relay.toggle(3);
        }
        break;
    case JSButton::button_function_t::k_relay_4_momentary:
        if (!held) {
            relay.on(3);
        }
        break;
#endif

    ////////////////////////////////////////////////
    // Servo functions
#if AP_SERVORELAYEVENTS_ENABLED
    case JSButton::button_function_t::k_servo_1_inc:
        sub.g2.actuators.increase_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_inc:
        sub.g2.actuators.increase_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_inc:
        sub.g2.actuators.increase_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_inc:
        sub.g2.actuators.increase_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_inc:
        sub.g2.actuators.increase_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_inc:
        sub.g2.actuators.increase_actuator(5);
        break;

    case JSButton::button_function_t::k_servo_1_dec:
        sub.g2.actuators.decrease_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_dec:
        sub.g2.actuators.decrease_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_dec:
        sub.g2.actuators.decrease_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_dec:
        sub.g2.actuators.decrease_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_dec:
        sub.g2.actuators.decrease_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_dec:
        sub.g2.actuators.decrease_actuator(5);
        break;

    case JSButton::button_function_t::k_servo_1_min:
    case JSButton::button_function_t::k_servo_1_min_momentary:
        sub.g2.actuators.min_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_min:
    case JSButton::button_function_t::k_servo_2_min_momentary:
        sub.g2.actuators.min_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_min:
    case JSButton::button_function_t::k_servo_3_min_momentary:
        sub.g2.actuators.min_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_min:
    case JSButton::button_function_t::k_servo_4_min_momentary:
        sub.g2.actuators.min_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_min:
    case JSButton::button_function_t::k_servo_5_min_momentary:
        sub.g2.actuators.min_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_min:
    case JSButton::button_function_t::k_servo_6_min_momentary:
        sub.g2.actuators.min_actuator(5);
        break;

    case JSButton::button_function_t::k_servo_1_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(0);
        }
        break;
    case JSButton::button_function_t::k_servo_2_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(1);
        }
        break;
    case JSButton::button_function_t::k_servo_3_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(2);
        }
        break;
    case JSButton::button_function_t::k_servo_4_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(3);
        }
        break;
    case JSButton::button_function_t::k_servo_5_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(4);
        }
        break;
    case JSButton::button_function_t::k_servo_6_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(5);
        }
        break;

    case JSButton::button_function_t::k_servo_1_max:
    case JSButton::button_function_t::k_servo_1_max_momentary:
        sub.g2.actuators.max_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_max:
    case JSButton::button_function_t::k_servo_2_max_momentary:
        sub.g2.actuators.max_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_max:
    case JSButton::button_function_t::k_servo_3_max_momentary:
        sub.g2.actuators.max_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_max:
    case JSButton::button_function_t::k_servo_4_max_momentary:
        sub.g2.actuators.max_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_max:
    case JSButton::button_function_t::k_servo_5_max_momentary:
        sub.g2.actuators.max_actuator(4);
        break;

    case JSButton::button_function_t::k_servo_1_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(0);
        }
        break;
    case JSButton::button_function_t::k_servo_2_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(1);
        }
        break;
    case JSButton::button_function_t::k_servo_3_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(2);
        }
        break;
    case JSButton::button_function_t::k_servo_4_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(3);
        }
        break;
    case JSButton::button_function_t::k_servo_5_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(4);
        }
        break;
    case JSButton::button_function_t::k_servo_6_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(5);
        }
        break;

    case JSButton::button_function_t::k_servo_1_center:
        sub.g2.actuators.center_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_center:
        sub.g2.actuators.center_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_center:
        sub.g2.actuators.center_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_center:
        sub.g2.actuators.center_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_center:
        sub.g2.actuators.center_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_center:
        sub.g2.actuators.center_actuator(5);
        break;
#endif  // AP_SERVORELAYEVENTS_ENABLED

    case JSButton::button_function_t::k_roll_pitch_toggle:
        if (!held) {
            roll_pitch_flag = !roll_pitch_flag;
            if (roll_pitch_flag) {
                gcs().send_text(MAV_SEVERITY_INFO, "#Attitude Control");
            }
            else {
                gcs().send_text(MAV_SEVERITY_INFO, "#Movement Control");
            }
        }
        break;

    case JSButton::button_function_t::k_custom_1:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_2:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_3:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_4:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_5:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_6:
        // Not implemented
        break;

#if AP_SCRIPTING_ENABLED
    case JSButton::button_function_t::k_script_1:
        sub.script_buttons[0].press();
        break;
    case JSButton::button_function_t::k_script_2:
        sub.script_buttons[1].press();
        break;
    case JSButton::button_function_t::k_script_3:
        sub.script_buttons[2].press();
        break;
    case JSButton::button_function_t::k_script_4:
        sub.script_buttons[3].press();
        break;
#endif // AP_SCRIPTING_ENABLED
    }
}

void Sub::handle_jsbutton_release(uint8_t _button, bool shift) {

    // Act based on the function assigned to this button
    switch (get_button(_button)->function(shift)) {
#if AP_RELAY_ENABLED
    case JSButton::button_function_t::k_relay_1_momentary:
        relay.off(0);
        break;
    case JSButton::button_function_t::k_relay_2_momentary:
        relay.off(1);
        break;
    case JSButton::button_function_t::k_relay_3_momentary:
        relay.off(2);
        break;
    case JSButton::button_function_t::k_relay_4_momentary:
        relay.off(3);
        break;
#endif
#if AP_SERVORELAYEVENTS_ENABLED
    case JSButton::button_function_t::k_servo_1_min_momentary:
    case JSButton::button_function_t::k_servo_1_max_momentary:
    {
        sub.g2.actuators.center_actuator(0);
    }
        break;
    case JSButton::button_function_t::k_servo_2_min_momentary:
    case JSButton::button_function_t::k_servo_2_max_momentary:
    {
        sub.g2.actuators.center_actuator(1);
    }
        break;
    case JSButton::button_function_t::k_servo_3_min_momentary:
    case JSButton::button_function_t::k_servo_3_max_momentary:
    {
        sub.g2.actuators.center_actuator(2);
    }
        break;
    case JSButton::button_function_t::k_servo_4_min_momentary:
    case JSButton::button_function_t::k_servo_4_max_momentary:
    {
        sub.g2.actuators.center_actuator(3);
    }
        break;
    case JSButton::button_function_t::k_servo_5_min_momentary:
    case JSButton::button_function_t::k_servo_5_max_momentary:
    {
        sub.g2.actuators.center_actuator(4);
    }
        break;
    case JSButton::button_function_t::k_servo_6_min_momentary:
    case JSButton::button_function_t::k_servo_6_max_momentary:
    {
        sub.g2.actuators.center_actuator(5);
    }
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case JSButton::button_function_t::k_script_1:
        sub.script_buttons[0].release();
        break;
    case JSButton::button_function_t::k_script_2:
        sub.script_buttons[1].release();
        break;
    case JSButton::button_function_t::k_script_3:
        sub.script_buttons[2].release();
        break;
    case JSButton::button_function_t::k_script_4:
        sub.script_buttons[3].release();
        break;
#endif // AP_SCRIPTING_ENABLED
    }
}

JSButton* Sub::get_button(uint8_t index)
{
    // Help to access appropriate parameter
    switch (index) {
    case 0:
        return &g.jbtn_0;
    case 1:
        return &g.jbtn_1;
    case 2:
        return &g.jbtn_2;
    case 3:
        return &g.jbtn_3;
    case 4:
        return &g.jbtn_4;
    case 5:
        return &g.jbtn_5;
    case 6:
        return &g.jbtn_6;
    case 7:
        return &g.jbtn_7;
    case 8:
        return &g.jbtn_8;
    case 9:
        return &g.jbtn_9;
    case 10:
        return &g.jbtn_10;
    case 11:
        return &g.jbtn_11;
    case 12:
        return &g.jbtn_12;
    case 13:
        return &g.jbtn_13;
    case 14:
        return &g.jbtn_14;
    case 15:
        return &g.jbtn_15;

    // add 16 more cases for 32 buttons with MANUAL_CONTROL extensions
    case 16:
        return &g.jbtn_16;
    case 17:
        return &g.jbtn_17;
    case 18:
        return &g.jbtn_18;
    case 19:
        return &g.jbtn_19;
    case 20:
        return &g.jbtn_20;
    case 21:
        return &g.jbtn_21;
    case 22:
        return &g.jbtn_22;
    case 23:
        return &g.jbtn_23;
    case 24:
        return &g.jbtn_24;
    case 25:
        return &g.jbtn_25;
    case 26:
        return &g.jbtn_26;
    case 27:
        return &g.jbtn_27;
    case 28:
        return &g.jbtn_28;
    case 29:
        return &g.jbtn_29;
    case 30:
        return &g.jbtn_30;
    case 31:
        return &g.jbtn_31;
    default:
        return &g.jbtn_0;
    }
}

void Sub::default_js_buttons()
{
    JSButton::button_function_t defaults[16][2] = {
        {JSButton::button_function_t::k_none,                   JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mode_manual,            JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mode_depth_hold,        JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mode_stabilize,         JSButton::button_function_t::k_none},

        {JSButton::button_function_t::k_disarm,                 JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_shift,                  JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_arm,                    JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mount_center,           JSButton::button_function_t::k_none},

        {JSButton::button_function_t::k_input_hold_set,         JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mount_tilt_down,        JSButton::button_function_t::k_mount_pan_left},
        {JSButton::button_function_t::k_mount_tilt_up,          JSButton::button_function_t::k_mount_pan_right},
        {JSButton::button_function_t::k_gain_inc,               JSButton::button_function_t::k_trim_pitch_dec},

        {JSButton::button_function_t::k_gain_dec,               JSButton::button_function_t::k_trim_pitch_inc},
        {JSButton::button_function_t::k_lights1_dimmer,         JSButton::button_function_t::k_trim_roll_dec},
        {JSButton::button_function_t::k_lights1_brighter,       JSButton::button_function_t::k_trim_roll_inc},
        {JSButton::button_function_t::k_none,                   JSButton::button_function_t::k_none},
    };

    for (int i = 0; i < 16; i++) {
        get_button(i)->set_default(defaults[i][0], defaults[i][1]);
    }
}

void Sub::set_neutral_controls()
{
    channel_roll->set_radio_in(channel_roll->get_radio_trim());
    channel_pitch->set_radio_in(channel_pitch->get_radio_trim());
    channel_yaw->set_radio_in(channel_yaw->get_radio_trim());
    channel_throttle->set_radio_in(channel_throttle->get_radio_trim());
    channel_forward->set_radio_in(channel_forward->get_radio_trim());
    channel_lateral->set_radio_in(channel_lateral->get_radio_trim());

    // Clear pitch/roll trim settings
    pitchTrim = 0;
    rollTrim  = 0;
}

void Sub::clear_input_hold()
{
    xTrim = 0;
    yTrim = 0;
    zTrim = 0;
    input_hold_engaged = false;
}

#if AP_SCRIPTING_ENABLED
bool Sub::is_button_pressed(uint8_t index)
{
    return script_buttons[index - 1].is_pressed();
}

uint8_t Sub::get_and_clear_button_count(uint8_t index)
{
    return script_buttons[index - 1].get_and_clear_count();
}
#endif // AP_SCRIPTING_ENABLED
