#include "Sub.h"

// Functions that will handle joystick/gamepad input
// ----------------------------------------------------------------------------

// Anonymous namespace to hold variables used only in this file
namespace {
float cam_tilt = 1500.0;
int16_t lights1 = 1100;
int16_t lights2 = 1100;
int16_t rollTrim = 0;
int16_t pitchTrim = 0;
int16_t zTrim = 0;
int16_t xTrim = 0;
int16_t yTrim = 0;
int16_t video_switch = 1100;
int16_t x_last, y_last, z_last;
uint16_t buttons_prev;
float gain;

// Servo control output channels
// TODO: Allow selecting output channels
const uint8_t SERVO_CHAN_1 = 9; // Pixhawk Aux1
const uint8_t SERVO_CHAN_2 = 10; // Pixhawk Aux2
const uint8_t SERVO_CHAN_3 = 11; // Pixhawk Aux3

uint8_t roll_pitch_flag = false; // Flag to adjust roll/pitch instead of forward/lateral
}

void Sub::init_joystick()
{
    default_js_buttons();

    set_mode(MANUAL, MODE_REASON_TX_COMMAND); // Initialize flight mode

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
}

void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons)
{

    int16_t channels[11];

    uint32_t tnow_ms = millis();

    float rpyScale = 0.4*gain; // Scale -1000-1000 to -400-400 with gain
    float throttleScale = 0.8*gain*g.throttle_gain; // Scale 0-1000 to 0-800 times gain
    int16_t rpyCenter = 1500;
    int16_t throttleBase = 1500-500*throttleScale;

    bool shift = false;
    static uint32_t buttonDebounce;

    // Debouncing timer
    if (tnow_ms - buttonDebounce > 100) {
        // Detect if any shift button is pressed
        for (uint8_t i = 0 ; i < 16 ; i++) {
            if ((buttons & (1 << i)) && get_button(i)->function() == JSButton::button_function_t::k_shift) {
                shift = true;
            }
        }

        // Act if button is pressed
        // Only act upon pressing button and ignore holding. This provides compatibility with Taranis as joystick.
        for (uint8_t i = 0 ; i < 16 ; i++) {
            if ((buttons & (1 << i))) {
                handle_jsbutton_press(i,shift,(buttons_prev & (1 << i)));
                buttonDebounce = tnow_ms;
            }
        }

        buttons_prev = buttons;
    }

    // Set channels to override
    if (!roll_pitch_flag) {
        channels[0] = 1500 + pitchTrim; // pitch
        channels[1] = 1500 + rollTrim;  // roll
    } else {
        // adjust roll and pitch with joystick input instead of forward and lateral
        channels[0] = constrain_int16((x+pitchTrim)*rpyScale+rpyCenter,1100,1900);
        channels[1] = constrain_int16((y+rollTrim)*rpyScale+rpyCenter,1100,1900);
    }

    channels[2] = constrain_int16((z+zTrim)*throttleScale+throttleBase,1100,1900); // throttle
    channels[3] = constrain_int16(r*rpyScale+rpyCenter,1100,1900);                 // yaw

    if (!roll_pitch_flag) {
        // adjust forward and lateral with joystick input instead of roll and pitch
        channels[4] = constrain_int16((x+xTrim)*rpyScale+rpyCenter,1100,1900); // forward for ROV
        channels[5] = constrain_int16((y+yTrim)*rpyScale+rpyCenter,1100,1900); // lateral for ROV
    } else {
        // neutralize forward and lateral input while we are adjusting roll and pitch
        channels[4] = constrain_int16(xTrim*rpyScale+rpyCenter,1100,1900); // forward for ROV
        channels[5] = constrain_int16(yTrim*rpyScale+rpyCenter,1100,1900); // lateral for ROV
    }

    channels[6] = 0;             // Unused
    channels[7] = cam_tilt;      // camera tilt
    channels[8] = lights1;       // lights 1
    channels[9] = lights2;       // lights 2
    channels[10] = video_switch; // video switch

    // Store old x, y, z values for use in input hold logic
    x_last = x;
    y_last = y;
    z_last = z;

    hal.rcin->set_overrides(channels, 10);
}

void Sub::handle_jsbutton_press(uint8_t button, bool shift, bool held)
{
    // Act based on the function assigned to this button
    switch (get_button(button)->function(shift)) {
    case JSButton::button_function_t::k_arm_toggle:
        if (motors.armed()) {
            init_disarm_motors();
        } else {
            init_arm_motors(true);
        }
        break;
    case JSButton::button_function_t::k_arm:
        init_arm_motors(true);
        break;
    case JSButton::button_function_t::k_disarm:
        init_disarm_motors();
        break;

    case JSButton::button_function_t::k_mode_manual:
        set_mode(MANUAL, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_stabilize:
        set_mode(STABILIZE, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_depth_hold:
        set_mode(ALT_HOLD, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_auto:
        set_mode(AUTO, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_guided:
        set_mode(GUIDED, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_circle:
        set_mode(CIRCLE, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_acro:
        set_mode(ACRO, MODE_REASON_TX_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_poshold:
        set_mode(POSHOLD, MODE_REASON_TX_COMMAND);
        break;

    case JSButton::button_function_t::k_mount_center:
        cam_tilt = g.cam_tilt_center;
        break;
    case JSButton::button_function_t::k_mount_tilt_up: {
        uint8_t i;

        // Find the first aux channel configured as mount tilt, if any
        if (SRV_Channels::find_channel(SRV_Channel::k_mount_tilt, i)) {

            // Get the channel output limits
            SRV_Channel *ch = SRV_Channels::srv_channel(i);
            uint16_t min = ch->get_output_min();
            uint16_t max = ch->get_output_max();

            cam_tilt = constrain_int16(cam_tilt-g.cam_tilt_step,min,max);
        }
    }
    break;
    case JSButton::button_function_t::k_mount_tilt_down: {
        uint8_t i;

        // Find the first aux channel configured as mount tilt, if any
        if (SRV_Channels::find_channel(SRV_Channel::k_mount_tilt, i)) {

            // Get the channel output limits
            SRV_Channel *ch = SRV_Channels::srv_channel(i);
            uint16_t min = ch->get_output_min();
            uint16_t max = ch->get_output_max();

            cam_tilt = constrain_int16(cam_tilt+g.cam_tilt_step,min,max);
        }
    }
    break;
    case JSButton::button_function_t::k_camera_trigger:
        break;
    case JSButton::button_function_t::k_camera_source_toggle:
        if (!held) {
            static bool video_toggle = false;
            video_toggle = !video_toggle;
            if (video_toggle) {
                video_switch = 1900;
                gcs().send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 2");
            } else {
                video_switch = 1100;
                gcs().send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 1");
            }
        }
        break;
    case JSButton::button_function_t::k_mount_pan_right:
        // Not implemented
        break;
    case JSButton::button_function_t::k_mount_pan_left:
        // Not implemented
        break;
    case JSButton::button_function_t::k_lights1_cycle:
        if (!held) {
            static bool increasing = true;
            if (increasing) {
                lights1 = constrain_float(lights1+g.lights_step,1100,1900);
            } else {
                lights1 = constrain_float(lights1-g.lights_step,1100,1900);
            }
            if (lights1 >= 1900 || lights1 <= 1100) {
                increasing = !increasing;
            }
        }
        break;
    case JSButton::button_function_t::k_lights1_brighter:
        if (!held) {
            lights1 = constrain_float(lights1+g.lights_step,1100,1900);
        }
        break;
    case JSButton::button_function_t::k_lights1_dimmer:
        if (!held) {
            lights1 = constrain_float(lights1-g.lights_step,1100,1900);
        }
        break;
    case JSButton::button_function_t::k_lights2_cycle:
        if (!held) {
            static bool increasing = true;
            if (increasing) {
                lights2 = constrain_float(lights2+g.lights_step,1100,1900);
            } else {
                lights2 = constrain_float(lights2-g.lights_step,1100,1900);
            }
            if (lights2 >= 1900 || lights2 <= 1100) {
                increasing = !increasing;
            }
        }
        break;
    case JSButton::button_function_t::k_lights2_brighter:
        if (!held) {
            lights2 = constrain_float(lights2+g.lights_step,1100,1900);
        }
        break;
    case JSButton::button_function_t::k_lights2_dimmer:
        if (!held) {
            lights2 = constrain_float(lights2-g.lights_step,1100,1900);
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
    case JSButton::button_function_t::k_input_hold_toggle:
        if (!held) {
            zTrim = z_last-500;
            xTrim = x_last;
            yTrim = y_last;
            gcs().send_text(MAV_SEVERITY_INFO,"#Input Hold Set");
        }
        break;
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

    ////////////////////////////////////////////////
    // Servo functions
    // TODO: initialize
    case JSButton::button_function_t::k_servo_1_inc:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_1 - 1); // 0-indexed
        uint16_t pwm_out = hal.rcout->read(SERVO_CHAN_1 - 1); // 0-indexed
        pwm_out = constrain_int16(pwm_out + 50, chan->get_output_min(), chan->get_output_max());
        ServoRelayEvents.do_set_servo(SERVO_CHAN_1, pwm_out); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_1_dec:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_1 - 1); // 0-indexed
        uint16_t pwm_out = hal.rcout->read(SERVO_CHAN_1 - 1); // 0-indexed
        pwm_out = constrain_int16(pwm_out - 50, chan->get_output_min(), chan->get_output_max());
        ServoRelayEvents.do_set_servo(SERVO_CHAN_1, pwm_out); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_1_min:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_1 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_1, chan->get_output_min()); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_1_max:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_1 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_1, chan->get_output_max()); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_1_center:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_1 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_1, chan->get_trim()); // 1-indexed
    }
        break;

    case JSButton::button_function_t::k_servo_2_inc:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_2 - 1); // 0-indexed
        uint16_t pwm_out = hal.rcout->read(SERVO_CHAN_2 - 1); // 0-indexed
        pwm_out = constrain_int16(pwm_out + 50, chan->get_output_min(), chan->get_output_max());
        ServoRelayEvents.do_set_servo(SERVO_CHAN_2, pwm_out); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_2_dec:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_2 - 1); // 0-indexed
        uint16_t pwm_out = hal.rcout->read(SERVO_CHAN_2 - 1); // 0-indexed
        pwm_out = constrain_int16(pwm_out - 50, chan->get_output_min(), chan->get_output_max());
        ServoRelayEvents.do_set_servo(SERVO_CHAN_2, pwm_out); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_2_min:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_2 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_2, chan->get_output_min()); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_2_max:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_2 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_2, chan->get_output_max()); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_2_center:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_2 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_2, chan->get_trim()); // 1-indexed
    }
        break;

    case JSButton::button_function_t::k_servo_3_inc:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_3 - 1); // 0-indexed
        uint16_t pwm_out = hal.rcout->read(SERVO_CHAN_3 - 1); // 0-indexed
        pwm_out = constrain_int16(pwm_out + 50, chan->get_output_min(), chan->get_output_max());
        ServoRelayEvents.do_set_servo(SERVO_CHAN_3, pwm_out); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_3_dec:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_3 - 1); // 0-indexed
        uint16_t pwm_out = hal.rcout->read(SERVO_CHAN_3 - 1); // 0-indexed
        pwm_out = constrain_int16(pwm_out - 50, chan->get_output_min(), chan->get_output_max());
        ServoRelayEvents.do_set_servo(SERVO_CHAN_3, pwm_out); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_3_min:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_3 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_3, chan->get_output_min()); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_3_max:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_3 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_3, chan->get_output_max()); // 1-indexed
    }
        break;
    case JSButton::button_function_t::k_servo_3_center:
    {
        SRV_Channel* chan = SRV_Channels::srv_channel(SERVO_CHAN_3 - 1); // 0-indexed
        ServoRelayEvents.do_set_servo(SERVO_CHAN_3, chan->get_trim()); // 1-indexed
    }
        break;

    case JSButton::button_function_t::k_roll_pitch_toggle:
        if (!held) {
            roll_pitch_flag = !roll_pitch_flag;
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

        {JSButton::button_function_t::k_input_hold_toggle,      JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mount_tilt_down,        JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mount_tilt_up,          JSButton::button_function_t::k_none},
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
    int16_t channels[11];

    for (uint8_t i = 0; i < 6; i++) {
        channels[i] = 1500;
    }

    for (uint8_t i = 6; i < 11; i++) {
        channels[i] = 0xffff;
    }

    channels[4] = 0xffff; // Leave mode switch where it was

    hal.rcin->set_overrides(channels, 10);
}
