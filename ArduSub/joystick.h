#pragma once

/**
 * @brief Handle joystick/gamepad inputs for Sub
 */
class Joystick_Sub {
public:
    void init_joystick();
    void transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons);
    void handle_jsbutton_press(uint8_t button, bool shift, bool held);
    void handle_jsbutton_release(uint8_t button, bool shift);
    JSButton* get_button(uint8_t index);
    void default_js_buttons();
    void set_neutral_controls();
    void clear_input_hold();

private:
    uint16_t buttons_prev;
    float cam_tilt = 1500.0;
    float cam_pan = 1500.0;
    int16_t lights1 = 1100;
    int16_t lights2 = 1100;
    int16_t pitchTrim = 0;
    int16_t rollTrim = 0;
    int16_t video_switch = 1100;
    int16_t x_last, y_last, z_last;
    int16_t xTrim = 0, yTrim = 0, zTrim = 0;

    // Servo control output channels
    // TODO: Allow selecting output channels
    const uint8_t SERVO_CHAN_1 = 9; // Pixhawk Aux1
    const uint8_t SERVO_CHAN_2 = 10; // Pixhawk Aux2
    const uint8_t SERVO_CHAN_3 = 11; // Pixhawk Aux3

    uint8_t roll_pitch_flag = false; // Flag to adjust roll/pitch instead of forward/lateral
};