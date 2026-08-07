#pragma once

#if AP_PERIPH_BATTERY_BMS_ENABLED

class BatteryBMS {
public:
    friend class AP_Periph_FW;

    // main update function
    void update(void);

private:

    // configure gpio pins. returns true once configured
    bool configured();

    // check and handle button press events
    void handle_button_press();

    // request display of battery SOC percentage using LEDs
    void request_display_percentage();

    // display battery SOC percentage using LEDs
    void display_percentage();

    // get battery SOC percentage (0-100). returns true on success
    bool get_percentage(uint8_t &percentage);

    // set LED pattern based on 8-bit bitmask
    void set_led_pattern(uint8_t pattern);

    // update the LEDs. called regularly from update()
    void update_led_state(void);

    // BMS state machine variables
    enum class BmsState : uint8_t {
        IDLE = 0,
        POWERING_ON,
        POWERED_ON,
        POWERING_OFF,
        POWERED_OFF
    };

    // request change in bms state. the only valid inputs are POWERED_ON and POWERING_OFF
    // return true on success
    bool request_bms_state(BmsState new_state);

    // update bms state.  transitions bms_state to req_bms_state
    void update_bms_state();

    // configuration variables
    bool config_complete;           // true once configuration has been completed

    // button handling variables
    struct {
        bool startup_complete;      // true once startup delay has completed
        bool pressed_prev;          // true if button was pressed during previous iteration
        uint32_t pressed_start_ms;  // system time that button was first detected as pressed
        bool short_press_handled;   // true once a short press has been detected and handled
        bool long_press_handled;    // true once a long pressed has been detected and handled
    } button;
    static const uint32_t BUTTON_SHORT_PRESS_THRESHOLD_MS = 10; // 10 ms for short press
    static const uint32_t BUTTON_LONG_PRESS_THRESHOLD_MS = 1000; // 1 second for long press
    static const uint32_t BUTTON_STARTUP_DELAY_MS = 2000; // ignore button presses for first 2 seconds after startup

    // BMS state machine variables
    BmsState bms_state, req_bms_state;  // current and requested BMS states
    uint32_t bms_last_update_ms;        // system time of last state machine update
    uint8_t bms_transition_counter;     // transition counter also used for animation

    // LED display variables
    uint32_t led_last_update_ms;        // system time of last LED update.  used to rate limit LED updates to 20hz
    uint32_t led_display_soc_start_ms;  // system time that SOC display started.  0 if not displaying SOC
    uint8_t led_charging_animation_step; // LED charging animation step
    static const uint32_t LED_UPDATE_INTERVAL_MS = 50;  // update LEDs at 20hz
    static const uint32_t LED_DISPLAY_SOC_DURATION_MS = 1000;   // Display SOC percentage for 1 second
    static const uint8_t led_gpios[];   // GPIO pins used for BMS LEDs
};

#endif // AP_PERIPH_BATTERY_BMS_ENABLED

