#pragma once

/*
  class to support "toy" mode for simplified user interaction for
  large volume consumer vehicles
 */

class ToyMode
{
public:
    friend class Copter;

    ToyMode();
    bool enabled(void) const {
        return enable.get() != 0;
    }
    
    void update(void);

    // get throttle mid-point
    int16_t get_throttle_mid(void) {
        return throttle_mid;
    }

    // adjust throttle for throttle takeoff
    void throttle_adjust(float &throttle_control);

    // handle mavlink message
    void handle_message(const mavlink_message_t &msg);

    void load_test_run(void);
    
    static const struct AP_Param::GroupInfo var_info[];
    
private:

    void trim_update(void);
    void action_arm(void);
    void blink_update(void);
    void send_named_int(const char *name, int32_t value);
    bool set_and_remember_mode(Mode::Number mode, ModeReason reason);

    void thrust_limiting(float *thrust, uint8_t num_motors);
    void arm_check_compass(void);

    // work out type of button setup
    bool is_v2450_buttons(void) const {
        return enable == 1;
    }
    bool is_f412_buttons(void) const {
        return enable == 2;
    }
    
    enum toy_action {
        ACTION_NONE         = 0,
        ACTION_TAKE_PHOTO   = 1,
        ACTION_TOGGLE_VIDEO = 2,
        ACTION_MODE_ACRO    = 3,
        ACTION_MODE_ALTHOLD = 4,
        ACTION_MODE_AUTO    = 5,
        ACTION_MODE_LOITER  = 6,
        ACTION_MODE_RTL     = 7,
        ACTION_MODE_CIRCLE  = 8,
        ACTION_MODE_LAND    = 9,
        ACTION_MODE_DRIFT   = 10,
        ACTION_MODE_SPORT   = 11,
        ACTION_MODE_AUTOTUNE= 12,
        ACTION_MODE_POSHOLD = 13,
        ACTION_MODE_BRAKE   = 14,
        ACTION_MODE_THROW   = 15,
        ACTION_MODE_FLIP    = 16,
        ACTION_MODE_STAB    = 17,
        ACTION_DISARM       = 18,
        ACTION_TOGGLE_MODE  = 19,
        ACTION_ARM_LAND_RTL = 20,
        ACTION_TOGGLE_SIMPLE = 21,
        ACTION_TOGGLE_SSIMPLE = 22,
        ACTION_LOAD_TEST = 23,
        ACTION_MODE_FLOW = 24,
    };

    enum toy_action last_action;

    // these are bitmask indexes for TMODE_FLAGS
    enum toy_flags {
        FLAG_THR_DISARM     = 1<<0,  // disarm on low throttle
        FLAG_THR_ARM        = 1<<1,  // arm on high throttle
        FLAG_UPGRADE_LOITER = 1<<2,  // auto upgrade from ALT_HOLD to LOITER
        FLAG_RTL_CANCEL     = 1<<3,  // cancel RTL on large stick input
    };

    enum blink_patterns {
        BLINK_FULL   = 0xFFFF,
        BLINK_OFF    = 0x0000,
        BLINK_1      = 0xBFFF,
        BLINK_2      = 0xAFFF,
        BLINK_3      = 0xABFF,
        BLINK_4      = 0xAAFF,
        BLINK_6      = 0xAAAF,
        BLINK_8      = 0xAAAA,
        BLINK_NO_RX  = 0x1111,
        BLINK_SLOW_1 = 0xF0FF,
        BLINK_VSLOW  = 0xF000,
        BLINK_MED_1  = 0xF0F0,
    };

    bool done_first_update;
    AP_Int8 enable;
    AP_Int8 primary_mode[2];
    AP_Int8 actions[9];
    AP_Int8 trim_auto;
    AP_Int16 flags;

    struct {
        uint32_t start_ms;
        uint16_t chan[4];
    } trim;
    
    uint32_t power_counter;
    uint32_t throttle_low_counter;
    uint32_t throttle_high_counter;
    uint16_t last_ch5;
    bool last_left_button;
    uint8_t last_mode_choice;
    int32_t left_press_counter;
    int32_t right_press_counter;
    bool ignore_left_change;
    int16_t throttle_mid = 500;
    uint32_t throttle_arm_ms;
    bool upgrade_to_loiter;
    uint32_t last_action_ms;
    uint32_t reset_turtle_start_ms;

    // time when we were last told we are recording video
    uint32_t last_video_ms;
    
    // current blink indexes
    uint16_t red_blink_pattern;
    uint16_t green_blink_pattern;
    uint8_t red_blink_index;
    uint8_t green_blink_index;
    uint16_t red_blink_count;
    uint16_t green_blink_count;
    uint8_t blink_disarm;

    struct {
        AP_Float volt_min;
        AP_Float volt_max;
        AP_Float thrust_min;
        AP_Float thrust_max;
    } filter;
    
    // low-pass voltage
    float filtered_voltage = 4.0;

    uint8_t motor_log_counter;

    // remember the last mode we set
    Mode::Number last_set_mode = Mode::Number::LOITER;

    struct load_data {
        uint16_t m[4];
    };

    enum load_type {
        LOAD_TYPE_CONSTANT=0,
        LOAD_TYPE_LOG1=1,
        LOAD_TYPE_LOG2=2,
    };
    
    struct {
        bool running;
        uint32_t row;
        uint8_t filter_counter;
        AP_Float load_mul;
        AP_Int8  load_filter;
        AP_Int8  load_type;
    } load_test;
    
    static const struct load_data load_data1[];
};
