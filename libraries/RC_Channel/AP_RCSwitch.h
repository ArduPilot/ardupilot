#pragma once

#include <stdint.h>

class AP_RCSwitch {
public:

    void init();
    virtual void read_aux_all();

    // Aux Switch enumeration
    enum aux_func {
        DO_NOTHING =           0, // aux switch disabled
        FLIP =                 2, // flip
        SIMPLE_MODE =          3, // change to simple mode
        RTL =                  4, // change to RTL flight mode
        SAVE_TRIM =            5, // save current position as level
        SAVE_WP =              7, // save mission waypoint or RTL if in auto mode
        CAMERA_TRIGGER =       9, // trigger camera servo or relay
        RANGEFINDER =         10, // allow enabling or disabling rangefinder in flight which helps avoid surface tracking when you are far above the ground
        FENCE =               11, // allow enabling or disabling fence in flight
        RESETTOARMEDYAW =     12, // UNUSED
        SUPERSIMPLE_MODE =    13, // change to simple mode in middle, super simple at top
        ACRO_TRAINER =        14, // low = disabled, middle = leveled, high = leveled and limited
        SPRAYER =             15, // enable/disable the crop sprayer
        AUTO =                16, // change to auto flight mode
        AUTOTUNE =            17, // auto tune
        LAND =                18, // change to LAND flight mode
        GRIPPER =             19, // Operate cargo grippers low=off, middle=neutral, high=on
        PARACHUTE_ENABLE  =   21, // Parachute enable/disable
        PARACHUTE_RELEASE =   22, // Parachute release
        PARACHUTE_3POS =      23, // Parachute disable, enable, release with 3 position switch
        MISSION_RESET =       24, // Reset auto mission to start from first command
        ATTCON_FEEDFWD =      25, // enable/disable the roll and pitch rate feed forward
        ATTCON_ACCEL_LIM =    26, // enable/disable the roll, pitch and yaw accel limiting
        RETRACT_MOUNT =       27, // Retract Mount
        RELAY =               28, // Relay pin on/off (only supports first relay)
        LANDING_GEAR =        29, // Landing gear controller
        LOST_COPTER_SOUND =   30, // Play lost copter sound
        MOTOR_ESTOP =         31, // Emergency Stop Switch
        MOTOR_INTERLOCK =     32, // Motor On/Off switch
        BRAKE =               33, // Brake flight mode
        RELAY2 =              34, // Relay2 pin on/off (in Mission planner set RC8_OPTION  = 34)
        RELAY3 =              35, // Relay3 pin on/off (in Mission planner set RC9_OPTION  = 35)
        RELAY4 =              36, // Relay4 pin on/off (in Mission planner set RC10_OPTION = 36)
        THROW =               37,  // change to THROW flight mode
        AVOID_ADSB =          38,  // enable AP_Avoidance library
        PRECISION_LOITER =    39,  // enable precision loiter
        AVOID_PROXIMITY =     40,  // enable object avoidance using proximity sensors (ie. horizontal lidar)
        ARMDISARM =           41,  // arm or disarm vehicle
        SMART_RTL =           42, // change to SmartRTL flight mode
        INVERTED  =           43,  // enable inverted flight
        WINCH_ENABLE =        44, // winch enable/disable
        WINCH_CONTROL =       45, // winch control
        RC_OVERRIDE_ENABLE =  46, // enable RC Override
    };
    typedef enum aux_func aux_func_t;

    class RC_Channel *find_channel_for_option(const aux_func_t option) const;
    bool duplicate_options_exist() const;

protected:
    // values used by the ap.ch7_opt and ap.ch8_opt flags
    enum aux_switch_pos {
        LOW,       // indicates auxiliary switch is in the low position (pwm <1200)
        MIDDLE,    // indicates auxiliary switch is in the middle position (pwm >1200, <1800)
        HIGH       // indicates auxiliary switch is in the high position (pwm >1800)
    };

    typedef enum aux_switch_pos aux_switch_pos_t;

    void init_aux_switches();
    virtual void init_aux_function(aux_func_t ch_option, aux_switch_pos_t);
    virtual void do_aux_function(aux_func_t ch_option, aux_switch_pos_t) = 0;
    virtual void reset_control_switch() = 0;

    virtual bool in_rc_failsafe() const = 0;

private:

    //Documentation of Aux Switch Flags:
    // 0 is low or false, 1 is center or true, 2 is high
    // pairs of bits in old_switch_positions give the old switch position for an RC input.
    uint32_t old_switch_positions;

    aux_switch_pos_t old_switch_position(const uint8_t chan) {
        return (aux_switch_pos_t)((old_switch_positions >> (chan*2)) & 0x3);
    }
    void set_old_switch_position(const uint8_t chan, const uint8_t value) {
        old_switch_positions &= ~(0x3 << (chan*2));
        old_switch_positions |= (value << (chan*2));
    }

    aux_switch_pos_t read_3pos_switch(const RC_Channel *channel) const;

    // pwm value above which the option will be invoked:
    static const uint16_t AUX_PWM_TRIGGER_HIGH = 1800;
    // pwm value below which the option will be disabled:
    static const uint16_t AUX_PWM_TRIGGER_LOW = 1200;


};
