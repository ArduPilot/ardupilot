#include "Copter.h"

#if TOY_MODE_ENABLED

// times in 0.1s units
#define TOY_COMMAND_DELAY 15
#define TOY_LONG_PRESS_COUNT 15
#define TOY_LAND_MANUAL_DISARM_COUNT 40
#define TOY_LAND_DISARM_COUNT 1
#define TOY_LAND_ARM_COUNT 1
#define TOY_RIGHT_PRESS_COUNT 1
#define TOY_ACTION_DELAY_MS 200
#define TOY_DESCENT_SLOW_HEIGHT 5
#define TOY_DESCENT_SLOW_RAMP 3
#define TOY_DESCENT_SLOW_MIN 300
#define TOY_RESET_TURTLE_TIME 5000

#define ENABLE_LOAD_TEST 0

const AP_Param::GroupInfo ToyMode::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: tmode enable 
    // @Description: tmode (or "toy" mode) gives a simplified user interface designed for mass market drones. Version1 is for the SkyViper V2450GPS. Version2 is for the F412 based boards
    // @Values: 0:Disabled,1:EnableVersion1,2:EnableVersion2
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, ToyMode, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MODE1
    // @DisplayName: Tmode first mode
    // @Description: This is the initial mode when the vehicle is first turned on. This mode is assumed to not require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:FlowHold
    // @User: Standard
    AP_GROUPINFO("_MODE1", 2, ToyMode, primary_mode[0], (float)Mode::Number::ALT_HOLD),

    // @Param: _MODE2
    // @DisplayName: Tmode second mode
    // @Description: This is the secondary mode. This mode is assumed to require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:FlowHold
    // @User: Standard
    AP_GROUPINFO("_MODE2", 3, ToyMode, primary_mode[1], (float)Mode::Number::LOITER),

    // @Param: _ACTION1
    // @DisplayName: Tmode action 1
    // @Description: This is the action taken when the left action button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION1", 4, ToyMode, actions[0], ACTION_TOGGLE_VIDEO),

    // @Param: _ACTION2
    // @DisplayName: Tmode action 2
    // @Description: This is the action taken when the right action button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION2", 5, ToyMode, actions[1], ACTION_TAKE_PHOTO),

    // @Param: _ACTION3
    // @DisplayName: Tmode action 3
    // @Description: This is the action taken when the power button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION3", 6, ToyMode, actions[2], ACTION_DISARM),

    // @Param: _ACTION4
    // @DisplayName: Tmode action 4
    // @Description: This is the action taken when the left action button is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION4", 7, ToyMode, actions[3], ACTION_NONE),

    // @Param: _ACTION5
    // @DisplayName: Tmode action 5
    // @Description: This is the action taken when the right action is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION5", 8, ToyMode, actions[4], ACTION_NONE),

    // @Param: _ACTION6
    // @DisplayName: Tmode action 6
    // @Description: This is the action taken when the power button is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION6", 9, ToyMode, actions[5], ACTION_NONE),

    // @Param: _LEFT
    // @DisplayName: Tmode left action
    // @Description: This is the action taken when the left (Mode) button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_LEFT", 10, ToyMode, actions[6], ACTION_TOGGLE_MODE),

    // @Param: _LEFT_LONG
    // @DisplayName: Tmode left long action
    // @Description: This is the action taken when the left (Mode) button is long-pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_LEFT_LONG", 11, ToyMode, actions[7], ACTION_NONE),

    // @Param: _TRIM_AUTO
    // @DisplayName: Stick auto trim limit
    // @Description: This is the amount of automatic stick trim that can be applied when disarmed with sticks not moving. It is a PWM limit value away from 1500
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_TRIM_AUTO", 12, ToyMode, trim_auto, 50),

    // @Param: _RIGHT
    // @DisplayName: Tmode right action
    // @Description: This is the action taken when the right (Return) button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest
    // @User: Standard
    AP_GROUPINFO("_RIGHT", 13, ToyMode, actions[8], ACTION_ARM_LAND_RTL),

    // @Param: _FLAGS
    // @DisplayName: Tmode flags
    // @Description: Bitmask of flags to change the behaviour of tmode. DisarmOnLowThrottle means to disarm if throttle is held down for 1 second when landed. ArmOnHighThrottle means to arm if throttle is above 80% for 1 second. UpgradeToLoiter means to allow takeoff in LOITER mode by switching to ALT_HOLD, then auto-upgrading to LOITER once GPS is available. RTLStickCancel means that on large stick inputs in RTL mode that LOITER mode is engaged
    // @Bitmask: 0:DisarmOnLowThrottle,1:ArmOnHighThrottle,2:UpgradeToLoiter,3:RTLStickCancel
    // @User: Standard
    AP_GROUPINFO("_FLAGS", 14, ToyMode, flags, FLAG_THR_DISARM),

    // @Param: _VMIN
    // @DisplayName: Min voltage for output limiting
    // @Description: This is the battery voltage below which no output limiting is done
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_VMIN", 15, ToyMode, filter.volt_min, 3.5),

    // @Param: _VMAX
    // @DisplayName: Max voltage for output limiting
    // @Description: This is the battery voltage above which thrust min is used
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_VMAX", 16, ToyMode, filter.volt_max, 3.8),
    
    // @Param: _TMIN
    // @DisplayName: Min thrust multiplier
    // @Description: This sets the thrust multiplier when voltage is high
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TMIN", 17, ToyMode, filter.thrust_min, 1.0),

    // @Param: _TMAX
    // @DisplayName: Max thrust multiplier
    // @Description: This sets the thrust multiplier when voltage is low
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TMAX", 18, ToyMode, filter.thrust_max, 1.0),

#if ENABLE_LOAD_TEST
    // @Param: _LOAD_MUL
    // @DisplayName: Load test multiplier
    // @Description: This scales the load test output, as a value between 0 and 1
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_LOAD_MUL", 19, ToyMode, load_test.load_mul, 1.0),
    
    // @Param: _LOAD_FILT
    // @DisplayName: Load test filter
    // @Description: This filters the load test output. A value of 1 means no filter. 2 means values are repeated once. 3 means values are repeated 3 times, etc
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_LOAD_FILT", 20, ToyMode, load_test.load_filter, 1),
    
    // @Param: _LOAD_TYPE
    // @DisplayName: Load test type
    // @Description: This sets the type of load test
    // @Values: 0:ConstantThrust,1:LogReplay1,2:LogReplay2
    // @User: Advanced
    AP_GROUPINFO("_LOAD_TYPE", 21, ToyMode, load_test.load_type, LOAD_TYPE_LOG1),
#endif
   
    AP_GROUPEND
};

ToyMode::ToyMode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  special mode handling for toys
 */
void ToyMode::update()
{
    if (!enable) {
        // not enabled
        return;
    }

#if ENABLE_LOAD_TEST
    if (!copter.motors->armed()) {
        load_test.running = false;
    }
#endif

    // keep filtered battery voltage for thrust limiting
    filtered_voltage = 0.99 * filtered_voltage + 0.01 * copter.battery.voltage();
    
    // update LEDs
    blink_update();
    
    if (!done_first_update) {
        done_first_update = true;
        copter.set_mode(Mode::Number(primary_mode[0].get()), ModeReason::TOY_MODE);
        copter.motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&ToyMode::thrust_limiting, void, float *, uint8_t));
    }

    // check if we should auto-trim
    if (trim_auto > 0) {
        trim_update();
    }
            
    // set ALT_HOLD as indoors for the EKF (disables GPS vertical velocity fusion)
#if 0
    copter.ahrs.set_indoor_mode(copter.flightmode->mode_number() == ALT_HOLD || copter.flightmode->mode_number() == FLOWHOLD);
#endif
    
    bool left_button = false;
    bool right_button = false;
    bool left_action_button = false;
    bool right_action_button = false;
    bool power_button = false;
    bool left_change = false;
    
    uint16_t ch5_in = RC_Channels::get_radio_in(CH_5);
    uint16_t ch6_in = RC_Channels::get_radio_in(CH_6);
    uint16_t ch7_in = RC_Channels::get_radio_in(CH_7);

    if (!rc().has_valid_input() || ch5_in < 900) {
        // failsafe handling is outside the scope of toy mode, it does
        // normal failsafe actions, just setup a blink pattern
        green_blink_pattern = BLINK_NO_RX;
        red_blink_pattern = BLINK_NO_RX;
        red_blink_index = green_blink_index;
        return;
    }

    uint32_t now = AP_HAL::millis();
    
    if (is_v2450_buttons()) {
        // V2450 button mapping from cypress radio. It maps the
        // buttons onto channels 5, 6 and 7 in a complex way, with the
        // left button latching
        left_change = ((ch5_in > 1700 && last_ch5 <= 1700) || (ch5_in <= 1700 && last_ch5 > 1700));
        
        last_ch5 = ch5_in;
                        
        // get buttons from channels
        left_button = (ch5_in > 2050 || (ch5_in > 1050 && ch5_in < 1150));
        right_button = (ch6_in > 1500);
        uint8_t ch7_bits = (ch7_in>1000)?uint8_t((ch7_in-1000)/100):0;
        left_action_button = (ch7_bits&1) != 0;
        right_action_button = (ch7_bits&2) != 0;
        power_button = (ch7_bits&4) != 0;
    } else if (is_f412_buttons()) {
        // F412 button setup for cc2500 radio. This maps the 6 buttons
        // onto channels 5 and 6, with no latching
        uint8_t ch5_bits = (ch5_in>1000)?uint8_t((ch5_in-1000)/100):0;
        uint8_t ch6_bits = (ch6_in>1000)?uint8_t((ch6_in-1000)/100):0;
        left_button = (ch5_bits & 4) != 0;
        right_button = (ch5_bits & 2) != 0;
        right_action_button = (ch6_bits & 1) != 0;
        left_action_button = (ch6_bits & 2) != 0;
        power_button = (ch6_bits & 4) != 0;
        left_change = (left_button != last_left_button);
        last_left_button = left_button;
    }
    
    // decode action buttons into an action
    uint8_t action_input = 0;    
    if (left_action_button) {
        action_input = 1;
    } else if (right_action_button) {
        action_input = 2;
    } else if (power_button) {
        action_input = 3;
    }
    
    if (action_input != 0 && left_button) {
        // combined button actions
        action_input += 3;
        left_press_counter = 0;
    } else if (left_button) {
        left_press_counter++;
    } else {
        left_press_counter = 0;
    }

    bool reset_combination = left_action_button && right_action_button;
    if (reset_combination && fabsf(copter.ahrs.get_roll_deg()) > 160) {
        /*
          if both shoulder buttons are pressed at the same time for 5
          seconds while the vehicle is inverted then we send a
          WIFIRESET message to the sonix to reset SSID and password
        */
        if (reset_turtle_start_ms == 0) {
            reset_turtle_start_ms = now;
        }
        if (now - reset_turtle_start_ms > TOY_RESET_TURTLE_TIME) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: WiFi reset");
            reset_turtle_start_ms = 0;
            send_named_int("WIFIRESET", 1);
        }
    } else {
        reset_turtle_start_ms = 0;
    }
    if (reset_combination) {
        // don't act on buttons when combination pressed
        action_input = 0;
        left_press_counter = 0;
    }

    /*
      work out commanded action, if any
     */
    enum toy_action action = action_input?toy_action(actions[action_input-1].get()):ACTION_NONE;
   
    // check for long left button press
    if (action == ACTION_NONE && left_press_counter > TOY_LONG_PRESS_COUNT) {
        left_press_counter = -TOY_COMMAND_DELAY;
        action = toy_action(actions[7].get());
        ignore_left_change = true;
    }

    // cope with long left press triggering a left change
    if (ignore_left_change && left_change) {
        left_change = false;
        ignore_left_change = false;
    }

    if (is_v2450_buttons()) {
        // check for left button latching change
        if (action == ACTION_NONE && left_change) {
            action = toy_action(actions[6].get());
        }
    } else if (is_f412_buttons()) {
        if (action == ACTION_NONE && left_change && !left_button) {
            // left release
            action = toy_action(actions[6].get());
        }

    }

    // check for right button
    if (action == ACTION_NONE && right_button) {
        right_press_counter++;
        if (right_press_counter >= TOY_RIGHT_PRESS_COUNT) {
            action = toy_action(actions[8].get());
            right_press_counter = -TOY_COMMAND_DELAY;
        }
    } else {
        right_press_counter = 0;
    }

    /*
      some actions shouldn't repeat too fast
     */
    switch (action) {
    case ACTION_TOGGLE_VIDEO:
    case ACTION_TOGGLE_MODE:
    case ACTION_TOGGLE_SIMPLE:
    case ACTION_TOGGLE_SSIMPLE:
    case ACTION_ARM_LAND_RTL:
    case ACTION_LOAD_TEST:
    case ACTION_MODE_FLOW:
        if (last_action == action ||
            now - last_action_ms < TOY_ACTION_DELAY_MS) {
            // for the above actions, button must be released before
            // it will activate again
            last_action = action;
            action = ACTION_NONE;
        }
        break;
        
    case ACTION_TAKE_PHOTO:
        // allow photo continuous shooting
        if (now - last_action_ms < TOY_ACTION_DELAY_MS) {
            last_action = action;
            action = ACTION_NONE;
        }
        break;

    default:
        last_action = action;
        break;
    }
    
    if (action != ACTION_NONE) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: action %u", action);
        last_action_ms = now;
    }

    // we use 150 for throttle_at_min to cope with varying stick throws
    bool throttle_at_min =
        copter.channel_throttle->get_control_in() < 150;

    // throttle threshold for throttle arming
    bool throttle_near_max =
        copter.channel_throttle->get_control_in() > 700;
    
    /*
      disarm if throttle is low for 1 second when landed
     */
    if ((flags & FLAG_THR_DISARM) && throttle_at_min && copter.motors->armed() && copter.ap.land_complete) {
        throttle_low_counter++;
        const uint8_t disarm_limit = copter.flightmode->has_manual_throttle()?TOY_LAND_MANUAL_DISARM_COUNT:TOY_LAND_DISARM_COUNT;
        if (throttle_low_counter >= disarm_limit) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: throttle disarm");
            copter.arming.disarm(AP_Arming::Method::TOYMODELANDTHROTTLE);
        }
    } else {
        throttle_low_counter = 0;
    }

    /*
      arm if throttle is high for 1 second when landed
     */
    if ((flags & FLAG_THR_ARM) && throttle_near_max && !copter.motors->armed()) {
        throttle_high_counter++;
        if (throttle_high_counter >= TOY_LAND_ARM_COUNT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: throttle arm");
            arm_check_compass();
            if (!copter.arming.arm(AP_Arming::Method::MAVLINK) && (flags & FLAG_UPGRADE_LOITER) && copter.flightmode->mode_number() == Mode::Number::LOITER) {
                /*
                  support auto-switching to ALT_HOLD, then upgrade to LOITER once GPS available
                 */
                if (set_and_remember_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Tmode: ALT_HOLD update arm");
#if AP_FENCE_ENABLED
                    copter.fence.enable(false, AC_FENCE_ALL_FENCES);
#endif
                    if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                        // go back to LOITER
                        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: ALT_HOLD arm failed");
                        set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE);
                    } else {
                        upgrade_to_loiter = true;
#if 0
                        AP_Notify::flags.hybrid_loiter = true;
#endif
                    }
                }
            } else {
                throttle_arm_ms = AP_HAL::millis();
            }
        }
    } else {
        throttle_high_counter = 0;
    }

    if (upgrade_to_loiter) {
        if (!copter.motors->armed() || copter.flightmode->mode_number() != Mode::Number::ALT_HOLD) {
            upgrade_to_loiter = false;
#if 0
            AP_Notify::flags.hybrid_loiter = false;
#endif
        } else if (copter.position_ok() && set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE)) {
#if AP_FENCE_ENABLED
            copter.fence.enable(true, AC_FENCE_ALL_FENCES);
#endif
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: LOITER update");            
        }
    }

    if (copter.flightmode->mode_number() == Mode::Number::RTL && (flags & FLAG_RTL_CANCEL) && throttle_near_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: RTL cancel");        
        set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE);
    }
    
    enum Mode::Number old_mode = copter.flightmode->mode_number();
    enum Mode::Number new_mode = old_mode;

    /*
      implement actions
     */
    switch (action) {
    case ACTION_NONE:
        break;

    case ACTION_TAKE_PHOTO:
        send_named_int("SNAPSHOT", 1);
        break;

    case ACTION_TOGGLE_VIDEO:
        send_named_int("VIDEOTOG", 1);
        break;

    case ACTION_MODE_ACRO:
#if MODE_ACRO_ENABLED
        new_mode = Mode::Number::ACRO;
#else
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: ACRO is disabled");
#endif
        break;

    case ACTION_MODE_ALTHOLD:
        new_mode = Mode::Number::ALT_HOLD;
        break;

    case ACTION_MODE_AUTO:
        new_mode = Mode::Number::AUTO;
        break;

    case ACTION_MODE_LOITER:
        new_mode = Mode::Number::LOITER;
        break;

    case ACTION_MODE_RTL:
        new_mode = Mode::Number::RTL;
        break;

    case ACTION_MODE_CIRCLE:
        new_mode = Mode::Number::CIRCLE;
        break;

    case ACTION_MODE_LAND:
        new_mode = Mode::Number::LAND;
        break;

    case ACTION_MODE_DRIFT:
        new_mode = Mode::Number::DRIFT;
        break;

    case ACTION_MODE_SPORT:
        new_mode = Mode::Number::SPORT;
        break;

    case ACTION_MODE_AUTOTUNE:
        new_mode = Mode::Number::AUTOTUNE;
        break;

    case ACTION_MODE_POSHOLD:
        new_mode = Mode::Number::POSHOLD;
        break;

    case ACTION_MODE_BRAKE:
        new_mode = Mode::Number::BRAKE;
        break;

    case ACTION_MODE_THROW:
#if MODE_THROW_ENABLED
        new_mode = Mode::Number::THROW;
#else
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: THROW is disabled");
#endif
        break;

    case ACTION_MODE_FLIP:
        new_mode = Mode::Number::FLIP;
        break;

    case ACTION_MODE_STAB:
        new_mode = Mode::Number::STABILIZE;
        break;

    case ACTION_MODE_FLOW:
        // toggle flow hold
        if (old_mode != Mode::Number::FLOWHOLD) {
            new_mode = Mode::Number::FLOWHOLD;
        } else {
            new_mode = Mode::Number::ALT_HOLD;
        }
        break;
        
    case ACTION_DISARM:
        if (copter.motors->armed()) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: Force disarm");
            copter.arming.disarm(AP_Arming::Method::TOYMODELANDFORCE);
        }
        break;

    case ACTION_TOGGLE_MODE:
        last_mode_choice = (last_mode_choice+1) % 2;
        new_mode = Mode::Number(primary_mode[last_mode_choice].get());
        break;

    case ACTION_TOGGLE_SIMPLE:
        copter.set_simple_mode(bool(copter.simple_mode)?Copter::SimpleMode::NONE:Copter::SimpleMode::SIMPLE);
        break;

    case ACTION_TOGGLE_SSIMPLE:
        copter.set_simple_mode(bool(copter.simple_mode)?Copter::SimpleMode::NONE:Copter::SimpleMode::SUPERSIMPLE);
        break;
        
    case ACTION_ARM_LAND_RTL:
        if (!copter.motors->armed()) {
            action_arm();
        } else if (old_mode == Mode::Number::RTL) {
            // switch between RTL and LOITER when in GPS modes
            new_mode = Mode::Number::LOITER;
        } else if (old_mode == Mode::Number::LAND) {
            if (last_set_mode == Mode::Number::LAND || !copter.position_ok()) {
                // this is a land that we asked for, or we don't have good positioning
                new_mode = Mode::Number::ALT_HOLD;
            } else if (copter.landing_with_GPS()) {
                new_mode = Mode::Number::LOITER;
            } else {
                new_mode = Mode::Number::ALT_HOLD;
            }
        } else if (copter.flightmode->requires_GPS()) {
            // if we're in a GPS mode, then RTL
            new_mode = Mode::Number::RTL;
        } else {
            // if we're in a non-GPS mode, then LAND
            new_mode = Mode::Number::LAND;
        }
        break;

    case ACTION_LOAD_TEST:
#if ENABLE_LOAD_TEST
        if (copter.motors->armed() && !load_test.running) {
            break;
        }
        if (load_test.running) {
            load_test.running = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test off");
            copter.init_disarm_motors();
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
        } else {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
#if AP_FENCE_ENABLED
            copter.fence.enable(false);
#endif
            if (copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                load_test.running = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test on");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test failed");
            }
        }
#endif
        break;
    }

    if (!copter.motors->armed() && (copter.flightmode->mode_number() == Mode::Number::LAND || copter.flightmode->mode_number() == Mode::Number::RTL)) {
        // revert back to last primary flight mode if disarmed after landing
        new_mode = Mode::Number(primary_mode[last_mode_choice].get());
    }
    
    if (new_mode != copter.flightmode->mode_number()) {
        load_test.running = false;
#if AP_FENCE_ENABLED
        copter.fence.enable(false, AC_FENCE_ALL_FENCES);
#endif
        if (set_and_remember_mode(new_mode, ModeReason::TOY_MODE)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: mode %s", copter.flightmode->name4());
            // force fence on in all GPS flight modes
#if AP_FENCE_ENABLED
            if (copter.flightmode->requires_GPS()) {
                copter.fence.enable(true, AC_FENCE_ALL_FENCES);
            }
#endif
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: %u FAILED", (unsigned)new_mode);
            if (new_mode == Mode::Number::RTL) {
                // if we can't RTL then land
                gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: LANDING");
                set_and_remember_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
#if AP_FENCE_ENABLED
                if (copter.landing_with_GPS()) {
                    copter.fence.enable(true, AC_FENCE_ALL_FENCES);
                }
#endif
            }
        }
    }
}


/*
  set a mode, remembering what mode we set, and the previous mode we were in
 */
bool ToyMode::set_and_remember_mode(Mode::Number mode, ModeReason reason)
{
    if (copter.flightmode->mode_number() == mode) {
        return true;
    }
    if (!copter.set_mode(mode, reason)) {
        return false;
    }
    last_set_mode = mode;
    return true;
}

/*
  automatic stick trimming. This works while disarmed by looking for
  zero rc-input changes for 4 seconds, and assuming sticks are
  centered. Trim is saved
 */
void ToyMode::trim_update(void)
{
    if (hal.util->get_soft_armed() || !rc().has_valid_input()) {
        // only when disarmed and with RC link
        trim.start_ms = 0;
        return;
    }

    // get throttle mid from channel trim
    uint16_t throttle_trim = copter.channel_throttle->get_radio_trim();
    if (abs(throttle_trim - 1500) <= trim_auto) {
        RC_Channel *c = copter.channel_throttle;
        uint16_t ch_min = c->get_radio_min();
        uint16_t ch_max = c->get_radio_max();
        // remember the throttle midpoint
        int16_t new_value = 1000UL * (throttle_trim - ch_min) / (ch_max - ch_min);
        if (new_value != throttle_mid) {
            throttle_mid = new_value;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: thr mid %d",
                                             throttle_mid);
        }
    }
    
    uint16_t chan[4];
    if (rc().get_radio_in(chan, 4) != 4) {
        trim.start_ms = 0;
        return;
    }

    const uint16_t noise_limit = 2;
    for (uint8_t i=0; i<4; i++) {
        if (abs(chan[i] - 1500) > trim_auto) {
            // not within limit
            trim.start_ms = 0;
            return;
        }
    }

    uint32_t now = AP_HAL::millis();
    
    if (trim.start_ms == 0) {
        // start timer
        memcpy(trim.chan, chan, 4*sizeof(uint16_t));
        trim.start_ms = now;
        return;
    }

    
    for (uint8_t i=0; i<4; i++) {
        if (abs(trim.chan[i] - chan[i]) > noise_limit) {
            // detected stick movement
            memcpy(trim.chan, chan, 4*sizeof(uint16_t));
            trim.start_ms = now;
            return;
        }
    }

    if (now - trim.start_ms < 4000) {
        // not steady for long enough yet
        return;
    }

    // reset timer so we don't trigger too often
    trim.start_ms = 0;
    
    uint8_t need_trim = 0;
    for (uint8_t i=0; i<4; i++) {
        RC_Channel *c = RC_Channels::rc_channel(i);
        if (c && abs(chan[i] - c->get_radio_trim()) > noise_limit) {
            need_trim |= 1U<<i;
        }
    }
    if (need_trim == 0) {
        return;
    }
    for (uint8_t i=0; i<4; i++) {
        if (need_trim & (1U<<i)) {
            RC_Channel *c = RC_Channels::rc_channel(i);
            c->set_and_save_radio_trim(chan[i]);
        }
    }

    gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: trim %u %u %u %u",
                                     chan[0], chan[1], chan[2], chan[3]);
}

/*
  handle arming action
 */
void ToyMode::action_arm(void)
{
    bool needs_gps = copter.flightmode->requires_GPS();

    // don't arm if sticks aren't in deadzone, to prevent pot problems
    // on TX causing flight control issues
    bool sticks_centered =
        copter.channel_roll->get_control_in() == 0 &&
        copter.channel_pitch->get_control_in() == 0 &&
        copter.channel_yaw->get_control_in() == 0;

    if (!sticks_centered) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: sticks not centered");
        return;
    }

    arm_check_compass();
    
    if (needs_gps && copter.arming.gps_checks(false)) {
#if AP_FENCE_ENABLED
        // we want GPS and checks are passing, arm and enable fence
        copter.fence.enable(true, AC_FENCE_ALL_FENCES);
#endif
        copter.arming.arm(AP_Arming::Method::TOYMODE);
        if (!copter.motors->armed()) {
            AP_Notify::events.arming_failed = true;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS armed motors");
        }
    } else if (needs_gps) {
        // notify of arming fail
        AP_Notify::events.arming_failed = true;
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
    } else {
#if AP_FENCE_ENABLED
        // non-GPS mode
        copter.fence.enable(false, AC_FENCE_ALL_FENCES);
#endif
        copter.arming.arm(AP_Arming::Method::TOYMODE);
        if (!copter.motors->armed()) {
            AP_Notify::events.arming_failed = true;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: non-GPS arming failed");
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: non-GPS armed motors");
        }
    }
}

/*
  adjust throttle for throttle takeoff
  This prevents sudden climbs when using throttle for arming
*/
void ToyMode::throttle_adjust(float &throttle_control)
{
    uint32_t now = AP_HAL::millis();
    const uint32_t soft_start_ms = 5000;
    const uint16_t throttle_start = 600 + copter.g.throttle_deadzone;
    if (!copter.motors->armed() && (flags & FLAG_THR_ARM)) {
        throttle_control = MIN(throttle_control, 500);
    } else if (now - throttle_arm_ms < soft_start_ms) {
        float p = (now - throttle_arm_ms) / float(soft_start_ms);
        throttle_control = MIN(throttle_control, throttle_start + p * (1000 - throttle_start));
    }

    // limit descent rate close to the ground
    float pos_d_m;  
    if (AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {  
        // we do not know how high we are, let alone where the ground is  
        return;  
    }  
    if (throttle_control >= 500) {  
        // user wants to climb - don't play with throttle  
        return;  
    }  
    const float height = -pos_d_m - copter.arming_altitude_m;  
    if (height >= TOY_DESCENT_SLOW_HEIGHT + TOY_DESCENT_SLOW_RAMP) {  
        // vehicle is above ramp-adjusted slow-down height  
        return;  
    }  
    if (!copter.motors->armed()) {  
        // vehicle isn't armed, so don't play with throttle  
        return;  
    }  
    if (copter.ap.land_complete) {  
        // we're landed, don't play with throttle  
        return;  
    }  
    const float limit = linear_interpolate(TOY_DESCENT_SLOW_MIN, 0, height,  
                                            TOY_DESCENT_SLOW_HEIGHT,  
                                            TOY_DESCENT_SLOW_HEIGHT+TOY_DESCENT_SLOW_RAMP);  
    if (throttle_control >= limit) {  
        // vehicle is above the now-calculated throttle-decrease height  
        return;  
    }  
    // limit descent rate close to the ground  
    throttle_control = limit; 
}


/*
  update blinking. Blinking is done with a 16 bit pattern for each
  LED. A count can be set for a pattern, which makes the pattern
  persist until the count is zero. When it is zero the normal pattern
  settings based on system status are used
 */
void ToyMode::blink_update(void)
{
    if (red_blink_pattern & (1U<<red_blink_index)) {
        copter.relay.on(1);
    } else {
        copter.relay.off(1);
    }
    if (green_blink_pattern & (1U<<green_blink_index)) {
        copter.relay.on(0);
    } else {
        copter.relay.off(0);
    }
    green_blink_index = (green_blink_index+1) % 16;
    red_blink_index = (red_blink_index+1) % 16;
    if (green_blink_index == 0 && green_blink_count > 0) {
        green_blink_count--;
    }
    if (red_blink_index == 0 && red_blink_count > 0) {
        red_blink_count--;
    }

    // let the TX know we are recording video
    uint32_t now = AP_HAL::millis();
    if (now - last_video_ms < 1000) {
        AP_Notify::flags.video_recording = true;
    } else {
        AP_Notify::flags.video_recording = false;
    }
    
    if (red_blink_count > 0 && green_blink_count > 0) {
        return;
    }
    
    // setup normal patterns based on flight mode and arming
    uint16_t pattern = 0;

    // full on when we can see the TX, except for battery failsafe,
    // when we blink rapidly
    if (copter.motors->armed() && AP_Notify::flags.failsafe_battery) {
        pattern = BLINK_8;
    } else if (!copter.motors->armed() && (blink_disarm > 0)) {
        pattern = BLINK_8;
        blink_disarm--;
    } else {
        pattern = BLINK_FULL;
    }
    
    if (copter.motors->armed()) {
        blink_disarm = 4;
    }
    
    if (red_blink_count == 0) {
        red_blink_pattern = pattern;
    }
    if (green_blink_count == 0) {
        green_blink_pattern = pattern;
    }
    if (red_blink_count == 0 && green_blink_count == 0) {
        // get LEDs in sync
        red_blink_index = green_blink_index;
    }
}

// handle a mavlink message
void ToyMode::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_NAMED_VALUE_INT) {
        return;
    }
    mavlink_named_value_int_t m;
    mavlink_msg_named_value_int_decode(&msg, &m);
    if (strncmp(m.name, "BLINKR", 10) == 0) {
        red_blink_pattern = (uint16_t)m.value;
        red_blink_count = m.value >> 16;
        red_blink_index = 0;
    } else if (strncmp(m.name, "BLINKG", 10) == 0) {
        green_blink_pattern = (uint16_t)m.value;
        green_blink_count = m.value >> 16;
        green_blink_index = 0;
    } else if (strncmp(m.name, "VNOTIFY", 10) == 0) {
        // taking photos or video
        if (green_blink_pattern != BLINK_2) {
            green_blink_index = 0;
        }
        green_blink_pattern = BLINK_2;
        green_blink_count = 1;
        last_video_ms = AP_HAL::millis();
        // immediately update AP_Notify recording flag
        AP_Notify::flags.video_recording = true;
    } else if (strncmp(m.name, "WIFICHAN", 10) == 0) {
#if AP_RADIO_ENABLED
        AP_Radio *radio = AP_Radio::get_singleton();
        if (radio) {
            radio->set_wifi_channel(m.value);
        }
#endif
    } else if (strncmp(m.name, "LOGDISARM", 10) == 0) {
        enum ap_var_type vtype;
        AP_Int8 *log_disarmed = (AP_Int8 *)AP_Param::find("LOG_DISARMED", &vtype);
        if (log_disarmed) {
            log_disarmed->set(int8_t(m.value));
        }
    }
}

/*
  send a named int to primary telem channel
 */
void ToyMode::send_named_int(const char *name, int32_t value)
{
    mavlink_msg_named_value_int_send(MAVLINK_COMM_1, AP_HAL::millis(), name, value);
}

/*
  limit maximum thrust based on voltage
 */
void ToyMode::thrust_limiting(float *thrust, uint8_t num_motors)
{
    float thrust_mul = linear_interpolate(filter.thrust_max, filter.thrust_min, filtered_voltage, filter.volt_min, filter.volt_max);
    for (uint8_t i=0; i<num_motors; i++) {
        thrust[i] *= thrust_mul;
    }
    uint16_t pwm[4];
    hal.rcout->read(pwm, 4);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: THST
// @Description: Maximum thrust limitation based on battery voltage in Toy Mode
// @Field: TimeUS: Time since system startup
// @Field: Vol: Filtered battery voltage
// @Field: Mul: Thrust multiplier between 0 and 1 to limit the output thrust based on battery voltage
// @Field: M1: Motor 1 pwm output
// @Field: M2: Motor 2 pwm output
// @Field: M3: Motor 3 pwm output
// @Field: M4: Motor 4 pwm output

    if (motor_log_counter++ % 10 == 0) {
        AP::logger().WriteStreaming("THST", "TimeUS,Vol,Mul,M1,M2,M3,M4", "QffHHHH",
                                               AP_HAL::micros64(),
                                               (double)filtered_voltage,
                                               (double)thrust_mul,
                                               pwm[0], pwm[1], pwm[2], pwm[3]);
    }
#endif
}

#if ENABLE_LOAD_TEST
/*
  run a motor load test - used for endurance checking in factory tests
 */
void ToyMode::load_test_run(void)
{
    uint16_t pwm[4] {};
    switch ((enum load_type)load_test.load_type.get()) {
    case LOAD_TYPE_LOG1:
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = load_data1[load_test.row].m[i];
        }
        load_test.filter_counter++;
        if (load_test.filter_counter >= load_test.load_filter.get()) {
            load_test.filter_counter = 0;
            load_test.row = (load_test.row + 1) % ARRAY_SIZE(load_data1);
        }
        break;
        
    case LOAD_TYPE_LOG2:
        // like log1, but all the same
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = load_data1[load_test.row].m[0];
        }
        load_test.filter_counter++;
        if (load_test.filter_counter >= load_test.load_filter.get()) {
            load_test.filter_counter = 0;
            load_test.row = (load_test.row + 1) % ARRAY_SIZE(load_data1);
        }
        break;

    case LOAD_TYPE_CONSTANT:
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = 500;
        }
        break;
    default:
        return;
    }
    for (uint8_t i=0; i<4; i++) {
        pwm[i] *= load_test.load_mul;
        // write, with conversion to 1000 to 2000 range
        hal.rcout->write(i, 1000 + pwm[i]*2);
    }

    if (copter.failsafe.battery) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test off (battery)");
        copter.init_disarm_motors();
        load_test.running = false;
    }    
}
#endif // ENABLE_LOAD_TEST

/*
  if we try to arm and the compass is out of range then we enable
  inflight compass learning
 */
void ToyMode::arm_check_compass(void)
{
    // check for unreasonable compass offsets
    Vector3f offsets = copter.compass.get_offsets();
    float field = copter.compass.get_field().length();
    
    char unused_compass_configured_error_message[20];
    if (offsets.length() > copter.compass.get_offsets_max() ||
        field < 200 || field > 800 ||
        !copter.compass.configured(unused_compass_configured_error_message, ARRAY_SIZE(unused_compass_configured_error_message))) {
        if (copter.compass.get_learn_type() != Compass::LearnType::INFLIGHT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: enable compass learning");
            copter.compass.set_learn_type(Compass::LearnType::INFLIGHT, false);
        }
    }
}

#endif // TOY_MODE_ENABLED
