#include "Plane.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Plane
#define RC_CHANNEL_SUBCLASS RC_Channel_Plane

#include <RC_Channel/RC_Channels_VarInfo.h>

// note that this callback is not presently used on Plane:
int8_t RC_Channels_Plane::flight_mode_channel_number() const
{
    return plane.g.flight_mode_channel.get();
}

bool RC_Channels_Plane::has_valid_input() const
{
    if (plane.rc_failsafe_active() || plane.failsafe.rc_failsafe) {
        return false;
    }
    if (plane.failsafe.throttle_counter != 0) {
        return false;
    }
    return true;
}

RC_Channel * RC_Channels_Plane::get_arming_channel(void) const
{
    return plane.channel_rudder;
}

void RC_Channel_Plane::do_aux_function_change_mode(const Mode::Number number,
                                                   const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        plane.set_mode_by_number(number, ModeReason::RC_COMMAND);
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (plane.control_mode->mode_number() == number) {
// TODO:           rc().reset_mode_switch();
            plane.reset_control_switch();
        }
    }
}

#if HAL_QUADPLANE_ENABLED
void RC_Channel_Plane::do_aux_function_q_assist_state(AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Force enabled");
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE);
            break;

        case AuxSwitchPos::MIDDLE:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Enabled");
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED);
            break;

        case AuxSwitchPos::LOW:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Disabled");
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
            break;
    }
}
#endif  // HAL_QUADPLANE_ENABLED

void RC_Channel_Plane::do_aux_function_crow_mode(AuxSwitchPos ch_flag)
{
        switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.crow_mode = Plane::CrowMode::CROW_DISABLED;
            gcs().send_text(MAV_SEVERITY_INFO, "Crow Flaps Disabled");
            break;
        case AuxSwitchPos::MIDDLE:
            gcs().send_text(MAV_SEVERITY_INFO, "Progressive Crow Flaps"); 
            plane.crow_mode = Plane::CrowMode::PROGRESSIVE;   
            break;
        case AuxSwitchPos::LOW:
            plane.crow_mode = Plane::CrowMode::NORMAL;
            gcs().send_text(MAV_SEVERITY_INFO, "Normal Crow Flaps");
            break;
        }    
}

void RC_Channel_Plane::do_aux_function_soaring_3pos(AuxSwitchPos ch_flag)
{
#if HAL_SOARING_ENABLED
    SoaringController::ActiveStatus desired_state = SoaringController::ActiveStatus::SOARING_DISABLED;

    switch (ch_flag) {
        case AuxSwitchPos::LOW:
            desired_state = SoaringController::ActiveStatus::SOARING_DISABLED;
            break;
        case AuxSwitchPos::MIDDLE:
            desired_state = SoaringController::ActiveStatus::MANUAL_MODE_CHANGE;
            break;
        case AuxSwitchPos::HIGH:
            desired_state = SoaringController::ActiveStatus::AUTO_MODE_CHANGE;
            break;
        }

    plane.g2.soaring_controller.set_pilot_desired_state(desired_state);
#endif
}

void RC_Channel_Plane::do_aux_function_flare(AuxSwitchPos ch_flag)
{
        switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.flare_mode = Plane::FlareMode::ENABLED_PITCH_TARGET;
#if HAL_QUADPLANE_ENABLED
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
#endif
            break;
        case AuxSwitchPos::MIDDLE:
            plane.flare_mode = Plane::FlareMode::ENABLED_NO_PITCH_TARGET;
#if HAL_QUADPLANE_ENABLED
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
#endif
            break;
        case AuxSwitchPos::LOW:
#if HAL_QUADPLANE_ENABLED
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED);
#endif
            plane.flare_mode = Plane::FlareMode::FLARE_DISABLED;
            break;
        }    
}


void RC_Channel_Plane::init_aux_function(const RC_Channel::aux_func_t ch_option,
                                         const RC_Channel::AuxSwitchPos ch_flag)
{
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::TRAINING:
    case AUX_FUNC::FLAP:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::LOITER:
    case AUX_FUNC::MANUAL:
    case AUX_FUNC::RTL:
    case AUX_FUNC::TAKEOFF:
    case AUX_FUNC::FBWA:
#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::QRTL:
#endif
    case AUX_FUNC::FBWA_TAILDRAGGER:
    case AUX_FUNC::FWD_THR:
    case AUX_FUNC::LANDING_FLARE:
    case AUX_FUNC::PARACHUTE_RELEASE:
    case AUX_FUNC::MODE_SWITCH_RESET:
    case AUX_FUNC::CRUISE:
#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::ARMDISARM_AIRMODE:
#endif
    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
    case AUX_FUNC::EMERGENCY_LANDING_EN:
    case AUX_FUNC::FW_AUTOTUNE:
        break;

    case AUX_FUNC::SOARING:
#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::Q_ASSIST:
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    case AUX_FUNC::ARSPD_CALIBRATE:
#endif
    case AUX_FUNC::TER_DISABLE:
    case AUX_FUNC::CROW_SELECT:
        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;

    case AUX_FUNC::REVERSE_THROTTLE:
        plane.have_reverse_throttle_rc_option = true;
        // setup input throttle as a range. This is needed as init_aux_function is called
        // after set_control_channels()
        if (plane.channel_throttle) {
            plane.channel_throttle->set_range(100);
        }
        // note that we don't call do_aux_function() here as we don't
        // want to startup with reverse thrust
        break;

    default:
        // handle in parent class
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function - implement the function invoked by auxillary switches
bool RC_Channel_Plane::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
    case AUX_FUNC::INVERTED:
        plane.inverted_flight = (ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::REVERSE_THROTTLE:
        plane.reversed_throttle = (ch_flag == AuxSwitchPos::HIGH);
        gcs().send_text(MAV_SEVERITY_INFO, "RevThrottle: %s", plane.reversed_throttle?"ENABLE":"DISABLE");
        break;

    case AUX_FUNC::AUTO:
        do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
        break;

    case AUX_FUNC::CIRCLE:
        do_aux_function_change_mode(Mode::Number::CIRCLE, ch_flag);
        break;

    case AUX_FUNC::ACRO:
        do_aux_function_change_mode(Mode::Number::ACRO, ch_flag);
        break;

    case AUX_FUNC::TRAINING:
        do_aux_function_change_mode(Mode::Number::TRAINING, ch_flag);
        break;
        
    case AUX_FUNC::LOITER:
        do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
        break;        

    case AUX_FUNC::GUIDED:
        do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
        break;

    case AUX_FUNC::MANUAL:
        do_aux_function_change_mode(Mode::Number::MANUAL, ch_flag);
        break;

    case AUX_FUNC::RTL:
        do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
        break;

    case AUX_FUNC::TAKEOFF:
        do_aux_function_change_mode(Mode::Number::TAKEOFF, ch_flag);
        break;

    case AUX_FUNC::FBWA:
        do_aux_function_change_mode(Mode::Number::FLY_BY_WIRE_A, ch_flag);
        break;

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::QRTL:
        do_aux_function_change_mode(Mode::Number::QRTL, ch_flag);
        break;
#endif

    case AUX_FUNC::SOARING:
        do_aux_function_soaring_3pos(ch_flag);
        break;

    case AUX_FUNC::FLAP:
    case AUX_FUNC::FBWA_TAILDRAGGER:
        break; // input labels, nothing to do

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::Q_ASSIST:
        do_aux_function_q_assist_state(ch_flag);
        break;
#endif

    case AUX_FUNC::FWD_THR:
        break; // VTOL forward throttle input label, nothing to do

    case AUX_FUNC::TER_DISABLE:
            switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                plane.non_auto_terrain_disable = true;
                if (plane.control_mode->allows_terrain_disable()) {
                    plane.set_target_altitude_current();
                }
                break;
            case AuxSwitchPos::MIDDLE:
                break;
            case AuxSwitchPos::LOW:
                plane.non_auto_terrain_disable = false;
                if (plane.control_mode->allows_terrain_disable()) {
                    plane.set_target_altitude_current();
                }
                break;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "NON AUTO TERRN: %s", plane.non_auto_terrain_disable?"OFF":"ON");
        break;

    case AUX_FUNC::CROW_SELECT:
        do_aux_function_crow_mode(ch_flag);
        break;

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::AIRMODE:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.quadplane.air_mode = AirMode::ON;
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            plane.quadplane.air_mode = AirMode::OFF;
            break;
        }
        break;
#endif

    case AUX_FUNC::ARSPD_CALIBRATE:
#if AP_AIRSPEED_AUTOCAL_ENABLE
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.airspeed.set_calibration_enabled(true);
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            plane.airspeed.set_calibration_enabled(false);
            break;
        }
#endif
        break;

    case AUX_FUNC::LANDING_FLARE:
        do_aux_function_flare(ch_flag);
        break;

    case AUX_FUNC::PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
        if (ch_flag == AuxSwitchPos::HIGH) {
            plane.parachute_manual_release();
        }
#endif
        break;

    case AUX_FUNC::MODE_SWITCH_RESET:
        plane.reset_control_switch();
        break;

    case AUX_FUNC::CRUISE:
        do_aux_function_change_mode(Mode::Number::CRUISE, ch_flag);
        break;

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::ARMDISARM_AIRMODE:
        RC_Channel::do_aux_function_armdisarm(ch_flag);
        if (plane.arming.is_armed()) {
            plane.quadplane.air_mode = AirMode::ON;
        }
        break;

    case AUX_FUNC::WEATHER_VANE_ENABLE: {
        if (plane.quadplane.weathervane != nullptr) {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    plane.quadplane.weathervane->allow_weathervaning(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    plane.quadplane.weathervane->allow_weathervaning(false);
                    break;
            }
        }
        break;
    }
#endif

    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
        if (ch_flag == AuxSwitchPos::HIGH) {
            plane.trim_radio();
        }
        break;

    case AUX_FUNC::EMERGENCY_LANDING_EN:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.emergency_landing = true;
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            plane.emergency_landing = false;
            break;
        }
        break;

    case AUX_FUNC::FW_AUTOTUNE:
        plane.autotune_enable(ch_flag == AuxSwitchPos::HIGH);
        break;

    default:
        return RC_Channel::do_aux_function(ch_option, ch_flag);
    }

    return true;
}
