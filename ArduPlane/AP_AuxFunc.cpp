#include "Plane.h"

#include "AP_AuxFunc.h"

void AP_AuxFunc_Plane::do_function_change_mode(const Mode::Number number,
                                               const SwitchPos pos)
{
    switch(pos) {
    case SwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        const bool success = plane.set_mode_by_number(number, ModeReason::RC_COMMAND);
        if (plane.control_mode != &plane.mode_initializing) {
            if (success) {
                AP_Notify::events.user_mode_change = 1;
            } else {
                AP_Notify::events.user_mode_change_failed = 1;
            }
        }
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

void AP_AuxFunc_Plane::do_function_q_assist_state(SwitchPos pos)
{
    switch(pos) {
        case SwitchPos::HIGH:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Force enabled");
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE);
            break;

        case SwitchPos::MIDDLE:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Enabled");
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED);
            break;

        case SwitchPos::LOW:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Disabled");
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
            break;
    }
}

void AP_AuxFunc_Plane::do_function_crow_mode(SwitchPos pos)
{
        switch(pos) {
        case SwitchPos::HIGH:
            plane.crow_mode = Plane::CrowMode::CROW_DISABLED;
            gcs().send_text(MAV_SEVERITY_INFO, "Crow Flaps Disabled");
            break;
        case SwitchPos::MIDDLE:
            gcs().send_text(MAV_SEVERITY_INFO, "Progressive Crow Flaps"); 
            plane.crow_mode = Plane::CrowMode::PROGRESSIVE;   
            break;
        case SwitchPos::LOW:
            plane.crow_mode = Plane::CrowMode::NORMAL;
            gcs().send_text(MAV_SEVERITY_INFO, "Normal Crow Flaps");
            break;
        }    
}

void AP_AuxFunc_Plane::do_function_soaring_3pos(SwitchPos pos)
{
#if HAL_SOARING_ENABLED
    SoaringController::ActiveStatus desired_state = SoaringController::ActiveStatus::SOARING_DISABLED;

    switch (pos) {
        case SwitchPos::LOW:
            desired_state = SoaringController::ActiveStatus::SOARING_DISABLED;
            break;
        case SwitchPos::MIDDLE:
            desired_state = SoaringController::ActiveStatus::MANUAL_MODE_CHANGE;
            break;
        case SwitchPos::HIGH:
            desired_state = SoaringController::ActiveStatus::AUTO_MODE_CHANGE;
            break;
        }

    plane.g2.soaring_controller.set_pilot_desired_state(desired_state);
#endif
}

void AP_AuxFunc_Plane::do_function_flare(SwitchPos pos)
{
        switch(pos) {
        case SwitchPos::HIGH:
            plane.flare_mode = Plane::FlareMode::ENABLED_PITCH_TARGET;
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
            break;
        case SwitchPos::MIDDLE:
            plane.flare_mode = Plane::FlareMode::ENABLED_NO_PITCH_TARGET;
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_DISABLED);
            break;
        case SwitchPos::LOW:
            plane.quadplane.set_q_assist_state(plane.quadplane.Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED);
            plane.flare_mode = Plane::FlareMode::FLARE_DISABLED;
            break;
        }    
}

void AP_AuxFunc_Plane::do_function_mission_reset(const SwitchPos pos)
{
    plane.mission.start();
    plane.prev_WP_loc = plane.current_loc;
}

bool AP_AuxFunc_Plane::init_function(const Function function,
                                         const SwitchPos pos)
{
    switch(function) {
    // the following functions do not need to be initialised:
    case Function::AUTO:
    case Function::CIRCLE:
    case Function::ACRO:
    case Function::TRAINING:
    case Function::FLAP:
    case Function::GUIDED:
    case Function::INVERTED:
    case Function::LOITER:
    case Function::MANUAL:
    case Function::RTL:
    case Function::TAKEOFF:
    case Function::FBWA:
    case Function::FBWA_TAILDRAGGER:
    case Function::FWD_THR:
    case Function::LANDING_FLARE:
    case Function::PARACHUTE_RELEASE:
    case Function::MODE_SWITCH_RESET:
        return true;

    case Function::Q_ASSIST:
    case Function::SOARING:
    case Function::AIRMODE:
#if AP_AIRSPEED_AUTOCAL_ENABLE
    case Function::ARSPD_CALIBRATE:
#endif
    case Function::TER_DISABLE:
    case Function::CROW_SELECT:
        return run_function(function, pos, TriggerSource::INIT);

    case Function::REVERSE_THROTTLE:
        plane.have_reverse_throttle_rc_option = true;
        // setup input throttle as a range. This is needed as init_aux_function is called
        // after set_control_channels()
        if (plane.channel_throttle) {
            plane.channel_throttle->set_range(100);
        }
        // note that we don't call do_function() here as we don't
        // want to startup with reverse thrust
        return true;

    default:
        // handle in parent class
        return AP_AuxFunc::init_function(function, pos);
    }
}

// do_function - implement the function invoked by auxillary switches
bool AP_AuxFunc_Plane::do_function(const Function function, const SwitchPos pos)
{
    switch(function) {
    case Function::INVERTED:
        plane.inverted_flight = (pos == SwitchPos::HIGH);
        break;

    case Function::REVERSE_THROTTLE:
        plane.reversed_throttle = (pos == SwitchPos::HIGH);
        gcs().send_text(MAV_SEVERITY_INFO, "RevThrottle: %s", plane.reversed_throttle?"ENABLE":"DISABLE");
        break;

    case Function::AUTO:
        do_function_change_mode(Mode::Number::AUTO, pos);
        break;

    case Function::CIRCLE:
        do_function_change_mode(Mode::Number::CIRCLE, pos);
        break;

    case Function::ACRO:
        do_function_change_mode(Mode::Number::ACRO, pos);
        break;

    case Function::TRAINING:
        do_function_change_mode(Mode::Number::TRAINING, pos);
        break;

    case Function::LOITER:
        do_function_change_mode(Mode::Number::LOITER, pos);
        break;        

    case Function::GUIDED:
        do_function_change_mode(Mode::Number::GUIDED, pos);
        break;

    case Function::MANUAL:
        do_function_change_mode(Mode::Number::MANUAL, pos);
        break;

    case Function::RTL:
        do_function_change_mode(Mode::Number::RTL, pos);
        break;

    case Function::TAKEOFF:
        do_function_change_mode(Mode::Number::TAKEOFF, pos);
        break;

    case Function::FBWA:
        do_function_change_mode(Mode::Number::FLY_BY_WIRE_A, pos);
        break;

    case Function::SOARING:
        do_function_soaring_3pos(pos);
        break;

    case Function::FLAP:
    case Function::FBWA_TAILDRAGGER:
        break; // input labels, nothing to do

    case Function::Q_ASSIST:
        do_function_q_assist_state(pos);
        break;

    case Function::FWD_THR:
        break; // VTOL forward throttle input label, nothing to do

    case Function::TER_DISABLE:
            switch (pos) {
            case SwitchPos::HIGH:
                plane.non_auto_terrain_disable = true;
                if (plane.control_mode->allows_terrain_disable()) {
                    plane.set_target_altitude_current();
                }
                break;
            case SwitchPos::MIDDLE:
                break;
            case SwitchPos::LOW:
                plane.non_auto_terrain_disable = false;
                if (plane.control_mode->allows_terrain_disable()) {
                    plane.set_target_altitude_current();
                }
                break;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "NON AUTO TERRN: %s", plane.non_auto_terrain_disable?"OFF":"ON");
        break;

    case Function::CROW_SELECT:
        do_function_crow_mode(pos);
        break;

    case Function::AIRMODE:
        switch (pos) {
        case SwitchPos::HIGH:
            plane.quadplane.air_mode = AirMode::ON;
            break;
        case SwitchPos::MIDDLE:
            break;
        case SwitchPos::LOW:
            plane.quadplane.air_mode = AirMode::OFF;
            break;
        }
        break;

    case Function::ARSPD_CALIBRATE:
#if AP_AIRSPEED_AUTOCAL_ENABLE
       switch (pos) {
        case SwitchPos::HIGH:
            plane.airspeed.set_calibration_enabled(true);
            break;
        case SwitchPos::MIDDLE:
            break;
        case SwitchPos::LOW:
            plane.airspeed.set_calibration_enabled(false);
            break;
        }
#endif
        break;

   case Function::LANDING_FLARE:
       do_function_flare(pos);
       break;

    case Function::PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
        plane.parachute_manual_release();
#endif
        break;

    case Function::MODE_SWITCH_RESET:
        plane.reset_control_switch();
        break;

    default:
        return AP_AuxFunc::do_function(function, pos);
    }

    return true;
}
