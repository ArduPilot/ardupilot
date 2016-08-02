// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// set_home_state - update home state
void Copter::set_home_state(enum HomeState new_home_state)
{
    // if no change, exit immediately
    if (ap.home_state == new_home_state)
        return;

    // update state
    ap.home_state = new_home_state;

    // log if home has been set
    if (new_home_state == HOME_SET_NOT_LOCKED || new_home_state == HOME_SET_AND_LOCKED) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// home_is_set - returns true if home positions has been set (to GPS location, armed location or EKF origin)
bool Copter::home_is_set()
{
    return (ap.home_state == HOME_SET_NOT_LOCKED || ap.home_state == HOME_SET_AND_LOCKED);
}

// ---------------------------------------------
void Copter::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
void Copter::set_simple_mode(uint8_t b)
{
    if(ap.simple_mode != b){
        if(b == 0){
            Log_Write_Event(DATA_SET_SIMPLE_OFF);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "SIMPLE mode off");
        }else if(b == 1){
            Log_Write_Event(DATA_SET_SIMPLE_ON);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "SIMPLE mode on");
        }else{
            // initialise super simple heading
            update_super_simple_bearing(true);
            Log_Write_Event(DATA_SET_SUPERSIMPLE_ON);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "SUPERSIMPLE mode on");
        }
        ap.simple_mode = b;
    }
}

// ---------------------------------------------
void Copter::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void Copter::set_failsafe_battery(bool b)
{
    failsafe.battery = b;
    AP_Notify::flags.failsafe_battery = b;
}

// ---------------------------------------------
void Copter::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;
}

// ---------------------------------------------

void Copter::set_pre_arm_check(bool b)
{
    if(ap.pre_arm_check != b) {
        ap.pre_arm_check = b;
        AP_Notify::flags.pre_arm_check = b;
    }
}

void Copter::set_pre_arm_rc_check(bool b)
{
    if(ap.pre_arm_rc_check != b) {
        ap.pre_arm_rc_check = b;
    }
}

void Copter::update_using_interlock()
{
#if FRAME_CONFIG == HELI_FRAME
    // helicopters are always using motor interlock
    ap.using_interlock = true;
#else
    // check if we are using motor interlock control on an aux switch or are in throw mode
    // which uses the interlock to stop motors while the copter is being thrown
    ap.using_interlock = check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK);
#endif
}

void Copter::set_motor_emergency_stop(bool b)
{
    if(ap.motor_emergency_stop != b) {
        ap.motor_emergency_stop = b;
    }

    // Log new status
    if (ap.motor_emergency_stop){
        Log_Write_Event(DATA_MOTORS_EMERGENCY_STOPPED);
    } else {
        Log_Write_Event(DATA_MOTORS_EMERGENCY_STOP_CLEARED);
    }
}
