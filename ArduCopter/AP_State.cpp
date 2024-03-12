#include "Copter.h"

// ---------------------------------------------
void Copter::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        AP::logger().Write_Event(LogEvent::AUTO_ARMED);
    }
}

// ---------------------------------------------
/**
 * Set Simple mode
 *
 * @param [in] b 0:false or disabled, 1:true or SIMPLE, 2:SUPERSIMPLE
 */
void Copter::set_simple_mode(SimpleMode b)
{
    if (simple_mode != b) {
        switch (b) {
            case SimpleMode::NONE:
                AP::logger().Write_Event(LogEvent::SET_SIMPLE_OFF);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode off");
                break;
            case SimpleMode::SIMPLE:
                AP::logger().Write_Event(LogEvent::SET_SIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode on");
                break;
            case SimpleMode::SUPERSIMPLE:
                // initialise super simple heading
                update_super_simple_bearing(true);
                AP::logger().Write_Event(LogEvent::SET_SUPERSIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SUPERSIMPLE mode on");
                break;
        }
        simple_mode = b;
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
void Copter::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gcs = b;
}

// ---------------------------------------------

void Copter::update_using_interlock()
{
#if FRAME_CONFIG == HELI_FRAME
    // helicopters are always using motor interlock
    ap.using_interlock = true;
#else
    // check if we are using motor interlock control on an aux switch or are in throw mode
    // which uses the interlock to stop motors while the copter is being thrown
    ap.using_interlock = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) != nullptr;
#endif
}
