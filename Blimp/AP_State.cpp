#include "Blimp.h"

// ---------------------------------------------
void Blimp::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if ( ap.auto_armed == b ) {
        return;
    }

    ap.auto_armed = b;
    if (b) {
        AP::logger().Write_Event(LogEvent::AUTO_ARMED);
    }
}

// ---------------------------------------------
void Blimp::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if (failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        } else {
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void Blimp::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gcs = b;
}
