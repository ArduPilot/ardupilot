/* -----------------------------------------------------------------------------------------
    This is currently a SITL only function until the project is complete.
    To trial this in SITL you will need to use Real Flight 8.
    Instructions for how to set this up in SITL can be found here:
    https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 -----------------------------------------------------------------------------------------*/

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#if MODE_AUTOROTATE_ENABLED

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    return false;
#endif

    // Check that mode is enabled, make sure this is the first check as this is the most
    // important thing for users to fix if they are planning to use autorotation mode
    if (!g2.arot.enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AROT: Mode not enabled");
        return false;
    }

    // Must be armed to use mode, prevent triggering state machine on the ground
    if (!motors->armed() || copter.ap.land_complete || copter.ap.land_complete_maybe) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AROT: Must be armed and flying");
        return false;
    }

    // Initialise controller
    g2.arot.init();

    // Setting default starting state
    current_phase = Phase::ENTRY_INIT;

    // Set entry timer
    _entry_time_start_ms = millis();

    // reset logging timer
    _last_logged_ms = 0;

    return true;
}

void ModeAutorotate::run()
{
    // Current time
    const uint32_t now_ms = millis();

    // Set dt in library
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    g2.arot.set_dt(last_loop_time_s);

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------
    // State machine progresses through the autorotation phases as you read down through the if statements.
    // More urgent phases (the ones closer to the ground) take precedence later in the if statements.

    if (current_phase < Phase::GLIDE_INIT && ((now_ms - _entry_time_start_ms) > g2.arot.entry_time_ms)) {
        // Flight phase can be progressed to steady state glide
        current_phase = Phase::GLIDE_INIT;
    }

    // Check if we believe we have landed. We need the landed state to zero all
    // controls and make sure that the copter landing detector will trip
    if (current_phase < Phase::LANDED && g2.arot.check_landed()) {
        current_phase = Phase::LANDED_INIT;
    }

    // Check if we are bailing out and need to re-set the spool state
    if (motors->autorotation_bailout()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Get norm input from yaw channel
    const float pilot_norm_input = channel_yaw->norm_input_dz();

    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (current_phase) {

        case Phase::ENTRY_INIT:
            // Entry phase functions to be run only once
            g2.arot.init_entry();
            current_phase = Phase::ENTRY;
            FALLTHROUGH;

        case Phase::ENTRY:
            // Smoothly transition the collective to enter autorotation regime and begin forward speed control
            g2.arot.run_entry(pilot_norm_input);
            break;

        case Phase::GLIDE_INIT:
            // Glide phase functions to be run only once
            g2.arot.init_glide();
            current_phase = Phase::GLIDE;
            FALLTHROUGH;

        case Phase::GLIDE:
            // Maintain head speed and forward speed as we glide to the ground
            g2.arot.run_glide(pilot_norm_input);
            break;

        case Phase::FLARE_INIT:
        case Phase::FLARE:
        case Phase::TOUCH_DOWN_INIT:
        case Phase::TOUCH_DOWN:
            break;

        case Phase::LANDED_INIT:
            // Landed phase functions to be run only once
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Landed");
            current_phase = Phase::LANDED;
            FALLTHROUGH;

        case Phase::LANDED:
            // Don't allow controller to continually ask for more pitch to build speed when we are on the ground, decay to zero smoothly
            g2.arot.run_landed();
            break;
    }

    // Slow rate (25 Hz) logging for the mode
    if (now_ms - _last_logged_ms > 40U) {
        g2.arot.log_write_autorotation();
        _last_logged_ms = now_ms;
    }

} // End function run()

#endif
