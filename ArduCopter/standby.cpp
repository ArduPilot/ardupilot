#include "Copter.h"

// Run standby functions at approximately 100 Hz to limit maximum variable build up
//
// When standby is active:
//      all I terms are continually reset
//      heading error is reset to zero
//      position errors are reset to zero
//      crash_check is disabled
//      thrust_loss_check is disabled
//      parachute_check is disabled
//      hover throttle learn is disabled
//      and landing detection is disabled.
void Copter::standby_update()
{
    if (!standby_active) {
        return;
    }

    attitude_control->reset_rate_controller_I_terms();
    attitude_control->set_yaw_target_to_current_heading();
    pos_control->standby_xyz_reset();
}

// enter standby mode, flight controller is no longer flying the vehicle
void Copter::enter_standby()
{
    standby_active = true;
    AP::logger().Write_Event(LogEvent::STANDBY_ENABLE);
    gcs().send_text(MAV_SEVERITY_INFO, "StandBy Enabled");
}

// Exit standby mode, takeover flying of the vehicle
void Copter::exit_standby()
{
    standby_active = false;
    AP::logger().Write_Event(LogEvent::STANDBY_DISABLE);
    gcs().send_text(MAV_SEVERITY_INFO, "StandBy Disabled");

    // if motors are armed make sure the vehicle does not think its on the ground
    set_land_complete(!motors->armed());

    // re init the current mode ignoring checks, should make the switch more seamless
    flightmode->init(true);
}
