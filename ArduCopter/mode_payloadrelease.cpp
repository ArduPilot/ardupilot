#include "Copter.h"
#include "mode.h"

bool ModePayloadRelease::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Initialise payload release");

    if (!ignore_checks) {   //if not ignore checks
        return true;
    }
    //if ignore checks
    return true;
}

void ModePayloadRelease::run()
{
    //This statement below makes sound in gcs
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "payload release run");
    //This statement below prints in terminal where autopilot is run
    gcs().send_text(MAV_SEVERITY_INFO,"payload release run");

    //added //etti matra gare hunxa. 
    //this below 4 lines make sure that at background the copter is moving towards given waypoint 
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    //copter.failsafe_terrain_set_status(wp_nav->update_wpnav()); //run way point controller
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    //add finish
}