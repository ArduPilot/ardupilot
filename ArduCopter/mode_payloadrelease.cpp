#include "Copter.h"
#include "mode.h"

bool ModePayloadRelease::init(bool ignore_checks)
{
    hal.console->printf("I am inside ModePayloadRelease::init.");

    gcs().send_text(MAV_SEVERITY_INFO, "Initialise payload release");

    if (!ignore_checks) {   //if not ignore checks
        return true;
    }
    //if ignore checks
    return true;
}

void ModePayloadRelease::run()
{
    // hal.console->printf("I am inside ModePayloadRelease::run");
    // ahrs.airspeed_estimate
    //This statement below makes sound in gcs
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "payload release run");
    //This statement below prints in terminal where autopilot is run
    // float *const_spd = 0;
    gcs().send_text(MAV_SEVERITY_INFO,"payload release run");
    Vector3f wind_vel = ahrs.wind_estimate();
    const AP_Airspeed *aspeed = ahrs.get_airspeed();
    bool air_spd = aspeed ->enabled();
    // float fire = 5 * air_sp->get_airspeed();
    gcs().send_text(MAV_SEVERITY_INFO,"air speed enable state: %d", air_spd);
    gcs().send_text(MAV_SEVERITY_INFO, "[%f %f %f]m/s wind estimates", wind_vel[0], wind_vel[1], wind_vel[2]);
    // gcs().send_text(MAV_SEVERITY_INFO, "%fm/s air estimates", air_sp);
    // hal.console->printf("[%f %f %f]m/s wind estimates", wind_vel[0], wind_vel[1], wind_vel[2]);

    //added //etti matra gare hunxa.
    //this below 4 lines make sure that at background the copter is moving towards given waypoint 
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    //copter.failsafe_terrain_set_status(wp_nav->update_wpnav()); //run way point controller
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    //add finish
}