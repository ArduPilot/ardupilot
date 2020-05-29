#include "mode.h"
#include "Plane.h"

bool ModePayloadRelease::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_enter");
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    return true;
}

void ModePayloadRelease::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModeModePayloadRelease::_exit");


}

void ModePayloadRelease::update()
{
    // Location tar_loc = plane.mission.get_current_nav_cmd().content.location;
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_update");
    // const AP_Airspeed *aspeed = plane.ahrs.get_airspeed();
    Vector3f wind_vel = plane.ahrs.wind_estimate();
    

    gcs().send_text(MAV_SEVERITY_INFO, "cruise heading: %d", plane.cruise_state.locked_heading);
    gcs().send_text(MAV_SEVERITY_INFO, "gps heading: %d", plane.gps.ground_course_cd());
    // gcs().send_text(MAV_SEVERITY_INFO, "%f m/s", plane.cruise_state.locked_heading);
    gcs().send_text(MAV_SEVERITY_INFO, "[%f %f %f]m/s wind estimates", wind_vel[0], wind_vel[1], wind_vel[2]);

    //These calculations is not required since mode_auto.cpp's update function calls this everytime in normal auto mission.
    //plane.calc_nav_pitch();
    //plane.calc_nav_roll();
    //plane.calc_throttle();
    ////////////////////////////////////////////////////////////////////////////////////////////////

}

void ModePayloadRelease::initialise_initial_condition() {
    //initialize all the values
    a = 1;  //sign changing variable
    vz = 0;
    az = g;
    z = 0;
    total_height = plane.current_loc.alt / 10; // divided by 10 because current altitude in cms.
    remaining_height = total_height;
    vx = AP::gps().ground_speed();
    ax = 0;
    x = 0;
    vrx = 0;
    v = 0;
    fd = 0;
    fdx = 0;
    fdz = 0;

    airspeed_uav = 5;

    wind = plane.ahrs.wind_estimate();
    // llh_to_local(drop_point,drop_point_neu);
    //intialize wind values
    wind_speed_north = wind.y;
    wind_speed_east = wind.x;
    wind_speed_normalized = (wind_speed_east*wind_speed_east) + (wind_speed_north*wind_speed_north);
    wind_speed_normalized = sqrt(wind_speed_normalized);
    
    //Perform initial calculation to initialize values
    // plane.cruise_state.locked_heading
    // Bearing in degrees
    // int32_t bearing_cd = plane.current_loc.get_;
    // get current heading.
    int32_t heading_cd = plane.gps.ground_course_cd();
    theta = wrap_2PI((heading_cd / 100) * DEG_TO_RAD) ;  //calculate heading in radian
    phi = wrap_2PI(atan2f(wind_speed_east,wind_speed_north)); //wind vector direction

    if(phi > theta ) {
        C = phi - theta;
    }
    else if (phi < theta) {
        C = theta - phi;
    }

    if(fabs(theta - phi) <= 0.001) { //if theta == phi but for floating point
        if(C < M_PI ) {
            C = M_PI  - C;
            a = 1;
        }
        else {
            C = C - M_PI ;
            a = -1;
        }
        relative = sqrt((airspeed_uav*airspeed_uav) + (wind_speed_normalized*wind_speed_normalized) - 2 * airspeed_uav * wind_speed_normalized * cos(C));
        dirn = acos(((relative*relative) + (airspeed_uav*airspeed_uav) - (wind_speed_normalized *wind_speed_normalized))/(2 * relative * airspeed_uav));
    }
    else {
        dirn = theta;
    }

}

void ModePayloadRelease::calculate_displacement() {
    while (remaining_height > 0.01) {
        //calculation along z direction in NED frame
        vz = vz + az * dt;
        z = z + vz * dt;

        //calculation of wind velocity according to variation in height
        vw = wind_speed_normalized*(logf(remaining_height/z_0)/logf(total_height/z_0));

        //calculation along displacement vector i.e in NE plane
        vx = vx + ax * dt;
        x = x + vx * dt;
        vrx = vx + vw;

        //calculation of drag force
        v = sqrt(vz * vz + vrx * vrx);
        fd = 0.5 * rho * cd * Area * v * v;
        fdx = fd * vrx /v;
        fdz = fd * vz / v;

        //calculation of acceleration
        az = g - fdz / m;
        ax = -fdx / m;
        remaining_height = remaining_height - z;
    }
}

void ModePayloadRelease::llh_to_local(Location &current_llh, Vector3d &current_neu) {
    Vector3d v_current_llh;
    //convert Location class to Vector3d structure form
    v_current_llh.x = current_llh.lat * 1.0e-7f * DEG_TO_RAD;
    v_current_llh.y = current_llh.lng * 1.0e-7f * DEG_TO_RAD;
    v_current_llh.z = current_llh.alt * 1.0e-2f; //in meters
    //convert to ecef and store in current_neu
    wgsllh2ecef(v_current_llh,current_neu);
}