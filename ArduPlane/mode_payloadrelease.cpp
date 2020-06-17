#include "mode.h"
#include "Plane.h"

bool ModePayloadRelease::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_enter");
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    calculated = false;
    // do the state update outside the function that does work
    // plane.mode_payloadrelease.set_state(plane.mode_payloadrelease.PayloadRelease_Start);
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
    
    // gcs().send_text(MAV_SEVERITY_INFO, "cruise heading: %d", plane.cruise_state.locked_heading);
    // gcs().send_text(MAV_SEVERITY_INFO, "gps heading: %d", plane.gps.ground_course_cd());
    gcs().send_text(MAV_SEVERITY_INFO, "%f m/s", plane.gps.ground_speed());
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
    total_height = plane.current_loc.alt / 100; // divided by 10 because current altitude in cms.
    remaining_height = total_height;
    vx = plane.gps.ground_speed();
    ax = 0;
    x = 0;
    vrx = 0;
    v = 0;
    fd = 0;
    fdx = 0;
    fdz = 0;

    airspeed_uav = plane.gps.ground_speed();

    wind = plane.ahrs.wind_estimate();
    //At first the drop_point which is in longitude and latitude must be changed into neu so that release point must be calculated
    llh_to_neu(drop_point,drop_point_neu);
    //intialize wind values
    //wind_speed_north = wind.y;
    //wind_speed_east = wind.x;
    wind_speed_north = 0;
    wind_speed_east = 0;
    wind_speed_normalized = (wind_speed_east*wind_speed_east) + (wind_speed_north*wind_speed_north);
    wind_speed_normalized = sqrt(wind_speed_normalized);
    
    //Perform initial calculation to initialize values
    // plane.cruise_state.locked_heading
    // Bearing in degrees
    // int32_t bearing_cd = plane.current_loc.get_;
    // get current heading.
    // gps.ground_course_cd() calculates heading in centi degrees so it needs to be divided by 100 to convert into degrees. 
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

void ModePayloadRelease::llh_to_ecef(Location &current_llh, Vector3d &current_neu) {
    Vector3d v_current_llh;
    //convert Location class to Vector3d structure form
    v_current_llh.x = current_llh.lat * 1.0e-7f * DEG_TO_RAD;
    v_current_llh.y = current_llh.lng * 1.0e-7f * DEG_TO_RAD;
    v_current_llh.z = current_llh.alt * 1.0e-2f; //in meters
    //convert to ecef and store in current_neu
    wgsllh2ecef(v_current_llh,current_neu);
}

void ModePayloadRelease::ecef_to_llh(Vector3d &current_neu, Location &current_llh) {
    Vector3d v_current_llh;
    //convert current ecef to lng,lat,alt.
    wgsecef2llh(current_neu, v_current_llh);
    //store vector structure to location class

    current_llh.lat = v_current_llh.x * RAD_TO_DEG * 1.0e7f;
    current_llh.lng = v_current_llh.y * RAD_TO_DEG * 1.0e7f;
    current_llh.alt = drop_point.alt; //same altitude as drop_point altitude
}

void ModePayloadRelease::llh_to_neu(Location &current_llh, Vector3d &current_neu){
    float cLat, cLon, sLat, sLon;
    double dx,dy,dz;
    Vector3d current_ecef,home_ecef;
    Location home = plane.ahrs.get_home();

    llh_to_ecef(current_llh,current_ecef);
    llh_to_ecef(home,home_ecef);

    cLat = cos(home.lat * 1.0e-7f * DEG_TO_RAD);
    sLat = sin(home.lat * 1.0e-7f * DEG_TO_RAD);

    cLon = cos(home.lng * 1.0e-7f * DEG_TO_RAD);
    sLon = sin(home.lng * 1.0e-7f * DEG_TO_RAD);

    dx = current_ecef.x - home_ecef.x;
    dy = current_ecef.y - home_ecef.y;
    dz = current_ecef.z - home_ecef.z;

    //x is N, y is E and z is U
    current_neu.x = (-cLon * sLat * dx) - (sLat * sLon * dy) + cLat * dz ;
    current_neu.y = (-dx * sLon) + (dy * cLon) ;
    current_neu.z = (cLat * cLon * dx) + (cLat * sLon * dy) + (sLat * dz) ;

}

void ModePayloadRelease::neu_to_llh(Vector3d &current_neu, Location &current_llh){
    float cLat, cLon, sLat, sLon;
    double n,e,u;
    Vector3d current_ecef,home_ecef;
    Location home = plane.ahrs.get_home();
    
    llh_to_ecef(home,home_ecef);

    cLat = cos(home.lat * 1.0e-7f * DEG_TO_RAD);
    sLat = sin(home.lat * 1.0e-7f * DEG_TO_RAD);

    cLon = cos(home.lng * 1.0e-7f * DEG_TO_RAD);
    sLon = sin(home.lng * 1.0e-7f * DEG_TO_RAD);

    n = current_neu.x;
    e = current_neu.y;
    u = current_neu.z;

    current_ecef.x = (-sLon * e) + (-sLat * cLon * n) + (cLat * cLon * u) + home_ecef.x;
    current_ecef.y = (cLon * e) + (-sLat * sLon * n) + (cLat * sLon * u) + home_ecef.y;
    current_ecef.z = (cLat * n) + (sLat * u) + home_ecef.z;

    ecef_to_llh(current_ecef,current_llh);

}


void ModePayloadRelease::calculate_release_point() {
    //If the relative payload path is same as uav path
    if(fabs(theta - phi) <= 0.001) {
        release_point_neu.x = drop_point_neu.x - x * cos(dirn);
        release_point_neu.y = drop_point_neu.y - x * sin(dirn);
        release_point_neu.z = drop_point_neu.z;
    }
    //If the relative payload path is above the uav path
    else if(theta - phi < 0.001) {
        release_point_neu.x = drop_point_neu.x - x * cos(theta + a * dirn);
        release_point_neu.y = drop_point_neu.y - x * sin(theta + a * dirn);
        release_point_neu.z = drop_point_neu.z;
    }
    //If the relative payload path is below the uav path OR no wind is blowing
    else if((theta - phi > 0.001) || phi <= 0.001) {
        release_point_neu.x = drop_point_neu.x - x * cos(theta - a * dirn);
        release_point_neu.y = drop_point_neu.y - x * sin(theta - a * dirn);
        release_point_neu.z = drop_point_neu.z;
    }
}

void ModePayloadRelease::get_intermediate_point(Vector3d RP){
    Vector3d int_point_neu;
    Vector3d center_point_neu;
    //int plus_minus = 1;  // 1 for center at left side , -1 for center at right side of intermediate point
    // Find the heading in which intermediate point should be calculated
    float angle = plane.prev_WP_loc.get_bearing_to(drop_point) * 0.01f * DEG_TO_RAD;
    gcs().send_text(MAV_SEVERITY_INFO, "angle = %f",angle);    

    int_point_neu.x = RP.x - intermediate_distance * cos(angle);
    int_point_neu.y = RP.y - intermediate_distance * sin(angle);
    int_point_neu.z = RP.z;

    gcs().send_text(MAV_SEVERITY_INFO, "rp_n = %f,rp_e = %f",release_point_neu.x,release_point_neu.y);
    gcs().send_text(MAV_SEVERITY_INFO, "ip_n = %f,ip_e = %f",int_point_neu.x,int_point_neu.y);

    //set loiter parameters for safety
    set_loiter_parameters();

    // after calculating intermediate point, calculate the center to loiter around  and update intermediate point
    // slope of line connecting tangent to leave the loiter and center i.e intermediate point 
    // use m1 * m2 = -1

    float slope = -1.0 / tan(angle);    

    //update intermediate point to be center of circle.
    //plane.loiter.direction is used in the calculation below to make circle in negative direction of loiter
    center_point_neu.x = int_point_neu.x + -1 * plane.loiter.direction * (radius) * (1.0 / sqrt(slope * slope + 1));
    center_point_neu.y = int_point_neu.y + -1 * plane.loiter.direction * (radius) * slope * (1.0 / sqrt(slope * slope + 1));
    center_point_neu.z = int_point_neu.z;

    gcs().send_text(MAV_SEVERITY_INFO,"loiter dirn = %d", plane.loiter.direction);
    gcs().send_text(MAV_SEVERITY_INFO, "cp_n = %f,cp_e = %f",center_point_neu.x,center_point_neu.y);
    float dist = sqrt(sq(center_point_neu.x-int_point_neu.x) + sq(center_point_neu.y - int_point_neu.y));
    gcs().send_text(MAV_SEVERITY_INFO,"dist = %f",dist);
    neu_to_llh(center_point_neu,int_point);
}

void ModePayloadRelease::set_loiter_parameters(){
    plane.loiter.direction = -1;
    plane.loiter.total_cd = 2 * 36000UL;
}
void ModePayloadRelease::release_payload(){
    AP_ServoRelayEvents ServoRelayEvents;
    ServoRelayEvents.do_set_servo(channel_payload, 1500);
}

void ModePayloadRelease::update_releasepoint() {

    bool result = false;

    if(get_state() == PayloadRelease_Start) {
        
        if(!calculated) {
            
            initialise_initial_condition();
            
            calculate_displacement();
            
            calculate_release_point();

            get_intermediate_point(release_point_neu);
            calculated = true;

            neu_to_llh(release_point_neu,release_point);
            
            
            gcs().send_text(MAV_SEVERITY_INFO, "rp_n = %d,rp_e = %d",release_point.lat,release_point.lng);
            gcs().send_text(MAV_SEVERITY_INFO, "ip_n = %d,ip_e = %d",int_point.lat,int_point.lng);
            int_point.loiter_xtrack = 1;
            release_point.loiter_xtrack = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "loiter_xtrack_int: %d",int_point.loiter_xtrack);
            gcs().send_text(MAV_SEVERITY_INFO, "loiter_xtrack_rp: %d",release_point.loiter_xtrack);
            plane.set_next_WP(int_point);
            
            //set loiter parameters in case of resetting waypoint
            //the payload release drop point behind current waypoint is not supported. the drop point should be selected in the direction of current waypoint.  
            set_loiter_parameters();
            
            
        }

    }
    
    if(get_state() == PayloadRelease_Intermediate_point_reached)
    {
        set_state(PayloadRelease_Loiter);
        
    }
    if(get_state() == PayloadRelease_Loiter)
    {
        plane.update_loiter(radius);

        if (plane.condition_value != 0) {
            // primary goal, loiter time
            if (plane.loiter.sum_cd > plane.loiter.total_cd && plane.loiter.sum_cd > 1) {
                // primary goal completed, initialize secondary heading goal
                plane.condition_value = 0;
                result = plane.verify_loiter_heading(true);
            }
        } else {
            // secondary goal, loiter to heading
            result = plane.verify_loiter_heading(false);
        }
        if(result){
            plane.set_next_WP(release_point);
            set_state(PayloadRelease_Loiter_complete);
        }
        
    }
}