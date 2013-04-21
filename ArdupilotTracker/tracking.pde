// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************

#include "tracking.h"
//#include "defines.h"

/*
 * Some azimuth and bearing calculation stuff!
 * Distance d (R is radius of Earth)
 * dlon = lon2 - lon1
 * dlat = lat2 - lat1
 * a = (sin(dlat/2)^2 + (cos(lat1)*cos(lat2) * sin(dlon/2)^2
 * Arc length in radians c
 * c = 2*arcsin(min(1, sqrt(a))
 * d = R*c
 * 
 * d is distance between p1 and p2 
 * var x = acos((sin(lat2)-sin(lat1)*cos(d)) / (sin(d/R)*cos(lat1))
 * 
 * Elevation flat Earth
 * lambda = asin((elev2 - elev1)/d)
 * 
 * Elevation curved Eearth
 * Angular drop in radians is: d/(2 R)
 * 
 * lambda = asin((elev2 - elev1)/d) - d/(2 R) 
 */

static void trackPosition(Location* here, Location* there, struct AzimuthElevation* result) {
	// We do it a little simpler here than above formula.
	float d = get_distance(here, there);
	int32_t bearing_cd = get_bearing_cd(here, there);
	// Flat Earth elevation.
	float elevation = asinf((there->alt-here->alt) / (d * 100));
	elevation -= d/(2*6378100); // double radius of Earth.

	bearing_cd -= neutral_bearing_cd;
	bearing_cd = wrap_180_cd(bearing_cd);
	
	result->azimuth_cd = bearing_cd;
	result->elevation_cd = elevation * DEGX100;
}

static void track_mavlink() {
	trackPosition(&current_loc, &target, &servoAzimuthElevation);
	servoAzimuthElevation.azimuth_cd += neutral_bearing_cd - ahrs.yaw_sensor;
	servoAzimuthElevation.azimuth_cd = wrap_180_cd(servoAzimuthElevation.azimuth_cd);
	servoAzimuthElevation.elevation_cd -= ahrs.pitch_sensor;
	servoAzimuthElevation.elevation_cd = wrap_180_cd(servoAzimuthElevation.elevation_cd);
}

static void track_ardutracker(){
	servoAzimuthElevation.azimuth_cd = neutral_bearing_cd + incomingAzimuthElevation.azimuth_cd - ahrs.yaw_sensor;
	servoAzimuthElevation.azimuth_cd = wrap_180_cd(servoAzimuthElevation.azimuth_cd);
	servoAzimuthElevation.elevation_cd = incomingAzimuthElevation.elevation_cd - ahrs.pitch_sensor;
	servoAzimuthElevation.elevation_cd = wrap_180_cd(servoAzimuthElevation.elevation_cd);
}

static void track() {
	switch(control_mode) {
	case ARDUTRACKER:
		track_ardutracker();
		break;
	case MAVLINK:
		track_mavlink();
	default:
		break;
	}
}

static void navigate() {
    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if(next_WP.lat == 0) {
        return;
    }
}

/*
// static void calc_altitude_error()
{
    if (control_mode == AUTO && offset_altitude_cm != 0) {
        // limit climb rates
        target_altitude_cm = next_WP.alt - (offset_altitude_cm*((float)(wp_distance-30) / (float)(wp_totalDistance-30)));

        // stay within a certain range
        if(prev_WP.alt > next_WP.alt) {
            target_altitude_cm = constrain_int32(target_altitude_cm, next_WP.alt, prev_WP.alt);
        }else{
            target_altitude_cm = constrain_int32(target_altitude_cm, prev_WP.alt, next_WP.alt);
        }
    } else if (non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) {
        target_altitude_cm = next_WP.alt;
    }

    altitude_error_cm       = target_altitude_cm - adjusted_altitude_cm();
}
*/
