// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_L1_Control.h
/// @brief   L1 Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#ifndef AP_L1_CONTROL_H
#define AP_L1_CONTROL_H

#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Navigation.h>

class AP_L1_Control : public AP_Navigation {
public:
	AP_L1_Control(AP_AHRS *ahrs) :
		_ahrs(ahrs)
		{
			AP_Param::setup_object_defaults(this, var_info);
		}

	/* see AP_Navigation.h for the definitions and units of these
	 * functions */
	int32_t nav_roll_cd(void);

	// return the desired track heading angle(centi-degrees)
	int32_t nav_bearing_cd(void);
	
	// return the heading error angle (centi-degrees) +ve to left of track
	int32_t bearing_error_cd(void);

	float crosstrack_error(void);

	int32_t target_bearing_cd(void);
	float turn_distance(float wp_radius);
	void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP);
	void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction);
	void update_heading_hold(int32_t navigation_heading_cd);
	void update_level_flight(void);
	bool reached_loiter_target(void);

	// this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
	// reference to the AHRS object
    AP_AHRS *_ahrs;

	// lateral acceration in m/s required to fly to the 
	// L1 reference point (+ve to right)
    float _latAccDem;
	
	// L1 tracking distance in meters which is dynamically updated
	float _L1_dist;
	
	// Status which is true when the vehicle has started circling the WP
	bool _WPcircle;
	
	// bearing angle (radians) to L1 point
	float _nav_bearing;
	
	// bearing error angle (radians) +ve to left of track
	float _bearing_error;

	// crosstrack error in meters
	float _crosstrack_error;

	// target bearing in centi-degrees from last update
	int32_t _target_bearing_cd;

	// L1 tracking loop period (sec)
	AP_Float _L1_period;
	// L1 tracking loop damping ratio
	AP_Float _L1_damping;
	
    // Convert a 2D vector from latitude and longitude to planar
    // coordinates based on a reference point
	Vector2f _geo2planar(const Vector2f &ref, const Vector2f &wp) const;

	//Calculate the maximum of two floating point numbers
	float _maxf(const float &num1, const float &num2) const;

};


#endif //AP_L1_CONTROL_H
