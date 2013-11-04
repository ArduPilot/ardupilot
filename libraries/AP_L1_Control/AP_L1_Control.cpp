// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_L1_Control.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] PROGMEM = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This needs to be larger for less responsive airframes. The default of 30 is very conservative, and for most RC aircraft will lead to slow and lazy turns. For smaller more agile aircraft a value closer to 20 is appropriate. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
	// @Units: seconds
	// @Range: 1-60
	// @Increment: 1
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 25),
	
    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
	// @Range: 0.6-1.0
	// @Increment: 0.05
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    AP_GROUPEND
};

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking," 
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle


/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_L1_Control::nav_roll_cd(void) const
{
	float ret;	
	ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
	ret = constrain_float(ret, -9000, 9000);
	return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
	return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) const
{
	return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
	return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
	return _target_bearing_cd;
}

float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
	return min(wp_radius, _L1_dist);
}

bool AP_L1_Control::reached_loiter_target(void)
{
	return _WPcircle;
}

float AP_L1_Control::crosstrack_error(void) const
{
	return _crosstrack_error;
}

// update L1 control for waypoint navigation
// this function costs about 3.5 milliseconds on a AVR2560
void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP)
{

	struct Location _current_loc;
	float Nu;
	float xtrackVel;
	float ltrackVel;
	
	// Calculate L1 gain required for specified damping
	float K_L1 = 4.0f * _L1_damping * _L1_damping;

	// Get current position and velocity
    _ahrs.get_position(_current_loc);

	Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update the position for lag. This helps especially for rovers
    // where waypoints may be very close together
    Vector2f lag_offset = _groundspeed_vector * _ahrs.get_position_lag();
    location_offset(_current_loc, lag_offset.x, lag_offset.y);

	// update _target_bearing_cd
	_target_bearing_cd = get_bearing_cd(_current_loc, next_WP);
	
	//Calculate groundspeed
	float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw)) * groundSpeed;
    }

	// Calculate time varying control parameters
	// Calculate the L1 length required for specified period
	// 0.3183099 = 1/1/pipi
	_L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;
	
	// Calculate the NE position of WP B relative to WP A
    Vector2f AB = location_diff(prev_WP, next_WP);
	
	// Check for AB zero length and track directly to the destination
	// if too small
	if (AB.length() < 1.0e-6f) {
		AB = location_diff(_current_loc, next_WP);
	}
	AB.normalize();

	// Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = location_diff(prev_WP, _current_loc);

	// calculate distance to target track, for reporting
	_crosstrack_error = AB % A_air;

	//Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
	//and further than L1 distance from WP A. Then use WP A as the L1 reference point
		//Otherwise do normal L1 guidance
	float WP_A_dist = A_air.length();
	float alongTrackDist = A_air * AB;
	if (WP_A_dist > _L1_dist && alongTrackDist/max(WP_A_dist, 1.0f) < -0.7071f) {

		//Calc Nu to fly To WP A
		Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
		xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
		ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
		Nu = atan2f(xtrackVel,ltrackVel);
		_nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
		
	} else { //Calc Nu to fly along AB line
			
		//Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
		xtrackVel = _groundspeed_vector % AB; // Velocity cross track
		ltrackVel = _groundspeed_vector * AB; // Velocity along track
		float Nu2 = atan2f(xtrackVel,ltrackVel);
		//Calculate Nu1 angle (Angle to L1 reference point)
		float xtrackErr = A_air % AB;
		float sine_Nu1 = xtrackErr/max(_L1_dist, 0.1f);
		//Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
		sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
		float Nu1 = asinf(sine_Nu1);
		Nu = Nu1 + Nu2;
		_nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point		
	}	
			
	//Limit Nu to +-pi
	Nu = constrain_float(Nu, -1.5708f, +1.5708f);
	_latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
	
	// Waypoint capture status is always false during waypoint following
	_WPcircle = false;
	
	_bearing_error = Nu; // bearing error angle (radians), +ve to left of track
}

// update L1 control for loitering
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
	struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius *= sq(_ahrs.get_EAS2TAS());

	// Calculate guidance gains used by PD loop (used during circle tracking)
	float omega = (6.2832f / _L1_period);
	float Kx = omega * omega;
	float Kv = 2.0f * _L1_damping * omega;

	// Calculate L1 gain required for specified damping (used during waypoint capture)
	float K_L1 = 4.0f * _L1_damping * _L1_damping;

	//Get current position and velocity
    _ahrs.get_position(_current_loc);

	Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update the position for lag. This helps especially for rovers
    // where waypoints may be very close together
    Vector2f lag_offset = _groundspeed_vector * _ahrs.get_position_lag();
    location_offset(_current_loc, lag_offset.x, lag_offset.y);

	//Calculate groundspeed
	float groundSpeed = max(_groundspeed_vector.length() , 1.0f);


	// update _target_bearing_cd
	_target_bearing_cd = get_bearing_cd(_current_loc, center_WP);


	// Calculate time varying control parameters
	// Calculate the L1 length required for specified period
	// 0.3183099 = 1/pi
	_L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

	//Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = location_diff(center_WP, _current_loc);
	
    //Calculate the unit vector from WP A to aircraft
    Vector2f A_air_unit = A_air.normalized();

	//Calculate Nu to capture center_WP
	float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
	float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
	float Nu = atan2f(xtrackVelCap,ltrackVelCap);
	Nu = constrain_float(Nu, -1.5708f, +1.5708f); //Limit Nu to +- Pi/2

	//Calculate lat accln demand to capture center_WP (use L1 guidance law)
	float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
	
	//Calculate radial position and velocity errors
	float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
	float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

	// keep crosstrack error for reporting
	_crosstrack_error = xtrackErrCirc;
	
	//Calculate PD control correction to circle waypoint
	float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);
	
	//Calculate tangential velocity
	float velTangent = xtrackVelCap * float(loiter_direction);
	
    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if ( velTangent < 0.0f ) {
        latAccDemCircPD =  max(latAccDemCircPD, 0.0f);
	}
	
	// Calculate centripetal acceleration demand
	float latAccDemCircCtr = velTangent * velTangent / max((0.5f * radius), (radius + xtrackErrCirc));

	//Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
	float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);
	
	// Perform switchover between 'capture' and 'circle' modes at the point where the commands cross over to achieve a seamless transfer
	// Only fly 'capture' mode if outside the circle
	if ((latAccDemCap < latAccDemCirc && loiter_direction > 0 && xtrackErrCirc > 0.0f) | (latAccDemCap > latAccDemCirc && loiter_direction < 0 && xtrackErrCirc > 0.0f)) {
		_latAccDem = latAccDemCap;
		_WPcircle = false;
		_bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
		_nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
	} else {
		_latAccDem = latAccDemCirc;
		_WPcircle = true;
		_bearing_error = 0.0f; // bearing error (radians), +ve to left of track
		_nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
	}
}


// update L1 control for heading hold navigation
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
{
	// Calculate normalised frequency for tracking loop						   
	const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
	// Calculate additional damping gain

	int32_t Nu_cd;
	float Nu;
	
	// copy to _target_bearing_cd and _nav_bearing
	_target_bearing_cd = wrap_180_cd(navigation_heading_cd);
	_nav_bearing = radians(navigation_heading_cd * 0.01f);

	Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
	Nu_cd = wrap_180_cd(Nu_cd);
	Nu = radians(Nu_cd * 0.01f);

	Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
	
	//Calculate groundspeed
	float groundSpeed = _groundspeed_vector.length();

	// Calculate time varying control parameters
	_L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
	float VomegaA = groundSpeed * omegaA;
	
	// Waypoint capture status is always false during heading hold
	_WPcircle = false;

	_crosstrack_error = 0;

	_bearing_error = Nu; // bearing error angle (radians), +ve to left of track

	// Limit Nu to +-pi
	Nu = constrain_float(Nu, -1.5708f, +1.5708f);
	_latAccDem = 2.0f*sinf(Nu)*VomegaA;
}

// update L1 control for level flight on current heading
void AP_L1_Control::update_level_flight(void)
{
	// copy to _target_bearing_cd and _nav_bearing
	_target_bearing_cd = _ahrs.yaw_sensor;
	_nav_bearing = _ahrs.yaw;
	_bearing_error = 0;
	_crosstrack_error = 0;

	// Waypoint capture status is always false during heading hold
	_WPcircle = false;

	_latAccDem = 0;
}
