// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

# if CONFIG_COLLISION_AVOIDANCE == ENABLED

#include <AP_Collision_Sensor.h>

static Collision_Sensor* collision_sensor;

static bool avoidance_is_enabled = true;

static bool is_collision_avoidance_enabled(){
	return avoidance_is_enabled;
}

static void init_collision_avoidance(){
	collision_sensor = new Collision_MaxsonarI2CXL();

	//at least bottom and front a required
	if(!collision_sensor->set_activated(CA_BOTTOM, true)){
		cliSerial->printf_P(PSTR("AVOID bottom not activated\n"));
		avoidance_is_enabled = false;
	}
	if(!avoidance_is_enabled || !collision_sensor->set_activated(CA_FRONT, true)){
		cliSerial->printf_P(PSTR("AVOID front not activated\n"));
		avoidance_is_enabled = false;
	}

//TODO
	if(!avoidance_is_enabled || !collision_sensor->set_activated(CA_RIGHT, true)){
		cliSerial->printf_P(PSTR("AVOID right not activated\n"));
	}
	if(!avoidance_is_enabled || !collision_sensor->set_activated(CA_LEFT, true)){
		cliSerial->printf_P(PSTR("AVOID left not activated\n"));
	}
//	if(!avoidance_is_enabled || !collision_sensor->set_activated(CA_BACK, true)){
//		cliSerial->printf_P(PSTR("AVOID back not activated\n"));
//	}
//	if(!avoidance_is_enabled || !collision_sensor->set_activated(CA_TOP, true)){
//		cliSerial->printf_P(PSTR("AVOID top not activated\n"));
//	}

	collision_sensor->set_threshold(CA_BOTTOM, g.avoid_collision_threshold);
	collision_sensor->set_threshold(CA_FRONT, g.avoid_collision_threshold);
	collision_sensor->set_threshold(CA_RIGHT, g.avoid_collision_threshold);
	collision_sensor->set_threshold(CA_LEFT, g.avoid_collision_threshold);
	collision_sensor->set_threshold(CA_BACK, g.avoid_collision_threshold);
	collision_sensor->set_threshold(CA_TOP, g.avoid_collision_threshold);
}

static void update_collision_sensors(){
	if(avoidance_is_enabled){
		collision_sensor->update();
	}
}

static void update_collision_avoidance(){
	if(avoidance_is_enabled){
		get_avoid_pos(0.1f);
	}
}

////////////////////////////////////////////////////
// Collision avoidance controller
////////////////////////////////////////////////////

// get_avoid_accel - collision avoidance controllers with desired accelerations provided in forward/right directions in cm/s/s
static void
get_avoid_accel(int16_t accel_req_forward, int16_t accel_req_right)
{
    float z_accel_meas = -GRAVITY_MSS * 100;    // gravity in cm/s/s

    // update angle targets that will be passed to stabilize controller
    avoid_roll = constrain((-accel_req_right/(-z_accel_meas))*(18000/M_PI), -4500, 4500);
    avoid_pitch = constrain((accel_req_forward/(-z_accel_meas*cos_roll_x))*(18000/M_PI), -4500, 4500);
}


// get_avoid_vel - collision avoidance controller with desired velocity provided in forward/right directions in cm/s
#define MAX_AVOID_VEL_ACCEL 800        // should be 1.5 times larger than MAX_LOITER_POS_ACCEL
static void
get_avoid_vel(int16_t vel_forward, int16_t vel_right, float dt)
{
    float speed_error_forward = 0;     // The velocity in cm/s.
    float speed_error_right = 0;     // The velocity in cm/s.

    float speed_lat = inertial_nav.get_latitude_velocity();
    float speed_lon = inertial_nav.get_longitude_velocity();
    //convert speed to drone coordinate system
    float speed_forward = speed_lat * cos_yaw + speed_lon*sin_yaw;
    float speed_right = -speed_lat * sin_yaw + speed_lon*cos_yaw;

//	cliSerial->printf("speed_forward %f\n",speed_forward);
//	cliSerial->printf("speed_right %f\n",speed_right);

    int32_t accel_forward;
    int32_t accel_right;
    int32_t accel_total;

    // calculate vel error
    if(vel_forward != 0){
    	speed_error_forward = vel_forward - speed_forward;
    }else{
    	g.pid_avoid_rate_forward.reset_I();
    }

    if(vel_right != 0){
    	speed_error_right = vel_right - speed_right;
    }else{
    	g.pid_avoid_rate_right.reset_I();
    }

//	cliSerial->printf("speed_error_forward %f\n", speed_error_forward);
//	cliSerial->printf("speed_error_right %f\n", speed_error_right);

    accel_forward = g.pid_avoid_rate_forward.get_pid(speed_error_forward, dt);
    accel_right = g.pid_avoid_rate_right.get_pid(speed_error_right, dt);

//	cliSerial->printf("accel_forward %d\n",accel_forward);
//	cliSerial->printf("accel_right %d\n",accel_right);

    accel_total = safe_sqrt(accel_forward*accel_forward + accel_right*accel_right);

    if( accel_total > MAX_AVOID_VEL_ACCEL ) {
    	accel_forward = MAX_AVOID_VEL_ACCEL * accel_forward/accel_total;
        accel_right = MAX_AVOID_VEL_ACCEL * accel_right/accel_total;
    }

    get_avoid_accel(accel_forward, accel_right);
}

// get_loiter_pos_lat_lon - loiter position controller with desired position provided as distance from home in lat/lon directions in cm
#define MAX_AVOID_POS_VELOCITY 750     // should be 1.5 ~ 2.0 times the pilot input's max velocity
#define MAX_AVOID_POS_ACCEL 250
static void get_avoid_pos(float dt)
{
	int16_t dist_error_forward = 0;
    int32_t desired_vel_forward;

    int16_t dist_error_right = 0;
    int32_t desired_vel_right;

    // calculate distance error

	int16_t bottom_distance = collision_sensor->get_collision_distance(CA_BOTTOM);
	int16_t front = collision_sensor->get_collision_depth(CA_FRONT);
	int16_t right = collision_sensor->get_collision_depth(CA_RIGHT);
	int16_t left = collision_sensor->get_collision_depth(CA_LEFT);
	int16_t back = collision_sensor->get_collision_depth(CA_BACK);
	int16_t top = collision_sensor->get_collision_depth(CA_TOP);

	if(top < 0){
		avoid_top_collision_depth = top;
	}

//	cliSerial->printf("Bottom %d\n",collision_sensor->get_collision_distance(CA_BOTTOM));
//	cliSerial->printf("Front %d\n",collision_sensor->get_collision_distance(CA_FRONT));
//	cliSerial->printf("Right %d\n",collision_sensor->get_collision_distance(CA_RIGHT));
//	cliSerial->printf("Left %d\n",collision_sensor->get_collision_distance(CA_LEFT));
//	cliSerial->printf("Back %d\n",collision_sensor->get_collision_distance(CA_BACK));
//	cliSerial->printf("Top %d\n",collision_sensor->get_collision_distance(CA_TOP));


	if(front < 0){
		dist_error_forward = front;
		// middle between the collision on both sides
		if(back < 0){
			dist_error_forward = (dist_error_forward - back) >> 1; //divide by 2
		}
	}else if(back < 0){
		dist_error_forward = back;
	}

	if(right < 0){
		dist_error_right = right;
		// middle between the collision on both sides
		if(left < 0){
			dist_error_right = (dist_error_right - left) >> 1; //divide by 2
		}
	}else if(left < 0){
		dist_error_right = back;
	}

	//do not react on collisions close to the ground
	//so because we can have a tilt angle up to 45 degree
	//and 90 degree between the sensors we have a equal
	//sided triangle and the bottom threshold will be same
	//to what we have configured for our distance threshold
	if(bottom_distance < g.avoid_collision_threshold){
		dist_error_forward = 0;
		dist_error_right = 0;
	}

//	cliSerial->printf("dist_error_forward %d\n",dist_error_forward);
//	cliSerial->printf("dist_error_right %d\n",dist_error_right);

	//using kD as leak rate
	desired_vel_forward = g.pid_avoid_forward.get_p(dist_error_forward)
			+ g.pid_avoid_forward.get_leaky_i(dist_error_forward, dt, g.pid_avoid_forward.kD());
    desired_vel_right = g.pid_avoid_right.get_p(dist_error_right)
		+ g.pid_avoid_right.get_leaky_i(dist_error_right, dt, g.pid_avoid_right.kD());

//	cliSerial->printf("desired_vel_forward raw %d\n",desired_vel_forward);
//	cliSerial->printf("desired_vel_right raw %d\n",desired_vel_right);

    int32_t vel_total = safe_sqrt(desired_vel_forward*desired_vel_forward + desired_vel_right * desired_vel_right);
    if( vel_total > MAX_AVOID_POS_VELOCITY ) {
        desired_vel_forward = MAX_AVOID_POS_VELOCITY * desired_vel_forward / vel_total;
        desired_vel_right = MAX_AVOID_POS_VELOCITY * desired_vel_right / vel_total;
    }

//	cliSerial->printf("desired_vel_forward %d\n",desired_vel_forward);
//	cliSerial->printf("desired_vel_right %d\n",desired_vel_right);

    get_avoid_vel(desired_vel_forward, desired_vel_right, dt);
}

#endif // CONFIG_COLLISION_AVOIDANCE
