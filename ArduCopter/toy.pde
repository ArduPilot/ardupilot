////////////////////////////////////////////////////////////////////////////////
// Toy Mode
////////////////////////////////////////////////////////////////////////////////


static void
get_yaw_toy()
{
	// convert pilot input to lean angles
	// moved to Yaw since it is called before get_roll_pitch_toy();
	get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

	// Grab inertial velocity
	Vector3f vel = inertial_nav.get_velocity();

	// rotate roll, pitch input from north facing to vehicle's perspective
	float roll_vel =  vel.y * cos_yaw - vel.x * sin_yaw; // body roll vel
	float pitch_vel = vel.y * sin_yaw + vel.x * cos_yaw; // body pitch vel

	pitch_vel = min(fabs(pitch_vel), 800);
	get_yaw_rate_stabilized_ef((float)(control_roll/2) * (1.0 - (pitch_vel / 2400.0)));

	roll_vel = constrain_float(roll_vel, -322, 322);
	get_stabilize_roll(roll_vel * -14);
	get_stabilize_pitch(control_pitch);
}


// The function call for managing the flight mode Toy
static void
get_roll_pitch_toy()
{
}
