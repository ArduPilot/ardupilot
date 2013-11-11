////////////////////////////////////////////////////////////////////////////////
// Toy Mode
////////////////////////////////////////////////////////////////////////////////

static float toy_gain;

static void
get_yaw_toy()
{
	// convert pilot input to lean angles
	// moved to Yaw since it is called before get_roll_pitch_toy();
	get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

	// Gain scheduling for Yaw input -
	// we reduce the yaw input based on the velocity of the copter
	// 0 = full control, 600cm/s = 20% control
    toy_gain = min(inertial_nav.get_velocity_xy(), 700);
    toy_gain = 1.0 - (toy_gain / 800.0);
	get_yaw_rate_stabilized_ef((float)control_roll * toy_gain);
}


// The function call for managing the flight mode Toy
static void
get_roll_pitch_toy()
{
	// pass desired roll, pitch to stabilize attitude controllers
	get_stabilize_roll((float)control_roll * (1.0 - toy_gain));
	get_stabilize_pitch(control_pitch);
}

