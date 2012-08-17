#if INERTIAL_NAV == ENABLED

// generates a new location and velocity in space based on inertia
// Calc 100 hz
void calc_inertia()
{
    // rotate accels based on DCM
    // --------------------------
    accels_rotated          = ahrs.get_dcm_matrix() * imu.get_accel();
    //accels_rotated		+= accels_offset;						// skew accels to account for long term error using calibration
    accels_rotated.z        += 9.805;                                                                   // remove influence of gravity

    // rising       = 2
    // neutral      = 0
    // falling      = -2

    // ACC Y POS = going EAST
    // ACC X POS = going North
    // ACC Z POS = going DOWN (lets flip this)

    // Integrate accels to get the velocity
    // ------------------------------------
    Vector3f temp           = accels_rotated * (G_Dt * 100);
    temp.z                          = -temp.z;     // Temp is changed to world frame and we can use it normaly
    accels_velocity         += temp;

    // Integrate velocity to get the Position
    // ------------------------------------
    accels_position         += accels_velocity * G_Dt;

    /*
     *       current_loc.lng += accels_velocity.x * G_Dt;
     *       current_loc.lat += accels_velocity.y * G_Dt;
     *       current_loc.alt += accels_velocity.z * G_Dt;
     */
}

void xy_error_correction()
{
    // Calculate speed error
    // ---------------------
    speed_error.x           = x_actual_speed - accels_velocity.x;
    speed_error.y           = y_actual_speed - accels_velocity.y;

    // Calculate position error
    // ------------------------
    //position_error.x	= accels_position.x - current_loc.lng;
    //position_error.y	= accels_position.y - current_loc.lat;

    // correct integrated velocity by speed_error
    // this number must be small or we will bring back sensor latency
    // -------------------------------------------
    accels_velocity.x       += speed_error.x * 0.03;                                                                    // g.speed_correction_x;
    accels_velocity.y       += speed_error.y * 0.03;

    // Error correct the accels to deal with calibration, drift and noise
    // ------------------------------------------------------------------
    //accels_position.x	-= position_error.x * 0.08;         // g.loiter_offset_correction; //.001;
    //accels_position.y	-= position_error.y * 0.08;         // g.loiter_offset_correction; //.001;

    accels_position.x = 0;
    accels_position.y = 0;
}

void z_error_correction()
{
    // Calculate speed error
    // ---------------------
    speed_error.z           = climb_rate - accels_velocity.z;
    //position_error.z	= accels_position.z - current_loc.alt;

    // correct integrated velocity by speed_error
    // this number must be small or we will bring back sensor latency
    // -------------------------------------------
    accels_velocity.z       += speed_error.z * 0.0350;                                                          //speed_correction_z;

    // ------------------------------------------------------------------
    //accels_position.z   -= position_error.z * 0.006;      //g.alt_offset_correction; // OK

    accels_position.z = 0;

    // For developement only
    // ---------------------
    if(motors.armed())
        Log_Write_Raw();
}

#endif