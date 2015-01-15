// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// read the barometer and return the updated altitude in meters
static void update_barometer(void)
{
    barometer.update();
}


/*
  update INS and attitude
 */
static void update_ahrs()
{
    ahrs.update();
}


/*
  read and update compass
 */
static void update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        compass.learn_offsets();
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}


/*
  read the GPS
 */
static void update_GPS(void)
{
    gps.update();

    static uint32_t last_gps_msg_ms;
    static uint8_t ground_start_count = 5;
    if (gps.last_message_time_ms() != last_gps_msg_ms && 
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();
        
        if(ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 5;

            } else {
                // Now have an initial GPS position
                // use it as the HOME position in future startups
                current_loc = gps.location();
                set_home(current_loc);

                // set system clock for log timestamps
                hal.util->set_system_clock(gps.time_epoch_usec());

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    compass.set_initial_location(gps.location().lat, gps.location().lng);
                }
                ground_start_count = 0;
            }
        }
    }
}

