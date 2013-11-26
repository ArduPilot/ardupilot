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
    barometer.read();
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
        compass.null_offsets();
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
    g_gps->update();
}

