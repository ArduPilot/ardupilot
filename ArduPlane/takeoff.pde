// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*   Check for automatic takeoff conditions being met using the following sequence:
 *   1) Check for adequate GPS lock - if not return false
 *   2) Check the gravity compensated longitudinal acceleration against the threshold and start the timer if true
 *   3) Wait until the timer has reached the specified value (increments of 0.1 sec) and then check the GPS speed against the threshold
 *   4) If the GPS speed is above the threshold and the attitude is within limits then return true and reset the timer
 *   5) If the GPS speed and attitude within limits has not been achieved after 2.5 seconds, return false and reset the timer
 *   6) If the time lapsed since the last timecheck is greater than 0.2 seconds, return false and reset the timer
 *   NOTE : This function relies on the TECS 50Hz processing for its acceleration measure.
 */
static bool auto_takeoff_check(void)
{
    // this is a more advanced check that relies on TECS
    uint32_t now = hal.scheduler->millis();
    static bool launchTimerStarted;
    static uint32_t last_tkoff_arm_time;
    static uint32_t last_check_ms;

    // Reset states if process has been interrupted
    if (last_check_ms && (now - last_check_ms) > 200) {
        gcs_send_text_fmt(PSTR("Timer Interrupted AUTO"));
	    launchTimerStarted = false;
	    last_tkoff_arm_time = 0;
        last_check_ms = now;
        return false;
    }

    last_check_ms = now;

    // Check for bad GPS
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        // no auto takeoff without GPS lock
        return false;
    }

    // Check for launch acceleration or timer started. NOTE: relies on TECS 50Hz processing
    if (!launchTimerStarted &&
        g.takeoff_throttle_min_accel != 0.0 &&
        SpdHgt_Controller->get_VXdot() < g.takeoff_throttle_min_accel) {
        goto no_launch;
    }

    // we've reached the acceleration threshold, so start the timer
    if (!launchTimerStarted) {
        launchTimerStarted = true;
        last_tkoff_arm_time = now;
        gcs_send_text_fmt(PSTR("Armed AUTO, xaccel = %.1f m/s/s, waiting %.1f sec"), 
                          SpdHgt_Controller->get_VXdot(), 0.1f*float(min(g.takeoff_throttle_delay,25)));
    }

    // Only perform velocity check if not timed out
    if ((now - last_tkoff_arm_time) > 2500) {
        gcs_send_text_fmt(PSTR("Timeout AUTO"));
        goto no_launch;
    }

    // Check aircraft attitude for bad launch
    if (ahrs.pitch_sensor <= -3000 ||
        ahrs.pitch_sensor >= 4500 ||
        abs(ahrs.roll_sensor) > 3000) {
        gcs_send_text_fmt(PSTR("Bad Launch AUTO"));
        goto no_launch;
    }

    // Check ground speed and time delay
    if (((gps.ground_speed() > g.takeoff_throttle_min_speed || g.takeoff_throttle_min_speed == 0.0)) && 
        ((now - last_tkoff_arm_time) >= min(uint16_t(g.takeoff_throttle_delay)*100,2500))) {
        gcs_send_text_fmt(PSTR("Triggered AUTO, GPSspd = %.1f"), gps.ground_speed());
        launchTimerStarted = false;
        last_tkoff_arm_time = 0;
        return true;
    }

    // we're not launching yet, but the timer is still going
    return false;

no_launch:
    launchTimerStarted = false;
    last_tkoff_arm_time = 0;
    return false;
}

