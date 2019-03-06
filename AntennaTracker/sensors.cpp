#include "Tracker.h"

/*
  update INS and attitude
 */
void Tracker::update_ahrs()
{
    ahrs.update();
}

// initialise compass
void Tracker::init_compass()
{
    if (!g.compass_enabled) {
        return;
    }

    if (!compass.init()|| !compass.read()) {
        hal.console->printf("Compass initialisation failed!\n");
        g.compass_enabled = false;
    } else {
        ahrs.set_compass(&compass);
    }
}

/*
  initialise compass's location used for declination
 */
void Tracker::init_compass_location(void)
{
    // update initial location used for declination
    if (!compass_init_location) {
        Location loc;
        if (ahrs.get_position(loc)) {
            compass.set_initial_location(loc.lat, loc.lng);
            compass_init_location = true;
        }
    }
}

/*
  read and update compass
 */
void Tracker::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        if (should_log(MASK_LOG_COMPASS)) {
            logger.Write_Compass();
        }
    }
}

/*
 calibrate compass
*/
void Tracker::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

// Save compass offsets
void Tracker::compass_save() {
    if (g.compass_enabled &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        compass.save_offsets();
    }
}

/*
    Accel calibration
*/
void Tracker::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    float trim_roll, trim_pitch;
    if (ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

/*
  read the GPS
 */
void Tracker::update_GPS(void)
{
    gps.update();

    static uint32_t last_gps_msg_ms;
    static uint8_t ground_start_count = 5;
    if (gps.last_message_time_ms() != last_gps_msg_ms && 
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();
        
        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else {
                // Now have an initial GPS position
                // use it as the HOME position in future startups
                current_loc = gps.location();
                if (!set_home(current_loc)) {
                    // silently ignored
                }

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    compass.set_initial_location(gps.location().lat, gps.location().lng);
                }
                ground_start_count = 0;
            }
        }
    }
}

void Tracker::handle_battery_failsafe(const char* type_str, const int8_t action)
{
    // NOP
    // useful failsafes in the future would include actually recalling the vehicle
    // that is tracked before the tracker loses power to continue tracking it
}
