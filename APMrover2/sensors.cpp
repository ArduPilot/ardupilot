#include "Rover.h"

#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_VisualOdom/AP_VisualOdom.h>

// initialise compass
void Rover::init_compass()
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
void Rover::init_compass_location(void)
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

// init beacons used for non-gps position estimates
void Rover::init_beacon()
{
    g2.beacon.init();
}

// init visual odometry sensor
void Rover::init_visual_odom()
{
    g2.visual_odom.init();
}

// update wheel encoders
void Rover::update_wheel_encoder()
{
    // exit immediately if not enabled
    if (g2.wheel_encoder.num_sensors() == 0) {
        return;
    }

    // update encoders
    g2.wheel_encoder.update();

    // initialise on first iteration
    const uint32_t now = AP_HAL::millis();
    if (wheel_encoder_last_ekf_update_ms == 0) {
        wheel_encoder_last_ekf_update_ms = now;
        for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
            wheel_encoder_last_angle_rad[i] = g2.wheel_encoder.get_delta_angle(i);
            wheel_encoder_last_update_ms[i] = g2.wheel_encoder.get_last_reading_ms(i);
        }
        return;
    }

    // calculate delta angle and delta time and send to EKF
    // use time of last ping from wheel encoder
    // send delta time (time between this wheel encoder time and previous wheel encoder time)
    // in case where wheel hasn't moved, count = 0 (cap the delta time at 50ms - or system time)
    //     use system clock of last update instead of time of last ping
    const float system_dt = (now - wheel_encoder_last_ekf_update_ms) / 1000.0f;
    for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
        // calculate angular change (in radians)
        const float curr_angle_rad = g2.wheel_encoder.get_delta_angle(i);
        const float delta_angle = curr_angle_rad - wheel_encoder_last_angle_rad[i];
        wheel_encoder_last_angle_rad[i] = curr_angle_rad;

        // save cumulative distances at current time (in meters)
        wheel_encoder_last_distance_m[i] = g2.wheel_encoder.get_distance(i);

        // calculate delta time
        float delta_time;
        const uint32_t latest_sensor_update_ms = g2.wheel_encoder.get_last_reading_ms(i);
        const uint32_t sensor_diff_ms = latest_sensor_update_ms - wheel_encoder_last_update_ms[i];

        // if we have not received any sensor updates, or time difference is too high then use time since last update to the ekf
        // check for old or insane sensor update times
        if (sensor_diff_ms == 0 || sensor_diff_ms > 100) {
            delta_time = system_dt;
            wheel_encoder_last_update_ms[i] = wheel_encoder_last_ekf_update_ms;
        } else {
            delta_time = sensor_diff_ms / 1000.0f;
            wheel_encoder_last_update_ms[i] = latest_sensor_update_ms;
        }

        /* delAng is the measured change in angular position from the previous measurement where a positive rotation is produced by forward motion of the vehicle (rad)
         * delTime is the time interval for the measurement of delAng (sec)
         * timeStamp_ms is the time when the rotation was last measured (msec)
         * posOffset is the XYZ body frame position of the wheel hub (m)
         */
        EKF3.writeWheelOdom(delta_angle, delta_time, wheel_encoder_last_update_ms[i], g2.wheel_encoder.get_pos_offset(i), g2.wheel_encoder.get_wheel_radius(i));
    }

    // record system time update for next iteration
    wheel_encoder_last_ekf_update_ms = now;
}

// Calibrate compass
void Rover::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

// Save compass offsets
void Rover::compass_save() {
    if (g.compass_enabled &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !arming.is_armed()) {
        compass.save_offsets();
    }
}

// Accel calibration

void Rover::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them    float trim_roll, trim_pitch;
    float trim_roll, trim_pitch;
    if (ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

// read the rangefinders
void Rover::read_rangefinders(void)
{
    rangefinder.update();

    AP_RangeFinder_Backend *s0 = rangefinder.get_backend(0);
    AP_RangeFinder_Backend *s1 = rangefinder.get_backend(1);

    if (s0 == nullptr || s0->status() == RangeFinder::RangeFinder_NotConnected) {
        // this makes it possible to disable rangefinder at runtime
        return;
    }

    if (s1 != nullptr && s1->has_data()) {
        // we have two rangefinders
        obstacle.rangefinder1_distance_cm = s0->distance_cm();
        obstacle.rangefinder2_distance_cm = s1->distance_cm();
        if (obstacle.rangefinder1_distance_cm < static_cast<uint16_t>(g.rangefinder_trigger_cm) &&
            obstacle.rangefinder1_distance_cm < static_cast<uint16_t>(obstacle.rangefinder2_distance_cm))  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.rangefinder_debounce) {
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder1 obstacle %u cm",
                        static_cast<uint32_t>(obstacle.rangefinder1_distance_cm));
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = g.rangefinder_turn_angle;
        } else if (obstacle.rangefinder2_distance_cm < static_cast<uint16_t>(g.rangefinder_trigger_cm)) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.rangefinder_debounce) {
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder2 obstacle %u cm",
                        static_cast<uint32_t>(obstacle.rangefinder2_distance_cm));
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = -g.rangefinder_turn_angle;
        }
    } else {
        // we have a single rangefinder
        obstacle.rangefinder1_distance_cm = s0->distance_cm();
        obstacle.rangefinder2_distance_cm = 0;
        if (obstacle.rangefinder1_distance_cm < static_cast<uint16_t>(g.rangefinder_trigger_cm))  {
            // obstacle detected in front
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.rangefinder_debounce) {
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder obstacle %u cm",
                        static_cast<uint32_t>(obstacle.rangefinder1_distance_cm));
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = g.rangefinder_turn_angle;
        }
    }

    Log_Write_Rangefinder();
    Log_Write_Depth();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.rangefinder_debounce &&
        AP_HAL::millis() > obstacle.detected_time_ms + g.rangefinder_turn_time*1000) {
        gcs().send_text(MAV_SEVERITY_INFO, "Obstacle passed");
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}

// initialise proximity sensor
void Rover::init_proximity(void)
{
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
}

/*
  ask airspeed sensor for a new value, duplicated from plane
 */
void Rover::read_airspeed(void)
{
    g2.airspeed.update(should_log(MASK_LOG_IMU));
}

/*
  update RPM sensors
 */
void Rover::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RC)) {
            logger.Write_RPM(rpm_sensor);
        }
    }
}
