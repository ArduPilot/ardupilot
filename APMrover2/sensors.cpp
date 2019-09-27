#include "Rover.h"

#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_VisualOdom/AP_VisualOdom.h>

// check for new compass data - 10Hz
void Rover::update_compass(void)
{
    if (AP::compass().enabled() && compass.read()) {
        ahrs.set_compass(&compass);
    }
}

// Save compass offsets
void Rover::compass_save() {
    if (AP::compass().enabled() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !arming.is_armed()) {
        compass.save_offsets();
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

    // save cumulative distances at current time (in meters) for reporting to GCS
    for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
        wheel_encoder_last_distance_m[i] = g2.wheel_encoder.get_distance(i);
    }

    // send wheel encoder delta angle and delta time to EKF
    // this should not be done at more than 50hz
    // initialise on first iteration
    if (!wheel_encoder_initialised) {
        wheel_encoder_initialised = true;
        for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
            wheel_encoder_last_angle_rad[i] = g2.wheel_encoder.get_delta_angle(i);
            wheel_encoder_last_reading_ms[i] = g2.wheel_encoder.get_last_reading_ms(i);
        }
        return;
    }

    // on each iteration send data from alternative wheel encoders
    wheel_encoder_last_index_sent++;
    if (wheel_encoder_last_index_sent >= g2.wheel_encoder.num_sensors()) {
        wheel_encoder_last_index_sent = 0;
    }

    // get current time, total delta angle (since startup) and update time from sensor
    const float curr_angle_rad = g2.wheel_encoder.get_delta_angle(wheel_encoder_last_index_sent);
    const uint32_t sensor_reading_ms = g2.wheel_encoder.get_last_reading_ms(wheel_encoder_last_index_sent);
    const uint32_t now_ms = AP_HAL::millis();

    // calculate angular change (in radians)
    const float delta_angle = curr_angle_rad - wheel_encoder_last_angle_rad[wheel_encoder_last_index_sent];
    wheel_encoder_last_angle_rad[wheel_encoder_last_index_sent] = curr_angle_rad;

    // calculate delta time using time between sensor readings or time since last send to ekf (whichever is shorter)
    uint32_t sensor_diff_ms = sensor_reading_ms - wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent];
    if (sensor_diff_ms == 0 || sensor_diff_ms > 100) {
        // if no sensor update or time difference between sensor readings is too long use time since last send to ekf
        sensor_diff_ms = now_ms - wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent];
        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent] = now_ms;
    } else {
        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent] = sensor_reading_ms;
    }
    const float delta_time = sensor_diff_ms * 0.001f;

    /* delAng is the measured change in angular position from the previous measurement where a positive rotation is produced by forward motion of the vehicle (rad)
     * delTime is the time interval for the measurement of delAng (sec)
     * timeStamp_ms is the time when the rotation was last measured (msec)
     * posOffset is the XYZ body frame position of the wheel hub (m)
     */
    EKF3.writeWheelOdom(delta_angle,
                        delta_time,
                        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent],
                        g2.wheel_encoder.get_pos_offset(wheel_encoder_last_index_sent),
                        g2.wheel_encoder.get_wheel_radius(wheel_encoder_last_index_sent));
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
    Log_Write_Depth();
}

// initialise proximity sensor
void Rover::init_proximity(void)
{
    g2.proximity.init();
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
