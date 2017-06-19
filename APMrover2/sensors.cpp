#include "Rover.h"

void Rover::init_barometer(bool full_calibration)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    } else {
        barometer.update_calibration();
    }
    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

void Rover::init_sonar(void)
{
    sonar.init();
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Rover::read_battery(void)
{
    battery.read();
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Rover::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

// Calibrate compass
void Rover::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
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

// read the sonars
void Rover::read_sonars(void)
{
    sonar.update();

    if (sonar.status(0) == RangeFinder::RangeFinder_NotConnected) {
        // this makes it possible to disable sonar at runtime
        return;
    }

    if (sonar.has_data(1)) {
        // we have two sonars
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = sonar.distance_cm(1);
        if (obstacle.sonar1_distance_cm < (uint16_t)g.sonar_trigger_cm &&
            obstacle.sonar1_distance_cm < (uint16_t)obstacle.sonar2_distance_cm)  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Sonar1 obstacle %u cm",
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        } else if (obstacle.sonar2_distance_cm < (uint16_t)g.sonar_trigger_cm) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Sonar2 obstacle %u cm",
                                  (unsigned)obstacle.sonar2_distance_cm);
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
    } else {
        // we have a single sonar
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = 0;
        if (obstacle.sonar1_distance_cm < (uint16_t)g.sonar_trigger_cm)  {
            // obstacle detected in front
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Sonar obstacle %u cm",
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = AP_HAL::millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }

    Log_Write_Sonar();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.sonar_debounce &&
        AP_HAL::millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Obstacle passed");
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}

/*
  update AP_Button
 */
void Rover::button_update(void)
{
    button.update();
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Rover::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;  // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    if (rover.DataFlash.logging_present()) {  // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }


    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION &
                                                         ~MAV_SYS_STATUS_SENSOR_YAW_POSITION &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING);

    switch (control_mode) {
    case MANUAL:
    case HOLD:
        break;

    case LEARNING:
    case STEERING:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;    // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;  // attitude stabilisation
        break;

    case AUTO:
    case RTL:
    case GUIDED:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;    // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;  // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;            // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;     // X/Y position control
        break;

    case INITIALISING:
        break;
    }

    if (rover.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default to all healthy except compass and gps which we set individually
    control_sensors_health = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_3D_MAG & ~MAV_SYS_STATUS_SENSOR_GPS);
    if (g.compass_enabled && compass.healthy(0) && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (sonar.num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (g.sonar_trigger_cm > 0) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (sonar.has_data(0)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }

    if (rover.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

    if (AP_Notify::flags.initialising) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }
#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}
