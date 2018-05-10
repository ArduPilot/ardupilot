#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{

    // notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight
    float height;
#if AP_TERRAIN_AVAILABLE
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // use the best available alt estimate via baro above home
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude;
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    if ((rangefinder.num_sensors() > 0) && should_log(MASK_LOG_SONAR)) {
        Log_Write_Sonar();
    }

    rangefinder_height_update();
}

/*
  calibrate compass
*/
void Plane::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

/*
    Accel calibration
*/
void Plane::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        if (should_log(MASK_LOG_IMU)) {
            DataFlash.Log_Write_Airspeed(airspeed);
        }

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }
    }

    // we calculate airspeed errors (and thus target_airspeed_cm) even
    // when airspeed is disabled as TECS may be using synthetic
    // airspeed for a quadplane transition
    calc_airspeed_errors();
    
    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }
}

/*
  update RPM sensors
 */
void Plane::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RC)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}

bool GCS_Plane::compass_enabled() const
{
    return plane.g.compass_enabled;
}

bool GCS_Plane::vehicle_initialised() const {
    return !AP_Notify::flags.initialising;
}

AP_GPS::GPS_Status GCS_Plane::min_gps_state() const {
    return AP_GPS::GPS_OK_FIX_3D;
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink.
void GCS_Plane::update_sensor_status_flags(void)
{
    GCS::update_sensor_status_flags();

     // default sensors present
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
        MAV_SYS_STATUS_SENSOR_RC_RECEIVER;

    // update airspeed sensor
    const AP_Airspeed &airspeed = plane.airspeed;
    if (airspeed.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        if (airspeed.use()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
        if (airspeed.all_healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
    }

    // update optical flow
#if OPTFLOW == ENABLED
    const OpticalFlow &optflow = plane.optflow;
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        if (optflow.healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        }
    }
#endif

    // update geofence status:
    if (plane.geofence_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
        if (plane.geofence_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
            if (!plane.geofence_breached()) {
                control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;
            }
        }
    }

    // update reverse-motor status:
    if (plane.aparm.throttle_min < 0) {
        control_sensors_present |= MAV_SYS_STATUS_REVERSE_MOTOR;
        if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < 0) {
            control_sensors_enabled |= MAV_SYS_STATUS_REVERSE_MOTOR;
            control_sensors_health |= MAV_SYS_STATUS_REVERSE_MOTOR;
        }
    }

    const FlightMode control_mode = plane.control_mode;
    uint32_t control_mode_sensors = 0;
    switch (control_mode) {
    case MANUAL:
        break;

    case ACRO:
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case QSTABILIZE:
    case QHOVER:
    case QLAND:
    case QLOITER:
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
    case CRUISE:
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case TRAINING:
        if (!plane.training_manual_roll || !plane.training_manual_pitch) {
            control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
            control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation        
        }
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
    case CIRCLE:
    case QRTL:
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
        control_mode_sensors |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        break;

    case INITIALISING:
        break;
    }

    control_sensors_enabled |= control_mode_sensors;
    // we could update these based on e.g. the ahrs attitude and LIM_*_CD
    control_sensors_health |= control_mode_sensors;

    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    if (millis() - plane.failsafe.last_valid_rc_ms < 200) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

#if AP_TERRAIN_AVAILABLE
    const AP_Terrain &terrain = plane.terrain;
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    const RangeFinder &rangefinder = plane.rangefinder;
    if (rangefinder.has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (plane.g.rangefinder_landing) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
    }

#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    plane.frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}
