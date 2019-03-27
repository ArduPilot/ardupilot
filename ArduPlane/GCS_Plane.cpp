#include "GCS_Plane.h"
#include "Plane.h"

void GCS_Plane::update_vehicle_sensor_status_flags(void)
{
    // first what sensors/controllers we have
    if (plane.g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }

    const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed && airspeed->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    const OpticalFlow *optflow = AP::opticalflow();
    if (optflow && optflow->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (plane.geofence_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }

    if (plane.have_reverse_thrust()) {
        control_sensors_present |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }

    if (airspeed && airspeed->enabled() && airspeed->use()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    if (plane.geofence_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }

    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    bool rate_controlled = false;
    bool attitude_stabilized = false;
    switch (plane.control_mode) {
    case MANUAL:
        break;

    case ACRO:
    case QACRO:
        rate_controlled = true;
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case QSTABILIZE:
    case QHOVER:
    case QLAND:
    case QLOITER:
    case QAUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
        rate_controlled = true;
        attitude_stabilized = true;
        break;

    case TRAINING:
        if (!plane.training_manual_roll || !plane.training_manual_pitch) {
            rate_controlled = true;
            attitude_stabilized = true;
        }
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
    case CIRCLE:
    case QRTL:
        rate_controlled = true;
        attitude_stabilized = true;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;

    case INITIALISING:
        break;
    }

    if (rate_controlled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
    }
    if (attitude_stabilized) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    }

    AP_AHRS &ahrs = AP::ahrs();

    const Compass &compass = AP::compass();
    if (plane.g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.is_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow && optflow->healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (airspeed && airspeed->all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
#if GEOFENCE_ENABLED
    if (!plane.geofence_breached()) {
        control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    if (millis() - plane.failsafe.last_valid_rc_ms < 200) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

#if AP_TERRAIN_AVAILABLE
    switch (plane.terrain.status()) {
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

    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (plane.g.rangefinder_landing) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
    }

    if (plane.have_reverse_thrust() && SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_REVERSE_MOTOR;
        control_sensors_health |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }
}
