#include "GCS_Copter.h"

#include "Copter.h"

const char* GCS_Copter::frame_string() const
{
    return copter.get_frame_string();
}

bool GCS_Copter::simple_input_active() const
{
    return copter.ap.simple_mode == 1;
}

bool GCS_Copter::supersimple_input_active() const
{
    return copter.ap.simple_mode == 2;
}

void GCS_Copter::update_vehicle_sensor_status_flags(void)
{
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

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
#if PRECISION_LANDING == ENABLED
    if (copter.precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    const AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom && visual_odom->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    const Copter::ap_t &ap = copter.ap;

    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif
#if RANGEFINDER_ENABLED == ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    switch (copter.control_mode) {
    case Mode::Number::AUTO:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
    case Mode::Number::RTL:
    case Mode::Number::CIRCLE:
    case Mode::Number::LAND:
    case Mode::Number::POSHOLD:
    case Mode::Number::BRAKE:
    case Mode::Number::THROW:
    case Mode::Number::SMART_RTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case Mode::Number::ALT_HOLD:
    case Mode::Number::GUIDED_NOGPS:
    case Mode::Number::SPORT:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLOWHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

    if (gps.is_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (ap.rc_receiver_present && !copter.failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if OPTFLOW == ENABLED
    if (optflow && optflow->healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (copter.precland.enabled() && copter.precland.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (visual_odom && visual_odom->enabled() && visual_odom->healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif

#if PROXIMITY_ENABLED == ENABLED
    if (!copter.g2.proximity.sensor_failed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (copter.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (copter.rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif
}
