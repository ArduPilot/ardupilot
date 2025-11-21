#include "GCS_Copter.h"

#include "Copter.h"

const char* GCS_Copter::frame_string() const
{
    if (copter.motors == nullptr) {
        return "MultiCopter";
    }
    return copter.motors->get_frame_string();
}

bool GCS_Copter::simple_input_active() const
{
    return copter.simple_mode == Copter::SimpleMode::SIMPLE;
}

bool GCS_Copter::supersimple_input_active() const
{
    return copter.simple_mode == Copter::SimpleMode::SUPERSIMPLE;
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

    // Update position controller flags
    if (copter.pos_control != nullptr) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

        // XY position controller
        if (copter.pos_control->NE_is_active()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        }

        // Z altitude controller
        if (copter.pos_control->D_is_active()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        }
    }

    // optional sensors, some of which are essentially always
    // available in the firmware:
#if HAL_PROXIMITY_ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (!copter.g2.proximity.sensor_failed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (copter.rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

#if AC_PRECLAND_ENABLED
    if (copter.precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    if (copter.precland.enabled() && copter.precland.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif

#if AP_TERRAIN_AVAILABLE
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

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    // only mark propulsion healthy if all of the motors are producing
    // nominal thrust
    if (!copter.motors->get_thrust_boost()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    }
}
