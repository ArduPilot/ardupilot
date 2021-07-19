#include "GCS_Rover.h"

#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

uint8_t GCS_Rover::sysid_this_mav() const
{
    return rover.g.sysid_this_mav;
}

bool GCS_Rover::simple_input_active() const
{
    if (rover.control_mode != &rover.mode_simple) {
        return false;
    }
    return (rover.g2.simple_type == ModeSimple::Simple_InitialHeading);
}

bool GCS_Rover::supersimple_input_active() const
{
    if (rover.control_mode != &rover.mode_simple) {
        return false;
    }
    return (rover.g2.simple_type == ModeSimple::Simple_CardinalDirections);
}

void GCS_Rover::update_vehicle_sensor_status_flags(void)
{
    // mode-specific:
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    if (rover.control_mode->attitude_stabilized()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // 3D angular rate control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // 3D angular rate control
    }
    if (rover.control_mode->is_autopilot_mode()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
    }

#if HAL_PROXIMITY_ENABLED
    const AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity && proximity->get_status() > AP_Proximity::Status::NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (proximity && proximity->get_status() != AP_Proximity::Status::NoData) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        AP_RangeFinder_Backend *s = rangefinder->get_backend(0);
        if (s != nullptr && s->has_data()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
}
