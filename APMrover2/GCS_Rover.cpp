#include "GCS_Rover.h"

#include "Rover.h"

#include <AP_RangeFinder/RangeFinder_Backend.h>

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
    // first what sensors/controllers we have
    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    const AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom && visual_odom->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    const AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity && proximity->get_status() > AP_Proximity::Proximity_NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }

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

    if (gps.is_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (visual_odom && visual_odom->enabled() && visual_odom->healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        AP_RangeFinder_Backend *s = rangefinder->get_backend(0);
        if (s != nullptr && s->has_data()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
    if (proximity && proximity->get_status() != AP_Proximity::Proximity_NoData) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
}
