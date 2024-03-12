#include "GCS_Blimp.h"

#include "Blimp.h"

uint8_t GCS_Blimp::sysid_this_mav() const
{
    return blimp.g.sysid_this_mav;
}

const char* GCS_Blimp::frame_string() const
{
    return blimp.get_frame_string();
}

void GCS_Blimp::update_vehicle_sensor_status_flags(void)
{
    // mode-specific flags:
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

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    const Blimp::ap_t &ap = blimp.ap;

    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (ap.rc_receiver_present && !blimp.failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
}
