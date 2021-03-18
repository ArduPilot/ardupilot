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

    const Blimp::ap_t &ap = blimp.ap;

    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    // switch (blimp.control_mode) {
    // case Mode::Number::AUTO:
    // case Mode::Number::AVOID_ADSB:
    // case Mode::Number::GUIDED:
    // case Mode::Number::LOITER:
    //     control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    //     control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    //     control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    //     control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    //     break;
    // case Mode::Number::ALT_HOLD:
    // case Mode::Number::GUIDED_NOGPS:
    // case Mode::Number::SPORT:
    // case Mode::Number::AUTOTUNE:
    // case Mode::Number::FLOWHOLD:
    //     control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    //     control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    //     break;
    // default:
    //     // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
    //     break;
    // }

    if (ap.rc_receiver_present && !blimp.failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

}
