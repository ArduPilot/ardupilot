#include "AP_Arming.h"
#include "Rover.h"

enum HomeState AP_Arming_Rover::home_status() const
{
    return rover.home_is_set;
}

bool AP_Arming_Rover::pre_arm_checks(bool report)
{
    if (rover.motor_type_class == Rover::UGV_TYPE_UNDEFINED) {
        if (report) {
            rover.gcs_send_text(MAV_SEVERITY_WARNING, "PreArm: FRAME_TYPE unset");
        }
        return false;
    }
    return hardware_safety_check(report)
           &  barometer_checks(report)
           &  ins_checks(report)
           &  compass_checks(report)
           &  gps_checks(report)
           &  battery_checks(report)
           &  logging_checks(report)
           &  manual_transmitter_checks(report)
           &  board_voltage_checks(report);
}
