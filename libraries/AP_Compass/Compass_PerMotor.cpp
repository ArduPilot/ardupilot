/*
  per-motor compass compensation
 */

#include "AP_Compass.h"
#include "LogStructure.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Compass_PerMotor::var_info[] = {
    // @Param: EN
    // @DisplayName: per-motor compass correction enable
    // @Description: This enables per-motor compass corrections
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("EN",  1, Compass_PerMotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _EXP
    // @DisplayName: per-motor exponential correction
    // @Description: This is the exponential correction for the power output of the motor for per-motor compass correction
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    // index 2

    // @Param: 1_X
    // @DisplayName: Compass per-motor1 X
    // @Description: Compensation for X axis of motor1
    // @User: Advanced

    // @Param: 1_Y
    // @DisplayName: Compass per-motor1 Y
    // @Description: Compensation for Y axis of motor1
    // @User: Advanced

    // @Param: 1_Z
    // @DisplayName: Compass per-motor1 Z
    // @Description: Compensation for Z axis of motor1
    // @User: Advanced
    AP_GROUPINFO("1",  3, Compass_PerMotor, compensation[0], 0),

    // @Param: 2_X
    // @DisplayName: Compass per-motor2 X
    // @Description: Compensation for X axis of motor2
    // @User: Advanced

    // @Param: 2_Y
    // @DisplayName: Compass per-motor2 Y
    // @Description: Compensation for Y axis of motor2
    // @User: Advanced

    // @Param: 2_Z
    // @DisplayName: Compass per-motor2 Z
    // @Description: Compensation for Z axis of motor2
    // @User: Advanced
    AP_GROUPINFO("2",  4, Compass_PerMotor, compensation[1], 0),

    // @Param: 3_X
    // @DisplayName: Compass per-motor3 X
    // @Description: Compensation for X axis of motor3
    // @User: Advanced

    // @Param: 3_Y
    // @DisplayName: Compass per-motor3 Y
    // @Description: Compensation for Y axis of motor3
    // @User: Advanced

    // @Param: 3_Z
    // @DisplayName: Compass per-motor3 Z
    // @Description: Compensation for Z axis of motor3
    // @User: Advanced
    AP_GROUPINFO("3",  5, Compass_PerMotor, compensation[2], 0),

    // @Param: 4_X
    // @DisplayName: Compass per-motor4 X
    // @Description: Compensation for X axis of motor4
    // @User: Advanced

    // @Param: 4_Y
    // @DisplayName: Compass per-motor4 Y
    // @Description: Compensation for Y axis of motor4
    // @User: Advanced

    // @Param: 4_Z
    // @DisplayName: Compass per-motor4 Z
    // @Description: Compensation for Z axis of motor4
    // @User: Advanced
    AP_GROUPINFO("4",  6, Compass_PerMotor, compensation[3], 0),

#if AP_COMPASS_PMOT_MAX_NUM_MOTORS > 4
    // @Param: 5_X
    // @DisplayName: Compass per-motor5 X
    // @Description: Compensation for X axis of motor5
    // @User: Advanced

    // @Param: 5_Y
    // @DisplayName: Compass per-motor4 Y
    // @Description: Compensation for Y axis of motor5
    // @User: Advanced

    // @Param: 5_Z
    // @DisplayName: Compass per-motor5 Z
    // @Description: Compensation for Z axis of motor5
    // @User: Advanced
    AP_GROUPINFO("5",  7, Compass_PerMotor, compensation[4], 0),
#endif
#if AP_COMPASS_PMOT_MAX_NUM_MOTORS > 5
    // @Param: 6_X
    // @DisplayName: Compass per-motor6 X
    // @Description: Compensation for X axis of motor6
    // @User: Advanced

    // @Param: 6_Y
    // @DisplayName: Compass per-motor6 Y
    // @Description: Compensation for Y axis of motor6
    // @User: Advanced

    // @Param: 6_Z
    // @DisplayName: Compass per-motor4 6
    // @Description: Compensation for Z axis of motor6
    // @User: Advanced
    AP_GROUPINFO("6",  8, Compass_PerMotor, compensation[5], 0),
#endif
#if AP_COMPASS_PMOT_MAX_NUM_MOTORS > 6
    // @Param: 7_X
    // @DisplayName: Compass per-motor7 X
    // @Description: Compensation for X axis of motor7
    // @User: Advanced

    // @Param: 7_Y
    // @DisplayName: Compass per-motor7 Y
    // @Description: Compensation for Y axis of motor7
    // @User: Advanced

    // @Param: 7_Z
    // @DisplayName: Compass per-motor74 Z
    // @Description: Compensation for Z axis of motor7
    // @User: Advanced
    AP_GROUPINFO("7",  9, Compass_PerMotor, compensation[6], 0),
#endif
#if AP_COMPASS_PMOT_MAX_NUM_MOTORS > 7
    // @Param: 8_X
    // @DisplayName: Compass per-motor8 X
    // @Description: Compensation for X axis of motor8
    // @User: Advanced

    // @Param: 8_Y
    // @DisplayName: Compass per-motor8 Y
    // @Description: Compensation for Y axis of motor8
    // @User: Advanced

    // @Param: 8_Z
    // @DisplayName: Compass per-motor8 Z
    // @Description: Compensation for Z axis of motor8
    // @User: Advanced
    AP_GROUPINFO("8",  10, Compass_PerMotor, compensation[7], 0),
#endif
    AP_GROUPEND
};

// constructor
Compass_PerMotor::Compass_PerMotor()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float Compass_PerMotor::scaled_output(uint8_t motor) const
{
#if APM_BUILD_COPTER_OR_HELI
    AP_Motors* motors = AP::motors();

    if (motors == nullptr || !motors->is_motor_enabled(motor)) {
        return 0.0f;
    }

    return motors->get_power_out(motor);
#else
    return 0.0f;
#endif
}

/*
  calculate total offset for per-motor compensation
 */
Vector3f Compass_PerMotor::compensate(float current)
{
    Vector3f offset;
    if (!enable) {
        // don't compensate while calibrating
        return offset;
    }

    for (uint8_t i=0; i<AP_COMPASS_PMOT_MAX_NUM_MOTORS; i++) {
        float output = scaled_output(i);

        const Vector3f &c = compensation[i].get();

        offset += c * output * current;
    }

    return offset;
}

void Compass_PerMotor::copy_from(const Compass_PerMotor per_motor)
{
    for (uint8_t i=0; i<AP_COMPASS_PMOT_MAX_NUM_MOTORS; i++) {
        compensation[i].set_and_save_ifchanged(per_motor.compensation[i]);
    }
}

void Compass_PerMotor::log_offsets(const uint64_t time_us, const uint8_t mag_instance) const
{
    Vector3f offsets[AP_COMPASS_PMOT_MAX_NUM_MOTORS];
    float current;
    // battery current is only updated at 10Hz so the motor compensation
    // is unlikely to change for long periods
    if (!enable || !AP::battery().current_amps(current)) {
        return;
    }

    for (uint8_t i=0; i<AP_COMPASS_PMOT_MAX_NUM_MOTORS; i++) {
        float output = scaled_output(i);

        const Vector3f &c = compensation[i].get();

        offsets[i] = c * output * current;
    }

    const struct log_MAG_PerMotor pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAG_PMOT_MSG),
        time_us         : time_us,
        instance        : mag_instance,
        motor1_offset_x  : offsets[0].x,
        motor1_offset_y  : offsets[0].y,
        motor1_offset_z  : offsets[0].z,
        motor2_offset_x  : offsets[1].x,
        motor2_offset_y  : offsets[1].y,
        motor2_offset_z  : offsets[1].z,
        motor3_offset_x  : offsets[2].x,
        motor3_offset_y  : offsets[2].y,
        motor3_offset_z  : offsets[2].z,
        motor4_offset_x  : offsets[3].x,
        motor4_offset_y  : offsets[3].y,
        motor4_offset_z  : offsets[3].z,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
