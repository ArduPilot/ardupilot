#include "AP_MotorsHawk.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHawk::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: H_CGAIN
    // @DisplayName: HAWK collective gain
    // @Description: Scales the collective throttle command before cyclic modulation
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("H_CGAIN", 1, AP_MotorsHawk, _collective_gain, 1.0f),

    // @Param: H_RGAIN
    // @DisplayName: HAWK roll cyclic gain
    // @Description: Gain applied to roll harmonic modulation
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_RGAIN", 2, AP_MotorsHawk, _cyclic_roll_gain, 0.10f),

    // @Param: H_PGAIN
    // @DisplayName: HAWK pitch cyclic gain
    // @Description: Gain applied to pitch harmonic modulation
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_PGAIN", 3, AP_MotorsHawk, _cyclic_pitch_gain, 0.10f),

    // @Param: H_YGAIN
    // @DisplayName: HAWK yaw gain
    // @Description: Gain applied to yaw bias term
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YGAIN", 4, AP_MotorsHawk, _yaw_gain, 0.05f),

    // @Param: H_CMAX
    // @DisplayName: HAWK cyclic limit
    // @Description: Maximum magnitude of cyclic contribution per motor
    // @Range: 0.0 0.5
    // @User: Advanced
    AP_GROUPINFO("H_CMAX", 5, AP_MotorsHawk, _cyclic_max, 0.20f),

    // @Param: H_RP1
    // @DisplayName: HAWK motor1 roll phase
    // @Description: Roll phase offset for motor1
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_RP1", 6, AP_MotorsHawk, _phase_roll_deg[0], 0.0f),

    // @Param: H_RP2
    // @DisplayName: HAWK motor2 roll phase
    // @Description: Roll phase offset for motor2
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_RP2", 7, AP_MotorsHawk, _phase_roll_deg[1], 120.0f),

    // @Param: H_RP3
    // @DisplayName: HAWK motor3 roll phase
    // @Description: Roll phase offset for motor3
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_RP3", 8, AP_MotorsHawk, _phase_roll_deg[2], -120.0f),

    // @Param: H_PP1
    // @DisplayName: HAWK motor1 pitch phase
    // @Description: Pitch phase offset for motor1
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_PP1", 9, AP_MotorsHawk, _phase_pitch_deg[0], 90.0f),

    // @Param: H_PP2
    // @DisplayName: HAWK motor2 pitch phase
    // @Description: Pitch phase offset for motor2
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_PP2", 10, AP_MotorsHawk, _phase_pitch_deg[1], -30.0f),

    // @Param: H_PP3
    // @DisplayName: HAWK motor3 pitch phase
    // @Description: Pitch phase offset for motor3
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_PP3", 11, AP_MotorsHawk, _phase_pitch_deg[2], -150.0f),

    // @Param: H_YB1
    // @DisplayName: HAWK motor1 yaw bias
    // @Description: Relative yaw bias for motor1
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YB1", 12, AP_MotorsHawk, _yaw_bias[0], 1.0f),

    // @Param: H_YB2
    // @DisplayName: HAWK motor2 yaw bias
    // @Description: Relative yaw bias for motor2
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YB2", 13, AP_MotorsHawk, _yaw_bias[1], -0.5f),

    // @Param: H_YB3
    // @DisplayName: HAWK motor3 yaw bias
    // @Description: Relative yaw bias for motor3
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YB3", 14, AP_MotorsHawk, _yaw_bias[2], -0.5f),

    AP_GROUPEND
};

AP_MotorsHawk::AP_MotorsHawk(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz),
    _encoders_initialized(false)
{
    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        _theta_rad[i] = 0.0f;
        _encoder_healthy[i] = false;
        _hawk_out[i] = 0.0f;
    }
}

const char* AP_MotorsHawk::_get_frame_string() const
{
    return "HAWK";
}

void AP_MotorsHawk::init(motor_frame_class frame_class,
                         motor_frame_type frame_type)
{
    set_frame_class_and_type(frame_class, frame_type);

    const bool ok = (frame_class == MOTOR_FRAME_HAWK) && _encoders_initialized;
    set_initialised_ok(ok);
}

void AP_MotorsHawk::set_frame_class_and_type(motor_frame_class frame_class,
                                             motor_frame_type frame_type)
{
    (void)frame_type;

    // clear state first
    _encoders_initialized = false;

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_enabled[i] = false;
    }

    if (frame_class != MOTOR_FRAME_HAWK) {
        return;
    }

    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);

    motor_enabled[MOTOR_HAWK_1] = true;
    motor_enabled[MOTOR_HAWK_2] = true;
    motor_enabled[MOTOR_HAWK_3] = true;

    _encoders.init();
    _encoders_initialized = true;
}

bool AP_MotorsHawk::arming_checks(size_t buflen, char *buffer) const
{
    if (!AP_MotorsMulticopter::arming_checks(buflen, buffer)) {
        return false;
    }

    if (!_encoders_initialized) {
        hal.util->snprintf(buffer, buflen, "HAWK encoders not initialized");
        return false;
    }

    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        if (!_encoder_healthy[i]) {
            hal.util->snprintf(buffer, buflen, "HAWK encoder %u invalid", unsigned(i + 1));
            return false;
        }
    }

    return true;
}

bool AP_MotorsHawk::motor_test_checks(size_t buflen, char *buffer) const
{
    if (!arming_checks(buflen, buffer)) {
        return false;
    }
    return true;
}

uint32_t AP_MotorsHawk::get_motor_mask()
{
    return (1U << 0) | (1U << 1) | (1U << 2);
}

void AP_MotorsHawk::update_encoder_state()
{
    if (!_encoders_initialized) {
        for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
            _encoder_healthy[i] = false;
        }
        return;
    }

    _encoders.update();

    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        const bool good = _encoders.healthy(i) &&
                          !_encoders.stale(i, ENCODER_TIMEOUT_US);

        _encoder_healthy[i] = good;

        if (good) {
            _theta_rad[i] = _encoders.get_angle_rad(i);
        }
    }
}

bool AP_MotorsHawk::encoders_healthy() const
{
    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        if (!_encoder_healthy[i]) {
            return false;
        }
    }
    return true;
}

float AP_MotorsHawk::compute_collective_thrust(float throttle_in) const
{
    return constrain_float(throttle_in * _collective_gain, 0.0f, 1.0f);
}

float AP_MotorsHawk::compute_cyclic_term(uint8_t motor_idx,
                                         float theta_rad,
                                         float roll_in,
                                         float pitch_in,
                                         float yaw_in) const
{
    const float roll_phase_rad  = radians((float)_phase_roll_deg[motor_idx]);
    const float pitch_phase_rad = radians((float)_phase_pitch_deg[motor_idx]);

    const float roll_term =
        constrain_float(roll_in, -1.0f, 1.0f) *
        _cyclic_roll_gain *
        cosf(theta_rad - roll_phase_rad);

    const float pitch_term =
        constrain_float(pitch_in, -1.0f, 1.0f) *
        _cyclic_pitch_gain *
        cosf(theta_rad - pitch_phase_rad);

    // first-pass yaw model: mean torque bias, not harmonic
    const float yaw_term =
        constrain_float(yaw_in, -1.0f, 1.0f) *
        _yaw_gain *
        _yaw_bias[motor_idx];

    const float total = roll_term + pitch_term + yaw_term;
    return constrain_float(total, -(float)_cyclic_max, (float)_cyclic_max);
}

float AP_MotorsHawk::apply_output_limits(float in) const
{
    return constrain_float(in, 0.0f, 1.0f);
}

void AP_MotorsHawk::set_actuator_safe()
{
    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        _hawk_out[i] = 0.0f;
    }
}

void AP_MotorsHawk::output_armed_stabilizing()
{
    update_encoder_state();

    if (!encoders_healthy()) {
        set_actuator_safe();

        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;
        limit.throttle_upper = false;
        return;
    }

    const float roll_in = _roll_in;
    const float pitch_in = _pitch_in;
    const float yaw_in = _yaw_in;
    const float throttle_in = _throttle_in;

    const float collective = compute_collective_thrust(throttle_in);

    bool hit_upper = false;
    bool hit_lower = false;

    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        const float cyclic = compute_cyclic_term(i,
                                                 _theta_rad[i],
                                                 roll_in,
                                                 pitch_in,
                                                 yaw_in);

        const float out = apply_output_limits(collective + cyclic);
        _hawk_out[i] = out;

        if (out >= 0.999f) {
            hit_upper = true;
        }
        if (out <= 0.001f) {
            hit_lower = true;
        }
    }

    limit.roll = hit_upper || hit_lower;
    limit.pitch = hit_upper || hit_lower;
    limit.yaw = hit_upper || hit_lower;
    limit.throttle_lower = hit_lower;
    limit.throttle_upper = hit_upper;
}

void AP_MotorsHawk::output_to_motors()
{
    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        set_actuator_safe();
        break;

    case SpoolState::GROUND_IDLE: {
        const float idle = actuator_spin_up_to_ground_idle();
        for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
            _hawk_out[i] = idle;
        }
        break;
    }

    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        output_armed_stabilizing();
        break;
    }

    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        set_actuator_with_slew(_actuator[i], _hawk_out[i]);
        rc_write(i, output_to_pwm(_actuator[i]));
    }
}

void AP_MotorsHawk::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    if (motor_seq < 1 || motor_seq > HAWK_NUM_MOTORS) {
        return;
    }

    const uint8_t motor_idx = motor_seq - 1;
    rc_write(motor_idx, pwm);
}