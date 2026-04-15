#pragma once

#include "AP_MotorsMulticopter.h"

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HawkEncoder/AP_HawkEncoder.h>

class AP_MotorsHawk : public AP_MotorsMulticopter
{
public:
    friend class AP_Motors;

    AP_MotorsHawk(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    void init(motor_frame_class frame_class,
              motor_frame_type frame_type) override;

    void set_frame_class_and_type(motor_frame_class frame_class,
                                  motor_frame_type frame_type) override;

    bool arming_checks(size_t buflen, char *buffer) const override;
    bool motor_test_checks(size_t buflen, char *buffer) const override;

    uint32_t get_motor_mask() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void output_to_motors() override;
    void output_armed_stabilizing() override;
    const char* _get_frame_string() const override;
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    void update_encoder_state();
    bool encoders_healthy() const;
    void set_actuator_safe();
    float compute_collective_thrust(float throttle_in) const;
    float compute_cyclic_term(uint8_t motor_idx,
                              float theta_rad,
                              float roll_in,
                              float pitch_in,
                              float yaw_in) const;
    float apply_output_limits(float in) const;

    // debug helpers
    void send_debug_text(MAV_SEVERITY severity, const char *fmt, ...) const;
    void send_encoder_debug_if_due();
    void send_encoder_fault_if_needed();

private:
    static constexpr uint8_t HAWK_NUM_MOTORS = 3;
    static constexpr uint32_t ENCODER_TIMEOUT_US = 20000U; // 20 ms

    enum MotorIndex : uint8_t {
        MOTOR_HAWK_1 = 0,
        MOTOR_HAWK_2 = 1,
        MOTOR_HAWK_3 = 2
    };

    AP_HawkEncoder _encoders;

    float _theta_rad[HAWK_NUM_MOTORS];
    bool  _encoder_healthy[HAWK_NUM_MOTORS];
    float _hawk_out[HAWK_NUM_MOTORS];

    AP_Float _cyclic_roll_gain;
    AP_Float _cyclic_pitch_gain;
    AP_Float _yaw_gain;
    AP_Float _collective_gain;
    AP_Float _cyclic_max;

    AP_Float _phase_roll_deg[HAWK_NUM_MOTORS];
    AP_Float _phase_pitch_deg[HAWK_NUM_MOTORS];
    AP_Float _yaw_bias[HAWK_NUM_MOTORS];

    bool _encoders_initialized;

    // debug state
    uint32_t _last_debug_ms;
    uint32_t _last_fault_ms;
    bool _sent_init_msg;
    bool _had_encoder_fault;
};