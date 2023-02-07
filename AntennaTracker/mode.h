#pragma once

#include <stdint.h>
#include <AP_Math/AP_Math.h>

class Mode {
public:
    enum class Number {
        MANUAL=0,
        STOP=1,
        SCAN=2,
        SERVOTEST=3,
        GUIDED=4,
        AUTO=10,
        INITIALISING=16
    };

    Mode() {}

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // returns a unique number specific to this mode
    virtual Mode::Number number() const = 0;

    virtual bool requires_armed_servos() const = 0;

    virtual void update() = 0;

protected:
    void update_scan();
    void update_auto();

    bool get_ef_yaw_direction();

    void calc_angle_error(float pitch, float yaw, bool direction_reversed);
    void convert_ef_to_bf(float pitch, float yaw, float& bf_pitch, float& bf_yaw);
    bool convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw);
};

class ModeAuto : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::AUTO; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

class ModeGuided : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::GUIDED; }
    bool requires_armed_servos() const override { return true; }
    void update() override;

    void set_angle(const Quaternion &target_att, bool use_yaw_rate, float yaw_rate_rads) {
        _target_att = target_att;
        _use_yaw_rate = use_yaw_rate;
        _yaw_rate_rads = yaw_rate_rads;
    }

private:
    Quaternion _target_att;
    bool _use_yaw_rate;
    float _yaw_rate_rads;
};

class ModeInitialising : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::INITIALISING; }
    bool requires_armed_servos() const override { return false; }
    void update() override {};
};

class ModeManual : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::MANUAL; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

class ModeScan : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::SCAN; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

class ModeServoTest : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::SERVOTEST; }
    bool requires_armed_servos() const override { return true; }
    void update() override {};

    bool set_servo(uint8_t servo_num, uint16_t pwm);
};

class ModeStop : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::STOP; }
    bool requires_armed_servos() const override { return false; }
    void update() override {};
};
