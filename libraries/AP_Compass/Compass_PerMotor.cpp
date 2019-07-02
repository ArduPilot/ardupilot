/*
  per-motor compass compensation
 */

#include "AP_Compass.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Compass_PerMotor::var_info[] = {
    // @Param: _EN
    // @DisplayName: per-motor compass correction enable
    // @Description: This enables per-motor compass corrections
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_EN",  1, Compass_PerMotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _EXP
    // @DisplayName: per-motor exponential correction
    // @Description: This is the exponential correction for the power output of the motor for per-motor compass correction
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_EXP", 2, Compass_PerMotor, expo, 0.65),

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
    
    AP_GROUPEND
};

// constructor
Compass_PerMotor::Compass_PerMotor(Compass &_compass) :
    compass(_compass)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// return current scaled motor output
float Compass_PerMotor::scaled_output(uint8_t motor)
{
    if (!have_motor_map) {
        if (SRV_Channels::find_channel(SRV_Channel::k_motor1, motor_map[0]) &&
            SRV_Channels::find_channel(SRV_Channel::k_motor2, motor_map[1]) &&
            SRV_Channels::find_channel(SRV_Channel::k_motor3, motor_map[2]) &&
            SRV_Channels::find_channel(SRV_Channel::k_motor4, motor_map[3])) {
            have_motor_map = true;
        }
    }
    if (!have_motor_map) {
        return 0;
    }
    
    // this currently assumes first 4 channels. 
    uint16_t pwm = hal.rcout->read_last_sent(motor_map[motor]);

    // get 0 to 1 motor demand
    float output = (hal.rcout->scale_esc_to_unity(pwm)+1) * 0.5f;

    if (output <= 0) {
        return 0;
    }
    
    // scale for voltage
    output *= voltage;

    // apply expo correction
    output = powf(output, expo);
    return output;
}

// per-motor calibration update
void Compass_PerMotor::calibration_start(void)
{
    for (uint8_t i=0; i<4; i++) {
        field_sum[i].zero();
        output_sum[i] = 0;
        count[i] = 0;
        start_ms[i] = 0;
    }

    // we need to ensure we get current data by throwing away several
    // samples. The offsets may have just changed from an offset
    // calibration
    for (uint8_t i=0; i<4; i++) {
        compass.read();
        hal.scheduler->delay(50);
    }
    
    base_field = compass.get_field(0);
    running = true;
}

// per-motor calibration update
void Compass_PerMotor::calibration_update(void)
{
    uint32_t now = AP_HAL::millis();
    
    // accumulate per-motor sums
    for (uint8_t i=0; i<4; i++) {
        float output = scaled_output(i);

        if (output <= 0) {
            // motor is off
            start_ms[i] = 0;
            continue;
        }
        if (start_ms[i] == 0) {
            start_ms[i] = now;
        }
        if (now - start_ms[i] < 500) {
            // motor must run for 0.5s to settle
            continue;
        }

        // accumulate a sample
        field_sum[i] += compass.get_field(0);
        output_sum[i] += output;
        count[i]++;
    }
}

// calculate per-motor calibration values
void Compass_PerMotor::calibration_end(void)
{
    for (uint8_t i=0; i<4; i++) {
        if (count[i] == 0) {
            continue;
        }

        // calculate effective output
        float output = output_sum[i] / count[i];

        // calculate amount that field changed from base field
        Vector3f field_change = base_field - (field_sum[i] / count[i]);
        if (output <= 0) {
            continue;
        }
        
        Vector3f c = field_change / output;
        compensation[i].set_and_save(c);
    }

    // enable per-motor compensation
    enable.set_and_save(1);
    
    running = false;
}

/*
  calculate total offset for per-motor compensation
 */
void Compass_PerMotor::compensate(Vector3f &offset)
{
    offset.zero();

    if (running) {
        // don't compensate while calibrating
        return;
    }

    for (uint8_t i=0; i<4; i++) {
        float output = scaled_output(i);

        const Vector3f &c = compensation[i].get();

        offset += c * output;
    }
}
