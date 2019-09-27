#include "Sub.h"

// stabilize_init - initialise stabilize controller
bool Sub::stabilize_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);
    last_roll = ahrs.roll_sensor;
    last_pitch = ahrs.pitch_sensor;
    last_yaw = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::stabilize_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        last_roll = ahrs.roll_sensor;
        last_pitch = ahrs.pitch_sensor;
        last_yaw = ahrs.yaw_sensor;
        return;
    }
    // Vehicle is armed, motors are free to run
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    handle_attitude();

    // output pilot's throttle
    attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
