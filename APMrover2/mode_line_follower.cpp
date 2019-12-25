#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

void ModeLineFollower::update()
{
    float desired_steering, desired_throttle,steering_out,ir_reading;

    get_pilot_input(desired_steering, desired_throttle);
    g2.motors.set_throttle(desired_throttle);
    g2.ir_sensor.read();
    g2.ir_sensor.calc_line_error(ir_reading); //read IR sensor values

    const float target_turn_rate = ir_reading* radians(g2.acro_turn_rate);
    steering_out = attitude_control.get_steering_out_rate(target_turn_rate, g2.motors.limit.steer_left, g2.motors.limit.steer_right, rover.G_Dt);
    set_steering(steering_out * 4500.0f);
}
