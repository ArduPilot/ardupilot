#include "Rover.h"

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        // get ground speed
        float speed;
        if (!g2.attitude_control.get_forward_speed(speed)) {
            speed = 0.0f;
        }

        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
