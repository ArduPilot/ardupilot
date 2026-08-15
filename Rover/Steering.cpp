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
        float speed = 0.0f;
        g2.attitude_control.get_forward_speed(speed);

        // Apply extra control after the active mode has updated its commands,
        // immediately before the motor mixer generates the servo outputs.
        extra_controller.apply();

        g2.motors.output(arming.is_armed() || extra_controller_armed,
                         speed, G_Dt);
    }
}
