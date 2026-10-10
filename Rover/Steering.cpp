#include "Rover.h"

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // drive the per-servo failsafe positions (SERVOn_FSPWM) when any
    // failsafe selected by FS_SERVO_MASK is active, before the outputs are
    // written, including during a motor test. Radio and GCS failsafes count
    // once they have lasted FS_TIMEOUT, whatever the mode and whether or not
    // an action was taken
    const bool fs_timed_out = failsafe.bits != 0 &&
                              (millis() - failsafe.start_time) > uint32_t(g.fs_timeout * 1000);
    uint16_t fs_bits = 0;
    if (fs_timed_out && (failsafe.bits & FAILSAFE_EVENT_THROTTLE)) { fs_bits |= (1U<<0); }
    if (battery.has_failsafed())                                  { fs_bits |= (1U<<1); }
    if (fs_timed_out && (failsafe.bits & FAILSAFE_EVENT_GCS))      { fs_bits |= (1U<<2); }
    if (failsafe.ekf)                                              { fs_bits |= (1U<<3); }
    SRV_Channels::set_failsafe_active((fs_bits & uint16_t(g.fs_servo_mask.get())) != 0);

    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        // get ground speed
        float speed = 0.0f;
        g2.attitude_control.get_forward_speed(speed);

        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
