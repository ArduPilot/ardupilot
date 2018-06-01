#include "Rover.h"

// start cruise throttle and speed learning
void Rover::cruise_learn_start()
{
    // if disarmed or no speed available do nothing
    float speed;
    if (!arming.is_armed() || !g2.attitude_control.get_forward_speed(speed)) {
        cruise_learn.learning = false;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learning NOT started");
        return;
    }
    // start learning
    cruise_learn.learning = true;
    cruise_learn.speed_filt.reset(speed);
    cruise_learn.throttle_filt.reset(g2.motors.get_throttle());
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learning started");
    // To-Do: add dataflash logging of learning started event
}

// update cruise learning with latest speed and throttle
// should be called at 50hz
void Rover::cruise_learn_update()
{
    float speed;
    if (cruise_learn.learning && g2.attitude_control.get_forward_speed(speed)) {
        // update filters with latest speed and throttle
        cruise_learn.speed_filt.apply(speed, 0.02f);
        cruise_learn.throttle_filt.apply(g2.motors.get_throttle(), 0.02f);
        return;
    }
}

// complete cruise learning and save results
void Rover::cruise_learn_complete()
{
    // when switch is moved low, save learned cruise value
    if (cruise_learn.learning) {
        const float thr = cruise_learn.throttle_filt.get();
        const float speed = cruise_learn.speed_filt.get();
        if (thr >= 10.0f && thr <= 100.0f && is_positive(speed)) {
            g.throttle_cruise.set_and_save(thr);
            g.speed_cruise.set_and_save(speed);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learned: Thr:%d Speed:%3.1f", (int)g.throttle_cruise, (double)g.speed_cruise);
        } else {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learning failed");
        }
        cruise_learn.learning = false;
        // To-Do: add dataflash logging of learning completion event
    }
}
