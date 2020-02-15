#include "Rover.h"

// start cruise throttle and speed learning
void Rover::cruise_learn_start()
{
    // if disarmed or no speed available do nothing
    float speed;
    if (!arming.is_armed() || !g2.attitude_control.get_forward_speed(speed)) {
        cruise_learn.learn_start_ms = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learning NOT started");
        return;
    }
    // start learning
    cruise_learn.speed_filt.reset(speed);
    cruise_learn.throttle_filt.reset(g2.motors.get_throttle());
    cruise_learn.learn_start_ms = AP_HAL::millis();
    cruise_learn.log_count = 0;
    log_write_cruise_learn();
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learning started");
}

// update cruise learning with latest speed and throttle
// should be called at 50hz
void Rover::cruise_learn_update()
{
    float speed;
    if (cruise_learn.learn_start_ms > 0 && g2.attitude_control.get_forward_speed(speed)) {
        // update filters with latest speed and throttle
        cruise_learn.speed_filt.apply(speed, 0.02f);
        cruise_learn.throttle_filt.apply(g2.motors.get_throttle(), 0.02f);
        // 10Hz logging
        if (cruise_learn.log_count % 5 == 0) {
            log_write_cruise_learn();
        }
        cruise_learn.log_count += 1;
        // check how long it took to learn
        if (AP_HAL::millis() - cruise_learn.learn_start_ms >= 2000) {
            cruise_learn_complete();
        }
        return;
    }
}

// complete cruise learning and save results
void Rover::cruise_learn_complete()
{
    // when switch is moved low, save learned cruise value
    if (cruise_learn.learn_start_ms > 0) {
        const float thr = cruise_learn.throttle_filt.get();
        const float speed = cruise_learn.speed_filt.get();
        if (thr >= 10.0f && thr <= 100.0f && is_positive(speed)) {
            g.throttle_cruise.set_and_save(thr);
            g.speed_cruise.set_and_save(speed);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learned: Thr:%d Speed:%3.1f", (int)g.throttle_cruise, (double)g.speed_cruise);
        } else {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cruise Learning failed");
        }
        cruise_learn.learn_start_ms = 0;
        log_write_cruise_learn();
    }
}

// logging for cruise learn
void Rover::log_write_cruise_learn()
{
    AP::logger().Write("CRSE", "TimeUS,State,Speed,Throttle", "Qbff",
                       AP_HAL::micros64(),
                       cruise_learn.learn_start_ms > 0,
                       cruise_learn.speed_filt.get(),
                       cruise_learn.throttle_filt.get());
}
