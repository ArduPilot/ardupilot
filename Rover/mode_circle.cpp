#include "Rover.h"

bool ModeCircle::_enter()
{
    ahrs.get_location(center);

    steering = 200.0f;
    throttle = 20.0f;

    gcs().send_text(MAV_SEVERITY_INFO, "Entering Mode Circle");

    return true;
}

void ModeCircle::update()
{
    Location current_location;
    ahrs.get_location(current_location);

    float dist = center.get_distance(current_location);
    float offset = (last_dist - dist) / radius;
    last_dist = dist;
    float val = abs(dist/radius);

    if (offset > 0) {
        // inside the circle -> decrease the steering
        steering = MAX(steering - val * M, 0);
        g2.motors.set_steering(steering);
        g2.motors.set_throttle(throttle);
    } else {
        // outside the circle -> increase the steering
        steering = MIN(steering + val * M, K);
        g2.motors.set_steering(steering);
        g2.motors.set_throttle(throttle);
    }
}

void ModeCircle::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Exiting Mode Circle");
}
