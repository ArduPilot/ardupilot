// Jacob Walser: jacob@bluerobotics.com

#include "Sub.h"

// counter to verify contact with bottom
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;

// checks if we have hit bottom or surface and updates the ap.at_bottom and ap.at_surface flags
// called at MAIN_LOOP_RATE
// ToDo: doesn't need to be called this fast
void Sub::update_surface_and_bottom_detector()
{
    if (!motors.armed()) { // only update when armed
        set_surfaced(false);
        set_bottomed(false);
        return;
    }

    Vector3f velocity;
    UNUSED_RESULT(ahrs.get_velocity_NED(velocity));

    // check that we are not moving up or down
    bool vel_stationary = velocity.z > -0.05 && velocity.z < 0.05;

    if (ap.depth_sensor_present && sensor_health.depth) { // we can use the external pressure sensor for a very accurate and current measure of our z axis position
        current_depth = barometer.get_altitude(); // cm


        if (ap.at_surface) {
            set_surfaced(current_depth > g.surface_depth/100.0 - 0.05); // add a 5cm buffer so it doesn't trigger too often
        } else {
            set_surfaced(current_depth > g.surface_depth/100.0); // If we are above surface depth, we are surfaced
        }


        if (motors.limit.throttle_lower && vel_stationary) {
            // bottom criteria met - increment the counter and check if we've triggered
            if (bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                bottom_detector_count++;
            } else {
                set_bottomed(true);
            }

        } else {
            set_bottomed(false);
        }

        // with no external baro, the only thing we have to go by is a vertical velocity estimate
    } else if (vel_stationary) {
        if (motors.limit.throttle_upper) {

            // surface criteria met, increment counter and see if we've triggered
            if (surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                surface_detector_count++;
            } else {
                set_surfaced(true);
            }

        } else if (motors.limit.throttle_lower) {
            // bottom criteria met, increment counter and see if we've triggered
            if (bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                bottom_detector_count++;
            } else {
                set_bottomed(true);
            }

        } else { // we're not at the limits of throttle, so reset both detectors
            set_surfaced(false);
            set_bottomed(false);
        }

    } else { // we're moving up or down, so reset both detectors
        set_surfaced(false);
        set_bottomed(false);
    }
}

void Sub::set_surfaced(bool at_surface)
{


    if (ap.at_surface == at_surface) { // do nothing if state unchanged
        return;
    }

    ap.at_surface = at_surface;

    surface_detector_count = 0;

    if (ap.at_surface) {
        LOGGER_WRITE_EVENT(LogEvent::SURFACED);
    } else {
        LOGGER_WRITE_EVENT(LogEvent::NOT_SURFACED);
    }
}

void Sub::set_bottomed(bool at_bottom)
{

    if (ap.at_bottom == at_bottom) { // do nothing if state unchanged
        return;
    }

    ap.at_bottom = at_bottom;

    bottom_detector_count = 0;

    if (ap.at_bottom) {
        LOGGER_WRITE_EVENT(LogEvent::BOTTOMED);
    } else {
        LOGGER_WRITE_EVENT(LogEvent::NOT_BOTTOMED);
    }
}
