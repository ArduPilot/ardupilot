#include "AC_SurfaceDistance.h"

#include <AP_RangeFinder/AP_RangeFinder.h>

#if AP_RANGEFINDER_ENABLED

#include <AP_AHRS/AP_AHRS.h>

#ifndef RANGEFINDER_TILT_CORRECTION         // by disable tilt correction for use of range finder data by EKF
 # define RANGEFINDER_TILT_CORRECTION 1
#endif

#ifndef RANGEFINDER_GLITCH_NUM_SAMPLES
 # define RANGEFINDER_GLITCH_NUM_SAMPLES  3   // number of rangefinder glitches in a row to take new reading
#endif

#ifndef RANGEFINDER_GLITCH_ALT_CM
 # define RANGEFINDER_GLITCH_ALT_CM  200      // amount of rangefinder change to be considered a glitch
#endif

#ifndef RANGEFINDER_HEALTH_MAX
 # define RANGEFINDER_HEALTH_MAX 3          // number of good reads that indicates a healthy rangefinder
#endif

#ifndef RANGEFINDER_TIMEOUT_MS
 # define RANGEFINDER_TIMEOUT_MS 1000        // rangefinder filter reset if no updates from sensor in 1 second
#endif

void AC_SurfaceDistance::update()
{
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        alt_healthy = false;
        return;
    }

#if RANGEFINDER_TILT_CORRECTION == 1
    const float tilt_correction = MAX(0.707f, AP::ahrs().get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    const uint32_t now = AP_HAL::millis();

    // update health
    alt_healthy = ((rangefinder->status_orient(rotation) == RangeFinder::Status::Good) &&
                            (rangefinder->range_valid_count_orient(rotation) >= RANGEFINDER_HEALTH_MAX));

    // tilt corrected but unfiltered, not glitch protected alt
    alt_cm = tilt_correction * rangefinder->distance_cm_orient(rotation);

    // remember inertial alt to allow us to interpolate rangefinder
    inertial_alt_cm = inertial_nav.get_position_z_up_cm();

    // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
    // are considered a glitch and glitch_count becomes non-zero
    // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
    // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
    const int32_t glitch_cm = alt_cm - alt_cm_glitch_protected;
    if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
        glitch_count = MAX(glitch_count+1, 1);
    } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
        glitch_count = MIN(glitch_count-1, -1);
    } else {
        glitch_count = 0;
        alt_cm_glitch_protected = alt_cm;
    }
    if (abs(glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
        glitch_count = 0;
        alt_cm_glitch_protected = alt_cm;
        glitch_cleared_ms = now;
    }

    // filter rangefinder altitude
    const bool timed_out = now - last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
    if (alt_healthy) {
        if (timed_out) {
            // reset filter if we haven't used it within the last second
            alt_cm_filt.reset(alt_cm);
        } else {
            alt_cm_filt.apply(alt_cm, 0.05f);
        }
        last_healthy_ms = now;
    }

}

#endif // AP_RANGEFINDER_ENABLED
