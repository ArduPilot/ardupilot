#include "AP_SurfaceDistance.h"

#include <AP_RangeFinder/AP_RangeFinder.h>

#ifndef RANGEFINDER_TIMEOUT_MS
 # define RANGEFINDER_TIMEOUT_MS 1000        // rangefinder filter reset if no updates from sensor in 1 second
#endif

#if AP_RANGEFINDER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

#ifndef RANGEFINDER_TILT_CORRECTION         // by disable tilt correction for use of range finder data by EKF
 # define RANGEFINDER_TILT_CORRECTION 1
#endif

#ifndef RANGEFINDER_GLITCH_NUM_SAMPLES
 # define RANGEFINDER_GLITCH_NUM_SAMPLES  3 // number of rangefinder glitches in a row to take new reading
#endif

#ifndef RANGEFINDER_GLITCH_ALT_CM
 # define RANGEFINDER_GLITCH_ALT_M 2.00     // amount of rangefinder change to be considered a glitch
#endif

#ifndef RANGEFINDER_HEALTH_MIN
 # define RANGEFINDER_HEALTH_MIN 3          // number of good reads that indicates a healthy rangefinder
#endif

void AP_SurfaceDistance::update()
{
    WITH_SEMAPHORE(sem);

    const RangeFinder *rangefinder = RangeFinder::get_singleton();
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

    // assemble bitmask assistance, definition is used to generate log documentation
    enum class Surface_Distance_Status : uint8_t {
        Enabled         = 1U<<0, // true if rangefinder has been set to enable by vehicle
        Unhealthy       = 1U<<1, // true if rangefinder is considered unhealthy
        Stale_Data      = 1U<<2, // true if the last healthy rangefinder reading is no longer valid
        Glitch_Detected = 1U<<3, // true if a measurement glitch detected
    };

    // reset status and add to the bitmask as we progress through the update
    status = 0;

    // update enabled status
    if (enabled) {
        status |= (uint8_t)Surface_Distance_Status::Enabled;
    }

    // update health
    alt_healthy = (rangefinder->status_orient(rotation) == RangeFinder::Status::Good) &&
                            (rangefinder->range_valid_count_orient(rotation) >= RANGEFINDER_HEALTH_MIN);
    if (!alt_healthy) {
        status |= (uint8_t)Surface_Distance_Status::Unhealthy;
    }

    // tilt corrected but unfiltered, not glitch protected alt
    alt_m = tilt_correction * rangefinder->distance_orient(rotation);

    // Glitch Handling. Rangefinder readings more than RANGEFINDER_GLITCH_ALT_M from the last good reading
    // are considered a glitch and glitch_count becomes non-zero
    // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
    // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
    const float glitch_m = alt_m - alt_glitch_protected_m;
    bool reset_terrain = false;
    if (glitch_m >= RANGEFINDER_GLITCH_ALT_M) {
        glitch_count = MAX(glitch_count+1, 1);
        status |= (uint8_t)Surface_Distance_Status::Glitch_Detected;
    } else if (glitch_m <= -RANGEFINDER_GLITCH_ALT_M) {
        glitch_count = MIN(glitch_count-1, -1);
        status |= (uint8_t)Surface_Distance_Status::Glitch_Detected;
    } else {
        glitch_count = 0;
        alt_glitch_protected_m = alt_m;
    }
    if (abs(glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
        glitch_count = 0;
        alt_glitch_protected_m = alt_m;
        glitch_cleared_ms = now;
        reset_terrain = true;
    }

    // filter rangefinder altitude
    const bool timed_out = now - last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
    if (alt_healthy) {
        if (timed_out) {
            // reset filter if we haven't used it within the last second
            alt_m_filt.reset(alt_m);
            reset_terrain = true;
            status |= (uint8_t)Surface_Distance_Status::Stale_Data;
        } else {
            // TODO: When we apply this library in plane we will need to be able to set the filter freq
            // we should be using the updated dt not 0.05
            alt_m_filt.apply(alt_m, 0.05);
        }
        last_healthy_ms = now;
    }

    // remember inertial alt to allow us to interpolate rangefinder
    float pos_d_m;
    if (AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {
        ref_pos_u_m = -pos_d_m;
    }

    // handle reset of terrain offset
    if (reset_terrain) {
        if (rotation == ROTATION_PITCH_90) {
            // upward facing
            terrain_u_m = ref_pos_u_m + alt_m;
        } else {
            // assume downward facing
            terrain_u_m = ref_pos_u_m - alt_m;
        }
    }
#if HAL_LOGGING_ENABLED
    Log_Write();
#endif
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool AP_SurfaceDistance::get_rangefinder_height_interpolated_m(float& height_m) const
{
    if (!enabled_and_healthy()) {
        return false;
    }

    // get inertial alt
    float pos_d_m;
    if (!AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {
        return false;
    }

    height_m = alt_m_filt.get();
    height_m += -pos_d_m - ref_pos_u_m;
    return true;
}

#if HAL_LOGGING_ENABLED
void AP_SurfaceDistance::Log_Write(void) const
{
    // @LoggerMessage: SURF
    // @Vehicles: Copter
    // @Description: Surface distance measurement
    // @Field: TimeUS: Time since system startup
    // @Field: I: Instance
    // @Field: St: Surface distance status
    // @FieldBitmaskEnum: St: Surface_Distance_Status
    // @Field: D: Raw Distance
    // @Field: FD: Filtered Distance
    // @Field: TO: Terrain Offset

    //Write to data flash log
    AP::logger().WriteStreaming("SURF",
                                "TimeUS,I,St,D,FD,TO",
                                "s#-mmm",
                                "F--000",
                                "QBBfff",
                                AP_HAL::micros64(),
                                instance,
                                status,
                                (float)alt_m,
                                (float)alt_m_filt.get(),
                                (float)terrain_u_m
                                );
}
#endif  // HAL_LOGGING_ENABLED

#endif // AP_RANGEFINDER_ENABLED

bool AP_SurfaceDistance::data_stale(void)
{
    WITH_SEMAPHORE(sem);
    return (AP_HAL::millis() - last_healthy_ms) > RANGEFINDER_TIMEOUT_MS;
}

bool AP_SurfaceDistance::enabled_and_healthy(void) const
{
    return enabled && alt_healthy;
}
