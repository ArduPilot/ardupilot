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

// Obstacle detection and floor tracking parameters
#ifndef OBSTACLE_DETECTION_ENABLED
 # define OBSTACLE_DETECTION_ENABLED 1      // enable obstacle detection (vs floor change discrimination)
#endif

#ifndef OBSTACLE_JUMP_THRESHOLD_M
 # define OBSTACLE_JUMP_THRESHOLD_M 0.8f    // altitude changes > 0.8m are considered potential obstacles
#endif

#ifndef OBSTACLE_HYSTERESIS_SAMPLES
 # define OBSTACLE_HYSTERESIS_SAMPLES 5     // require 5 consecutive samples to accept new floor height
#endif

#ifndef MAX_FLOOR_CHANGE_RATE_MS
 # define MAX_FLOOR_CHANGE_RATE_MS 0.3f     // maximum floor change rate: 0.3 m/s (normal walking speed)
#endif

#ifndef FLOOR_TRACKING_TAU
 # define FLOOR_TRACKING_TAU 0.1f           // time constant for smooth floor height tracking (10% per update)
#endif

#ifndef TILT_AGGRESSIVE_THRESHOLD
 # define TILT_AGGRESSIVE_THRESHOLD 0.866f  // cos(30°) - above 30° tilt, be more aggressive with obstacle detection
#endif

// Detect obstacles vs floor changes and track floor height smoothly
// This prevents altitude jumps when flying over objects (beds, tables, furniture)
// Takes tilt into account - higher tilt angles make obstacle detection more sensitive
// Returns true if current reading is likely an obstacle (should be filtered out)
static bool detect_obstacle_and_track_floor(float current_alt_m,
                                            float& floor_height_estimate_m,
                                            int8_t& obstacle_counter,
                                            uint32_t& last_floor_update_ms,
                                            uint32_t now_ms,
                                            float dt,
                                            float tilt_correction)
{
#if OBSTACLE_DETECTION_ENABLED
    // Adjust sensitivity based on tilt angle
    // Higher tilt = more likely to see obstacles early (like table edges when flying forward)
    // So reduce threshold and be more aggressive with obstacle detection
    float jump_threshold = OBSTACLE_JUMP_THRESHOLD_M;
    float max_floor_rate = MAX_FLOOR_CHANGE_RATE_MS;

    if (tilt_correction < TILT_AGGRESSIVE_THRESHOLD) {
        // Tilt > 30° - reduce threshold by 30% to catch obstacles earlier
        // Also reduce acceptable floor change rate
        jump_threshold *= 0.7f;      // 0.8m → 0.56m threshold
        max_floor_rate *= 0.7f;      // 0.3m/s → 0.21m/s max rate
    }

    // Calculate difference between current reading and tracked floor height
    const float delta_m = current_alt_m - floor_height_estimate_m;
    const bool potential_obstacle = (fabsf(delta_m) > jump_threshold);

    if (potential_obstacle) {
        // Check rate of change - real floor changes happen gradually
        const float time_since_update_s = (now_ms - last_floor_update_ms) * 0.001f;
        const float floor_change_rate_ms = (time_since_update_s > 0.01f) ?
                                           fabsf(delta_m) / time_since_update_s : 0.0f;

        // If change rate exceeds maximum expected floor gradient, likely an obstacle
        if (floor_change_rate_ms > max_floor_rate) {
            // Increment obstacle counter (positive = likely obstacle)
            obstacle_counter = constrain_int16(obstacle_counter + 1, -OBSTACLE_HYSTERESIS_SAMPLES, OBSTACLE_HYSTERESIS_SAMPLES);
        } else {
            // Slow change = legitimate floor change, decrement counter
            obstacle_counter = constrain_int16(obstacle_counter - 1, -OBSTACLE_HYSTERESIS_SAMPLES, OBSTACLE_HYSTERESIS_SAMPLES);
        }

        // Only accept new floor height after enough samples confirm it's not an obstacle
        if (abs(obstacle_counter) >= OBSTACLE_HYSTERESIS_SAMPLES) {
            // Confirmed: this is a new floor level, not a temporary obstacle
            floor_height_estimate_m = current_alt_m;
            obstacle_counter = 0;
            last_floor_update_ms = now_ms;
            return false;  // Not an obstacle, accept reading
        } else {
            // Still uncertain, treat as obstacle for now
            return true;  // Likely obstacle, filter it out
        }
    } else {
        // Small change = normal variation, track floor smoothly with low-pass filter
        obstacle_counter = 0;
        const float alpha = dt / (dt + FLOOR_TRACKING_TAU);
        floor_height_estimate_m += alpha * delta_m;
        last_floor_update_ms = now_ms;
        return false;  // Accept reading
    }
#else
    // Obstacle detection disabled, always accept readings
    floor_height_estimate_m = current_alt_m;
    last_floor_update_ms = now_ms;
    obstacle_counter = 0;
    return false;
#endif
}

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

    // Obstacle detection: discriminate between obstacles (furniture, etc.) and real floor changes
    // Initialize floor height estimate on first valid reading
    if (last_floor_update_ms == 0 && alt_healthy) {
        floor_height_estimate_m = alt_m;
        last_floor_update_ms = now;
        obstacle_counter = 0;
    }

    // Detect obstacles vs floor changes (tilt-aware)
    const bool is_obstacle = detect_obstacle_and_track_floor(
        alt_m,
        floor_height_estimate_m,
        obstacle_counter,
        last_floor_update_ms,
        now,
        0.05f,            // dt = 0.05s (20Hz update rate assumed)
        tilt_correction   // Pass tilt for adaptive thresholds
    );

    // If obstacle detected, use tracked floor height to prevent altitude jumps
    if (is_obstacle && alt_healthy) {
        alt_m = floor_height_estimate_m;
        alt_glitch_protected_m = floor_height_estimate_m;
        // Don't reset terrain - we're intentionally ignoring this obstacle
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
    // @FieldBitmaskEnum: St: Surface_Distance_Status
    // @Field: D: Raw Distance
    // @Field: FD: Filtered Distance
    // @Field: TO: Terrain Offset
    // @Field: FH: Floor Height Estimate (obstacle detection)
    // @Field: OC: Obstacle Counter (positive = likely obstacle)

    //Write to data flash log
    AP::logger().WriteStreaming("SURF",
                                "TimeUS,I,St,D,FD,TO,FH,OC",
                                "s#-mmmm-",
                                "F--0000-",
                                "QBBffffb",
                                AP_HAL::micros64(),
                                instance,
                                status,
                                (float)alt_m,
                                (float)alt_m_filt.get(),
                                (float)terrain_u_m,
                                (float)floor_height_estimate_m,
                                (int8_t)obstacle_counter
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
