#include "Plane.h"

/*
  is_flying and crash detection logic
 */

#define CRASH_DETECTION_DELAY_MS            500
#define IS_FLYING_IMPACT_TIMER_MS           3000
#define GPS_IS_FLYING_SPEED_CMS             150

/*
  Do we think we are flying?
  Probabilistic method where a bool is low-passed and considered a probability.
*/
void Plane::update_is_flying_5Hz(void)
{
    float aspeed;
    bool is_flying_bool;
    uint32_t now_ms = AP_HAL::millis();

    uint32_t ground_speed_thresh_cm = (aparm.min_gndspeed_cm > 0) ? ((uint32_t)(aparm.min_gndspeed_cm*0.9f)) : GPS_IS_FLYING_SPEED_CMS;
    bool gps_confirmed_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_3D) &&
                                    (gps.ground_speed_cm() >= ground_speed_thresh_cm);

    // airspeed at least 75% of stall speed?
    bool airspeed_movement = ahrs.airspeed_estimate(&aspeed) && (aspeed >= (aparm.airspeed_min*0.75f));


    if (quadplane.is_flying()) {
        is_flying_bool = true;

    } else if(arming.is_armed()) {
        // when armed assuming flying and we need overwhelming evidence that we ARE NOT flying
        // short drop-outs of GPS are common during flight due to banking which points the antenna in different directions
        bool gps_lost_recently = (gps.last_fix_time_ms() > 0) && // we have locked to GPS before
                        (gps.status() < AP_GPS::GPS_OK_FIX_2D) && // and it's lost now
                        (now_ms - gps.last_fix_time_ms() < 5000); // but it wasn't that long ago (<5s)

        if ((auto_state.last_flying_ms > 0) && gps_lost_recently) {
            // we've flown before, remove GPS constraints temporarily and only use airspeed
            is_flying_bool = airspeed_movement; // moving through the air
        } else {
            // we've never flown yet, require good GPS movement
            is_flying_bool = airspeed_movement || // moving through the air
                                gps_confirmed_movement; // locked and we're moving
        }

        if (control_mode == AUTO) {
            /*
              make is_flying() more accurate during various auto modes
             */

            // Detect X-axis deceleration for probable ground impacts.
            // Limit the max probability so it can decay faster. This
            // will not change the is_flying state, anything above 0.1
            // is "true", it just allows it to decay faster once we decide we
            // aren't flying using the normal schemes
            if (g.crash_accel_threshold == 0) {
                crash_state.impact_detected = false;
            } else if (ins.get_accel_peak_hold_neg_x() < -(g.crash_accel_threshold)) {
                // large deceleration detected, lets lower confidence VERY quickly
                crash_state.impact_detected = true;
                crash_state.impact_timer_ms = now_ms;
                if (isFlyingProbability > 0.2f) {
                    isFlyingProbability = 0.2f;
                }
            } else if (crash_state.impact_detected &&
                (now_ms - crash_state.impact_timer_ms > IS_FLYING_IMPACT_TIMER_MS)) {
                // no impacts seen in a while, clear the flag so we stop clipping isFlyingProbability
                crash_state.impact_detected = false;
            }

            switch (flight_stage)
            {
            case AP_Vehicle::FixedWing::FLIGHT_TAKEOFF:
                break;

            case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
                if (in_preLaunch_flight_stage()) {
                    // while on the ground, an uncalibrated airspeed sensor can drift to 7m/s so
                    // ensure we aren't showing a false positive.
                    is_flying_bool = false;
                    crash_state.is_crashed = false;
                    auto_state.started_flying_in_auto_ms = 0;
                }
                break;

            case AP_Vehicle::FixedWing::FLIGHT_VTOL:
                // TODO: detect ground impacts
                break;

            case AP_Vehicle::FixedWing::FLIGHT_LAND:
                switch (landing.get_stage()) {
                case AP_Landing::STAGE_APPROACH:
                    if (fabsf(auto_state.sink_rate) > 0.2f) {
                        is_flying_bool = true;
                    }
                    break;

                case AP_Landing::STAGE_UNKNOWN:
                case AP_Landing::STAGE_PREFLARE:
                case AP_Landing::STAGE_FINAL:
                    break;
                }
                break;

            case AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND:
                if (auto_state.sink_rate < -0.5f) {
                    // steep climb
                    is_flying_bool = true;
                }
                break;

            default:
                break;
            } // switch
        }
    } else {
        // when disarmed assume not flying and need overwhelming evidence that we ARE flying
        is_flying_bool = airspeed_movement && gps_confirmed_movement;

        if ((control_mode == AUTO) &&
            ((flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF) ||
             (landing.get_stage() == AP_Landing::STAGE_FINAL)) ) {
            is_flying_bool = false;
        }
    }

    if (!crash_state.impact_detected || !is_flying_bool) {
        // when impact is detected, enforce a clip. Only allow isFlyingProbability to go down, not up.
        // low-pass the result.
        // coef=0.15f @ 5Hz takes 3.0s to go from 100% down to 10% (or 0% up to 90%)
        isFlyingProbability = (0.85f * isFlyingProbability) + (0.15f * (float)is_flying_bool);
    }

    /*
      update last_flying_ms so we always know how long we have not
      been flying for. This helps for crash detection and auto-disarm
     */
    bool new_is_flying = is_flying();

    // we are flying, note the time
    if (new_is_flying) {

        auto_state.last_flying_ms = now_ms;

        if (!previous_is_flying) {
            // just started flying in any mode
            started_flying_ms = now_ms;
        }

        if ((control_mode == AUTO) &&
            ((auto_state.started_flying_in_auto_ms == 0) || !previous_is_flying) ) {

            // We just started flying, note that time also
            auto_state.started_flying_in_auto_ms = now_ms;
        }
    }
    previous_is_flying = new_is_flying;
    adsb.set_is_flying(new_is_flying);
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.set_is_flying(new_is_flying);
#endif
    g2.stats.set_flying(new_is_flying);

    crash_detection_update();

    if (should_log(MASK_LOG_MODE)) {
        Log_Write_Status();
    }
}

/*
  return true if we think we are flying. This is a probabilistic
  estimate, and needs to be used very carefully. Each use case needs
  to be thought about individually.
 */
bool Plane::is_flying(void)
{
    if (hal.util->get_soft_armed()) {
        if (quadplane.is_flying_vtol()) {
            return true;
        }
        // when armed, assume we're flying unless we probably aren't
        return (isFlyingProbability >= 0.1f);
    }

    // when disarmed, assume we're not flying unless we probably are
    return (isFlyingProbability >= 0.9f);
}

/*
 * Determine if we have crashed
 */
void Plane::crash_detection_update(void)
{
    if (control_mode != AUTO || !aparm.crash_detection_enable)
    {
        // crash detection is only available in AUTO mode
        crash_state.debounce_timer_ms = 0;
        crash_state.is_crashed = false;
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    bool crashed_near_land_waypoint = false;
    bool crashed = false;
    bool been_auto_flying = (auto_state.started_flying_in_auto_ms > 0) &&
                            (now_ms - auto_state.started_flying_in_auto_ms >= 2500);

    if (!is_flying() && arming.is_armed())
    {
        switch (flight_stage)
        {
        case AP_Vehicle::FixedWing::FLIGHT_TAKEOFF:
            if (g.takeoff_throttle_min_accel > 0 &&
                    !throttle_suppressed) {
                // if you have an acceleration holding back throttle, but you met the
                // accel threshold but still not fying, then you either shook/hit the
                // plane or it was a failed launch.
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
            }
            // TODO: handle auto missions without NAV_TAKEOFF mission cmd
            break;

        case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
            if (!in_preLaunch_flight_stage() && been_auto_flying) {
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
            }
            break;

        case AP_Vehicle::FixedWing::FLIGHT_VTOL:
            // we need a totally new method for this
            crashed = false;
            break;
            
        case AP_Vehicle::FixedWing::FLIGHT_LAND:
            switch (landing.get_stage())
            {
            case AP_Landing::STAGE_APPROACH:
                if (been_auto_flying) {
                    crashed = true;
                    crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
                }
                // when altitude gets low, we automatically progress to AP_Landing::STAGE_FINAL
                // so ground crashes most likely can not be triggered from here. However,
                // a crash into a tree would be caught here.
                break;

            case AP_Landing::STAGE_PREFLARE:
            case AP_Landing::STAGE_FINAL:
                // We should be nice and level-ish in this flight stage. If not, we most
                // likely had a crazy landing. Throttle is inhibited already at the flare
                // but go ahead and notify GCS and perform any additional post-crash actions.
                // Declare a crash if we are oriented more that 60deg in pitch or roll
                if (!crash_state.checkedHardLanding && // only check once
                    been_auto_flying &&
                    (labs(ahrs.roll_sensor) > 6000 || labs(ahrs.pitch_sensor) > 6000)) {
                    crashed = true;
                    crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;

                    // did we "crash" within 75m of the landing location? Probably just a hard landing
                    crashed_near_land_waypoint =
                            get_distance(current_loc, mission.get_current_nav_cmd().content.location) < 75;

                    // trigger hard landing event right away, or never again. This inhibits a false hard landing
                    // event when, for example, a minute after a good landing you pick the plane up and
                    // this logic is still running and detects the plane is on its side as you carry it.
                    crash_state.debounce_timer_ms = now_ms + CRASH_DETECTION_DELAY_MS;
                }

                crash_state.checkedHardLanding = true;
                break;

            case AP_Landing::STAGE_UNKNOWN:
                break;
            } // switch landing.get_stage()

        default:
            break;
        } // switch
    } else {
        crash_state.checkedHardLanding = false;
    }

    if (!crashed) {
        // reset timer
        crash_state.debounce_timer_ms = 0;

    } else if (crash_state.debounce_timer_ms == 0) {
        // start timer
        crash_state.debounce_timer_ms = now_ms;

    } else if ((now_ms - crash_state.debounce_timer_ms >= crash_state.debounce_time_total_ms) && !crash_state.is_crashed) {
        crash_state.is_crashed = true;

        if (aparm.crash_detection_enable == CRASH_DETECT_ACTION_BITMASK_DISABLED) {
            if (crashed_near_land_waypoint) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "Hard landing detected. No action taken");
            } else {
                gcs_send_text(MAV_SEVERITY_EMERGENCY, "Crash detected. No action taken");
            }
        }
        else {
            if (aparm.crash_detection_enable & CRASH_DETECT_ACTION_BITMASK_DISARM) {
                disarm_motors();
            }
            landing.complete = true;
            if (crashed_near_land_waypoint) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "Hard landing detected");
            } else {
                gcs_send_text(MAV_SEVERITY_EMERGENCY, "Crash detected");
            }
        }
    }
}

/*
 * return true if we are in a pre-launch phase of an auto-launch, typically used in bungee launches
 */
bool Plane::in_preLaunch_flight_stage(void) {
    return (control_mode == AUTO &&
            throttle_suppressed &&
            flight_stage == AP_Vehicle::FixedWing::FLIGHT_NORMAL &&
            mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF);
}


