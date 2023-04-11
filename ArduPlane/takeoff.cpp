#include "Plane.h"

/*   Check for automatic takeoff conditions being met using the following sequence:
 *   1) Check for adequate GPS lock - if not return false
 *   2) Check the gravity compensated longitudinal acceleration against the threshold and start the timer if true
 *   3) Wait until the timer has reached the specified value (increments of 0.1 sec) and then check the GPS speed against the threshold
 *   4) If the GPS speed is above the threshold and the attitude is within limits then return true and reset the timer
 *   5) If the GPS speed and attitude within limits has not been achieved after 2.5 seconds, return false and reset the timer
 *   6) If the time lapsed since the last timecheck is greater than 0.2 seconds, return false and reset the timer
 *   NOTE : This function relies on the TECS 50Hz processing for its acceleration measure.
 */
bool Plane::auto_takeoff_check(void)
{
    // this is a more advanced check that relies on TECS
    uint32_t now = millis();
    uint16_t wait_time_ms = MIN(uint16_t(g.takeoff_throttle_delay)*100,12700);

    // reset all takeoff state if disarmed
    if (!arming.is_armed_and_safety_off()) {
        memset(&takeoff_state, 0, sizeof(takeoff_state));
        auto_state.baro_takeoff_alt = barometer.get_altitude();
        return false;
    }

    // Reset states if process has been interrupted
    if (takeoff_state.last_check_ms && (now - takeoff_state.last_check_ms) > 200) {
        memset(&takeoff_state, 0, sizeof(takeoff_state));
        return false;
    }

    takeoff_state.last_check_ms = now;
    
    //check if waiting for rudder neutral after rudder arm
    if (plane.arming.last_arm_method() == AP_Arming::Method::RUDDER &&
        !seen_neutral_rudder) {
        // we were armed with rudder but have not seen rudder neutral yet
        takeoff_state.waiting_for_rudder_neutral = true;
        // warn if we have been waiting a long time
        if (now - takeoff_state.rudder_takeoff_warn_ms > TAKEOFF_RUDDER_WARNING_TIMEOUT) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff waiting for rudder release");
            takeoff_state.rudder_takeoff_warn_ms = now;
        }
        // since we are still waiting, dont takeoff
        return false;
    } else {
       // we did not arm by rudder or rudder has returned to neutral
       // make sure we dont indicate we are in the waiting state with servo position indicator
       takeoff_state.waiting_for_rudder_neutral = false;
    }  

    // Check for bad GPS
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        // no auto takeoff without GPS lock
        return false;
    }

    bool do_takeoff_attitude_check = !(g2.flight_options & FlightOptions::DISABLE_TOFF_ATTITUDE_CHK);
#if HAL_QUADPLANE_ENABLED
    // disable attitude check on tailsitters
    do_takeoff_attitude_check = !quadplane.tailsitter.enabled();
#endif

    if (!takeoff_state.launchTimerStarted && !is_zero(g.takeoff_throttle_min_accel)) {
        // we are requiring an X acceleration event to launch
        float xaccel = TECS_controller.get_VXdot();
        if (g2.takeoff_throttle_accel_count <= 1) {
            if (xaccel < g.takeoff_throttle_min_accel) {
                goto no_launch;
            }
        } else {
            // we need multiple accel events
            if (now - takeoff_state.accel_event_ms > 500) {
                takeoff_state.accel_event_counter = 0;
            }
            bool odd_event = ((takeoff_state.accel_event_counter & 1) != 0);
            bool got_event = (odd_event?xaccel < -g.takeoff_throttle_min_accel : xaccel > g.takeoff_throttle_min_accel);
            if (got_event) {
                takeoff_state.accel_event_counter++;
                takeoff_state.accel_event_ms = now;
            }
            if (takeoff_state.accel_event_counter < g2.takeoff_throttle_accel_count) {
                goto no_launch;
            }
        }
    }

    // we've reached the acceleration threshold, so start the timer
    if (!takeoff_state.launchTimerStarted) {
        takeoff_state.launchTimerStarted = true;
        takeoff_state.last_tkoff_arm_time = now;
        if (now - takeoff_state.last_report_ms > 2000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Armed AUTO, xaccel = %.1f m/s/s, waiting %.1f sec",
                              (double)TECS_controller.get_VXdot(), (double)(wait_time_ms*0.001f));
            takeoff_state.last_report_ms = now;
        }
    }

    // Only perform velocity check if not timed out
    if ((now - takeoff_state.last_tkoff_arm_time) > wait_time_ms+100U) {
        if (now - takeoff_state.last_report_ms > 2000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Timeout AUTO");
            takeoff_state.last_report_ms = now;
        }
        goto no_launch;
    }

    if (do_takeoff_attitude_check) {
        // Check aircraft attitude for bad launch
        if (ahrs.pitch_sensor <= -3000 || ahrs.pitch_sensor >= 4500 ||
            (!fly_inverted() && labs(ahrs.roll_sensor) > 3000)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Bad launch AUTO");
            takeoff_state.accel_event_counter = 0;
            goto no_launch;
        }
    }

    // Check ground speed and time delay
    if (((gps.ground_speed() > g.takeoff_throttle_min_speed || is_zero(g.takeoff_throttle_min_speed))) &&
        ((now - takeoff_state.last_tkoff_arm_time) >= wait_time_ms)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Triggered AUTO. GPS speed = %.1f", (double)gps.ground_speed());
        takeoff_state.launchTimerStarted = false;
        takeoff_state.last_tkoff_arm_time = 0;
        takeoff_state.start_time_ms = now;
        steer_state.locked_course_err = 0; // use current heading without any error offset
        return true;
    }

    // we're not launching yet, but the timer is still going
    return false;

no_launch:
    takeoff_state.launchTimerStarted = false;
    takeoff_state.last_tkoff_arm_time = 0;
    return false;
}

/*
  calculate desired bank angle during takeoff, setting nav_roll_cd
 */
void Plane::takeoff_calc_roll(void)
{
    if (steer_state.hold_course_cd == -1) {
        // we don't yet have a heading to hold - just level
        // the wings until we get up enough speed to get a GPS heading
        nav_roll_cd = 0;
        return;
    }

    calc_nav_roll();

    // during takeoff use the level flight roll limit to prevent large
    // wing strike. Slowly allow for more roll as we get higher above
    // the takeoff altitude
    int32_t takeoff_roll_limit_cd = roll_limit_cd;

    if (auto_state.highest_airspeed < g.takeoff_rotate_speed) {
        // before Vrotate (aka, on the ground)
        takeoff_roll_limit_cd = g.level_roll_limit * 100;
    } else {
        // lim1 - below altitude TKOFF_LVL_ALT, restrict roll to LEVEL_ROLL_LIMIT
        // lim2 - above altitude (TKOFF_LVL_ALT * 3) allow full flight envelope of LIM_ROLL_CD
        // In between lim1 and lim2 use a scaled roll limit.
        // The *3 scheme should scale reasonably with both small and large aircraft
        const float lim1 = MAX(mode_takeoff.level_alt, 0);
        const float lim2 = MIN(mode_takeoff.level_alt*3, mode_takeoff.target_alt);
        const float current_baro_alt = barometer.get_altitude();

        takeoff_roll_limit_cd = linear_interpolate(g.level_roll_limit*100, roll_limit_cd,
                                        current_baro_alt,
                                        auto_state.baro_takeoff_alt+lim1, auto_state.baro_takeoff_alt+lim2);
    }

    nav_roll_cd = constrain_int32(nav_roll_cd, -takeoff_roll_limit_cd, takeoff_roll_limit_cd);
}

        
/*
  calculate desired pitch angle during takeoff, setting nav_pitch_cd
 */
void Plane::takeoff_calc_pitch(void)
{
    if (auto_state.highest_airspeed < g.takeoff_rotate_speed) {
        // we have not reached rotate speed, use the specified takeoff target pitch angle
        nav_pitch_cd = int32_t(100.0f * mode_takeoff.ground_pitch);
        return;
    }

    if (ahrs.airspeed_sensor_enabled()) {
        int16_t takeoff_pitch_min_cd = get_takeoff_pitch_min_cd();
        calc_nav_pitch();
        if (nav_pitch_cd < takeoff_pitch_min_cd) {
            nav_pitch_cd = takeoff_pitch_min_cd;
        }
    } else {
        if (g.takeoff_rotate_speed > 0) {
            // Rise off ground takeoff so delay rotation until ground speed indicates adequate airspeed
            nav_pitch_cd = ((gps.ground_speed()*100) / (float)aparm.airspeed_cruise_cm) * auto_state.takeoff_pitch_cd;
            nav_pitch_cd = constrain_int32(nav_pitch_cd, 500, auto_state.takeoff_pitch_cd); 
        } else {
            // Doing hand or catapult launch so need at least 5 deg pitch to prevent initial height loss
            nav_pitch_cd = MAX(auto_state.takeoff_pitch_cd, 500);
        }
    }

    if (aparm.stall_prevention != 0) {
        if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF ||
            control_mode == &mode_takeoff) {
            // during takeoff we want to prioritise roll control over
            // pitch. Apply a reduction in pitch demand if our roll is
            // significantly off. The aim of this change is to
            // increase the robustness of hand launches, particularly
            // in cross-winds. If we start to roll over then we reduce
            // pitch demand until the roll recovers
            float roll_error_rad = radians(constrain_float(labs(nav_roll_cd - ahrs.roll_sensor) * 0.01, 0, 90));
            float reduction = sq(cosf(roll_error_rad));
            nav_pitch_cd *= reduction;
        }
    }
}

/*
 * get the pitch min used during takeoff. This matches the mission pitch until near the end where it allows it to levels off
 */
int16_t Plane::get_takeoff_pitch_min_cd(void)
{
    if (flight_stage != AP_FixedWing::FlightStage::TAKEOFF) {
        return auto_state.takeoff_pitch_cd;
    }

    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    int32_t remaining_height_to_target_cm = (auto_state.takeoff_altitude_rel_cm - relative_alt_cm);

    // seconds to target alt method
    if (g.takeoff_pitch_limit_reduction_sec > 0) {
        // if height-below-target has been initialized then use it to create and apply a scaler to the pitch_min
        if (auto_state.height_below_takeoff_to_level_off_cm != 0) {
            float scalar = remaining_height_to_target_cm / (float)auto_state.height_below_takeoff_to_level_off_cm;
            return auto_state.takeoff_pitch_cd * scalar;
        }

        // are we entering the region where we want to start leveling off before we reach takeoff alt?
        if (auto_state.sink_rate < -0.1f) {
            float sec_to_target = (remaining_height_to_target_cm * 0.01f) / (-auto_state.sink_rate);
            if (sec_to_target > 0 &&
                relative_alt_cm >= 1000 &&
                sec_to_target <= g.takeoff_pitch_limit_reduction_sec) {
                // make a note of that altitude to use it as a start height for scaling
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff level-off starting at %dm", int(remaining_height_to_target_cm/100));
                auto_state.height_below_takeoff_to_level_off_cm = remaining_height_to_target_cm;
            }
        }
    }
    return auto_state.takeoff_pitch_cd;
}

/*
  return a tail hold percentage during initial takeoff for a tail
  dragger

  This can be used either in auto-takeoff or in FBWA mode with
  FBWA_TDRAG_CHAN enabled
 */
int8_t Plane::takeoff_tail_hold(void)
{
    bool in_takeoff = ((control_mode == &mode_auto && !auto_state.takeoff_complete) ||
                       (control_mode == &mode_fbwa && auto_state.fbwa_tdrag_takeoff_mode));
    if (!in_takeoff) {
        // not in takeoff
        return 0;
    }
    if (g.takeoff_tdrag_elevator == 0) {
        // no takeoff elevator set
        goto return_zero;
    }
    if (auto_state.highest_airspeed >= g.takeoff_tdrag_speed1) {
        // we've passed speed1. We now raise the tail and aim for
        // level pitch. Return 0 meaning no fixed elevator setting
        goto return_zero;
    }
    if (ahrs.pitch_sensor > auto_state.initial_pitch_cd + 1000) {
        // the pitch has gone up by more then 10 degrees over the
        // initial pitch. This may mean the nose is coming up for an
        // early liftoff, perhaps due to a bad setting of
        // g.takeoff_tdrag_speed1. Go to level flight to prevent a
        // stall
        goto return_zero;
    }
    // we are holding the tail down
    return g.takeoff_tdrag_elevator;

return_zero:
    if (auto_state.fbwa_tdrag_takeoff_mode) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "FBWA tdrag off");
        auto_state.fbwa_tdrag_takeoff_mode = false;
    }
    return 0;
}

#if AP_LANDINGGEAR_ENABLED
/*
  update landing gear
 */
void Plane::landing_gear_update(void)
{
    g2.landing_gear.update(relative_ground_altitude(g.rangefinder_landing));
}
#endif
