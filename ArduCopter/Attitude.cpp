#include "Copter.h"

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

/*
  update rate controller when run from main thread (normal operation)
*/
void Copter::run_rate_controller_main()
{
    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt_s(last_loop_time_s);
    attitude_control->set_dt_s(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt_s(last_loop_time_s);
        // only run the rate controller if we are not using the rate thread
        attitude_control->rate_controller_run();
    }
    // reset sysid and other temporary inputs
    attitude_control->rate_controller_target_reset();
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
    update_hover_bias_rest();

    // if not armed or landed or on standby then exit
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_U_ms())) {
        return;
    }

    // do not update if no vertical velocity estimate
    float vel_d_ms;
    if (!AP::ahrs().get_velocity_D(vel_d_ms, vibration_check.high_vibes)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if ((throttle > 0.0f) && (fabsf(vel_d_ms) < 0.6) &&
        (fabsf(ahrs.get_roll_rad() - attitude_control->get_roll_trim_rad()) < radians(5)) && (fabsf(ahrs.get_pitch_rad()) < radians(5))) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
        // update learned hover accel bias (checks ground effect flags internally)
        update_hover_bias_learning(0.01f);
    }
}

// get_pilot_desired_climb_rate_ms - transform pilot's throttle input to climb rate in m/s
// without any deadzone at the bottom.
// The max climb and descent rates are adjusted for any climb/descent rate currently being
// commanded by surface tracking, so full stick deflection always yields net vehicle climb
// or descent at PILOT_SPD_UP  / PILOT_SPD_DN  (see get_pilot_speed_up_adjusted_ms()).
float Copter::get_pilot_desired_climb_rate_ms()
{
    // throttle failsafe check
    if (!rc().has_valid_input()) {
        return 0.0f;
    }

    float throttle_control = copter.channel_throttle->get_control_in();

#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control, 0.0f, 1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float desired_rate_ms = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate_ms = get_pilot_speed_dn_adjusted_ms() * (throttle_control - deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate_ms = get_pilot_speed_up_adjusted_ms() * (throttle_control - deadband_top) / (1000.0 - deadband_top);
    } else {
        // must be in the deadband
        desired_rate_ms = 0.0f;
    }

    return desired_rate_ms;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover() / 2.0);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0, 1.0);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->D_get_accel_pid().set_integrator(-(pilot_throttle - motors->get_throttle_hover()));
}

// It will return the PILOT_SPD_DN value if non zero, otherwise if zero it returns the PILOT_SPD_UP value.
float Copter::get_pilot_speed_dn_ms() const
{
    if (is_zero(g2.pilot_speed_dn_ms)) {
        return fabsf(g2.pilot_speed_up_ms);
    } else {
        return fabsf(g2.pilot_speed_dn_ms);
    }
}

// Returns the maximum pilot climb rate (m/s) adjusted for the climb/descent rate currently
// being commanded by surface tracking, to always result in at least PILOT_SPD_UP on full up stick.
float Copter::get_pilot_speed_up_adjusted_ms() const
{
    // terrain velocity in Up-positive frame (m/s)
    const float terrain_climb_ms = -pos_control->get_vel_terrain_D_ms();
    // floored at zero to prevent sign flip if terrain velocity exceeds PILOT_SPD_UP
    return MAX(0.0f, g2.pilot_speed_up_ms - terrain_climb_ms);
}

// Returns the maximum pilot descent rate (m/s, positive magnitude) adjusted for the
// climb/descent rate currently being commanded by surface tracking, to always result in
// at least PILOT_SPD_DN on full down stick.
float Copter::get_pilot_speed_dn_adjusted_ms() const
{
    // terrain velocity in Up-positive frame (m/s)
    const float terrain_climb_ms = -pos_control->get_vel_terrain_D_ms();
    // floored at zero to prevent sign flip if terrain velocity exceeds PILOT_SPD_DN
    return MAX(0.0f, get_pilot_speed_dn_ms() + terrain_climb_ms);
}

// time constant of the hover bias learning filter
static const float HOVER_BIAS_TC_S = 2.0f;

// time at rest the EKF takes to settle the Z accel bias after restarting it, within 0.01 m/s/s in SITL
static const uint32_t HOVER_BIAS_REST_MS = 10000;

// age by which the 100Hz learner hook has seen an EKF accel bias zeroing, with margin for overruns; an estimate
// that comes into view sooner after its zeroing stalled for at most a tenth of the rest wait
static const uint32_t HOVER_BIAS_ZERO_SEEN_MS = 1000;

// how far a zero time the seed reads can be stamped after the seed read the clock: an arm from Lua or DDS can be
// held off behind a main thread EKF update, which stamps a zeroing with its frame start, one loop of at most 20 ms
// as SCHED_LOOP_RATE is 50 Hz or more, with the same margin for overruns as HOVER_BIAS_ZERO_SEEN_MS. It costs only
// a misread of a time within it of the 49.7-day millis() wrap
static const uint32_t HOVER_BIAS_STAMP_LEAD_MS = 1000;

// age of a time on the millis() clock, across its wrap; 0 for a time up to HOVER_BIAS_STAMP_LEAD_MS after now_ms
static uint32_t hover_bias_age_ms(uint32_t now_ms, uint32_t time_ms)
{
    if (time_ms - now_ms < HOVER_BIAS_STAMP_LEAD_MS) {
        return 0;
    }
    return now_ms - time_ms;
}

enum class AccZBiasLearn : uint8_t {
    SAVE             = (1U << 0),
    USE              = (1U << 1),
    INHIBIT_DISARMED = (1U << 2),
};

// init_hover_bias_correction - enables the hover Z-bias correction in AHRS and
// seeds the learner from the saved INS parameters
// called once from startup_INS_ground() after ahrs.reset()
void Copter::init_hover_bias_correction(void)
{
    const uint8_t learn_or_use = uint8_t(AccZBiasLearn::SAVE) | uint8_t(AccZBiasLearn::USE);
    if ((g2.accel_zbias_learn & learn_or_use) == 0) {
        return;
    }

    if (g2.accel_zbias_learn & uint8_t(AccZBiasLearn::USE)) {
        ahrs.set_hover_z_bias_enabled(true);
    }

    seed_hover_bias_learning();
}

// seed_hover_bias_learning - start a flight from the saved bias, which a calibration may have
// changed, and record the bias the EKF holds for each IMU before the motors run and whether
// that is the motors-off bias it learnt at rest
void Copter::seed_hover_bias_learning(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    _hover_bias_inhibit_disarmed = (g2.accel_zbias_learn & uint8_t(AccZBiasLearn::INHIBIT_DISARMED)) != 0;
    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        _hover_bias_learning[imu] = AP::ins().get_accel_vrf_bias_z(imu);
        _hover_bias_learned[imu] = false;
        _hover_bias_restarted[imu] = false;
        // read before the bias, so that a restart between the two reads shows as a change
        _hover_bias_reset_count[imu] = ahrs.get_accel_bias_reset_count_for_imu(imu);
        _hover_bias_zero_ms[imu] = ahrs.get_accel_bias_zero_time_ms_for_imu(imu);
        _hover_bias_at_arm_ok[imu] = ahrs.get_accel_bias_z_for_imu(imu, _hover_bias_at_arm[imu]);
        // ages rather than times, so that the comparisons hold across the millis() wrap
        const uint32_t zero_age_ms = hover_bias_age_ms(now_ms, _hover_bias_zero_ms[imu]);
        uint32_t rest_ms = zero_age_ms;
        if (_hover_bias_rest_start_ms[imu] != 0) {
            rest_ms = MIN(rest_ms, now_ms - _hover_bias_rest_start_ms[imu]);
        }
        if (_hover_bias_inhibit_disarmed) {
            // with bit 2 the EKF learns no bias on the ground, and what it holds is the last flight's
            _hover_bias_at_arm[imu] = 0;
            _hover_bias_baseline[imu] = HoverBiasBaseline::OK;
        } else if ((_hover_bias_zero_ms[imu] == 0) ||
                   ((_hover_bias_flight_end_ms != 0) && (zero_age_ms >= now_ms - _hover_bias_flight_end_ms))) {
            // after a flight the bias at rest stays the flight's, through the ~10 s measured at rest. Only
            // zeroing it restarts it: an accel switch takes the estimate held for the other accel, which
            // after a landing is a hover bias too
            _hover_bias_baseline[imu] = HoverBiasBaseline::NOT_FIRST;
        } else if (_hover_bias_switched[imu]) {
            // a switch hands back an estimate the EKF did not settle itself, frozen part settled while the
            // accel was out of use; it settles far slower than the rest wait, so only a zeroing reopens it
            _hover_bias_baseline[imu] = HoverBiasBaseline::SWITCHED;
        } else if (rest_ms < HOVER_BIAS_REST_MS) {
            // a zeroed bias holds only part of the motors-off bias until it settles at rest
            _hover_bias_baseline[imu] = HoverBiasBaseline::TOO_SOON;
        } else {
            _hover_bias_baseline[imu] = HoverBiasBaseline::OK;
        }
    }
    _hover_bias_flew = false;
}

// update_hover_bias_rest - note an accel switch that hands an IMU a bias estimate without zeroing it,
// and while armed, whether the vehicle has left the ground
// called at 100Hz from update_throttle_hover(); an arm or disarm from Lua or DDS can interleave with
// it for a few microseconds, as with the learner's other state
void Copter::update_hover_bias_rest(void)
{
    if ((g2.accel_zbias_learn & uint8_t(AccZBiasLearn::SAVE)) == 0) {
        return;
    }
    if (motors->armed() && !ap.land_complete) {
        _hover_bias_flew = true;
    }
    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        const uint16_t count = ahrs.get_accel_bias_reset_count_for_imu(imu);
        const uint32_t zero_ms = ahrs.get_accel_bias_zero_time_ms_for_imu(imu);
        // polled armed or not, a zeroing is seen at once; an older time is an estimate coming into view, as
        // when an accel out of use when the EKF zeroed it comes back, and is left to the count
        if ((zero_ms != 0) && (zero_ms != _hover_bias_rest_zero_ms[imu]) && (now_ms - zero_ms < HOVER_BIAS_ZERO_SEEN_MS)) {
            _hover_bias_rest_zero_ms[imu] = zero_ms;
            _hover_bias_switched[imu] = false;
        } else if (count != _hover_bias_rest_count[imu]) {
            // a restart that zeroed nothing is a switch onto or off the accel; a time of 0 is the accel out of use
            _hover_bias_switched[imu] = true;
        }
        _hover_bias_rest_count[imu] = count;
    }
}

// report_hover_z_bias - announce the correction EKF3 will apply, once per boot
// called from one_hz_loop while disarmed
void Copter::report_hover_z_bias(void)
{
    if (_hover_z_bias_reported || (g2.accel_zbias_learn & uint8_t(AccZBiasLearn::USE)) == 0) {
        return;
    }
    _hover_z_bias_reported = true;

    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        if (!is_zero(_hover_bias_learning[imu])) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hover Z-bias IMU%u: %.3f m/s/s", imu, _hover_bias_learning[imu]);
        }
    }
}

// update_hover_bias_learning - learns Z-axis accelerometer bias during hover
// to compensate for vibration rectification effects, stopping for an IMU whose
// bias the EKF has restarted since arming
// called at 100Hz from update_throttle_hover() when already in a stable hover
// (hover conditions checked by caller: armed, level, low velocity, throttle > 0)
void Copter::update_hover_bias_learning(float dt)
{
    if ((g2.accel_zbias_learn & uint8_t(AccZBiasLearn::SAVE)) == 0) {
        return;
    }

    // Don't learn during ground effect (vibration rectification differs near ground)
    if (ahrs.get_takeoff_expected() || ahrs.get_touchdown_expected()) {
        return;
    }

    const float alpha = dt / (dt + HOVER_BIAS_TC_S);

    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        // no core used this IMU when the vehicle armed
        if (!_hover_bias_at_arm_ok[imu]) {
            continue;
        }
        float bias_z_mss;
        const bool in_use = ahrs.get_accel_bias_z_for_imu(imu, bias_z_mss);

        // the EKF has restarted the bias since arming, and from here holds a relearning; read the count
        // after the bias, so that a restart between the two reads is not missed, and before whether the
        // IMU is in use, as a core switching off it is itself a restart
        if (ahrs.get_accel_bias_reset_count_for_imu(imu) != _hover_bias_reset_count[imu]) {
            _hover_bias_restarted[imu] = true;
            continue;
        }
        if (!in_use) {
            continue;
        }

        // add back the correction already applied; the bias the EKF learnt on the ground is left out,
        // as it learns that again before every flight and applying it from here would count it twice
        const float total_bias_mss = bias_z_mss + ahrs.get_hover_z_bias_correction(imu);
        const float motors_on_bias_mss = total_bias_mss - _hover_bias_at_arm[imu];

        _hover_bias_learning[imu] += alpha * (motors_on_bias_mss - _hover_bias_learning[imu]);
        _hover_bias_learned[imu] = true;
    }
}

// save_hover_bias_learning - saves learned hover bias to EEPROM, from the first flight after
// the EKF restarted the IMU's bias at rest only, as only then is the bias at arming motors-off
// called on disarm
void Copter::save_hover_bias_learning(void)
{
    if ((g2.accel_zbias_learn & uint8_t(AccZBiasLearn::SAVE)) == 0) {
        return;
    }
    // bit 2 as at arming chose the baseline; a value learnt against one setting is not saved under the other
    const bool inhibit_disarmed = _hover_bias_inhibit_disarmed;
    const bool bit2_changed = inhibit_disarmed != ((g2.accel_zbias_learn & uint8_t(AccZBiasLearn::INHIBIT_DISARMED)) != 0);
    bool reset = false;
    bool changed = false;
    bool too_soon = false;
    bool switched = false;
    bool not_first = false;
    bool not_learned = false;
    // a zeroing in a flight relearns the hover bias, so it opens no first flight
    const uint32_t now_ms = AP_HAL::millis();
    if (_hover_bias_flew) {
        _hover_bias_flight_end_ms = now_ms;
    }
    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        // armed on the ground the EKF fuses no zero velocity, so a zeroing there, or a wait that had not
        // settled the bias, settles again at rest from here
        if (!_hover_bias_flew &&
            ((ahrs.get_accel_bias_zero_time_ms_for_imu(imu) != _hover_bias_zero_ms[imu]) ||
             (_hover_bias_baseline[imu] == HoverBiasBaseline::TOO_SOON))) {
            _hover_bias_rest_start_ms[imu] = now_ms;
        }

        // the EKF restarted the bias while the learner was taking it, perhaps before its first sample or by
        // switching the core off the IMU, so tested before whether the IMU is in use; the count is not
        // compared here, as an accel brought back into use after landing moves the other accel's count
        if (_hover_bias_restarted[imu]) {
            reset = true;
            continue;
        }
        // only an IMU some EKF core is actually using is saved
        float bias_z;
        if (!ahrs.get_accel_bias_z_for_imu(imu, bias_z)) {
            continue;
        }
        // an IMU that learnt nothing keeps its saved value, which may have been set since the learner
        // was seeded; without bit 2 a first flight that could have learnt says so, as it was the only one
        if (!_hover_bias_learned[imu]) {
            not_learned |= !inhibit_disarmed && _hover_bias_at_arm_ok[imu] && _hover_bias_flew &&
                           (_hover_bias_baseline[imu] == HoverBiasBaseline::OK);
            continue;
        }
        if (bit2_changed) {
            changed = true;
            continue;
        }
        if (_hover_bias_baseline[imu] == HoverBiasBaseline::TOO_SOON) {
            too_soon = true;
            continue;
        }
        if (_hover_bias_baseline[imu] == HoverBiasBaseline::SWITCHED) {
            switched = true;
            continue;
        }
        if (_hover_bias_baseline[imu] == HoverBiasBaseline::NOT_FIRST) {
            not_first = true;
            continue;
        }
        AP::ins().set_accel_vrf_bias_z(imu, _hover_bias_learning[imu]);
        AP::ins().save_accel_vrf_bias_z(imu);
    }
    if (reset) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hover Z-bias not saved: EKF accel bias reset");
    } else if (changed) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hover Z-bias not saved: ACC_ZBIAS_LEARN changed");
    } else if (too_soon) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hover Z-bias not saved: armed <10s after EKF reset");
    } else if (switched) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hover Z-bias not saved: accel switched on ground");
    } else if (not_first) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hover Z-bias not saved: not first flight");
    } else if (not_learned) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hover Z-bias not learned: no steady hover");
    }
}

// update_accel_bias_inhibit - inhibit EKF accel bias learning while disarmed when
// ACC_ZBIAS_LEARN bit 2 is set, so the motors-off bias is not learned as the correction.
// arm() clears it at once; clearing it here too undoes a set that raced an arm from
// another thread.
// called from one_hz_loop
void Copter::update_accel_bias_inhibit(void)
{
    const bool inhibit_disarmed = (g2.accel_zbias_learn & uint8_t(AccZBiasLearn::INHIBIT_DISARMED)) != 0;
    ahrs.set_inhibit_accel_bias_learning(inhibit_disarmed && !motors->armed());
}
