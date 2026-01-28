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

// get_pilot_desired_climb_rate_ms - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
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
        desired_rate_ms = get_pilot_speed_dn() * 0.01 * (throttle_control - deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate_ms = g.pilot_speed_up_cms * 0.01 * (throttle_control - deadband_top) / (1000.0 - deadband_top);
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

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn_cms == 0) {
        return abs(g.pilot_speed_up_cms);
    } else {
        return abs(g2.pilot_speed_dn_cms);
    }
}

// Time constant for hover bias learning filter (seconds)
#define HOVER_BIAS_TC 2.0f

// init_hover_bias_correction - loads saved hover Z-bias from INS parameters
// into _hover_bias_learning array. The frozen correction in EKF is set later
// from one_hz_loop once EKF3 is active.
// called once from startup_INS_ground() after ahrs.reset()
void Copter::init_hover_bias_correction(void)
{
    if (g2.accel_zbias_learn <= 0) {
        return;
    }

    // Initialize learning state from INS parameters
    // The actual EKF correction will be set later once EKF3 is active
    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        const float raw_bias = AP::ins().get_accel_vrf_bias_z(imu);
        _hover_bias_learning[imu] = raw_bias;
    }
}

// set_hover_z_bias_correction - sets the frozen hover Z-bias correction in EKF
// called from one_hz_loop while disarmed until values match
void Copter::set_hover_z_bias_correction(void)
{
    if (g2.accel_zbias_learn <= 0) {
        return;
    }

    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        const float saved_bias = _hover_bias_learning[imu];
        const float current_correction = ahrs.get_hover_z_bias_correction(imu);

        // Skip if already set correctly or nothing to set
        if (is_equal(saved_bias, current_correction)) {
            continue;
        }

        // Try to set the correction
        if (ahrs.set_hover_z_bias_correction(imu, saved_bias)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hover Z-bias IMU%u: %.3f m/s^2", imu, saved_bias);
        }
    }
}

// update_hover_bias_learning - learns Z-axis accelerometer bias during hover
// to compensate for vibration rectification effects
// called at 100Hz from update_throttle_hover() when already in a stable hover
// (hover conditions checked by caller: armed, level, low velocity, throttle > 0)
void Copter::update_hover_bias_learning(float dt)
{
    if (g2.accel_zbias_learn <= 0) {
        return;
    }

    // Don't learn during ground effect (motor thrust offset corrupts bias)
    if (ahrs.get_takeoff_expected() || ahrs.get_touchdown_expected()) {
        return;
    }

    const float alpha = dt / (dt + HOVER_BIAS_TC);

    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        // Get current EKF bias for this IMU (returns false if no core uses it)
        float currentBiasZ;
        if (!ahrs.get_accel_bias_z_for_imu(imu, currentBiasZ)) {
            continue;
        }

        // Get frozen correction (already applied at IMU level)
        const float frozenCorrection = ahrs.get_hover_z_bias_correction(imu);

        // Total bias = EKF residual + frozen correction
        const float totalBias = currentBiasZ + frozenCorrection;

        // Apply low-pass filter
        _hover_bias_learning[imu] += alpha * (totalBias - _hover_bias_learning[imu]);

        // Update INS parameter (RAM only, not saved yet)
        AP::ins().set_accel_vrf_bias_z(imu, _hover_bias_learning[imu]);
    }
}

// save_hover_bias_learning - saves learned hover bias to EEPROM
// called on disarm
void Copter::save_hover_bias_learning(void)
{
    if (g2.accel_zbias_learn != 2) {
        return;
    }

    for (uint8_t imu = 0; imu < INS_MAX_INSTANCES; imu++) {
        // Only save if an EKF core is using this IMU
        float bias_z;
        if (ahrs.get_accel_bias_z_for_imu(imu, bias_z)) {
            AP::ins().save_accel_vrf_bias_z(imu);
        }
    }
}
