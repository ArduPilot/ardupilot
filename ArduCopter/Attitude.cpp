#include "Copter.h"
#include <AP_HAL/CondMutex.h>

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

#if AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED

//#define RATE_LOOP_TIMING_DEBUG
/*
  thread for rate control
*/
void Copter::rate_controller_thread()
{
    uint8_t target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1,
                                                     uint8_t((ins.get_raw_gyro_rate_hz() + 400 - 1) / 400));
    uint8_t rate_decimation = target_rate_decimation;
    uint32_t rate_loop_count = 0;
    uint32_t prev_loop_count = 0;

    HAL_CondMutex cmutex;
    ins.set_rate_loop_mutex(&cmutex);

    uint32_t last_run_us = AP_HAL::micros();
    float max_dt = 0.0;
    float min_dt = 1.0;
    uint32_t now_ms = AP_HAL::millis();
    uint32_t last_rate_check_ms = 0;
    uint32_t last_rate_increase_ms = 0;
#if HAL_LOGGING_ENABLED
    uint32_t last_rtdt_log_ms = now_ms;
#endif
    uint32_t last_notch_sample_ms = now_ms;
    bool was_using_rate_thread = false;
    bool was_armed = false;
    uint32_t running_slow = 0;
#ifdef RATE_LOOP_TIMING_DEBUG
    uint32_t gyro_sample_time_us = 0;
    uint32_t rate_controller_time_us = 0;
    uint32_t motor_output_us = 0;
    uint32_t timing_count = 0;
    uint32_t last_timing_msg_us = 0;
#endif

    // run the filters at half the gyro rate
    uint8_t filter_rate_decimate = 2;
#if HAL_LOGGING_ENABLED
    uint8_t log_fast_rate_decimate = uint8_t((ins.get_raw_gyro_rate_hz() + 1000 - 1) / 1000);   // 1Khz
    uint8_t log_med_rate_decimate = uint8_t((ins.get_raw_gyro_rate_hz() + 10 - 1) / 10);        // 10Hz
    uint8_t log_loop_count = 0;
#endif
    uint8_t filter_loop_count = 0;

    while (true) {

#ifdef RATE_LOOP_TIMING_DEBUG
        uint32_t rate_now_us = AP_HAL::micros();
#endif

        // allow changing option at runtime
        if ((get_fast_rate_type() != FastRateType::FAST_RATE_DYNAMIC &&
             get_fast_rate_type() != FastRateType::FAST_RATE_FIXED) ||
            ap.motor_test) {
            using_rate_thread = false;
            if (was_using_rate_thread) {
                // if we were using the rate thread, we need to
                // setup the notch filter sample rate
                const float loop_rate_hz = AP::scheduler().get_filtered_loop_rate_hz();
                attitude_control->set_notch_sample_rate(loop_rate_hz);
                motors->set_dt(1.0/loop_rate_hz);
                ins.set_rate_decimation(0);
                hal.rcout->force_trigger_groups(false);
                was_using_rate_thread = false;
            }
            hal.scheduler->delay_microseconds(500);
            last_run_us = AP_HAL::micros();
            continue;
        }

        // set up rate thread requirements
        using_rate_thread = true;
        hal.rcout->force_trigger_groups(true);
        ins.set_rate_decimation(rate_decimation);

        // wait for an IMU sample
        Vector3f gyro;
        ins.get_next_gyro_sample(gyro);

#ifdef RATE_LOOP_TIMING_DEBUG
        gyro_sample_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // we must use multiples of the actual sensor rate
        const float sensor_dt = 1.0f * rate_decimation / ins.get_raw_gyro_rate_hz();
        const uint32_t now_us = AP_HAL::micros();
        const uint32_t dt_us = now_us - last_run_us;
        const float dt = dt_us * 1.0e-6;
        last_run_us = now_us;

        max_dt = MAX(dt, max_dt);
        min_dt = MIN(dt, min_dt);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: RTDT
// @Description: Attitude controller time deltas
// @Field: TimeUS: Time since system startup
// @Field: dt: current time delta
// @Field: dtAvg: current time delta average
// @Field: dtMax: Max time delta since last log output
// @Field: dtMin: Min time delta since last log output

        if (now_ms - last_rtdt_log_ms >= 10) {    // 100 Hz
            AP::logger().WriteStreaming("RTDT", "TimeUS,dt,dtAvg,dtMax,dtMin", "Qffff",
                                                AP_HAL::micros64(),
                                                dt, sensor_dt, max_dt, min_dt);
            max_dt = sensor_dt;
            min_dt = sensor_dt;
            last_rtdt_log_ms = now_ms;
        }
#endif

        motors->set_dt(sensor_dt);
        // check if we are falling behind
        if (ins.get_num_gyro_samples() > 2) {
            running_slow++;
        } else if (running_slow > 0) {
            running_slow--;
        }
        if (AP::scheduler().get_extra_loop_us() == 0) {
            rate_loop_count++;
        }

        // run the rate controller on all available samples
        // it is important not to drop samples otherwise the filtering will be fubar
        // there is no need to output to the motors more than once for every batch of samples
        attitude_control->rate_controller_run_dt(sensor_dt, gyro + ahrs.get_gyro_drift());

#ifdef RATE_LOOP_TIMING_DEBUG
        rate_controller_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // immediately output the new motor values
        motors_output();

        // process filter updates
        if (filter_loop_count++ >= filter_rate_decimate) {
            filter_loop_count = 0;
            rate_controller_filter_update();
        }

#if HAL_LOGGING_ENABLED
        // fast logging output
        if (should_log(MASK_LOG_ATTITUDE_FAST)) {
            if (log_fast_rate_decimate > 0 && log_loop_count++ >= log_fast_rate_decimate) {
                log_loop_count = 0;
                rate_controller_log_update();
            }
        } else if (should_log(MASK_LOG_ATTITUDE_MED)) {
            if (log_med_rate_decimate > 0 && log_loop_count++ >= log_med_rate_decimate) {
                log_loop_count = 0;
                rate_controller_log_update();
            }
        }
#endif

#ifdef RATE_LOOP_TIMING_DEBUG
        motor_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        now_ms = AP_HAL::millis();

        // make sure we have the latest target rate
        target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1,
                                                 uint8_t((ins.get_raw_gyro_rate_hz() + 400 - 1) / 400));
        if (now_ms - last_notch_sample_ms >= 1000 || !was_using_rate_thread) {
            // update the PID notch sample rate at 1Hz if we are
            // enabled at runtime
            last_notch_sample_ms = now_ms;
            attitude_control->set_notch_sample_rate(1.0 / sensor_dt);
        }

        // Once armed, switch to the fast rate if configured to do so
        if ((rate_decimation != target_rate_decimation || !was_armed) && motors->armed() && get_fast_rate_type() == FastRateType::FAST_RATE_FIXED) {
            rate_decimation = target_rate_decimation;
            attitude_control->set_notch_sample_rate(ins.get_raw_gyro_rate_hz() / rate_decimation);
            gcs().send_text(MAV_SEVERITY_INFO, "Attitude rate active at %uHz", (unsigned)ins.get_raw_gyro_rate_hz() / rate_decimation);
            was_armed = motors->armed();
        } else if (!motors->armed()) {
            was_armed = false;
        }
        
        // check that the CPU is not pegged, if it is drop the attitude rate
        if (now_ms - last_rate_check_ms >= 100
            && (get_fast_rate_type() != FastRateType::FAST_RATE_FIXED || !motors->armed() || target_rate_decimation > rate_decimation)) {
            last_rate_check_ms = now_ms;
            const uint32_t att_rate = ins.get_raw_gyro_rate_hz()/rate_decimation;
            if (running_slow > 5 || AP::scheduler().get_extra_loop_us() > 0
#if HAL_LOGGING_ENABLED
                || AP::logger().in_log_download()
#endif
                || target_rate_decimation > rate_decimation) {
                const uint8_t new_rate_decimation = MAX(rate_decimation + 1, target_rate_decimation);
                const uint32_t new_attitude_rate = ins.get_raw_gyro_rate_hz() / new_rate_decimation;
                if (new_attitude_rate > AP::scheduler().get_filtered_loop_rate_hz()) {
                    rate_decimation = new_rate_decimation;
                    attitude_control->set_notch_sample_rate(new_attitude_rate);
                    gcs().send_text(MAV_SEVERITY_WARNING, "Attitude CPU high, dropping rate to %uHz",
                                    (unsigned)new_attitude_rate);
                    prev_loop_count = rate_loop_count;
                    rate_loop_count = 0;
                    running_slow = 0;
#if HAL_LOGGING_ENABLED
                    if (new_attitude_rate > 1000) {
                        log_fast_rate_decimate = uint8_t((ins.get_raw_gyro_rate_hz() + 1000 - 1) / 1000);
                    } else {
                        log_fast_rate_decimate = uint8_t((ins.get_raw_gyro_rate_hz()
                            + AP::scheduler().get_filtered_loop_rate_hz() - 1) / AP::scheduler().get_filtered_loop_rate_hz());
                    }
#endif
                }
            } else if (rate_decimation > target_rate_decimation && rate_loop_count > att_rate/10 // ensure 100ms worth of good readings
                && (prev_loop_count > att_rate/10   // ensure there was 100ms worth of good readings at the higher rate
                    || prev_loop_count == 0         // last rate was actually a lower rate so keep going quickly
                    || now_ms - last_rate_increase_ms >= 10000)) { // every 10s retry
                const uint32_t new_attitude_rate = ins.get_raw_gyro_rate_hz()/(rate_decimation-1);
                rate_decimation = rate_decimation - 1;
                attitude_control->set_notch_sample_rate(new_attitude_rate);
                gcs().send_text(MAV_SEVERITY_INFO, "Attitude CPU normal, increasing rate to %uHz",
                                (unsigned) new_attitude_rate);
                prev_loop_count = 0;
                rate_loop_count = 0;
                last_rate_increase_ms = now_ms;
#if HAL_LOGGING_ENABLED
                if (new_attitude_rate > 1000) {
                    log_fast_rate_decimate = uint8_t((ins.get_raw_gyro_rate_hz() + 1000 - 1) / 1000);
                } else {
                    log_fast_rate_decimate = uint8_t((ins.get_raw_gyro_rate_hz()
                        + AP::scheduler().get_filtered_loop_rate_hz() - 1) / AP::scheduler().get_filtered_loop_rate_hz());
                }
#endif
            }
        }

#ifdef RATE_LOOP_TIMING_DEBUG
        timing_count++;

        if (rate_now_us - last_timing_msg_us > 1e6) {
            hal.console->printf("Rate loop timing: gyro=%uus, rate=%uus, motors=%uus\n",
                                unsigned(gyro_sample_time_us/timing_count), unsigned(rate_controller_time_us/timing_count),
                                unsigned(motor_output_us/timing_count));
            last_timing_msg_us = rate_now_us;
            timing_count = 0;
            gyro_sample_time_us = rate_controller_time_us = motor_output_us = 0;
        }
#endif

        was_using_rate_thread = true;
    }
}

/*
  update rate controller filters. on an H7 this is about 30us
*/
void Copter::rate_controller_filter_update()
{
    // update the frontend center frequencies of notch filters
    for (auto &notch : ins.harmonic_notches) {
        update_dynamic_notch(notch);
    }

    // this copies backend data to the frontend and updates the notches
    ins.update_backend_filters();
}

/*
  log only those items that are updated at the rate loop rate
 */
void Copter::rate_controller_log_update()
{
#if HAL_LOGGING_ENABLED
    if (!copter.flightmode->logs_attitude()) {
        Log_Write_Rate();
        Log_Write_PIDS(); // only logs if PIDS bitmask is set
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        AP::ins().write_notch_log_messages();
    }
#endif
#endif
}

#endif // AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED

/*
  update rate controller when run from main thread (normal operation)
*/
void Copter::run_rate_controller_main()
{
    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt(last_loop_time_s);
    attitude_control->set_dt(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt(last_loop_time_s);
        // only run the rate controller if we are not using the rate thread
        attitude_control->rate_controller_run();
    }
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
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        fabsf(ahrs.roll_sensor-attitude_control->get_roll_trim_cd()) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }

#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float desired_rate = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    } else {
        return abs(g2.pilot_speed_dn);
    }
}
