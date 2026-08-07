/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

#pragma GCC optimize("O2")

/*
 Attitude Rate controller thread design.

 Rationale: running rate outputs linked to fast gyro outputs achieves two goals:

 1. High frequency gyro processing allows filters to be applied with high sample rates
    which is advantageous in removing high frequency noise and associated aliasing
 2. High frequency rate control reduces the latency between control and action leading to 
    better disturbance rejection and faster responses which generally means higher
    PIDs can be used without introducing control oscillation

 (1) is already mostly achieved through the higher gyro rates that are available via
 INS_GYRO_RATE. (2) requires running the rate controller at higher rates via a separate thread


 Goal: the ideal scenario is to run in a single cycle:

    gyro read->filter->publish->rate control->motor output

 This ensures the minimum latency between gyro sample and motor output. Other functions need 
 to also run faster than they would normally most notably logging and filter frequencies - most
 notably the harmonic notch frequency.

 Design assumptions:

 1. The sample rate of the IMUs is consistent and accurate.
    This is the most basic underlying assumption. An alternative approach would be to rely on
    the timing of when samples are received but this proves to not work in practice due to
    scheduling delays. Thus the dt used by the attitude controller is the delta between IMU
    measurements, not the delta between processing cycles in the rate thread.
 2. Every IMU reading must be processed or consistently sub-sampled.
    This is an assumption that follows from (1) - so it means that attitude control should
    process every sample or every other sample or every third sample etc. Note that these are
    filtered samples - all incoming samples are processed for filtering purposes, it is only
    for the purposes of rate control that we are sub-sampling.
 3. The data that the rate loop requires is timely, consistent and accurate.
    Rate control essentially requires two components - the target and the actuals. The actuals
    come from the incoming gyro sample combined with the state of the PIDs. The target comes
    from attitude controller which is running at a slower rate in the main loop. Since the rate
    thread can read the attitude target at any time it is important that this is always available
    consistently and is updated consistently.
 4. The data that the rest of the vehicle uses is the same data that the rate thread uses.
    Put another way any gyro value that the vehicle uses (e.g. in the EKF etc), must have already
    been processed by the rate thread. Where this becomes important is with sub-sampling - if 
    rate gyro values are sub-sampled we need to make sure that the vehicle is also only using
    the sub-sampled values.

 Design:

 1. Filtered gyro samples are (sub-sampled and) pushed into an ObjectBuffer from the INS backend.
 2. The pushed sample is published to the INS front-end so that the rest of the vehicle only
    sees published values that have been used by the rate controller. When the rate thread is not 
    in use the filtered samples are effectively sub-sampled at the main loop rate. The EKF is unaffected
    as it uses delta angles calculated from the raw gyro values. (It might be possible to avoid publishing
    from the rate thread by only updating _gyro_filtered when a value is pushed).
 3. A notification is sent that a sample is available
 4. The rate thread is blocked waiting for a sample. When it receives a notification it:
    4a. Runs the rate controller
    4b. Pushes the new pwm values. Periodically at the main loop rate all of the SRV_Channels::push()
        functionality is run as well.
 5. The rcout dshot thread is blocked waiting for a new pwm value. When it is signalled by the
    rate thread it wakes up and runs the dshot motor output logic.
 6. Periodically the rate thread:
    6a. Logs the rate outputs (1Khz)
    6b. Updates the notch filter centers (Gyro rate/2)
    6c. Checks the ObjectBuffer length and main loop delay (10Hz)
        If the ObjectBuffer length has been longer than 2 for the last 5 cycles or the main loop has
        been slowed down then the rate thread is slowed down by telling the INS to sub-sample. This
        mechanism is continued until the rate thread is able to keep up with the sub-sample rate.
        The inverse of this mechanism is run if the rate thread is able to keep up but is running slower
        than the gyro sample rate.
    6d. Updates the PID notch centers (1Hz)
 7. When the rate rate changes through sub-sampling the following values are updated:
    7a. The PID notch sample rate
    7b. The dshot rate is constrained to be never greater than the gyro rate or rate rate
    7c. The motors dt
 8. Independently of the rate thread the attitude control target is updated in the main loop. In order
    for target values to be consistent all updates are processed using local variables and the final
    target is only written at the end of the update as a vector. Direct control of the target (e.g. in
    autotune) is also constrained to be on all axes simultaneously using the new desired value. The
    target makes use of the current PIDs and the "latest" gyro, it might be possible to use a loop
    delayed gyro value, but that is currently out-of-scope.

 Performance considerations:

 On an H754 using ICM42688 and gyro sampling at 4KHz and rate thread at 4Khz the main CPU users are:

 ArduCopter    PRI=182 sp=0x30000600 STACK=4392/7168 LOAD=18.6%
 idle          PRI=  1 sp=0x300217B0 STACK= 296/ 504 LOAD= 4.3%
 rcout         PRI=181 sp=0x3001DAF0 STACK= 504/ 952 LOAD=10.7%
 SPI1          PRI=181 sp=0x3002DAB8 STACK= 856/1464 LOAD=17.5%
 SPI4          PRI=181 sp=0x3002D4A0 STACK= 888/1464 LOAD=18.3%
 rate          PRI=182 sp=0x3002B1D0 STACK=1272/1976 LOAD=22.4%

 There is a direct correlation between the rate rate and CPU load, so if the rate rate is half the gyro
 rate (i.e. 2Khz) we observe the following:

 ArduCopter    PRI=182 sp=0x30000600 STACK=4392/7168 LOAD=16.7%
 idle          PRI=  1 sp=0x300217B0 STACK= 296/ 504 LOAD=21.3%
 rcout         PRI=181 sp=0x3001DAF0 STACK= 504/ 952 LOAD= 6.2%
 SPI1          PRI=181 sp=0x3002DAB8 STACK= 856/1464 LOAD=16.7%
 SPI4          PRI=181 sp=0x3002D4A0 STACK= 888/1464 LOAD=17.8%
 rate          PRI=182 sp=0x3002B1D0 STACK=1272/1976 LOAD=11.5%

 So we get almost a halving of CPU load in the rate and rcout threads. This is the main way that CPU
 load can be handled on lower-performance boards, with the other mechanism being lowering the gyro rate.
 So at a very respectable gyro rate and rate rate both of 2Khz (still 5x standard main loop rate) we see:

 ArduCopter    PRI=182 sp=0x30000600 STACK=4440/7168 LOAD=15.6%
 idle          PRI=  1 sp=0x300217B0 STACK= 296/ 504 LOAD=39.4%
 rcout         PRI=181 sp=0x3001DAF0 STACK= 504/ 952 LOAD= 5.9%
 SPI1          PRI=181 sp=0x3002DAB8 STACK= 856/1464 LOAD= 8.9%
 SPI4          PRI=181 sp=0x3002D4A0 STACK= 896/1464 LOAD= 9.1%
 rate          PRI=182 sp=0x30029FB0 STACK=1296/1976 LOAD=11.8%

 This essentially means that its possible to run this scheme successfully on all MCUs by careful setting of 
 the maximum rates.

 Enabling rate thread timing debug for 4Khz reads with fast logging and armed we get the following data:

 Rate loop timing: gyro=178us, rate=13us, motors=45us, log=7us, ctrl=1us
 Rate loop timing: gyro=178us, rate=13us, motors=45us, log=7us, ctrl=1us
 Rate loop timing: gyro=177us, rate=13us, motors=46us, log=7us, ctrl=1us

 The log output is an average since it only runs at 1Khz, so roughly 28us elapsed. So the majority of the time
 is spent waiting for a gyro sample (higher is better here since it represents the idle time) updating the PIDs
 and outputting to the motors. Everything else is relatively cheap. Since the total cycle time is 250us the duty
 cycle is thus 29%
 */

#define DIV_ROUND_INT(x, d) ((x + d/2) / d)

uint8_t Copter::calc_gyro_decimation(uint8_t gyro_decimation, uint16_t rate_hz)
{
    return MAX(uint8_t(DIV_ROUND_INT(ins.get_raw_gyro_rate_hz() / gyro_decimation, rate_hz)), 1U);
}

static inline bool run_decimated_callback(uint8_t decimation_rate, uint8_t& decimation_count)
{
    return decimation_rate > 0 && ++decimation_count >= decimation_rate;
}

//#define RATE_LOOP_TIMING_DEBUG
/*
  thread for rate control
*/
void Copter::rate_controller_thread()
{
    uint8_t target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1,
                                                     DIV_ROUND_INT(ins.get_raw_gyro_rate_hz(), AP::scheduler().get_loop_rate_hz()));
    uint8_t rate_decimation = target_rate_decimation;

    // set up the decimation rates
    RateControllerRates rates;
    rate_controller_set_rates(rate_decimation, rates, false);

    uint32_t rate_loop_count = 0;
    uint32_t prev_loop_count = 0;

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
    bool notify_fixed_rate_active = true;
    bool was_armed = false;
    uint32_t running_slow = 0;
#ifdef RATE_LOOP_TIMING_DEBUG
    uint32_t gyro_sample_time_us = 0;
    uint32_t rate_controller_time_us = 0;
    uint32_t motor_output_us = 0;
    uint32_t log_output_us = 0;
    uint32_t ctrl_output_us = 0;
    uint32_t timing_count = 0;
    uint32_t last_timing_msg_us = 0;
#endif

    // run the filters at half the gyro rate
#if HAL_LOGGING_ENABLED
    uint8_t log_loop_count = 0;
#endif
    uint8_t main_loop_count = 0;
    uint8_t filter_loop_count = 0;

    while (true) {

#ifdef RATE_LOOP_TIMING_DEBUG
        uint32_t rate_now_us = AP_HAL::micros();
#endif

        // allow changing option at runtime
        if (get_fast_rate_type() == FastRateType::FAST_RATE_DISABLED) {
            if (was_using_rate_thread) {
                disable_fast_rate_loop(rates);
                was_using_rate_thread = false;
            }
            hal.scheduler->delay_microseconds(500);
            last_run_us = AP_HAL::micros();
            continue;
        }

        // set up rate thread requirements
        if (!using_rate_thread) {
            enable_fast_rate_loop(rate_decimation, rates);
        }
        ins.set_rate_decimation(rate_decimation);

        // wait for an IMU sample
        Vector3f gyro;
        if (!ins.get_next_gyro_sample(gyro)) {
            continue;   // go around again
        }

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
        attitude_control->rate_controller_run_dt(gyro + ahrs.get_gyro_drift(), sensor_dt);

#ifdef RATE_LOOP_TIMING_DEBUG
        rate_controller_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // immediately output the new motor values
        if (run_decimated_callback(rates.main_loop_rate, main_loop_count)) {
            main_loop_count = 0;
        }
        motors_output(main_loop_count == 0);

        // process filter updates
        if (run_decimated_callback(rates.filter_rate, filter_loop_count)) {
            filter_loop_count = 0;

            rate_controller_filter_update();
        }

        max_dt = MAX(dt, max_dt);
        min_dt = MIN(dt, min_dt);

#if HAL_LOGGING_ENABLED
        if (now_ms - last_rtdt_log_ms >= 100) {    // 10 Hz
            Log_Write_Rate_Thread_Dt(dt, sensor_dt, max_dt, min_dt);
            max_dt = sensor_dt;
            min_dt = sensor_dt;
            last_rtdt_log_ms = now_ms;
        }
#endif

#ifdef RATE_LOOP_TIMING_DEBUG
        motor_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

#if HAL_LOGGING_ENABLED
        // fast logging output
        if (should_log(MASK_LOG_ATTITUDE_FAST)) {
            if (run_decimated_callback(rates.fast_logging_rate, log_loop_count)) {
                log_loop_count = 0;
                rate_controller_log_update();

            }
        } else if (should_log(MASK_LOG_ATTITUDE_MED)) {
            if (run_decimated_callback(rates.medium_logging_rate, log_loop_count)) {
                log_loop_count = 0;
                rate_controller_log_update();
            }
        }
#endif

#ifdef RATE_LOOP_TIMING_DEBUG
        log_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        now_ms = AP_HAL::millis();

        // make sure we have the latest target rate
        target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1,
                                                 DIV_ROUND_INT(ins.get_raw_gyro_rate_hz(), AP::scheduler().get_loop_rate_hz()));
        if (now_ms - last_notch_sample_ms >= 1000 || !was_using_rate_thread) {
            // update the PID notch sample rate at 1Hz if we are
            // enabled at runtime
            last_notch_sample_ms = now_ms;
            attitude_control->set_notch_sample_rate(1.0 / sensor_dt);
#ifdef RATE_LOOP_TIMING_DEBUG
            hal.console->printf("Sample rate %.1f, main loop %u, fast rate %u, med rate %u\n", 1.0 / sensor_dt,
                                 rates.main_loop_rate, rates.fast_logging_rate, rates.medium_logging_rate);
#endif
        }

        // interlock for printing fixed rate active
        if (was_armed != motors->armed()) {
            notify_fixed_rate_active = !was_armed;
            was_armed = motors->armed();
        }

        // Once armed, switch to the fast rate if configured to do so
        if ((rate_decimation != target_rate_decimation || notify_fixed_rate_active)
            && ((get_fast_rate_type() == FastRateType::FAST_RATE_FIXED_ARMED && motors->armed())
                || get_fast_rate_type() == FastRateType::FAST_RATE_FIXED)) {
            rate_decimation = target_rate_decimation;
            rate_controller_set_rates(rate_decimation, rates, false);
            notify_fixed_rate_active = false;
        }

        // check that the CPU is not pegged, if it is drop the attitude rate
        if (now_ms - last_rate_check_ms >= 100
            && (get_fast_rate_type() == FastRateType::FAST_RATE_DYNAMIC
                || (get_fast_rate_type() == FastRateType::FAST_RATE_FIXED_ARMED && !motors->armed())
                || target_rate_decimation > rate_decimation)) {
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
                    rate_controller_set_rates(rate_decimation, rates, true);
                    prev_loop_count = rate_loop_count;
                    rate_loop_count = 0;
                    running_slow = 0;
                }
            } else if (rate_decimation > target_rate_decimation && rate_loop_count > att_rate/10 // ensure 100ms worth of good readings
                && (prev_loop_count > att_rate/10   // ensure there was 100ms worth of good readings at the higher rate
                    || prev_loop_count == 0         // last rate was actually a lower rate so keep going quickly
                    || now_ms - last_rate_increase_ms >= 10000)) { // every 10s retry
                rate_decimation = rate_decimation - 1;

                rate_controller_set_rates(rate_decimation, rates, false);
                prev_loop_count = 0;
                rate_loop_count = 0;
                last_rate_increase_ms = now_ms;
            }
        }

#ifdef RATE_LOOP_TIMING_DEBUG
        timing_count++;
        ctrl_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();

        if (rate_now_us - last_timing_msg_us > 1e6) {
            hal.console->printf("Rate loop timing: gyro=%uus, rate=%uus, motors=%uus, log=%uus, ctrl=%uus\n",
                                unsigned(gyro_sample_time_us/timing_count), unsigned(rate_controller_time_us/timing_count),
                                unsigned(motor_output_us/timing_count), unsigned(log_output_us/timing_count), unsigned(ctrl_output_us/timing_count));
            last_timing_msg_us = rate_now_us;
            timing_count = 0;
            gyro_sample_time_us = rate_controller_time_us = motor_output_us = log_output_us = ctrl_output_us = 0;
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
  update rate controller rates and return the logging rate
*/
void Copter::rate_controller_set_rates(uint8_t rate_decimation, RateControllerRates& rates, bool warn_cpu_high)
{
    const uint32_t attitude_rate = ins.get_raw_gyro_rate_hz() / rate_decimation;
    attitude_control->set_notch_sample_rate(attitude_rate);
    hal.rcout->set_dshot_rate(SRV_Channels::get_dshot_rate(), attitude_rate);
    motors->set_dt_s(1.0f / attitude_rate);
    gcs().send_text(warn_cpu_high ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                    "Rate CPU %s, rate set to %uHz",
                    warn_cpu_high ? "high" : "normal", (unsigned) attitude_rate);
#if HAL_LOGGING_ENABLED
    if (attitude_rate > 1000) {
        rates.fast_logging_rate = calc_gyro_decimation(rate_decimation, 1000);   // 1Khz
    } else {
         rates.fast_logging_rate = calc_gyro_decimation(rate_decimation, AP::scheduler().get_filtered_loop_rate_hz());
    }
    rates.medium_logging_rate = calc_gyro_decimation(rate_decimation, 10);   // 10Hz
#endif
    rates.main_loop_rate = calc_gyro_decimation(rate_decimation, AP::scheduler().get_filtered_loop_rate_hz());
    rates.filter_rate = calc_gyro_decimation(rate_decimation, ins.get_raw_gyro_rate_hz() / 2);
}

// enable the fast rate thread using the provided decimation rate and record the new output rates
void Copter::enable_fast_rate_loop(uint8_t rate_decimation, RateControllerRates& rates)
{
    ins.enable_fast_rate_buffer();
    rate_controller_set_rates(rate_decimation, rates, false);
    hal.rcout->force_trigger_groups(true);
    using_rate_thread = true;
}

// disable the fast rate thread and record the new output rates
void Copter::disable_fast_rate_loop(RateControllerRates& rates)
{
    using_rate_thread = false;
    uint8_t rate_decimation = calc_gyro_decimation(1, AP::scheduler().get_filtered_loop_rate_hz());
    rate_controller_set_rates(rate_decimation, rates, false);
    hal.rcout->force_trigger_groups(false);
    ins.disable_fast_rate_buffer();
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

// run notch update at either loop rate or 200Hz
void Copter::update_dynamic_notch_at_specified_rate_main()
{
    if (using_rate_thread) {
        return;
    }

    update_dynamic_notch_at_specified_rate();
}

#endif // AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
