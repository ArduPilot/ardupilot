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

/**
   The strategy for roll/pitch autotune is to give the user a AUTOTUNE
   flight mode which behaves just like FBWA, but does automatic
   tuning.
*/

#include "AP_AutoTune.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

extern const AP_HAL::HAL& hal;

// step size for changing FF gains, percentage
#define AUTOTUNE_INCREASE_FF_STEP 12
#define AUTOTUNE_DECREASE_FF_STEP 15

// limits on IMAX
#define AUTOTUNE_MIN_IMAX 0.4
#define AUTOTUNE_MAX_IMAX 0.9

// ratio of I to P
#define AUTOTUNE_I_RATIO 0.75

// time constant of rate trim loop
#define TRIM_TCONST 1.0f

// constructor
AP_AutoTune::AP_AutoTune(ATGains &_gains, ATType _type,
                         const AP_FixedWing &parms,
                         AC_PID &_rpid) :
    current(_gains),
    rpid(_rpid),
    type(_type),
    aparm(parms),
    ff_filter(2)
{}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
# define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define Debug(fmt, args ...)
#endif

/*
  auto-tuning table. This table gives the starting values for key
  tuning parameters based on a user chosen AUTOTUNE_LEVEL parameter
  from 1 to 10. Level 1 is a very soft tune. Level 10 is a very
  aggressive tune.
  Level 0 means use the existing RMAX and TCONST parameters
 */
static const struct {
    float tau;
    float rmax;
} tuning_table[] = {
    { 1.00, 20 },   // level 1
    { 0.90, 30 },   // level 2
    { 0.80, 40 },   // level 3
    { 0.70, 50 },   // level 4
    { 0.60, 60 },   // level 5
    { 0.50, 75 },   // level 6
    { 0.30, 90 },   // level 7
    { 0.2, 120 },   // level 8
    { 0.15, 160 },   // level 9
    { 0.1, 210 },   // level 10
    { 0.1, 300 },   // (yes, it goes to 11)
};

/*
  start an autotune session
*/
void AP_AutoTune::start(void)
{
    running = true;
    state = ATState::IDLE;

    current = restore = last_save = get_gains();

    // do first update of rmax and tau now
    update_rmax();

    dt = AP::scheduler().get_loop_period_s();

    rpid.kIMAX().set(constrain_float(rpid.kIMAX(), AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX));

    // use 0.75Hz filters on the actuator, rate and target to reduce impact of noise
    actuator_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 0.75);
    rate_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 0.75);

    // target filter is a bit broader
    target_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 4);

    ff_filter.reset();
    actuator_filter.reset();
    rate_filter.reset();
    D_limit = 0;
    P_limit = 0;
    ff_count = 0;
    D_set_ms = 0;
    P_set_ms = 0;
    done_count = 0;

    if (!is_positive(rpid.slew_limit())) {
        // we must have a slew limit, default to 150 deg/s
        rpid.slew_limit().set_and_save(150);
    }

    if (current.FF < 0.01) {
        // don't allow for zero FF
        current.FF = 0.01;
        rpid.ff().set(current.FF);
    }

    Debug("START FF -> %.3f\n", rpid.ff().get());
}

/*
  called when we change state to see if we should change gains
 */
void AP_AutoTune::stop(void)
{
    if (running) {
        running = false;
        if (is_positive(D_limit) && is_positive(P_limit)) {
            save_gains();
        } else {
            restore_gains();
        }
    }
}

const char *AP_AutoTune::axis_string(void) const
{
    return axis_string(type);
}

const char *AP_AutoTune::axis_string(ATType _type)
{
    switch (_type) {
    case AUTOTUNE_ROLL:
        return "Roll";
    case AUTOTUNE_PITCH:
        return "Pitch";
    case AUTOTUNE_YAW:
        return "Yaw";
    }
    return "";
}

/*
  one update cycle of the autotuner
 */
void AP_AutoTune::update(AP_PIDInfo &pinfo, float scaler, float angle_err_deg)
{
    if (!running) {
        return;
    }

    // see what state we are in
    ATState new_state = state;
    const float desired_rate = target_filter.apply(pinfo.target);

    // filter actuator without I term so we can take ratios without
    // accounting for trim offsets. We first need to include the I and
    // clip to 45 degrees to get the right value of the real surface
    const float clipped_actuator = constrain_float(pinfo.FF + pinfo.P + pinfo.D + pinfo.DFF + pinfo.I, -45, 45) - pinfo.I;
    const float actuator = actuator_filter.apply(clipped_actuator);
    const float actual_rate = rate_filter.apply(pinfo.actual);

    max_actuator = MAX(max_actuator, actuator);
    min_actuator = MIN(min_actuator, actuator);
    max_rate = MAX(max_rate, actual_rate);
    min_rate = MIN(min_rate, actual_rate);
    max_target = MAX(max_target, desired_rate);
    min_target = MIN(min_target, desired_rate);
    max_P = MAX(max_P, fabsf(pinfo.P));
    max_D = MAX(max_D, fabsf(pinfo.D));
    min_Dmod = MIN(min_Dmod, pinfo.Dmod);
    max_Dmod = MAX(max_Dmod, pinfo.Dmod);

    // update the P and D slew rates, using P and D values from before Dmod was applied
    const float slew_limit_scale = 45.0 / degrees(1);
    slew_limit_max = rpid.slew_limit();
    slew_limit_tau = 1.0;
    slew_limiter_P.modifier((pinfo.P/pinfo.Dmod)*slew_limit_scale, dt);
    slew_limiter_D.modifier((pinfo.D/pinfo.Dmod)*slew_limit_scale, dt);

    // remember maximum slew rates for this cycle
    max_SRate_P = MAX(max_SRate_P, slew_limiter_P.get_slew_rate());
    max_SRate_D = MAX(max_SRate_D, slew_limiter_D.get_slew_rate());

    float att_limit_deg = 0;
    switch (type) {
    case AUTOTUNE_ROLL:
        att_limit_deg = aparm.roll_limit;
        break;
    case AUTOTUNE_PITCH:
        att_limit_deg = MIN(abs(aparm.pitch_limit_max*100),abs(aparm.pitch_limit_min*100))*0.01;
        break;
    case AUTOTUNE_YAW:
        // arbitrary value for yaw angle
        att_limit_deg = 20;
        break;
    }


    // thresholds for when we consider an event to start and end
    const float rate_threshold1 = 0.4 * MIN(att_limit_deg / current.tau.get(), current.rmax_pos);
    const float rate_threshold2 = 0.25 * rate_threshold1;
    bool in_att_demand = fabsf(angle_err_deg) >= 0.3 * att_limit_deg;

    switch (state) {
    case ATState::IDLE:
        if (desired_rate > rate_threshold1 && in_att_demand) {
            new_state = ATState::DEMAND_POS;
        } else if (desired_rate < -rate_threshold1 && in_att_demand) {
            new_state = ATState::DEMAND_NEG;
        }
        break;
    case ATState::DEMAND_POS:
        if (desired_rate < rate_threshold2) {
            new_state = ATState::IDLE;
        }
        break;
    case ATState::DEMAND_NEG:
        if (desired_rate > -rate_threshold2) {
            new_state = ATState::IDLE;
        }
        break;
    }

    const uint32_t now = AP_HAL::millis();

#if HAL_LOGGING_ENABLED
    if (now - last_log_ms >= 40) {
        // log at 25Hz
        const struct log_ATRP pkt {
            LOG_PACKET_HEADER_INIT(LOG_ATRP_MSG),
            time_us : AP_HAL::micros64(),
            type : uint8_t(type),
            state: uint8_t(new_state),
            actuator : actuator,
            P_slew : max_SRate_P,
            D_slew : max_SRate_D,
            FF_single: FF_single,
            FF: current.FF,
            P: current.P,
            I: current.I,
            D: current.D,
            action: uint8_t(action),
            rmax: float(current.rmax_pos.get()),
            tau: current.tau.get()
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
        last_log_ms = now;
    }
#endif

    if (new_state == state) {
        if (state == ATState::IDLE &&
            now - state_enter_ms > 500 &&
            max_Dmod < 0.9) {
            // we've been oscillating while idle, reduce P or D
            const float slew_sum = max_SRate_P + max_SRate_D;
            const float gain_mul = 0.5;
            current.P *= linear_interpolate(gain_mul, 1.0,
                                            max_SRate_P,
                                            slew_sum, 0);
            current.D *= linear_interpolate(gain_mul, 1.0,
                                            max_SRate_D,
                                            slew_sum, 0);
            rpid.kP().set(current.P);
            rpid.kD().set(current.D);
            action = Action::IDLE_LOWER_PD;
            P_limit = MIN(P_limit, current.P);
            D_limit = MIN(D_limit, current.D);
            state_change(state);
        }
        return;
    }

    if (new_state != ATState::IDLE) {
        // starting an event
        min_actuator = max_actuator = min_rate = max_rate = 0;
        state_enter_ms = now;
        state = new_state;
        return;
    }

    if ((state == ATState::DEMAND_POS && max_rate < 0.01 * current.rmax_pos) ||
        (state == ATState::DEMAND_NEG && min_rate > -0.01 * current.rmax_neg)) {
        // we didn't get enough rate
        action = Action::LOW_RATE;
        state_change(ATState::IDLE);
        return;
    }

    if (now - state_enter_ms < 100) {
        // not long enough sample
        action = Action::SHORT;
        state_change(ATState::IDLE);
        return;
    }

    // we've finished an event. calculate the single-event FF value
    if (state == ATState::DEMAND_POS) {
        FF_single = max_actuator / (max_rate * scaler);
    } else {
        FF_single = min_actuator / (min_rate * scaler);
    }

    // apply median filter
    float FF = ff_filter.apply(FF_single);
    ff_count++;

    const float old_FF = rpid.ff();

    // limit size of change in FF
    FF = constrain_float(FF,
                         old_FF*(1-AUTOTUNE_DECREASE_FF_STEP*0.01),
                         old_FF*(1+AUTOTUNE_INCREASE_FF_STEP*0.01));

    // adjust P and D
    float D = rpid.kD();
    float P = rpid.kP();

    if (ff_count == 1) {
        // apply minimum D and P values
        D = MAX(D, 0.0005);
        P = MAX(P, 0.01);
    } else if (ff_count == 4) {
        // we got a good ff estimate, halve P ready to start raising D
        P *= 0.5;
    }

    // see if the slew limiter kicked in
    if (min_Dmod < 1.0 && !is_positive(D_limit)) {
        // oscillation, without D_limit set
        if (max_P > 0.5 * max_D) {
            // lower P and D to get us to a non-oscillating state
            P *= 0.35;
            D *= 0.75;
            action = Action::LOWER_PD;
        } else {
            // set D limit to 30% of current D, remember D limit and start to work on P
            D *= 0.3;
            D_limit = D;
            D_set_ms = now;
            action = Action::LOWER_D;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%sD: %.4f", axis_string(), D_limit);
        }
    } else if (min_Dmod < 1.0) {
        // oscillation, with D_limit set
        if (now - D_set_ms > 2000) {
            // leave 2s for Dmod to settle after lowering D
            if (max_D > 0.8 * max_P) {
                // lower D limit some more
                D *= 0.35;
                D_limit = D;
                D_set_ms = now;
                action = Action::LOWER_D;
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%sD: %.4f", axis_string(), D_limit);
                done_count = 0;
            } else if (now - P_set_ms > 2500) {
                if (is_positive(P_limit)) {
                    // if we've already got a P estimate then don't
                    // reduce as quickly, stopping small spikes at the
                    // later part of the tune from giving us a very
                    // low P gain
                    P *= 0.7;
                } else {
                    P *= 0.35;
                }
                P_limit = P;
                P_set_ms = now;
                action = Action::LOWER_P;
                done_count = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%sP: %.4f", axis_string(), P_limit);
            }
        }
    } else if (ff_count < 4) {
        // we don't have a good FF estimate yet, keep going

    } else if (!is_positive(D_limit)) {
        /* we haven't detected D oscillation yet, keep raising D */
        D *= 1.3;
        action = Action::RAISE_D;
    } else if (!is_positive(P_limit)) {
        /* not oscillating, increase P gain */
        P *= 1.3;
        action = Action::RAISE_PD;
    } else {
        // after getting P_limit we consider the tune done when we
        // have done 3 cycles without reducing P
        if (done_count < 3) {
            if (++done_count == 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s: Finished", axis_string());
                save_gains();
            }
        }
    }

    rpid.ff().set(FF);
    rpid.kP().set(P);
    rpid.kD().set(D);
    if (type == AUTOTUNE_ROLL) {  // for roll set I = smaller of FF or P
        rpid.kI().set(MIN(P, (FF / TRIM_TCONST)));
    } else {                      // for pitch/yaw naturally damped axes) set I usually = FF to get 1 sec I closure
        rpid.kI().set(MAX(P*AUTOTUNE_I_RATIO, (FF / TRIM_TCONST)));
    }

    // setup filters to be suitable for time constant and gyro filter
    // filtering T can  prevent P/D oscillation being seen, so allow the
    // user to switch it off
    if (!has_option(DISABLE_FLTT_UPDATE)) {
        rpid.filt_T_hz().set(10.0/(current.tau * 2 * M_PI));
    }
    rpid.filt_E_hz().set(0);
    // filtering D at the same level as VTOL can allow unwanted oscillations to be seen,
    // so allow the user to switch it off and select their own (usually lower) value
    if (!has_option(DISABLE_FLTD_UPDATE)) {
        rpid.filt_D_hz().set(AP::ins().get_gyro_filter_hz()*0.5);
    }

    current.FF = FF;
    current.P = P;
    current.I = rpid.kI().get();
    current.D = D;

    Debug("FPID=(%.3f, %.3f, %.3f, %.3f) Dmod=%.2f\n",
          rpid.ff().get(),
          rpid.kP().get(),
          rpid.kI().get(),
          rpid.kD().get(),
          min_Dmod);

    // move rmax and tau towards target
    update_rmax();

    state_change(new_state);
}

/*
  record a state change
 */
void AP_AutoTune::state_change(ATState new_state)
{
    min_Dmod = 1;
    max_Dmod = 0;
    max_SRate_P = 1;
    max_SRate_D = 1;
    max_P = max_D = 0;
    state = new_state;
    state_enter_ms = AP_HAL::millis();
}

/*
  save a float if it has changed
 */
void AP_AutoTune::save_float_if_changed(AP_Float &v, float old_value)
{
    if (!is_equal(old_value, v.get())) {
        v.save();
    }
}

/*
  save a int16_t if it has changed
 */
void AP_AutoTune::save_int16_if_changed(AP_Int16 &v, int16_t old_value)
{
    if (old_value != v.get()) {
        v.save();
    }
}


/*
  save a set of gains
 */
void AP_AutoTune::save_gains(void)
{
    const auto &v = last_save;
    save_float_if_changed(current.tau, v.tau);
    save_int16_if_changed(current.rmax_pos, v.rmax_pos);
    save_int16_if_changed(current.rmax_neg, v.rmax_neg);
    save_float_if_changed(rpid.ff(), v.FF);
    save_float_if_changed(rpid.kP(), v.P);
    save_float_if_changed(rpid.kI(), v.I);
    save_float_if_changed(rpid.kD(), v.D);
    save_float_if_changed(rpid.kIMAX(), v.IMAX);
    save_float_if_changed(rpid.filt_T_hz(), v.flt_T);
    save_float_if_changed(rpid.filt_E_hz(), v.flt_E);
    save_float_if_changed(rpid.filt_D_hz(), v.flt_D);
    last_save = get_gains();
}

/*
  get gains with PID components
 */
AP_AutoTune::ATGains AP_AutoTune::get_gains(void)
{
    ATGains ret = current;
    ret.FF = rpid.ff();
    ret.P = rpid.kP();
    ret.I = rpid.kI();
    ret.D = rpid.kD();
    ret.IMAX = rpid.kIMAX();
    ret.flt_T = rpid.filt_T_hz();
    ret.flt_E = rpid.filt_E_hz();
    ret.flt_D = rpid.filt_D_hz();
    return ret;
}

/*
  set gains with PID components
 */
void AP_AutoTune::restore_gains(void)
{
    current = restore;
    rpid.ff().set(restore.FF);
    rpid.kP().set(restore.P);
    rpid.kI().set(restore.I);
    rpid.kD().set(restore.D);
    rpid.kIMAX().set(restore.IMAX);
    rpid.filt_T_hz().set(restore.flt_T);
    rpid.filt_E_hz().set(restore.flt_E);
    rpid.filt_D_hz().set(restore.flt_D);
}

/*
  update RMAX and TAU parameters on each step. We move them gradually
  towards the target to allow for a user going straight to a level 10
  tune while starting with a poorly tuned plane
 */
void AP_AutoTune::update_rmax(void)
{
    uint8_t level = constrain_int32(aparm.autotune_level, 0, ARRAY_SIZE(tuning_table));

    int16_t target_rmax;
    float target_tau;

    if (level == 0) {
        // this level means to keep current values of RMAX and TCONST
        target_rmax = constrain_float(current.rmax_pos, 20, 720);
        target_tau = constrain_float(current.tau, 0.1, 2);
    } else {
        target_rmax = tuning_table[level-1].rmax;
        target_tau = tuning_table[level-1].tau;
        if (type == AUTOTUNE_PITCH) {
            // 50% longer time constant on pitch
            target_tau *= 1.5;
        }
    }

    if (level > 0 && is_positive(current.FF)) {
        const float invtau = ((1.0f / target_tau) + (current.I / current.FF));
        if (is_positive(invtau)) {
            target_tau = MAX(target_tau,1.0f / invtau);
        }
    }

    if (current.rmax_pos == 0) {
        // conservative initial value
        current.rmax_pos.set(75);
    }
    // move RMAX by 20 deg/s per step
    current.rmax_pos.set(constrain_int32(target_rmax,
                                         current.rmax_pos.get()-20,
                                         current.rmax_pos.get()+20));

    if (level != 0 || current.rmax_neg.get() == 0) {
        current.rmax_neg.set(current.rmax_pos.get());
    }

    // move tau by max 15% per loop
    current.tau.set(constrain_float(target_tau,
                                    current.tau*0.85,
                                    current.tau*1.15));
}
