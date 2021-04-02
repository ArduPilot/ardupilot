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

   While the user is flying in AUTOTUNE the gains are saved every 10
   seconds, but the saved gains are not the current gains, instead it
   saves the gains from 10s ago. When the user exits AUTOTUNE the
   gains are restored from 10s ago.

   This allows the user to fly as much as they want in AUTOTUNE mode,
   and if they are ever unhappy they just exit the mode. If they stay
   in AUTOTUNE for more than 10s then their gains will have changed.

   Using this approach users don't need any special switches, they
   just need to be able to enter and exit AUTOTUNE mode
*/

#include "AP_AutoTune.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// time in milliseconds between autotune saves
#define AUTOTUNE_SAVE_PERIOD 10000U

// step size for increasing gains, percentage
#define AUTOTUNE_INCREASE_FF_STEP 12
#define AUTOTUNE_INCREASE_PD_STEP 5

// step size for increasing gains when low impact, percentage
#define AUTOTUNE_INCREASE_PD_LOW_STEP 30

// step size for decreasing gains, percentage
#define AUTOTUNE_DECREASE_FF_STEP 15
#define AUTOTUNE_DECREASE_PD_STEP 20

// limits on IMAX
#define AUTOTUNE_MIN_IMAX 0.4
#define AUTOTUNE_MAX_IMAX 0.9

// ratio of I to P
#define AUTOTUNE_I_RATIO 0.75

// overshoot threshold
#define AUTOTUNE_OVERSHOOT 1.1

// constructor
AP_AutoTune::AP_AutoTune(ATGains &_gains, ATType _type,
                         const AP_Vehicle::FixedWing &parms,
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
    { 0.25, 90 },   // level 7
    { 0.12, 120 },   // level 8
    { 0.06, 160 },   // level 9
    { 0.03, 210 },   // level 10
    { 0.01, 300 },   // (yes, it goes to 11)
};

/*
  start an autotune session
*/
void AP_AutoTune::start(void)
{
    running = true;
    state = ATState::IDLE;
    uint32_t now = AP_HAL::millis();

    last_save_ms = now;

    restore = last_save = get_gains(current);

    // do first update of rmax and tau now
    update_rmax();

    rpid.kIMAX().set(constrain_float(rpid.kIMAX(), AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX));

    next_save = current;

    // use 2Hz filters on the actuator and rate to reduce impact of noise
    actuator_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 2);
    rate_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 2);

    // scale slew limit to more agressively find oscillations during autotune
    rpid.set_slew_limit_scale(1.5*45);

    Debug("START FF -> %.3f\n", rpid.ff().get());
}

/*
  called when we change state to see if we should change gains
 */
void AP_AutoTune::stop(void)
{
    if (running) {
        running = false;
        save_gains(restore);
        rpid.set_slew_limit_scale(45);
    }
}


// @LoggerMessage: ATRP
// @Description: Plane AutoTune
// @Vehicles: Plane
// @Field: TimeUS: Time since system startup
// @Field: Axis: tuning axis
// @Field: State: tuning state
// @Field: Sur: control surface deflection
// @Field: Tar: target rate
// @Field: FF0: FF value single sample
// @Field: FF: FF value
// @Field: P: P value
// @Field: D: D value
// @Field: Action: action taken

/*
  one update cycle of the autotuner
 */
void AP_AutoTune::update(AP_Logger::PID_Info &pinfo, float scaler)
{
    if (!running) {
        return;
    }
    check_save();
    // see what state we are in
    ATState new_state = state;
    const float desired_rate = pinfo.target;
    // filter actuator without I term
    const float actuator = actuator_filter.apply(pinfo.FF + pinfo.P + pinfo.D);
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

    int16_t att_limit_cd;
    if (type == AUTOTUNE_ROLL) {
        att_limit_cd = aparm.roll_limit_cd;
    } else {
        att_limit_cd = MIN(abs(aparm.pitch_limit_max_cd),abs(aparm.pitch_limit_min_cd));
    }
    const float rate_threshold1 = 0.75 * MIN(att_limit_cd * 0.01 / current.tau.get(), current.rmax_pos);
    const float rate_threshold2 = 0.25 * rate_threshold1;

    switch (state) {
    case ATState::IDLE:
        if (desired_rate > rate_threshold1) {
            new_state = ATState::DEMAND_POS;
        } else if (desired_rate < -rate_threshold1) {
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

    // unfortunately the LoggerDocumentation test doesn't
    // like two different log msgs in one Write call
    AP::logger().Write(
        "ATRP",
        "TimeUS,Axis,State,Sur,Tar,FF0,FF,P,D,Action",
        "s#-dk-----",
        "F--000000-",
        "QBBffffffB",
        AP_HAL::micros64(),
        unsigned(type),
        unsigned(new_state),
        actuator,
        desired_rate,
        FF_single,
        current.FF,
        current.P,
        current.D,
        unsigned(action));

    if (new_state == state) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

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
        state = ATState::IDLE;
        action = Action::LOW_RATE;
        min_Dmod = 1;
        max_P = max_D = 0;
        return;
    }

    if (now - state_enter_ms < 100) {
        // not long enough sample
        state = ATState::IDLE;
        action = Action::SHORT;
        min_Dmod = 1;
        max_P = max_D = 0;
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

    const float old_FF = rpid.ff();

    // limit size of change in FF
    FF = constrain_float(FF,
                         old_FF*(1-AUTOTUNE_DECREASE_FF_STEP*0.01),
                         old_FF*(1+AUTOTUNE_INCREASE_FF_STEP*0.01));

    // did the P or D components go over 15% of total actuator?
    const float abs_actuator = MAX(max_actuator, fabsf(min_actuator));
    const float PD_high = 0.15 * abs_actuator;
    bool PD_significant = (max_P > PD_high || max_D > PD_high);

    // see if we overshot
    bool overshot = (state == ATState::DEMAND_POS)?
        (max_rate > max_target*AUTOTUNE_OVERSHOOT):
        (min_rate < min_target*AUTOTUNE_OVERSHOOT);

    // adjust P and D
    float D = rpid.kD();
    float P = rpid.kP();

    D = MAX(D, 0.0005);
    P = MAX(P, 0.01);

    // if the slew limiter kicked in or
    if (min_Dmod < 1.0 || (overshot && PD_significant)) {
        // we're overshooting or oscillating, decrease gains. We
        // assume the gain that needs to be reduced is the one that
        // peaked at a higher value
        if (max_P < max_D) {
            D *= (100 - AUTOTUNE_DECREASE_PD_STEP)*0.01;
        } else {
            P *= (100 - AUTOTUNE_DECREASE_PD_STEP)*0.01;
        }
        action = Action::LOWER_PD;
    } else {
        const float low_PD = 0.05 * MAX(max_actuator, fabsf(min_actuator));
        // not oscillating or overshooting, increase the gains
        if (max_P < low_PD) {
            // P is very small, increase rapidly
            P *= (100 + AUTOTUNE_INCREASE_PD_LOW_STEP)*0.01;
        } else {
            P *= (100 + AUTOTUNE_INCREASE_PD_STEP)*0.01;
        }
        if (max_D < low_PD) {
            // D is very small, increase rapidly
            D *= (100 + AUTOTUNE_INCREASE_PD_LOW_STEP)*0.01;
        } else {
            D *= (100 + AUTOTUNE_INCREASE_PD_STEP)*0.01;
        }
        action = Action::RAISE_PD;
    }


    rpid.ff().set(FF);
    rpid.kP().set(P);
    rpid.kD().set(D);
    rpid.kI().set(P*AUTOTUNE_I_RATIO);
    current.FF = FF;
    current.P = P;
    current.I = rpid.kI().get();
    current.D = D;

    Debug("FPID=(%.3f, %.3f, %.3f, %.3f)\n",
          rpid.ff().get(),
          rpid.kP().get(),
          rpid.kI().get(),
          rpid.kD().get());

    // move rmax and tau towards target
    update_rmax();

    min_Dmod = 1;
    max_P = max_D = 0;
    state = new_state;
    state_enter_ms = now;
}

/*
  see if we should save new values
 */
void AP_AutoTune::check_save(void)
{
    if (AP_HAL::millis() - last_save_ms < AUTOTUNE_SAVE_PERIOD) {
        return;
    }

    // save the next_save values, which are the autotune value from
    // the last save period. This means the pilot has
    // AUTOTUNE_SAVE_PERIOD milliseconds to decide they don't like the
    // gains and switch out of autotune
    ATGains tmp = get_gains(current);

    save_gains(next_save);
    last_save = next_save;

    // restore our current gains
    set_gains(tmp);

    // if the pilot exits autotune they get these saved values
    restore = next_save;

    // the next values to save will be the ones we are flying now
    next_save = tmp;
    last_save_ms = AP_HAL::millis();
}

/*
  set a float and save a float if it has changed by more than
  0.1%. This reduces the number of insignificant EEPROM writes
 */
void AP_AutoTune::save_float_if_changed(AP_Float &v, float value)
{
    float old_value = v.get();
    v.set(value);
    if (value <= 0 || fabsf((value-old_value)/value) > 0.001f) {
        v.save();
    }
}

/*
  set a int16 and save if changed
 */
void AP_AutoTune::save_int16_if_changed(AP_Int16 &v, int16_t value)
{
    int16_t old_value = v.get();
    v.set(value);
    if (old_value != v.get()) {
        v.save();
    }
}


/*
  save a set of gains
 */
void AP_AutoTune::save_gains(const ATGains &v)
{
    ATGains tmp = current;
    current = last_save;
    save_float_if_changed(current.tau, v.tau);
    save_int16_if_changed(current.rmax_pos, v.rmax_pos);
    save_int16_if_changed(current.rmax_neg, v.rmax_neg);
    save_float_if_changed(rpid.ff(), v.FF);
    save_float_if_changed(rpid.kP(), v.P);
    save_float_if_changed(rpid.kI(), v.I);
    save_float_if_changed(rpid.kD(), v.D);
    save_float_if_changed(rpid.kIMAX(), v.IMAX);
    last_save = get_gains(current);
    current = tmp;
}

/*
  get gains with PID components
 */
AP_AutoTune::ATGains AP_AutoTune::get_gains(const ATGains &v)
{
    ATGains ret = v;
    ret.FF = rpid.ff();
    ret.P = rpid.kP();
    ret.I = rpid.kI();
    ret.D = rpid.kD();
    ret.IMAX = rpid.kIMAX();
    return ret;
}

/*
  set gains with PID components
 */
void AP_AutoTune::set_gains(const ATGains &v)
{
    current = v;
    rpid.ff().set(v.FF);
    rpid.kP().set(v.P);
    rpid.kI().set(v.I);
    rpid.kD().set(v.D);
    rpid.kIMAX().set(v.IMAX);
}

/*
  update RMAX and TAU parameters on each step. We move them gradually
  towards the target to allow for a user going straight to a level 10
  tune while starting with a poorly tuned plane
 */
void AP_AutoTune::update_rmax(void)
{
    uint8_t level = constrain_int32(aparm.autotune_level, 1, ARRAY_SIZE(tuning_table));

    int16_t target_rmax = tuning_table[level-1].rmax;
    float target_tau = tuning_table[level-1].tau;

    if (current.rmax_pos == 0) {
        // conservative initial value
        current.rmax_pos.set(75);
    }
    // move RMAX by 20 deg/s per step
    current.rmax_pos.set(constrain_int32(target_rmax,
                                         current.rmax_pos.get()-20,
                                         current.rmax_pos.get()+20));
    current.rmax_neg.set(current.rmax_pos.get());

    // move tau by max 15% per loop
    current.tau.set(constrain_float(target_tau,
                                    current.tau*0.85,
                                    current.tau*1.15));
}
