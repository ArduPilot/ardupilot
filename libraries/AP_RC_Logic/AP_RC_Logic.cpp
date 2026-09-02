#include "AP_RC_Logic.h"

#if AP_RC_LOGIC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL& hal;

// a settling term must hold its new value this long before it is committed
#define AP_RC_LOGIC_DEBOUNCE_MS 40

// Term::var_info lives in AP_RC_Logic_Term.cpp (see the @Path below)

const AP_Param::GroupInfo AP_RC_Logic::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: RC logic engine enable
    // @Description: Enable the RC logic / range-function engine
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 0, AP_RC_Logic, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: 1_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[0],  "1_",  1,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 2_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[1],  "2_",  2,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 3_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[2],  "3_",  3,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 4_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[3],  "4_",  4,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 5_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[4],  "5_",  5,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 6_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[5],  "6_",  6,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 7_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[6],  "7_",  7,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 8_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[7],  "8_",  8,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 9_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[8],  "9_",  9,  AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 10_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[9],  "10_", 10, AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 11_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[10], "11_", 11, AP_RC_Logic, AP_RC_Logic::Term),
    // @Group: 12_
    // @Path: AP_RC_Logic_Term.cpp
    AP_SUBGROUPINFO(terms[11], "12_", 12, AP_RC_Logic, AP_RC_Logic::Term),

    AP_GROUPEND
};

AP_RC_Logic *AP_RC_Logic::_singleton;

AP_RC_Logic::AP_RC_Logic()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    // runtime (non-parameter) state must start cleared
    memset(_states, 0, sizeof(_states));
}

// ARM-class functions (those that arm the vehicle on their HIGH edge) are
// treated conservatively: range terms only (never links or conditions),
// negate ignored, and only honoured with valid RC input.
static bool is_arm_function(RC_Channel::AUX_FUNC f)
{
    switch (f) {
    case RC_Channel::AUX_FUNC::ARMDISARM:
    case RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE:
    case RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP:
        return true;
    default:
        return false;
    }
}

// functions whose LOW edge is a motor-affecting transition that must not be
// emitted implicitly when a slot is released on reconfiguration
static bool low_edge_affects_motors(RC_Channel::AUX_FUNC f)
{
    // arm functions LOW = disarm; ARM_EMERGENCY_STOP LOW also = motor e-stop
    if (is_arm_function(f)) {
        return true;
    }
    // MOTOR_ESTOP LOW clears the e-stop, which can let motors spin
    return f == RC_Channel::AUX_FUNC::MOTOR_ESTOP;
}

bool AP_RC_Logic::eval_condition(Condition c) const
{
    const AP_Notify &notify = AP::notify();
    switch (c) {
    case Condition::RC_FAILSAFE:
        return notify.flags.failsafe_radio;
    case Condition::BATTERY_FAILSAFE:
        return notify.flags.failsafe_battery;
    case Condition::GCS_FAILSAFE:
        return notify.flags.failsafe_gcs;
    case Condition::EKF_FAILSAFE:
        return notify.flags.failsafe_ekf;
    case Condition::ARMED:
        return hal.util->get_soft_armed();
    }
    return false;
}

bool AP_RC_Logic::committed_state(RC_Channel::AUX_FUNC f) const
{
    // func 0 doubles as the "unused slot" marker, so never match on it
    if (f == RC_Channel::AUX_FUNC::DO_NOTHING) {
        return false;
    }
    for (uint8_t i = 0; i < AP_RC_LOGIC_NUM_TERMS; i++) {
        if (_states[i].func == (uint16_t)f) {
            // a link watches whether the function is fully active (HIGH)
            return _states[i].committed_pos == (uint8_t)RC_Channel::AuxSwitchPos::HIGH;
        }
    }
    return false;
}

bool AP_RC_Logic::func_has_link_term(RC_Channel::AUX_FUNC f) const
{
    for (uint8_t i = 0; i < AP_RC_LOGIC_NUM_TERMS; i++) {
        if (terms[i].enabled() && terms[i].func() == f &&
            terms[i].source_type() == Term::SourceType::LINK) {
            return true;
        }
    }
    return false;
}

// raw (pre-debounce) truth of a single term
bool AP_RC_Logic::eval_term(const Term &t) const
{
    bool v = false;
    switch (t.source_type()) {
    case Term::SourceType::RANGE: {
        const int16_t ch = t.source.get();
        if (ch >= 1 && ch <= NUM_RC_CHANNELS) {
            const RC_Channel *c = rc().channel(ch - 1);
            if (c != nullptr) {
                const uint16_t pwm = c->get_radio_in();
                if (pwm >= RC_Channel::RC_MIN_LIMIT_PWM &&
                    pwm <= RC_Channel::RC_MAX_LIMIT_PWM) {
                    v = pwm >= t.pwm_min.get() && pwm <= t.pwm_max.get();
                }
            }
        }
        break;
    }
    case Term::SourceType::LINK: {
        const auto watched = (RC_Channel::AUX_FUNC)t.source.get();
        // Betaflight rule: a link may not reference a link-driven function,
        // and never an arming function. This blocks chains and cycles.
        if (!is_arm_function(watched) && !func_has_link_term(watched)) {
            v = committed_state(watched);
        }
        break;
    }
    case Term::SourceType::CONDITION:
        v = eval_condition((Condition)t.source.get());
        break;
    }
    // never invert an arming term: a negated range would be "active" at
    // centre stick and at boot, which must not be able to request arm
    if (t.negated() && !is_arm_function(t.func())) {
        v = !v;
    }
    return v;
}

AP_RC_Logic::FuncState *AP_RC_Logic::state_for(RC_Channel::AUX_FUNC f)
{
    FuncState *first_free = nullptr;
    for (uint8_t i = 0; i < AP_RC_LOGIC_NUM_TERMS; i++) {
        if (_states[i].func == (uint16_t)f) {
            return &_states[i];
        }
        if (first_free == nullptr && _states[i].func == 0) {
            first_free = &_states[i];
        }
    }
    if (first_free != nullptr) {
        first_free->func = (uint16_t)f;
        first_free->committed_pos = (uint8_t)RC_Channel::AuxSwitchPos::LOW;
        first_free->committed_level = 0;
        first_free->candidate_pos = (uint8_t)RC_Channel::AuxSwitchPos::LOW;
        first_free->candidate_level = 0;
        first_free->since_ms = 0;
    }
    return first_free;
}

void AP_RC_Logic::release_slot(FuncState &st)
{
    if (st.func != 0 && st.committed_pos != (uint8_t)RC_Channel::AuxSwitchPos::LOW) {
        const auto f = (RC_Channel::AUX_FUNC)st.func;
        // Releasing drives the function LOW so latching outputs (relays) do
        // not stay stuck on after a re-target or engine disable. Never do
        // this for functions whose LOW edge affects the motors (arm =
        // disarm, MOTOR_ESTOP LOW = clear e-stop): forcing that on a config
        // change could cut motors in flight or let them spin unexpectedly.
        // Genuine RC-loss disarm is handled by the has_valid_input gate
        // while the term is still active.
        if (!low_edge_affects_motors(f)) {
            rc().run_aux_function(f, RC_Channel::AuxSwitchPos::LOW,
                                  RC_Channel::AuxFuncTrigger::Source::LOGIC, 0);
        }
    }
    st = FuncState{};
}

void AP_RC_Logic::update()
{
    if (!enable) {
        // engine turned off: release any (non-arming) functions we were
        // holding so latching outputs like relays do not stay stuck on
        for (uint8_t i = 0; i < AP_RC_LOGIC_NUM_TERMS; i++) {
            release_slot(_states[i]);
        }
        return;
    }

    const bool rc_valid = rc().has_valid_input();
    const uint32_t now_ms = AP_HAL::millis();

    // reclaim debounce slots whose function is no longer referenced by any
    // enabled term, releasing the function so a runtime re-target frees the
    // slot without leaving a stale committed state or a stuck output
    for (uint8_t i = 0; i < AP_RC_LOGIC_NUM_TERMS; i++) {
        if (_states[i].func == 0) {
            continue;
        }
        bool referenced = false;
        for (uint8_t j = 0; j < AP_RC_LOGIC_NUM_TERMS; j++) {
            if (terms[j].enabled() && (uint16_t)terms[j].func() == _states[i].func) {
                referenced = true;
                break;
            }
        }
        if (!referenced) {
            release_slot(_states[i]);
        }
    }

    for (uint8_t i = 0; i < AP_RC_LOGIC_NUM_TERMS; i++) {
        const Term &ti = terms[i];
        if (!ti.enabled()) {
            continue;
        }
        // evaluate each distinct function once, at its first row
        const RC_Channel::AUX_FUNC f = ti.func();
        bool already_done = false;
        for (uint8_t j = 0; j < i; j++) {
            if (terms[j].enabled() && terms[j].func() == f) {
                already_done = true;
                break;
            }
        }
        if (already_done) {
            continue;
        }

        const bool arm_func = is_arm_function(f);

        // A row may set a specific output level (OPT bit 4 = level mode, bits
        // 5-7 = level index) to drive a multi-level function (e.g. VTX power) to
        // a chosen level. If any row for this function does so the function runs
        // in "selector" mode: the lowest-index active row wins and drives its
        // level; if none is active the output is LOW/off. Otherwise the classic
        // boolean AND/OR combine is used. Arm functions are always strictly
        // boolean (HIGH = arm, LOW = disarm) and never enter selector mode.
        bool selector = false;
        if (!arm_func) {
            for (uint8_t j = i; j < AP_RC_LOGIC_NUM_TERMS; j++) {
                if (terms[j].enabled() && terms[j].func() == f &&
                    terms[j].uses_level()) {
                    selector = true;
                    break;
                }
            }
        }

        RC_Channel::AuxSwitchPos target_pos = RC_Channel::AuxSwitchPos::LOW;
        uint8_t target_level = 0;
        if (selector) {
            for (uint8_t j = i; j < AP_RC_LOGIC_NUM_TERMS; j++) {
                const Term &tj = terms[j];
                if (!tj.enabled() || tj.func() != f) {
                    continue;
                }
                if (eval_term(tj)) {
                    // priority: the first (lowest-index) active row wins. Emit
                    // HIGH plus the row's one-based level so a multi-level
                    // target selects that exact level.
                    target_pos = RC_Channel::AuxSwitchPos::HIGH;
                    target_level = tj.output_level() + 1;
                    break;
                }
            }
        } else {
            // combine all rows for this function using the Betaflight rule:
            // active when all AND terms are true, or any OR term is true.
            bool has_and = false, all_and = true, any_or = false;
            for (uint8_t j = i; j < AP_RC_LOGIC_NUM_TERMS; j++) {
                const Term &tj = terms[j];
                if (!tj.enabled() || tj.func() != f) {
                    continue;
                }
                // arming functions honour range terms only
                if (arm_func && tj.source_type() != Term::SourceType::RANGE) {
                    continue;
                }
                const bool v = eval_term(tj);
                if (tj.combine_is_and()) {
                    has_and = true;
                    all_and = all_and && v;
                } else {
                    any_or = any_or || v;
                }
            }
            bool raw = (has_and && all_and) || any_or;
            // ARM-class functions fail toward disarm without valid RC input.
            if (raw && arm_func && !rc_valid) {
                raw = false;
            }
            target_pos = raw ? RC_Channel::AuxSwitchPos::HIGH
                             : RC_Channel::AuxSwitchPos::LOW;
        }

        FuncState *st = state_for(f);
        if (st == nullptr) {
            continue;  // no free slot (more distinct funcs than terms)
        }

        // debounce: a changed (position, level) must settle before committing
        const uint8_t tpos = (uint8_t)target_pos;
        if (tpos == st->committed_pos && target_level == st->committed_level) {
            st->candidate_pos = tpos;  // nothing pending
            st->candidate_level = target_level;
            continue;
        }
        if (st->candidate_pos != tpos || st->candidate_level != target_level) {
            st->candidate_pos = tpos;
            st->candidate_level = target_level;
            st->since_ms = now_ms;
        } else if (now_ms - st->since_ms >= AP_RC_LOGIC_DEBOUNCE_MS) {
            st->committed_pos = tpos;
            st->committed_level = target_level;
            rc().run_aux_function(f, target_pos,
                                  RC_Channel::AuxFuncTrigger::Source::LOGIC, i,
                                  target_level);
        }
    }
}

namespace AP {
    AP_RC_Logic &rc_logic()
    {
        return *AP_RC_Logic::get_singleton();
    }
};

#endif  // AP_RC_LOGIC_ENABLED
