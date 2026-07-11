#pragma once

#include "AP_RC_Logic_config.h"

#if AP_RC_LOGIC_ENABLED

#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

/*
  Table-driven engine that activates RC_Channel AUX_FUNCs from RC channel
  PWM ranges and boolean combinations. See README.md for the full design
  and the parameter-schema contract shared with the configurator.
 */
class AP_RC_Logic {
public:
    AP_RC_Logic();

    CLASS_NO_COPY(AP_RC_Logic);

    static AP_RC_Logic *get_singleton() { return _singleton; }

    // evaluate all terms and drive aux functions; call once per RC frame
    void update();

    static const struct AP_Param::GroupInfo var_info[];

    // AP-native conditions selectable by a CONDITION source term. These map
    // directly onto existing ArduPilot state, not any Betaflight concept.
    enum class Condition : uint8_t {
        RC_FAILSAFE      = 0,
        BATTERY_FAILSAFE = 1,
        GCS_FAILSAFE     = 2,
        EKF_FAILSAFE     = 3,
        ARMED            = 4,
    };

    // one row of the logic table
    class Term {
    public:
        Term();
        static const struct AP_Param::GroupInfo var_info[];

        // source category held in bits 0-1 of options
        enum class SourceType : uint8_t {
            RANGE     = 0,   // SRC is an RC channel, active when MIN<=pwm<=MAX
            LINK      = 1,   // SRC is an AUX_FUNC, active when that func is active
            CONDITION = 2,   // SRC is a condition id (failsafe, etc.)
        };

        bool enabled() const { return function.get() != 0; }
        RC_Channel::AUX_FUNC func() const {
            return (RC_Channel::AUX_FUNC)function.get();
        }
        SourceType source_type() const {
            return (SourceType)(options.get() & 0x3);
        }
        bool combine_is_and() const { return (options.get() & 0x4) != 0; }
        bool negated() const { return (options.get() & 0x8) != 0; }
        // Output aux position emitted when this row is active, encoded in
        // options bits 4-5: 0=HIGH (default), 1=MIDDLE, 2=LOW. This lets a row
        // drive a multi-position function (e.g. VTX power low/mid/high) to a
        // specific level instead of a plain on/off. Any row with a non-default
        // (non-HIGH) output position puts the whole function into "selector"
        // mode: the lowest-index active row wins and drives its position.
        RC_Channel::AuxSwitchPos output_position() const {
            switch ((options.get() >> 4) & 0x3) {
            case 1:  return RC_Channel::AuxSwitchPos::MIDDLE;
            case 2:  return RC_Channel::AuxSwitchPos::LOW;
            default: return RC_Channel::AuxSwitchPos::HIGH;
            }
        }
        bool has_output_position() const { return ((options.get() >> 4) & 0x3) != 0; }

        AP_Int16 function;   // target AUX_FUNC (0 = row disabled)
        AP_Int16 options;    // packed: type(0-1), combine(2), negate(3), outpos(4-5)
        AP_Int16 source;     // channel / watched func / condition id
        AP_Int16 pwm_min;
        AP_Int16 pwm_max;
    };

private:
    static AP_RC_Logic *_singleton;

    AP_Int8 enable;
    Term terms[AP_RC_LOGIC_NUM_TERMS];

    // per-function debounce + committed state, keyed by target AUX_FUNC
    struct FuncState {
        uint16_t func;         // AUX_FUNC, 0 = slot unused
        uint8_t committed_pos; // last emitted position (RC_Channel::AuxSwitchPos)
        uint8_t candidate_pos; // position currently settling
        uint32_t since_ms;     // when the candidate first appeared
    } _states[AP_RC_LOGIC_NUM_TERMS];

    // raw evaluation of a single term (no debounce)
    bool eval_term(const Term &t) const;
    bool eval_condition(Condition c) const;
    // committed state of a watched function, for link terms
    bool committed_state(RC_Channel::AUX_FUNC f) const;
    // true if any enabled term targets f with a LINK source
    bool func_has_link_term(RC_Channel::AUX_FUNC f) const;
    // find or allocate the debounce slot for a function
    FuncState *state_for(RC_Channel::AUX_FUNC f);

    // drive a slot's function to LOW (unless its LOW edge affects the motors)
    // and clear the slot; used when a function stops being referenced
    void release_slot(FuncState &st);
};

namespace AP {
    AP_RC_Logic &rc_logic();
};

#endif  // AP_RC_LOGIC_ENABLED
