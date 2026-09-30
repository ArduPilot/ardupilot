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
        // Output level selection for multi-level targets. Options bit 4 enables
        // level mode; bits 5-7 hold a zero-based level index. In level mode an
        // active row drives the target function to that specific level (for VTX
        // power, the one-based table power index) instead of a plain on/off.
        // Any row using level mode puts the whole function into "selector" mode:
        // the lowest-index active row wins and drives its level.
        bool uses_level() const { return (options.get() & 0x10) != 0; }
        uint8_t output_level() const { return (options.get() >> 5) & 0x7; }

        AP_Int16 function;   // target AUX_FUNC (0 = row disabled)
        AP_Int16 options;    // packed: type(0-1), combine(2), negate(3), levelmode(4), level(5-7)
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
        uint16_t func;           // AUX_FUNC, 0 = slot unused
        uint8_t committed_pos;   // last emitted position (RC_Channel::AuxSwitchPos)
        uint8_t committed_level; // last emitted level (0 = none)
        uint8_t candidate_pos;   // position currently settling
        uint8_t candidate_level; // level currently settling
        uint32_t since_ms;       // when the candidate first appeared
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
