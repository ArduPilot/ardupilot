#include "AP_RC_Logic.h"

#if AP_RC_LOGIC_ENABLED

// per-term parameters, kept in their own file so the parent @Group @Path can
// point here rather than at the file that declares the group (which would
// make the parameter documentation parser recurse). @CopyFieldsFrom pulls the
// full AUX_FUNC value list into FUNC so the configurator and param docs stay
// in sync with RCx_OPTION.
const AP_Param::GroupInfo AP_RC_Logic::Term::var_info[] = {
    // @Param: FUNC
    // @DisplayName: RC logic target function
    // @Description: Auxiliary function this table row activates (0 disables the row)
    // @CopyFieldsFrom: RC1_OPTION
    AP_GROUPINFO_FLAGS("FUNC", 1, AP_RC_Logic::Term, function, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: OPT
    // @DisplayName: RC logic term options
    // @Description: Packed options for this row
    // @Bitmask: 0:SourceTypeBit0,1:SourceTypeBit1,2:CombineAND,3:Negate
    // @User: Advanced
    AP_GROUPINFO("OPT", 2, AP_RC_Logic::Term, options, 0),

    // @Param: SRC
    // @DisplayName: RC logic term source
    // @Description: Meaning depends on source type - RC channel (1-16) for a range term, watched function for a link term, or condition id for a condition term
    // @User: Advanced
    AP_GROUPINFO("SRC", 3, AP_RC_Logic::Term, source, 0),

    // @Param: MIN
    // @DisplayName: RC logic range low
    // @Description: Lower PWM bound for a range term
    // @Units: PWM
    // @Range: 800 2200
    // @User: Advanced
    AP_GROUPINFO("MIN", 4, AP_RC_Logic::Term, pwm_min, 1700),

    // @Param: MAX
    // @DisplayName: RC logic range high
    // @Description: Upper PWM bound for a range term
    // @Units: PWM
    // @Range: 800 2200
    // @User: Advanced
    AP_GROUPINFO("MAX", 5, AP_RC_Logic::Term, pwm_max, 2100),

    AP_GROUPEND
};

AP_RC_Logic::Term::Term()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_RC_LOGIC_ENABLED
