#include <AP_Common/AP_Common.h>
#include "AP_Relay_Params.h"

const AP_Param::GroupInfo AP_Relay_Params::var_info[] = {
    // @Param: FUNCTION
    // @DisplayName: Relay function
    // @Description: The function the relay channel is mapped to.
    // @Values: 0:None,1:Relay,2:Ignition,3:Starter
    // @User: Standard
    AP_GROUPINFO_FLAGS("FUNCTION", 1, AP_Relay_Params, function, (float)Function::none, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN
    // @DisplayName: Relay pin
    // @Description: Digital pin number for relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,62:BBBMini Pin P8.13,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    // @User: Standard
    AP_GROUPINFO("PIN", 2, AP_Relay_Params, pin, -1),
    
    // @Param: DEFAULT
    // @DisplayName: Relay default state
    // @Description: Should the relay default to on or off, this only applies to function Relay. All other uses will pick the appropriate default output state.
    // @Values: 0: Off,1:On,2:NoChange
    // @User: Standard
    AP_GROUPINFO("DEFAULT", 3, AP_Relay_Params, default_state, (float)DefaultState::OFF),

    AP_GROUPEND

};

AP_Relay_Params::AP_Relay_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
