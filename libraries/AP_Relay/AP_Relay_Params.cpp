#include <AP_Common/AP_Common.h>
#include "AP_Relay_Params.h"

const AP_Param::GroupInfo AP_Relay_Params::var_info[] = {
    // @Param: FUNCTION
    // @DisplayName: Relay function
    // @Description: The function the relay channel is mapped to.
    // @Values{Copter, Rover, Plane, Blimp,Sub}: 0:None
    // @Values{Copter, Rover, Plane, Blimp,Sub}: 1:Relay
    // @Values{Plane}: 2:Ignition
    // @Values{Plane, Copter}: 3:Parachute
    // @Values{Copter, Rover, Plane, Blimp,Sub}: 4:Camera
    // @Values{Rover}: 5:Bushed motor reverse 1 throttle or throttle-left or omni motor 1
    // @Values{Rover}: 6:Bushed motor reverse 2 throttle-right or omni motor 2
    // @Values{Rover}: 7:Bushed motor reverse 3 omni motor 3
    // @Values{Rover}: 8:Bushed motor reverse 4 omni motor 4
    // @Values{Plane}: 9:ICE Starter
    // @Values{AP_Periph}: 10: DroneCAN Hardpoint ID 0
    // @Values{AP_Periph}: 11: DroneCAN Hardpoint ID 1
    // @Values{AP_Periph}: 12: DroneCAN Hardpoint ID 2
    // @Values{AP_Periph}: 13: DroneCAN Hardpoint ID 3
    // @Values{AP_Periph}: 14: DroneCAN Hardpoint ID 4
    // @Values{AP_Periph}: 15: DroneCAN Hardpoint ID 5
    // @Values{AP_Periph}: 16: DroneCAN Hardpoint ID 6
    // @Values{AP_Periph}: 17: DroneCAN Hardpoint ID 7
    // @Values{AP_Periph}: 18: DroneCAN Hardpoint ID 8
    // @Values{AP_Periph}: 19: DroneCAN Hardpoint ID 9
    // @Values{AP_Periph}: 20: DroneCAN Hardpoint ID 10
    // @Values{AP_Periph}: 21: DroneCAN Hardpoint ID 11
    // @Values{AP_Periph}: 22: DroneCAN Hardpoint ID 12
    // @Values{AP_Periph}: 23: DroneCAN Hardpoint ID 13
    // @Values{AP_Periph}: 24: DroneCAN Hardpoint ID 14
    // @Values{AP_Periph}: 25: DroneCAN Hardpoint ID 15

    // @User: Standard
    AP_GROUPINFO_FLAGS("FUNCTION", 1, AP_Relay_Params, function, (float)FUNCTION::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN
    // @DisplayName: Relay pin
    // @Description: Digital pin number for relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,62:BBBMini Pin P8.13,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    // @Values: 1000: DroneCAN Hardpoint ID 0
    // @Values: 1001: DroneCAN Hardpoint ID 1
    // @Values: 1002: DroneCAN Hardpoint ID 2
    // @Values: 1003: DroneCAN Hardpoint ID 3
    // @Values: 1004: DroneCAN Hardpoint ID 4
    // @Values: 1005: DroneCAN Hardpoint ID 5
    // @Values: 1006: DroneCAN Hardpoint ID 6
    // @Values: 1007: DroneCAN Hardpoint ID 7
    // @Values: 1008: DroneCAN Hardpoint ID 8
    // @Values: 1009: DroneCAN Hardpoint ID 9
    // @Values: 1010: DroneCAN Hardpoint ID 10
    // @Values: 1011: DroneCAN Hardpoint ID 11
    // @Values: 1012: DroneCAN Hardpoint ID 12
    // @Values: 1013: DroneCAN Hardpoint ID 13
    // @Values: 1014: DroneCAN Hardpoint ID 14
    // @Values: 1015: DroneCAN Hardpoint ID 15
    // @User: Standard
    AP_GROUPINFO("PIN", 2, AP_Relay_Params, pin, -1),

    // @Param: DEFAULT
    // @DisplayName: Relay default state
    // @Description: Should the relay default to on or off, this only applies to RELAYx_FUNC "Relay" (1). All other uses will pick the appropriate default output state from within the controlling function's parameters.
    // @Values: 0: Off,1:On,2:NoChange
    // @User: Standard
    AP_GROUPINFO("DEFAULT", 3, AP_Relay_Params, default_state, (float)DefaultState::OFF),

    AP_GROUPEND

};

AP_Relay_Params::AP_Relay_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
