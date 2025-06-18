#include "AP_Periph.h"

#if AP_PERIPH_RELAY_ENABLED

#include <dronecan_msgs.h>

void AP_Periph_FW::handle_hardpoint_command(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_hardpoint_Command cmd {};
    if (uavcan_equipment_hardpoint_Command_decode(transfer, &cmd)) {
        // Failed to decode
        return;
    }

    // Command must be 0 or 1, other values may be supported in the future
    // rejecting them now ensures no change in behaviour
    if ((cmd.command != 0) && (cmd.command != 1)) {
        return;
    }

    // Translate hardpoint ID to relay function
    AP_Relay_Params::FUNCTION fun;
    switch (cmd.hardpoint_id) {
        case 0 ... 15:
            // 0 to 15 are continuous
            fun = AP_Relay_Params::FUNCTION(cmd.hardpoint_id + (uint8_t)AP_Relay_Params::FUNCTION::DroneCAN_HARDPOINT_0);
            break;

        default:
            // ID not supported
            return;
    }

    // Set relay
    relay.set(fun, cmd.command);

}
#endif
