#include "GCS.h"

#include "AP_ServoRelayEvents/AP_ServoRelayEvents.h"

MAV_RESULT GCS_MAVLINK::handle_servorelay_message(mavlink_command_long_t &packet)
{
    AP_ServoRelayEvents *handler = get_servorelayevents();
    if (handler == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    MAV_RESULT result = MAV_RESULT_FAILED;

    switch (packet.command) {
    case MAV_CMD_DO_SET_SERVO:
        if (handler->do_set_servo(packet.param1, packet.param2)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

    case MAV_CMD_DO_REPEAT_SERVO:
        if (handler->do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4 * 1000)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

    case MAV_CMD_DO_SET_RELAY:
        if (handler->do_set_relay(packet.param1, packet.param2)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
        if (handler->do_repeat_relay(packet.param1, packet.param2, packet.param3 * 1000)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    return result;
}
