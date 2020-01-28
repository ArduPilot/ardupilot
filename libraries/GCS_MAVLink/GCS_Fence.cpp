#include "GCS.h"

#include <AC_Fence/AC_Fence.h>

MAV_RESULT GCS_MAVLINK::handle_command_do_fence_enable(const mavlink_command_long_t &packet)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    switch ((uint16_t)packet.param1) {
    case 0:
        fence->enable(false);
        return MAV_RESULT_ACCEPTED;
    case 1:
        fence->enable(true);
        return MAV_RESULT_ACCEPTED;
    default:
        return MAV_RESULT_FAILED;
    }
}

void GCS_MAVLINK::handle_fence_message(const mavlink_message_t &msg)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }
    // send or receive fence points with GCS
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_FENCE_POINT:
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        fence->polyfence().handle_msg(*this, msg);
        break;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled common fence message");
#endif
        break;
    }
}

// fence_send_mavlink_status - send fence status to ground station
void GCS_MAVLINK::send_fence_status() const
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }
    if (!fence->enabled()) {
        return;
    }

    // traslate fence library breach types to mavlink breach types
    uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
    const uint8_t breaches = fence->get_breaches();
    if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
        mavlink_breach_type = FENCE_BREACH_MAXALT;
    }
    if ((breaches & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) != 0) {
        mavlink_breach_type = FENCE_BREACH_BOUNDARY;
    }

    // send status
    mavlink_msg_fence_status_send(chan,
                                  static_cast<int8_t>(fence->get_breaches() != 0),
                                  fence->get_breach_count(),
                                  mavlink_breach_type,
                                  fence->get_breach_time(),
                                  0);
}
