#include "GCS.h"

#include <AC_Fence/AC_Fence.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

MAV_RESULT GCS_MAVLINK::handle_command_do_fence_enable(const mavlink_command_long_t &packet)
{
#if AP_FENCE_ENABLED
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    switch ((uint16_t)packet.param1) {
    case 0: // disable fence
        fence->enable(false);
        return MAV_RESULT_ACCEPTED;
    case 1: // enable fence
        if (!fence->present())
        {
            return MAV_RESULT_FAILED;
        }
    
        fence->enable(true);
        return MAV_RESULT_ACCEPTED;
    case 2: // disable fence floor only
        fence->disable_floor();
        return MAV_RESULT_ACCEPTED;
    default:
        return MAV_RESULT_FAILED;
    }
#else
    return MAV_RESULT_UNSUPPORTED;
#endif // AP_FENCE_ENABLED
}

void GCS_MAVLINK::handle_fence_message(const mavlink_message_t &msg)
{
#if AP_FENCE_ENABLED
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
#endif // AP_FENCE_ENABLED
}

// fence_send_mavlink_status - send fence status to ground station
void GCS_MAVLINK::send_fence_status() const
{
#if AP_FENCE_ENABLED
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
    if ((breaches & AC_FENCE_TYPE_ALT_MIN) != 0) {
        mavlink_breach_type = FENCE_BREACH_MINALT;
    }
    if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
        mavlink_breach_type = FENCE_BREACH_MAXALT;
    }
    if ((breaches & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) != 0) {
        mavlink_breach_type = FENCE_BREACH_BOUNDARY;
    }

    // report on Avoidance liminting
    uint8_t breach_mitigation = FENCE_MITIGATE_UNKNOWN;
#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    const AC_Avoid* avoid =  AC_Avoid::get_singleton();
    if (avoid != nullptr) {
        if (avoid->limits_active()) {
            breach_mitigation = FENCE_MITIGATE_VEL_LIMIT;
        } else {
            breach_mitigation = FENCE_MITIGATE_NONE;
        }
    }
#endif

    // send status
    mavlink_msg_fence_status_send(chan,
                                  static_cast<int8_t>(fence->get_breaches() != 0),
                                  fence->get_breach_count(),
                                  mavlink_breach_type,
                                  fence->get_breach_time(),
                                  breach_mitigation);
#endif // AP_FENCE_ENABLED
}
