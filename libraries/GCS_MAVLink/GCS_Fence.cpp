#include "GCS.h"

#include <AC_Fence/AC_Fence.h>

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
                                  fence->get_breach_time());
}

