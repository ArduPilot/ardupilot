#include "AP_DAL_VisualOdom.h"

#include <AP_VisualOdom/AP_VisualOdom.h>

#if HAL_VISUALODOM_ENABLED

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

void AP_DAL_VisualOdom::start_frame()
{
    const auto *vo = AP::visualodom();

    const log_RVOH old = RVOH;
    RVOH.ptr_is_nullptr = (vo == nullptr);
    if (vo != nullptr) {
        RVOH.pos_offset = vo->get_pos_offset();
        RVOH.delay_ms = vo->get_delay_ms();
        RVOH.healthy = vo->healthy();
        RVOH.enabled = vo->enabled();
    }

    WRITE_REPLAY_BLOCK_IFCHANGED(RVOH, RVOH, old);
}

#endif // HAL_VISUALODOM_ENABLED
