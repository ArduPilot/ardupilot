#include "AP_DAL_VisualOdom.h"

#include <AP_VisualOdom/AP_VisualOdom.h>

#if HAL_VISUALODOM_ENABLED

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_VisualOdom::AP_DAL_VisualOdom()
{
}

void AP_DAL_VisualOdom::start_frame()
{
    const auto *vo = AP::visualodom();

    const log_RVOH old = RVOH;
    RVOH.ptr_is_nullptr = (vo == nullptr);
    if (vo != nullptr) {
        RVOH.healthy = vo->healthy();
    }

    WRITE_REPLAY_BLOCK_IFCHANGD(RVOH, RVOH, old);
}

#endif // HAL_VISUALODOM_ENABLED
