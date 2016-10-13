#include "AP_Stats.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Stats::var_info[] = {

    // @Param: _BOOTCNT
    // @DisplayName: Boot Count
    // @Description: Number of times board has been booted
    // @User: Standard
    AP_GROUPINFO("_BOOTCNT",    0, AP_Stats, params.bootcount, 0),

    AP_GROUPEND
};

void AP_Stats::init()
{
    params.bootcount.set_and_save(params.bootcount+1);
}
