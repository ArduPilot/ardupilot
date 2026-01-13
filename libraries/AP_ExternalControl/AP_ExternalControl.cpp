#include "AP_ExternalControl.h"

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Vehicle/AP_Vehicle.h>

// singleton instance
AP_ExternalControl *AP_ExternalControl::singleton;

bool AP_ExternalControl::arm(AP_Arming::Method method, bool do_arming_checks)
{
    return AP::arming().arm(method, do_arming_checks);
}

bool AP_ExternalControl::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    return AP::arming().disarm(method, do_disarm_checks);
}

bool AP_ExternalControl::set_mode(const uint8_t mode, const ModeReason reason)
{
    return AP::vehicle()->set_mode(mode, reason);
}

AP_ExternalControl::AP_ExternalControl()
{
    singleton = this;
}


namespace AP
{

AP_ExternalControl *externalcontrol()
{
    return AP_ExternalControl::get_singleton();
}

};

#endif // AP_EXTERNAL_CONTROL_ENABLED
