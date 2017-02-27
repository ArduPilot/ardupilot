#include "AP_Arming_Sub.h"
#include "Sub.h"

enum HomeState AP_Arming_Sub::home_status() const
{
    return sub.ap.home_state;
}
