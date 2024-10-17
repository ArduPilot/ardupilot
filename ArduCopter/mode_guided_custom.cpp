#include "Copter.h"

#if MODE_GUIDED_ENABLED && AP_SCRIPTING_ENABLED
// constructor registers custom number and names
ModeGuidedCustom::ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name):
    number(_number),
    full_name(_full_name),
    short_name(_short_name)
{
}
#endif
