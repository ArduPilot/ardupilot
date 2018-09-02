/* dummy methods to avoid having to link against AP_Camera */

#include <AP_Camera/AP_Camera.h>

namespace AP {
    AP_Camera *camera() {
        return nullptr;
    }
};

void AP_Camera::take_picture()
{
}
