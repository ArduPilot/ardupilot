#include "AC_PrecLand_Backend.h"

bool AC_PrecLand_Backend::get_los_body(Vector3f& dir_body) const
{
    if (_have_los_meas) {
        dir_body = _los_meas_body;
        return true;
    }
    return false;
}
