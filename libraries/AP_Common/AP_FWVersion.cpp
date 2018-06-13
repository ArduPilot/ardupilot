#include "AP_FWVersion.h"

namespace AP {

const AP_FWVersion &fwversion()
{
    return AP_FWVersion::get_fwverz();
}

}
