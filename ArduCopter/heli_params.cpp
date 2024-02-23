#include "heli_params.h"

const AP_Param::GroupInfo Heli_Params::var_info[] = {

    // @Param: OPTIONS
    // @DisplayName: Heli Options
    // @Description: Bitmask of heli options. Bit 0 changes how the pitch, roll, and yaw axis integrator term is managed for low speed and takeoff/landing. In AC 4.0 and earlier, scheme uses a leaky integrator for ground speeds less than 5 m/s and won't let the steady state integrator build above ILMI. The integrator is allowed to build to the ILMI value when it is landed. The other integrator management scheme bases integrator limiting on takeoff and landing. Whenever the aircraft is landed the integrator is set to zero. When the aicraft is airborne, the integrator is only limited by IMAX.
    // @Bitmask: 0:Use Leaky I
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 1, Heli_Params, _heli_options, (uint8_t)HeliOption::USE_LEAKY_I),

    AP_GROUPEND
};

// Determines if _heli_options bit is set
bool Heli_Params::heli_option(HeliOption opt) const
{
    return (_heli_options.get() & (uint8_t)opt);
}
