#include <AP_Param/AP_Param.h>

class Heli_Params
{
public:

    // Enum for heli optional features
    enum class HeliOption {
        USE_LEAKY_I = (1<<0), // 1
    };

    // Determines if _heli_options bit is set
    bool heli_option(HeliOption opt) const;

    // Use leaking integrator management scheme
    bool using_leaky_integrator() const { return heli_option(HeliOption::USE_LEAKY_I); }

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters
    AP_Int8 _heli_options; // Bitmask for optional features

};
