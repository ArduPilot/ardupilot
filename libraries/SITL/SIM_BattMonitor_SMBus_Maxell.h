#include "SIM_BattMonitor_SMBus_Generic.h"

#include <AP_Common/Bitmask.h>

/*

Testing:

param set BATT_MONITOR 16
reboot

*/

namespace SITL {

class Maxell : public SIM_BattMonitor_SMBus_Generic
{
public:

    Maxell();

    uint8_t cellcount() const override { return 3; }

};

} // namespace SITL
