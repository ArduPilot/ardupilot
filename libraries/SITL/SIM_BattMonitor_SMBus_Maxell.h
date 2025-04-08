#include "SIM_config.h"

#if AP_SIM_BATT_MONITOR_SMBUS_MAXELL_ENABLED

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

    uint8_t cellcount() const override { return 14; }
    uint8_t connected_cells() const override { return 14; }

};

} // namespace SITL

#endif  // AP_SIM_BATT_MONITOR_SMBUS_MAXELL_ENABLED
