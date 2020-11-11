#include "SIM_BattMonitor_SMBus_Generic.h"

#include <AP_Common/Bitmask.h>

/*

Testing:

param set BATT_MONITOR 19
reboot

*/

namespace SITL {

class SMBusBattRotoyeDevReg : public SMBusBattGenericDevReg {
public:
    static const uint8_t TEMP_EXT = 0x07;
};

class Rotoye : public SIM_BattMonitor_SMBus_Generic
{
public:

    Rotoye();

    uint8_t cellcount() const override { return 3; }

    void update(const class Aircraft &aircraft) override;

private:
    uint32_t last_temperature_update_ms;
};

} // namespace SITL
