#include "SIM_BattMonitor_SMBus_Maxell.h"

SITL::Maxell::Maxell() :
    SIM_BattMonitor_SMBus_Generic()
{
    // TO DO set maxell batteries PEC version to V1
    
    // Note Maxell batteries do not support PEC and ArduPilot checks for this by
    // checking that the manufacturer name matches "Hitachi maxell"
    const char *manufacturer_name = "Hitachi maxell";
    set_block(SMBusBattDevReg::MANUFACTURE_NAME, manufacturer_name);

    const char *device_name = "SITL_maxell";
    set_block(SMBusBattDevReg::DEVICE_NAME, device_name);
    
    set_register(SMBusBattGenericDevReg::SERIAL, (uint16_t)37);
}
