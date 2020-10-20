#include "SIM_BattMonitor_SMBus_Maxell.h"

SITL::Maxell::Maxell() :
    SIM_BattMonitor_SMBus_Generic()
{
    set_register(SMBusBattGenericDevReg::SERIAL, (uint16_t)37);
}
