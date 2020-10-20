#include "SIM_BattMonitor_SMBus_Generic.h"

SITL::SIM_BattMonitor_SMBus_Generic::SIM_BattMonitor_SMBus_Generic() :
    SIM_BattMonitor_SMBus()
{
}

void SITL::SIM_BattMonitor_SMBus_Generic::init()
{
    switch (cellcount()) {
    case 12:
        add_register("Cell12", SMBusBattGenericDevReg::CELL12, O_RDONLY);
        FALLTHROUGH;
    case 11:
        add_register("Cell11", SMBusBattGenericDevReg::CELL11, O_RDONLY);
        FALLTHROUGH;
    case 10:
        add_register("Cell10", SMBusBattGenericDevReg::CELL10, O_RDONLY);
        FALLTHROUGH;
    case 9:
        add_register("Cell9", SMBusBattGenericDevReg::CELL9, O_RDONLY);
        FALLTHROUGH;
    case 8:
        add_register("Cell8", SMBusBattGenericDevReg::CELL8, O_RDONLY);
        FALLTHROUGH;
    case 7:
        add_register("Cell7", SMBusBattGenericDevReg::CELL7, O_RDONLY);
        FALLTHROUGH;
    case 6:
        add_register("Cell6", SMBusBattGenericDevReg::CELL6, O_RDONLY);
        FALLTHROUGH;
    case 5:
        add_register("Cell5", SMBusBattGenericDevReg::CELL5, O_RDONLY);
        FALLTHROUGH;
    case 4:
        add_register("Cell4", SMBusBattGenericDevReg::CELL4, O_RDONLY);
        FALLTHROUGH;
    case 3:
        add_register("Cell3", SMBusBattGenericDevReg::CELL3, O_RDONLY);
        FALLTHROUGH;
    case 2:
        add_register("Cell2", SMBusBattGenericDevReg::CELL2, O_RDONLY);
        FALLTHROUGH;
    case 1:
        add_register("Cell1", SMBusBattGenericDevReg::CELL1, O_RDONLY);
        return;
    default:
        AP_HAL::panic("Bad cellcount %u", cellcount());
    }
}

void SITL::SIM_BattMonitor_SMBus_Generic::update(const class Aircraft &aircraft)
{
    SIM_BattMonitor_SMBus::update(aircraft);

    // pretend to have three cells connected
    const float millivolts = AP::sitl()->state.battery_voltage * 1000.0f;
    set_register(SMBusBattGenericDevReg::CELL1, uint16_t(millivolts/3.0f - 100.0f));
    set_register(SMBusBattGenericDevReg::CELL2, uint16_t(millivolts/3.0f));
    set_register(SMBusBattGenericDevReg::CELL3, uint16_t(millivolts/3.0f + 100.0f));
}
