#include "SIM_BattMonitor_SMBus_Generic.h"

SITL::SIM_BattMonitor_SMBus_Generic::SIM_BattMonitor_SMBus_Generic() :
    SIM_BattMonitor_SMBus()
{
}

void SITL::SIM_BattMonitor_SMBus_Generic::init()
{
    switch (cellcount()) {
    case 14:
        add_register("Cell14", SMBusBattGenericDevReg::CELL14, O_RDONLY);
        FALLTHROUGH;
    case 13:
        add_register("Cell13", SMBusBattGenericDevReg::CELL13, O_RDONLY);
        FALLTHROUGH;
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
    const float millivolts = aircraft.get_battery_voltage() * 1000.0f;
    uint16_t value_even = -1;
    uint16_t value_odd = -1;
    const uint8_t _connected_cells = connected_cells();
    if (millivolts > 0) {
        const float volts_per_cell = millivolts/float(_connected_cells);
        value_even = uint16_t(volts_per_cell - 100.0f);
        value_odd = uint16_t(volts_per_cell + 100.0f);
    }
    switch (_connected_cells) {
    case 14:
        set_register(SMBusBattGenericDevReg::CELL14, value_even);
        FALLTHROUGH;
    case 13:
        set_register(SMBusBattGenericDevReg::CELL13, value_odd);
        FALLTHROUGH;
    case 12:
        set_register(SMBusBattGenericDevReg::CELL12, value_even);
        FALLTHROUGH;
    case 11:
        set_register(SMBusBattGenericDevReg::CELL11, value_odd);
        FALLTHROUGH;
    case 10:
        set_register(SMBusBattGenericDevReg::CELL10, value_even);
        FALLTHROUGH;
    case 9:
        set_register(SMBusBattGenericDevReg::CELL9, value_odd);
        FALLTHROUGH;
    case 8:
        set_register(SMBusBattGenericDevReg::CELL8, value_even);
        FALLTHROUGH;
    case 7:
        set_register(SMBusBattGenericDevReg::CELL7, value_odd);
        FALLTHROUGH;
    case 6:
        set_register(SMBusBattGenericDevReg::CELL6, value_even);
        FALLTHROUGH;
    case 5:
        set_register(SMBusBattGenericDevReg::CELL5, value_odd);
        FALLTHROUGH;
    case 4:
        set_register(SMBusBattGenericDevReg::CELL4, value_even);
        FALLTHROUGH;
    case 3:
        set_register(SMBusBattGenericDevReg::CELL3, value_odd);
        FALLTHROUGH;
    case 2:
        set_register(SMBusBattGenericDevReg::CELL2, value_even);
        FALLTHROUGH;
    case 1:
        set_register(SMBusBattGenericDevReg::CELL1, value_odd);
        return;
    default:
        AP_HAL::panic("Bad connected_cellcount %u", _connected_cells);
    }
}
