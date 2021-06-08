#include "SIM_BattMonitor_SMBus.h"

#pragma once

namespace SITL {

class SMBusBattGenericDevReg : public SMBusBattDevReg {
public:
    static const uint8_t CELL1 = 0x3f;
    static const uint8_t CELL2 = 0x3e;
    static const uint8_t CELL3 = 0x3d;
    static const uint8_t CELL4 = 0x3c;
    static const uint8_t CELL5 = 0x3b;
    static const uint8_t CELL6 = 0x3a;
    static const uint8_t CELL7 = 0x39;
    static const uint8_t CELL8 = 0x38;
    static const uint8_t CELL9 = 0x37;
    static const uint8_t CELL10 = 0x36;
    static const uint8_t CELL11 = 0x35;
    static const uint8_t CELL12 = 0x34;
    static const uint8_t CELL13 = 0x33;
    static const uint8_t CELL14 = 0x32;
};

class SIM_BattMonitor_SMBus_Generic : public SIM_BattMonitor_SMBus
{
public:

    SIM_BattMonitor_SMBus_Generic();
    void init() override;
    void update(const class Aircraft &aircraft) override;

    virtual uint8_t cellcount() const = 0;

    virtual uint8_t connected_cells() const { return 3; }
};

} // namespace SITL
