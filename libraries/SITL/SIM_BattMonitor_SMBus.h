#include "SIM_SMBusDevice.h"

#pragma once

namespace SITL {

class SMBusBattDevReg : public SMBusRegEnum {
public:
    static const uint8_t TEMP = 0x08;                 // Temperature
    static const uint8_t VOLTAGE = 0x09;              // Voltage
    static const uint8_t CURRENT = 0x0A;              // Current
    static const uint8_t REMAINING_CAPACITY = 0x0F;   // Remaining Capacity
    static const uint8_t FULL_CHARGE_CAPACITY = 0x10; // Full Charge Capacity (accounting for battery degradation)
    static const uint8_t CYCLE_COUNT = 0x17;          // Cycle Count
    static const uint8_t DESIGN_CAPACITY = 0x18;      // Design capacity (capacity when newly manufactured)
    static const uint8_t DESIGN_VOLTAGE = 0x19;       // Design voltage
    static const uint8_t SPECIFICATION_INFO = 0x1A;   // Specification Info
    static const uint8_t MANUFACTURE_DATE = 0x1B;     // Manufacture date
    static const uint8_t SERIAL = 0x1C;               // Serial Number
    static const uint8_t MANUFACTURE_NAME = 0x20;     // Manufacture Name
    static const uint8_t DEVICE_NAME = 0x21;          // Device Name
    static const uint8_t DEVICE_CHEMISTRY = 0x22;     // Battery chemistry type
    static const uint8_t MANUFACTURE_DATA = 0x23;     // Manufacture Data
};

class SIM_BattMonitor_SMBus : public SMBusDevice
{
public:

    SIM_BattMonitor_SMBus();

    virtual void update(const class Aircraft &aircraft) override;

private:

    uint32_t last_update_ms;
};

} // namespace SITL
