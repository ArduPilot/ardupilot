#include "SIM_BattMonitor_SMBus.h"

#include <AP_Stats/AP_Stats.h>

SITL::SIM_BattMonitor_SMBus::SIM_BattMonitor_SMBus() :
    SMBusDevice()
{
    add_register("Temperature", SMBusBattDevReg::TEMP, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Voltage", SMBusBattDevReg::VOLTAGE, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Current", SMBusBattDevReg::CURRENT, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Remaining Capacity", SMBusBattDevReg::REMAINING_CAPACITY, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Full Charge Capacity", SMBusBattDevReg::FULL_CHARGE_CAPACITY, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Cycle_Count", SMBusBattDevReg::CYCLE_COUNT, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Design Charge Capacity", SMBusBattDevReg::DESIGN_CAPACITY, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Design Maximum Voltage", SMBusBattDevReg::DESIGN_VOLTAGE, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Specification Info", SMBusBattDevReg::SPECIFICATION_INFO, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Manufacture Date", SMBusBattDevReg::MANUFACTURE_DATE, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Serial", SMBusBattDevReg::SERIAL, SITL::I2CRegisters::RegMode::RDONLY);
    add_block("Manufacture Name", SMBusBattDevReg::MANUFACTURE_NAME, SITL::I2CRegisters::RegMode::RDONLY);
    add_block("Device Name", SMBusBattDevReg::DEVICE_NAME, SITL::I2CRegisters::RegMode::RDONLY);
    add_block("Device Chemistry", SMBusBattDevReg::DEVICE_CHEMISTRY, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("Manufacture Data", SMBusBattDevReg::MANUFACTURE_DATA, SITL::I2CRegisters::RegMode::RDONLY);

    set_register(SMBusBattDevReg::TEMP, (int16_t)((C_TO_KELVIN(15))*10));
    // see update for voltage
    // see update for current

    set_register(SMBusBattDevReg::DESIGN_VOLTAGE, (uint16_t(50400U)));          // (mV) Design maximum voltage

    // TODO: remaining capacity connect to sim capacity add to update method below?
    // TODO: Battery mode bit set to mAh vs 10 mWh
    set_register(SMBusBattDevReg::REMAINING_CAPACITY, (uint16_t(42042U)));      // (mAh) Remaining Capacity

    // TODO: full capacity fill via SIM parameter
    set_register(SMBusBattDevReg::FULL_CHARGE_CAPACITY, (uint16_t(45000U)));    // (mAh) Full charge capacity
    set_register(SMBusBattDevReg::DESIGN_CAPACITY, (uint16_t(52500U)));         // (mAh) Design capacity

    set_register(SMBusBattDevReg::CYCLE_COUNT, (uint16_t(42U)));

    // Set SPECIFICATION_INFO
    union {
        struct {
            uint8_t revision : 4;
            uint8_t version: 4;
            uint8_t vscale: 4;
            uint8_t ipscale: 4;
        } fields;
        uint16_t word;
    } specinfo;

    // TODO: compute PEC for relevant monitors
    specinfo.fields.revision = 0b0001;  // version 1
//    specinfo.fields.version = 0b0011;  // 1.1 with PEC; TODO!
    specinfo.fields.version = 0b0001;  // 1.0
    specinfo.fields.vscale = 0b0000;  // unknown...
    specinfo.fields.ipscale = 0b0000;  // unknown...
    set_register(SMBusBattDevReg::SPECIFICATION_INFO, specinfo.word);

    set_register(SMBusBattDevReg::SERIAL, (uint16_t)12345);

    const char *manufacturer_name = "ArduPilot";
    set_block(SMBusBattDevReg::MANUFACTURE_NAME, manufacturer_name);

    const char *device_name = "SITLBatMon_V0.99";
    set_block(SMBusBattDevReg::DEVICE_NAME, device_name);

    const char *device_chemistry = "LION";
    set_block(SMBusBattDevReg::DEVICE_CHEMISTRY, device_chemistry);

    // Set Manufacture date to 2021 APR 24th
    const uint16_t manufacturer_date = ((2021 - 1980) << 9) + (04 << 5) + 24;
    set_register(SMBusBattDevReg::MANUFACTURE_DATE, manufacturer_date);

    // TODO: manufacturer data
}

void SITL::SIM_BattMonitor_SMBus::update(const class Aircraft &aircraft)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms > 100) {
        const float millivolts = AP::sitl()->state.battery_voltage * 1000.0f;
        set_register(SMBusBattDevReg::VOLTAGE, uint16_t(millivolts));
        // FIXME: is this REALLY what the hardware will do?
        const int16_t current = constrain_int32(AP::sitl()->state.battery_current*-1000, -32768, 32767);
        set_register(SMBusBattDevReg::CURRENT, current);
        last_update_ms = now;
    }
}
