#include "SIM_BattMonitor_SMBus.h"

#include <AP_Stats/AP_Stats.h>

SITL::SIM_BattMonitor_SMBus::SIM_BattMonitor_SMBus() :
    I2CRegisters_16Bit()
{
    add_register("Temperature", SMBusBattDevReg::TEMP, O_RDONLY);
    add_register("Voltage", SMBusBattDevReg::VOLTAGE, O_RDONLY);
    add_register("Current", SMBusBattDevReg::CURRENT, O_RDONLY);
    add_register("Remaining Capacity", SMBusBattDevReg::REMAINING_CAPACITY, O_RDONLY);
    add_register("Full Charge Capacity", SMBusBattDevReg::FULL_CHARGE_CAPACITY, O_RDONLY);
    add_register("Cycle_Count", SMBusBattDevReg::CYCLE_COUNT, O_RDONLY);
    add_register("Specification Info", SMBusBattDevReg::SPECIFICATION_INFO, O_RDONLY);
    add_register("Serial", SMBusBattDevReg::SERIAL, O_RDONLY);
    add_register("Manufacture Name", SMBusBattDevReg::MANUFACTURE_NAME, O_RDONLY);
    add_register("Manufacture Data", SMBusBattDevReg::MANUFACTURE_DATA, O_RDONLY);

    set_register(SMBusBattDevReg::TEMP, (int16_t)((15 + 273.15)*10));
     // see update for voltage
     // see update for current
     // TODO: remaining capacity
     // TODO: full capacity
     set_register(SMBusBattDevReg::CYCLE_COUNT, (uint16_t(39U)));

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

     specinfo.fields.revision = 0b0001;  // version 1
//     specinfo.fields.version = 0b0011;  // 1.1 with PEC; TODO!
     specinfo.fields.version = 0b0001;  // 1.0
     specinfo.fields.vscale = 0b0000;  // unknown...
     specinfo.fields.ipscale = 0b0000;  // unknown...
     set_register(SMBusBattDevReg::SPECIFICATION_INFO, specinfo.word);

     set_register(SMBusBattDevReg::SERIAL, (uint16_t)12345);

     // Set MANUFACTURE_NAME
     const char *manufacturer_name = "ArduPilot";
     set_register(SMBusBattDevReg::MANUFACTURE_NAME, uint16_t(manufacturer_name[0]<<8|strlen(manufacturer_name)));
     uint8_t i = 1;  // already sent the first byte out....
     while (i < strlen(manufacturer_name)) {
         const uint8_t a = manufacturer_name[i++];
         uint8_t b = 0;
         if (i < strlen(manufacturer_name)) {
             b = manufacturer_name[i];
         }
         i++;
         const uint16_t value = b<<8 | a;
         add_register("Name", SMBusBattDevReg::MANUFACTURE_NAME + i/2, O_RDONLY);
         set_register(SMBusBattDevReg::MANUFACTURE_NAME + i/2, value);
     }
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
