#include "SIM_Temperature_MCP9600.h"

using namespace SITL;

#include <GCS_MAVLink/GCS.h>
#include <signal.h>

void MCP9600::init()
{
    set_debug(true);

    add_register("WHOAMI", MCP9600DevReg::WHOAMI, 2, I2CRegisters::RegMode::RDONLY);
    set_register(MCP9600DevReg::WHOAMI, (uint16_t)0x40);

    add_register("SENSOR_CONFIG", MCP9600DevReg::SENSOR_CONFIG, 1, I2CRegisters::RegMode::RDWR);
    set_register(MCP9600DevReg::SENSOR_CONFIG, (uint8_t)0x00);

    add_register("HOT_JUNC", MCP9600DevReg::HOT_JUNC, 2, I2CRegisters::RegMode::RDONLY);
}

void MCP9600::update(const class Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_temperature_update_ms < 100) {  // 10Hz
        return;
    }
    last_temperature_update_ms = now_ms;

    uint8_t config;
    get_reg_value(MCP9600DevReg::SENSOR_CONFIG, config);
    if (config == 0) {
        // unconfigured; FIXME lack of fidelity
        return;
    }
    if ((config & 0b111) != 1) {  // FIXME: this is just the default config
        AP_HAL::panic("Unexpected filter configuration");
    }
    if ((config >> 4) != 0) {  // this is a K-type thermocouple, the default in the driver
        AP_HAL::panic("Unexpected thermocouple configuration");
    }
    static constexpr uint16_t factor = (1/0.0625);
    set_register(MCP9600DevReg::HOT_JUNC, uint16_t(htobe16(some_temperature + degrees(sinf(now_ms)) * factor)));
}

int MCP9600::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    return I2CRegisters_ConfigurableLength::rdwr(data);
}
