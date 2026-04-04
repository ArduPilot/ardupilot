#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MCP9600_ENABLED

#include "SIM_Temperature_MCP9600.h"

using namespace SITL;

#include <GCS_MAVLink/GCS.h>
#include <signal.h>

enum class MCP9600StatusBits {
    TH_UPDATE = (1 << 6)  // data ready to read
};

void MCP9600::init()
{
    set_debug(true);

    add_register("WHOAMI", MCP9600DevReg::WHOAMI, 1, I2CRegisters::RegMode::RDONLY);
    set_register(MCP9600DevReg::WHOAMI, (uint8_t)0x40);

    add_register("SENSOR_STATUS", MCP9600DevReg::SENSOR_STATUS, 1, I2CRegisters::RegMode::RDWR);
    set_register(MCP9600DevReg::SENSOR_STATUS, (uint8_t)0x00);

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
    if ((config & 0b111) != 0b10) {
        AP_HAL::panic("Unexpected filter configuration");
    }
    if ((config >> 4) != 0) {  // this is a K-type thermocouple, the default in the driver
        AP_HAL::panic("Unexpected thermocouple configuration");
    }

    // dont update if we already have data ready to read (CHECKME: fidelity)
    uint8_t status = 0;
    get_reg_value(MCP9600DevReg::SENSOR_STATUS, config);

    if (status & (uint8_t)MCP9600StatusBits::TH_UPDATE) {
        return;
    }

    static constexpr uint16_t factor = (1/0.0625);
    set_register(MCP9600DevReg::HOT_JUNC, uint16_t(htobe16(some_temperature + degrees(sinf(now_ms)) * factor)));

    // indicate we have data ready to read
    set_register(MCP9600DevReg::SENSOR_STATUS, (uint8_t)MCP9600StatusBits::TH_UPDATE);
}

int MCP9600::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    return I2CRegisters_ConfigurableLength::rdwr(data);
}

#endif  // AP_SIM_TEMPERATURE_MCP9600_ENABLED
