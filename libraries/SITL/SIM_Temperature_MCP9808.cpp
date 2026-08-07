/*
 * SITL simulation for MCP9808 temperature sensor
 */

#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MCP9808_ENABLED

#include "SIM_Temperature_MCP9808.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
using namespace SITL;

void MCP9808::init()
{
    set_debug(true);
    // Configuration register
    add_register("CONFIG", MCP9808DevReg::CONFIG, 2, I2CRegisters::RegMode::RDWR);
    set_register(MCP9808DevReg::CONFIG, uint16_t(htobe16(0x0000)));
    // Ambient temperature register
    add_register("AMBIENT_TEMP", MCP9808DevReg::AMBIENT_TEMP, 2, I2CRegisters::RegMode::RDONLY);
    // Resolution register
    add_register("RESOLUTION", MCP9808DevReg::RESOLUTION, 1, I2CRegisters::RegMode::RDWR);
    // Default power-up resolution
    // 0x03 = 0.0625°C, 250 ms
    set_register(MCP9808DevReg::RESOLUTION, uint8_t(0x03));
}

void MCP9808::update(const Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    // Read current resolution selected by ArduPilot
    uint8_t resolution;
    get_reg_value(MCP9808DevReg::RESOLUTION, resolution);

    uint32_t conversion_time_ms = 250;

    switch (resolution)
    {
    case 0x00:
        conversion_time_ms = 30;
        break;

    case 0x01:
        conversion_time_ms = 65;
        break;

    case 0x02:
        conversion_time_ms = 130;
        break;

    case 0x03:
    default:
        conversion_time_ms = 250;
        break;
    }

    if ((now_ms - last_temperature_update_ms) < conversion_time_ms) {
        return;
    }

    last_temperature_update_ms = now_ms;

    const float period_s = 10.0f;  // One complete cycle every 10 sec
    const float phase = (2.0f * M_PI * (now_ms * 0.001f)) / period_s;
    const float temperature = 60.0f + 35.0f * sinf(phase);

    // MCP9808 resolution = 0.0625°C
    int16_t raw_temperature = int16_t(temperature / 0.0625f);
    uint16_t value = uint16_t(raw_temperature & 0x0FFF);

    if (temperature < 0) {
        value |= (1 << 12);
    }
    set_register(
        MCP9808DevReg::AMBIENT_TEMP, uint16_t(htobe16(value)));
}

int MCP9808::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    return I2CRegisters_ConfigurableLength::rdwr(data);
}

#endif // AP_SIM_TEMPERATURE_MCP9808_ENABLED