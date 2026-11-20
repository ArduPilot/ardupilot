#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MLX90614_ENABLED

#include "SIM_Temperature_MLX90614.h"
#include "SIM_Aircraft.h"

#include <AP_Math/AP_Math.h>

using namespace SITL;

uint16_t MLX90614::raw_for_degC(float degC)
{
    return uint16_t(C_TO_KELVIN(degC) / 0.02f);
}

void MLX90614::update(const class Aircraft &aircraft)
{
    const float ambient_degC = aircraft.ambient_outside_temperature_degC();
    ta_raw = raw_for_degC(ambient_degC);
    tobj1_raw = raw_for_degC(ambient_degC + OBJECT_OFFSET_DEGC);
}

int MLX90614::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    // only SMBus read word is supported: a command write then a
    // 3 byte read of the LSB, MSB and PEC
    if (data->nmsgs != 2 ||
        data->msgs[0].flags != 0 || data->msgs[0].len != 1 ||
        data->msgs[1].flags != I2C_M_RD || data->msgs[1].len != 3) {
        return -1;
    }

    const uint8_t cmd = data->msgs[0].buf[0];
    uint16_t value;
    switch ((Command)cmd) {
    case Command::TA:
        value = ta_raw;
        break;
    case Command::TOBJ1:
        value = tobj1_raw;
        break;
    default:
        return -1;
    }

    uint8_t *buf = data->msgs[1].buf;
    buf[0] = value & 0xff;
    buf[1] = value >> 8;

    // CRC-8 (polynomial 0x07) over the write address, command, read
    // address and the two data bytes
    const uint8_t addr = data->msgs[0].addr << 1;
    const uint8_t pec_data[] { addr, cmd, uint8_t(addr | 1), buf[0], buf[1] };
    buf[2] = crc8_generic(pec_data, sizeof(pec_data), 0x07);

    return 0;
}

#endif  // AP_SIM_TEMPERATURE_MLX90614_ENABLED
