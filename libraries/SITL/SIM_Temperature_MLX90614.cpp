#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MLX90614_ENABLED

#include "SIM_Temperature_MLX90614.h"

using namespace SITL;

void MLX90614::update(const class Aircraft &aircraft)
{
    _temperature_degC = aircraft.ambient_outside_temperature_degC();
}

// The driver reads register 0x06 (MLX90614_TA = ambient temperature) via a
// write-then-read transaction: send 1-byte command, receive 3 bytes (LSB, MSB, PEC).
// raw = (T_degC + 273.15) / 0.02; returned little-endian (LSB first, then MSB).
int MLX90614::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 2) {
        return -1;
    }

    const auto &write_msg = data->msgs[0];
    auto &read_msg = data->msgs[1];

    if (write_msg.flags != 0 || write_msg.len != 1) {
        return -1;
    }
    if (read_msg.flags != I2C_M_RD || read_msg.len != 3) {
        return -1;
    }

    const uint8_t cmd = write_msg.buf[0];
    if (cmd != 0x06) {  // MLX90614_TA
        return -1;
    }

    const uint16_t raw = uint16_t((_temperature_degC + 273.15f) / 0.02f);
    read_msg.buf[0] = raw & 0xFF;         // LSB
    read_msg.buf[1] = (raw >> 8) & 0xFF;  // MSB
    read_msg.buf[2] = 0x00;               // PEC (not validated by driver)

    return 0;
}

#endif  // AP_SIM_TEMPERATURE_MLX90614_ENABLED
