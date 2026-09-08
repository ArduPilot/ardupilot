#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MAX31865_ENABLED

#include "SIM_Temperature_MAX31865.h"

using namespace SITL;

void MAX31865::update(const class Aircraft &aircraft)
{
    _temperature_degC = aircraft.ambient_outside_temperature_degC();
}

// The driver issues two types of SPI transaction:
//
// 1. write_register(0x80, config)  →  1LONG: tx=[0x80, config], len=2
// 2. read_registers(0x01, buf, 2)  →  2LONG: tx=[0x01] then rx=2 bytes
//
// Encoding (nominal=100Ω, reference=400Ω driver defaults, T in °C):
//   R(T) = 100 * (1 + 3.9083e-3*T + (-5.775e-7)*T²)   (Callendar-Van Dusen, T ≥ 0)
//   raw_temp = R(T) / 400 * 32768
//   data = raw_temp << 1  (fault LSB = 0)
//   bytes returned big-endian: [data >> 8, data & 0xFF]
int MAX31865::rdwr(uint8_t count, SPI::spi_ioc_transfer *&data)
{
    if (count == 1) {
        // write_register: buf = [reg_addr | 0x80, value]
        const uint8_t *tx = (const uint8_t *)data[0].tx_buf;
        if (data[0].len >= 2 && tx[0] == 0x80) {
            config_reg = tx[1];
        }
        return 0;
    }

    if (count == 2) {
        // read_registers: tx=[reg_addr], rx=response
        const uint8_t *tx = (const uint8_t *)data[0].tx_buf;
        uint8_t *rx = (uint8_t *)data[1].rx_buf;
        if (tx[0] == 0x01 && data[1].len >= 2) {
            const float T = _temperature_degC;
            const float R = 100.0f * (1.0f + 3.9083e-3f * T + (-5.775e-7f) * T * T);
            const uint16_t raw_temp = uint16_t(R / 400.0f * 32768.0f);
            const uint16_t d = raw_temp << 1;
            rx[0] = d >> 8;
            rx[1] = d & 0xFF;
            return 0;
        }
        return -1;
    }

    return -1;
}

#endif  // AP_SIM_TEMPERATURE_MAX31865_ENABLED
