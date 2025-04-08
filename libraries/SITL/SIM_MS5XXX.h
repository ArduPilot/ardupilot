#pragma once

#include "SIM_I2CDevice.h"

#include <AP_Common/Bitmask.h>

namespace SITL {

class MS5XXX : public I2CDevice
{
public:

    MS5XXX();

protected:

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

private:

    // float pressure;
    // float temperature;

    void reset();

    enum class Command : uint8_t {
        RESET = 0x1E,

        READ_CONVERSION = 0x00,

        // prom reading commands:
        READ_C0 = 0xa0,
        READ_C1 = 0xa2,
        READ_C2 = 0xa4,
        READ_C3 = 0xa6,
        READ_C4 = 0xa8,
        READ_C5 = 0xaa,
        READ_C6 = 0xac,
        READ_CRC = 0xae,

        // conversion start commands:
        CONVERT_D2_OSR_1024 = 0x54,
        CONVERT_D1_OSR_1024 = 0x44,
    };

    enum class State : uint8_t {
        COLD = 5,
        COLD_WAIT = 6,

        UNINITIALISED = 7,

        RUNNING = 17,

        RESET_START = 27,
        RESET_WAIT = 28,

        CONVERSION_D1_START = 37,
        CONVERSION_D1_WAIT = 38,

        CONVERSION_D2_START = 47,
        CONVERSION_D2_WAIT = 48,
    };

    State state = State::COLD;

    uint32_t command_start_us;

    uint8_t convert_out[3];

    bool prom_loaded = false;
    uint16_t loaded_prom[128/16];
    virtual void load_prom(uint16_t *loaded_prom, uint8_t len) const = 0;

    uint16_t conversion_time_osr_1024_us = 2280;

    virtual void convert(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2) =0;
    virtual void convert_forward(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C) = 0;
    virtual void get_pressure_temperature_readings(float &P_Pa, float &Temp_C) = 0;
    virtual void check_conversion_accuracy(float P_Pa, float Temp_C, uint32_t D1, uint32_t D2) = 0;

    void convert_D1();
    void convert_D2();
};

} // namespace SITL
