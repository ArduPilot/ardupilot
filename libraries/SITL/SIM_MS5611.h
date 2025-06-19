#include "SIM_config.h"

#if AP_SIM_MS5611_ENABLED

#include "SIM_MS5XXX.h"

#include <AP_Common/Bitmask.h>

namespace SITL {

class MS5611 : public MS5XXX
{
public:

    using MS5XXX::MS5XXX;

protected:

    void get_pressure_temperature_readings(float &P_Pa, float &Temp_C) override;

    void convert(float P_Pa, float Temp_C, uint32_t &D1, uint32_t &D2) override;
    void convert_forward(int32_t D1, int32_t D2, float &P_Pa, float &Temp_C) override;
    void check_conversion_accuracy(float P_Pa, float Temp_C, uint32_t D1, uint32_t D2) override;

    void load_prom(uint16_t *_loaded_prom, uint8_t len) const override {
        memcpy(_loaded_prom, prom, len);
    }

private:

    // this data comes from the datasheet page 7
    const uint16_t prom[8] {
        0xFFFF,  // reserved
        40127, // C1, pressure sensitivity
        36924, // C2, pressure offset
        23317, // C3, temperature coeff of press sensit
        23282, // C4, temperature cofff of press offs
        33464, // C5, ref temperature
        28312,  // C6, temperature coeff of temperature
        0x0008  // checksum
    };

    const uint8_t Qx_coeff[6] {
        15, 16, 8, 7, 8, 23
    };

};

} // namespace SITL

#endif  // AP_SIM_MS5611_ENABLED
