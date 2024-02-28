#include "SIM_MS5XXX.h"

#include <AP_Common/Bitmask.h>

namespace SITL {

class MS5525 : public MS5XXX
{
public:

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
        36402, // C1, pressure sensitivity
        39473, // C2, pressure offset
        40393, // C3, temperature coeff of press sensit
        29523, // C4, temperature cofff of press offs
        29854, // C5, ref temperature
        21917,  // C6, temperature coeff of temperature
        0x000c  // checksum
    };

    // for 5525DSO-pp001DS
    const uint8_t Qx_coeff[6] {
        15, 17, 7, 5, 7, 21
    };


};

} // namespace SITL
