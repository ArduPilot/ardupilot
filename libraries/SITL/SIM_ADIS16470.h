/*
  Simulated Analog Devices ADIS16470 (flat register map, 16-bit gyro/accel
  "basic" burst with an 8-bit additive checksum).  Driven by the real
  AP_InertialSensor_ADIS1647x driver in OpMode::Basic.
*/
#pragma once

#include "SIM_config.h"

#if AP_SIM_ADIS_ENABLED

#include "SIM_ADIS.h"

namespace SITL {

class ADIS16470 : public ADIS
{
public:
    void init() override {
        set_reg(0, REG_PROD_ID, PROD_ID);
    }

protected:
    float nominal_rate_hz() const override { return 2000; }
    uint16_t burst_command() const override { return 0x6800; }  // read of GLOB_CMD triggers burst
    uint8_t build_burst(uint8_t *out) override;

private:
    static const uint8_t  REG_PROD_ID = 0x72;
    static const uint16_t PROD_ID     = 0x4056;
};

} // namespace SITL

#endif  // AP_SIM_ADIS_ENABLED
