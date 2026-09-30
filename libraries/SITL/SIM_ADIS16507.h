/*
  Simulated Analog Devices ADIS16507 (flat register map, 32-bit delta-angle /
  delta-velocity burst with an 8-bit additive checksum).  Driven by the real
  AP_InertialSensor_ADIS1647x driver in OpMode::Delta32.
*/
#pragma once

#include "SIM_config.h"

#if AP_SIM_ADIS_ENABLED

#include "SIM_ADIS.h"

namespace SITL {

class ADIS16507 : public ADIS
{
public:
    void init() override {
        set_reg(0, REG_PROD_ID, PROD_ID);
        set_reg(0, REG_RANG_MDL, RANG_MDL);   // gyro range identifier (0 => +/-125 deg/s)
    }

protected:
    float nominal_rate_hz() const override { return 1000; }
    uint16_t burst_command() const override { return 0x6800; }
    uint8_t build_burst(uint8_t *out) override;

private:
    static const uint8_t  REG_PROD_ID  = 0x72;
    static const uint8_t  REG_RANG_MDL = 0x5e;
    static const uint16_t PROD_ID      = 0x407b;
    static const uint16_t RANG_MDL     = 0x0000;  // (rng>>2)&3 == 0 => +/-125 deg/s
};

} // namespace SITL

#endif  // AP_SIM_ADIS_ENABLED
