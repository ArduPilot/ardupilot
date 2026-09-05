/*
  Simulated Analog Devices ADIS16547 (paged register map, 32-bit delta-angle /
  delta-velocity burst prefixed by a BURST_ID and protected by CRC-32).

  This is exercised by AP_InertialSensor_ADIS1654x; the scale factors, register
  map and burst layout here must be kept in lock-step with that driver.  The
  SITL_Nexus board wires this device up as its third IMU.
*/
#pragma once

#include "SIM_config.h"

#if AP_SIM_ADIS_ENABLED

#include "SIM_ADIS.h"

namespace SITL {

class ADIS16547 : public ADIS
{
public:
    void init() override {
        set_reg(0, PAGE0_REG_PROD_ID, PROD_ID);   // PROD_ID lives on page 0
        set_reg(3, PAGE3_REG_RANG_MDL, RANG_MDL); // RANG_MDL lives on page 3
    }

protected:
    bool is_paged() const override { return true; }
    float nominal_rate_hz() const override { return 4000; }
    uint16_t burst_command() const override { return 0x7c00; }  // BURST_CMD
    uint8_t build_burst(uint8_t *out) override;

private:
    static const uint8_t  PAGE0_REG_PROD_ID  = 0x7e;
    static const uint8_t  PAGE3_REG_RANG_MDL = 0x12;
    static const uint16_t PROD_ID   = 0x40a3;   // 16547
    static const uint16_t RANG_MDL  = 0x0003;   // model -1: +/-125 deg/s, +/-360 deg
    static const uint16_t BURST_ID  = 0xc3c3;   // delta-angle/velocity burst marker
};

} // namespace SITL

#endif  // AP_SIM_ADIS_ENABLED
