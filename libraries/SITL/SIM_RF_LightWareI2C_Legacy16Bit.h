#pragma once

/*
  Simulator for the legacy-protocol LightWareI2C rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

param set RNGFND1_TYPE 7
param set RNGFND1_ADDR 0x66
param set RNGFND1_MAX 30
graph DISTANCE_SENSOR[0].current_distance*0.01
graph GLOBAL_POSITION_INT.relative_alt/1000-DISTANCE_SENSOR[0].current_distance*0.01
reboot

arm throttle
rc 3 1600
*/

#include "SIM_config.h"

#if AP_SIM_RF_LIGHTWAREI2C_LEGACY16BIT_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class LightWareI2C_Legacy16BitDevReg : public I2CRegEnum {
public:
    static const uint8_t RANGE                               =  0;
    static const uint8_t LOST_SIGNAL_TIMEOUT_READ            = 22;
    static const uint8_t LOST_SIGNAL_TIMEOUT_WRITE           = 23;
};

class LightWareI2C_Legacy16Bit : public I2CDevice, private I2CRegisters_16Bit
{
public:

    LightWareI2C_Legacy16Bit() :
        I2CDevice(),
        I2CRegisters_16Bit()
        { }

    void init() override;

    void update(const class Aircraft &aircraft) override {
        const uint32_t now_ms = AP_HAL::millis();
        if (last_update_ms - now_ms > 50) {
            const float range = aircraft.rangefinder_range();
            if (range > 30) {
                set_register(LightWareI2C_Legacy16BitDevReg::RANGE, htobe16(uint16_t(65436)));  // firmware can send this value for out-of-range
            } else {
                set_register(LightWareI2C_Legacy16BitDevReg::RANGE, htobe16(uint16_t(range*100)));
            }
        }
    }

private:

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    uint32_t last_update_ms;
};

} // namespace SITL

#endif  // AP_SIM_RF_LIGHTWAREI2C_LEGACY16BIT_ENABLED
