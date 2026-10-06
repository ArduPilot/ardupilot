#pragma once

/*
  Simulator for the TeraRangerI2C rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

param set RNGFND1_TYPE 14  # TRI2C
param ftp
param set RNGFND1_ADDR 49
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600
*/

#include "SIM_config.h"

#if AP_SIM_TERARANGERI2C_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class TeraRangerI2CDevReg : public I2CRegEnum {
public:
    // static constexpr uint8_t TRIGGER_READING = 0x00;
    static constexpr uint8_t WHO_AM_I = 0x01;
    static constexpr uint8_t CHANGE_BASE_ADDR = 0xA2;
};

class TeraRangerI2C : public I2CDevice, public I2CRegisters_8Bit
{
public:

    void init() override {
        // add_register("TRIGGER_READING", TeraRangerI2CDevReg::TRIGGER_READING, I2CRegisters::RegMode::WRONLY);
        add_register("WHO_AM_I", TeraRangerI2CDevReg::WHO_AM_I, I2CRegisters::RegMode::RDONLY);
        add_register("CHANGE_BASE_ADDR", TeraRangerI2CDevReg::CHANGE_BASE_ADDR, I2CRegisters::RegMode::WRONLY);

        set_register(TeraRangerI2CDevReg::WHO_AM_I, (uint8_t)0xA1);
    }

    void update(const class Aircraft &aircraft) override {
        // free running
        rangefinder_range = aircraft.rangefinder_range();
    }

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    float rangefinder_range;

    uint32_t reading_start_us;
};

} // namespace SITL

#endif  // AP_SIM_TERARANGERI2C_ENABLED
