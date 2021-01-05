#include "SIM_INA3221.h"

#include <SITL/SITL.h>

#include <stdio.h>

SITL::INA3221::INA3221()
{
    writable_registers.set(0);
    writable_registers.set(7);
    writable_registers.set(8);
    writable_registers.set(9);
    writable_registers.set(10);
    writable_registers.set(11);
    writable_registers.set(12);
    writable_registers.set(14);
    writable_registers.set(15);
    writable_registers.set(16);
    writable_registers.set(254);
    writable_registers.set(255);

    reset();
}

void SITL::INA3221::reset()
{
    // from page 24 of datasheet:
    registers.byname.configuration.word = 0x7127;
    registers.byname.Channel_1_Shunt_Voltage = 0x0;
    registers.byname.Channel_1_Bus_Voltage = 0x0;
    registers.byname.Channel_2_Shunt_Voltage = 0x0;
    registers.byname.Channel_2_Bus_Voltage = 0x0;
    registers.byname.Channel_3_Shunt_Voltage = 0x0;
    registers.byname.Channel_3_Bus_Voltage = 0x0;
    registers.byname.Channel_1_CriticalAlertLimit = 0x7FF8;
    registers.byname.Channel_1_WarningAlertLimit = 0x7FF8;
    registers.byname.Channel_2_CriticalAlertLimit = 0x7FF8;
    registers.byname.Channel_2_WarningAlertLimit = 0x7FF8;
    registers.byname.Channel_3_CriticalAlertLimit = 0x7FF8;
    registers.byname.Channel_3_WarningAlertLimit = 0x7FF8;
    registers.byname.Shunt_VoltageSum = 0x0;
    registers.byname.Shunt_VoltageSumLimit = 0x7FFE;
    registers.byname.Mask_Enable = 0x0002;
    registers.byname.Power_ValidUpperLimit = 0x2710;
    registers.byname.Power_ValidLowerLimit = 0x2328;
    registers.byname.ManufacturerID = MANUFACTURER_ID;
    registers.byname.Die_ID = DIE_ID;
}

int SITL::INA3221::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_addr = data->msgs[0].buf[0];
        const uint16_t register_value = registers.word[reg_addr];
        data->msgs[1].buf[0] = register_value >> 8;
        data->msgs[1].buf[1] = register_value & 0xFF;
        data->msgs[1].len = 2;
        return 0;
    }

    if (data->nmsgs == 1) {
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_addr = data->msgs[0].buf[0];
        if (!writable_registers.get(reg_addr)) {
            AP_HAL::panic("Register 0x%02x is not writable!", reg_addr);
        }
        const uint16_t register_value = data->msgs[0].buf[2] << 8 | data->msgs[0].buf[1];
        // ::fprintf(stderr, "Setting register (0x%x) to 0x%x\n", reg_addr, register_value);
        registers.word[reg_addr] = register_value;
        return 0;
    }

    return -1;
};

void SITL::INA3221::update(const class Aircraft &aircraft)
{
    if (registers.byname.configuration.bits.reset != 0) {
        reset();
    }

    // update readings
    if (registers.byname.configuration.bits.mode == 0b000 ||
        registers.byname.configuration.bits.mode == 0b100) {
        // power-off
        return;
    }

    const bool update_shunt = registers.byname.configuration.bits.mode & 0b001;
    const bool update_bus = registers.byname.configuration.bits.mode & 0b010;
    if ((registers.byname.configuration.bits.mode & 0b100) == 0) {
        // single-shot only
        registers.byname.configuration.bits.mode &= ~0b011;
    }

    // channel 1 gets the first simulated battery's voltage and current:
    // see 8.6.6.2 on page 27 for the whole "40uV" thing
    if (registers.byname.configuration.bits.ch1_enable) {
        if (update_bus) {
            float fakevoltage = 12.3;
            registers.byname.Channel_1_Bus_Voltage = fakevoltage; // FIXME
        }
        if (update_shunt) {
            float fakecurrent = 7.6;
            registers.byname.Channel_1_Shunt_Voltage = fakecurrent/0.56; // FIXME
        }
    }

    if (registers.byname.configuration.bits.ch2_enable) {
        if (update_shunt) {
            registers.byname.Channel_2_Shunt_Voltage = AP::sitl()->state.battery_current/26 * 32768; // FIXME
        }
        if (update_bus) {
            registers.byname.Channel_2_Bus_Voltage = AP::sitl()->state.battery_voltage/26 * 32768; // FIXME
        }
    }
}
