#include "SIM_config.h"

#if AP_SIM_INA3221_ENABLED

#include "SIM_INA3221.h"

#include <SITL/SITL.h>

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
        registers.word[reg_addr] = register_value;
        return 0;
    }

    return -1;
};

static uint16_t convert_voltage(float voltage_v) {
    // 8mV per count, register value is x8 (3 lowest bits not used)
    float volt_counts = (voltage_v/8e-3)*8;
    if (volt_counts < INT16_MIN) {
        volt_counts = INT16_MIN;
    } else if (volt_counts > INT16_MAX) {
        volt_counts = INT16_MAX;
    }

    // register value is signed
    return (uint16_t)((int16_t)volt_counts);
}

static uint16_t convert_current(float current_a) {
    // V = IR
    const float shunt_resistance_ohms = 0.001; // default value in driver
    const float shunt_voltage = current_a*shunt_resistance_ohms;

    // 40uV per count, register value is x8 (3 lowest bits not used)
    float shunt_counts = (shunt_voltage/40e-6)*8;
    if (shunt_counts < INT16_MIN) {
        shunt_counts = INT16_MIN;
    } else if (shunt_counts > INT16_MAX) {
        shunt_counts = INT16_MAX;
    }

    // register value is signed
    return (uint16_t)((int16_t)shunt_counts);
}


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

    // channel 2 gets the first simulated battery's voltage and current, others are test values:
    if (registers.byname.configuration.bits.ch1_enable) {
        // values close to chip limits (assuming 1mOhm current shunt)
        if (update_bus) {
            registers.byname.Channel_1_Bus_Voltage = convert_voltage(25); // max of 26V
        }
        if (update_shunt) {
            registers.byname.Channel_1_Shunt_Voltage = convert_current(160); // max of 163.8A
        }
    }

    if (registers.byname.configuration.bits.ch2_enable) {
        if (update_bus) {
            registers.byname.Channel_2_Bus_Voltage = convert_voltage(AP::sitl()->state.battery_voltage);
        }
        if (update_shunt) {
            registers.byname.Channel_2_Shunt_Voltage = convert_current(AP::sitl()->state.battery_current);
        }
    }

    if (registers.byname.configuration.bits.ch3_enable) {
        // values different from above to test channel switching
        if (update_bus) {
            registers.byname.Channel_3_Bus_Voltage = convert_voltage(3.14159);
        }
        if (update_shunt) {
            registers.byname.Channel_3_Shunt_Voltage = convert_current(2.71828);
        }
    }
}

#endif  // AP_SIM_INA3221_ENABLED
