#include "SIM_TI_DACx3204.h"

using namespace SITL;

TI_DACx3204::TI_DACx3204() :
    I2CDevice(),
    I2CRegisters_16Bit()
{

    add_register("NOP", TI_DACx3204RegEnum::NOP, SITL::I2CRegisters::RegMode::RDONLY);  // RO as we should never be touching it

    add_register("DAC-0-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-MARGIN-LOW", TI_DACx3204RegEnum::DAC_0_MARGIN_LOW, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-1-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-MARGIN-LOW", TI_DACx3204RegEnum::DAC_1_MARGIN_LOW, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-2-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-MARGIN-LOW", TI_DACx3204RegEnum::DAC_2_MARGIN_LOW, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-3-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-MARGIN-LOW", TI_DACx3204RegEnum::DAC_3_MARGIN_LOW, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-0-DATA", TI_DACx3204RegEnum::DAC_0_DATA, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-DATA", TI_DACx3204RegEnum::DAC_1_DATA, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-DATA", TI_DACx3204RegEnum::DAC_2_DATA, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-DATA", TI_DACx3204RegEnum::DAC_3_DATA, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("COMMON-CONFIG", TI_DACx3204RegEnum::COMMON_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("COMMON-TRIGGER", TI_DACx3204RegEnum::COMMON_TRIGGER, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("COMMON-DAC-TRIGGER", TI_DACx3204RegEnum::COMMON_DAC_TRIGGER, SITL::I2CRegisters::RegMode::RDONLY);

    // create objects for each of the channels to handle the dynamics
    // of updating each.  The channel gets references to the relevant
    // configuration words.
    dacs[0] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG]
    };
    dacs[1] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG]
    };
    dacs[2] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG]
    };
    dacs[3] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG]
    };
}

void TI_DACx3204::reset()
{
    set_register(TI_DACx3204RegEnum::NOP, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_0_DATA, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_DATA, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_DATA, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_DATA, 0);

    set_register(TI_DACx3204RegEnum::COMMON_CONFIG, 0xFFF);
    set_register(TI_DACx3204RegEnum::COMMON_TRIGGER, 0);
    set_register(TI_DACx3204RegEnum::COMMON_DAC_TRIGGER, 0);

    for (auto &dac : dacs) {
        dac->reset();
    }
}

// assert that data is either not a register write, or that it's OK
// for that register write to happen
void TI_DACx3204::assert_register_write_ok(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (!registers_are_locked) {
        return;
    }
    if (data->nmsgs != 1) {
        // not a register write
        return;
    }
    const uint8_t reg_base_addr = data->msgs[0].buf[0];
    if (reg_base_addr != TI_DACx3204RegEnum::COMMON_TRIGGER) {
        AP_HAL::panic("write to registers while they are locked");
    }
    const uint16_t register_value = data->msgs[0].buf[2] << 8 | data->msgs[0].buf[1];
    if (((register_value & common_trigger_reset_bits_mask) >> 8) != common_trigger_reset_bits_reset_value) {
        AP_HAL::panic("bad reset register value");
    }
}

int TI_DACx3204::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    assert_register_write_ok(data);
    return I2CRegisters_16Bit::rdwr(data);
}

// called periodically by simulation code; should read registers
// written by autopilot and act on them, updating state based on those
// registers and time passing.
void TI_DACx3204::update(const class Aircraft &aircraft)
{
    // handle changed in COMMON-TRIGGER
    const uint16_t common_trigger = get_reg_value(TI_DACx3204RegEnum::COMMON_TRIGGER);
    // reset triggered by 0b1010 in common-trigger; see page 61
    const uint16_t reset_bits = (common_trigger & common_trigger_reset_bits_mask) >> 8;
    if (reset_bits == common_trigger_reset_bits_reset_value) {
        reset();
    } else if (reset_bits) {
        AP_HAL::panic("Unexpected reset bits value");
    }

    // handle changes in COMMON-CONFIG (page 60)
    const uint16_t common_config = get_reg_value(TI_DACx3204RegEnum::COMMON_CONFIG);
    const bool lock_bit_set = (common_config & 1U<<14);  // 14 is DEV-LOCK
    if (lock_bit_set && !registers_are_locked) {
        registers_are_locked = true;
    } else if (!lock_bit_set && registers_are_locked) {
        if (((common_trigger >> 12) & 0b111) != 0b101) {  // DEV-UNLOCK magic value
            AP_HAL::panic("Attempt to unlock registers with bad DEV-UNLOCK value");
        }
    }

    for (auto &dac : dacs) {
        dac->update();
    }
}

// actual DAC constructor:
TI_DACx3204::DAC::DAC(uint16_t &_margin_high,
                      uint16_t &_margin_low,
                      uint16_t &_vout_cmp_config,
                      uint16_t &_vout_misc_config,
                      uint16_t &_vout_mode_config,
                      uint16_t &_vout_func_config) :
    margin_high{_margin_high},
    margin_low{_margin_low},
    vout_cmp_config{_vout_cmp_config},
    vout_misc_config{_vout_misc_config},
    vout_mode_config{_vout_mode_config},
    vout_func_config{_vout_func_config}
{
}

// power-on-reset:
void TI_DACx3204::DAC::reset()
{
}

void TI_DACx3204::DAC::update()
{
}
