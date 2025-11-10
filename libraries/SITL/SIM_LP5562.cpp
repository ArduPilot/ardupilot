#include "SIM_config.h"

#if AP_SIM_LP5562_ENABLED

#include "SIM_LP5562.h"

using namespace SITL;

#include <stdio.h>

void LP5562::init()
{
    add_register("ENABLE", LP5562DevReg::ENABLE, I2CRegisters::RegMode::RDWR);
    add_register("OP_MODE", LP5562DevReg::OP_MODE, I2CRegisters::RegMode::RDWR);

    add_register("B_CURRENT", LP5562DevReg::B_CURRENT, I2CRegisters::RegMode::RDWR);
    add_register("G_CURRENT", LP5562DevReg::G_CURRENT, I2CRegisters::RegMode::RDWR);
    add_register("R_CURRENT", LP5562DevReg::R_CURRENT, I2CRegisters::RegMode::RDWR);

    add_register("B_PWM", LP5562DevReg::B_PWM, I2CRegisters::RegMode::RDWR);
    add_register("G_PWM", LP5562DevReg::G_PWM, I2CRegisters::RegMode::RDWR);
    add_register("R_PWM", LP5562DevReg::R_PWM, I2CRegisters::RegMode::RDWR);

    add_register("CONFIG", LP5562DevReg::CONFIG, I2CRegisters::RegMode::RDWR);

    add_register("STATUS", LP5562DevReg::STATUS, I2CRegisters::RegMode::RDONLY);
    add_register("RESET", LP5562DevReg::RESET, I2CRegisters::RegMode::RDWR);

    add_register("LED_MAP", LP5562DevReg::LED_MAP, I2CRegisters::RegMode::RDWR);

    reset_registers();

    // create objects for each of the channels to handle the dynamics
    // of updating each.  The channel gets references to the relevant
    // configuration bytes.  This seems over-kill for the way we
    // currently use this device but is a structure we use elsewhere.
    // Note that this assumed that the LED Map register is set to its
    // default value.
    b = NEW_NOTHROW LEDChannel(
        byte[(uint8_t)LP5562DevReg::B_PWM]
        );
    g = NEW_NOTHROW LEDChannel(
        byte[(uint8_t)LP5562DevReg::G_PWM]
        );
    r = NEW_NOTHROW LEDChannel(
        byte[(uint8_t)LP5562DevReg::R_PWM]
        );

    rgbled.init();
}

void LP5562::reset_registers()
{
    set_register(LP5562DevReg::LED_MAP, (uint8_t)0b00111001);
    set_register(LP5562DevReg::B_CURRENT, (uint8_t)0xAF);
    set_register(LP5562DevReg::G_CURRENT, (uint8_t)0xAF);
    set_register(LP5562DevReg::R_CURRENT, (uint8_t)0xAF);
    set_register(LP5562DevReg::RESET, (uint8_t)0);
}

void LP5562::update(const class Aircraft &aircraft)
{
    {    // check the state of the reset register
        union {
            struct {
                uint8_t reset;
            } bits;
            uint8_t byte;
        } reset_reg;
        reset_reg.byte = get_register(LP5562DevReg::RESET);

        // 0xff in this register resets the device:
        if (reset_reg.bits.reset == 0xFF) {
            reset_registers();
        }
    }

    {    // check the state of the enable register
        union {
            struct {
                uint8_t eng3_exc : 2;
                uint8_t eng2_exc : 2;
                uint8_t eng1_exc : 2;
                uint8_t chip_en : 1;
                uint8_t log_en : 1;
            } bits;
            uint8_t byte;
        } enable_reg;
        enable_reg.byte = get_register(LP5562DevReg::ENABLE);

        // we don't get fancy with the device:
        if (enable_reg.bits.eng3_exc || enable_reg.bits.eng2_exc || enable_reg.bits.eng1_exc) {
            AP_HAL::panic("engine execute bits set");
        }

        if (!enable_reg.bits.chip_en) {
            // do nothing, maintain outputs; not sure what actual behaviour is
            return;
        }
    }

    {    // check the state of the config register
        union {
            struct {
                uint8_t internal_clock : 1;
                uint8_t autodetect : 1;
                uint8_t unused2 : 1;
                uint8_t unused3 : 1;
                uint8_t unused4 : 1;
                uint8_t pwrsav_en : 1;
                uint8_t pwm_hf : 1;
            } bits;
            uint8_t byte;
        } config_reg;
        config_reg.byte = get_register(LP5562DevReg::CONFIG);
        // we don't get fancy with the device.  Until the driver sets
        // internal clock enabled we do nothing (assume no external clock)
        if (config_reg.bits.internal_clock != 1) {
            fprintf(stderr, "Waiting for internal clock bits to be set (current=0x%02x)\n", config_reg.byte);
            return;
        }
    }

    {    // check the state of the LED Map register
        union {
            struct {
                uint8_t b_eng_sel : 2;
                uint8_t g_eng_sel : 2;
                uint8_t r_eng_sel : 2;
                uint8_t w_eng_sel : 2;
            } bits;
            uint8_t byte;
        } led_map_reg;
        led_map_reg.byte = get_register(LP5562DevReg::LED_MAP);
        // we don't get fancy with the device.  We expect this to be
        // at its reset value
        if (led_map_reg.bits.b_eng_sel != 0x00 ||
            led_map_reg.bits.g_eng_sel != 0x00 ||
            led_map_reg.bits.r_eng_sel != 0x00
            ) {
            fprintf(stderr, "Waiting for LED mode to be set to take-from-register\n");
            return;
        }
    }

    // update each channel.
    b->update();
    g->update();
    r->update();

    rgbled.set_colours(
        r->current_value(),
        g->current_value(),
        b->current_value()
        );
}

void LP5562::LEDChannel::update()
{
    // nothing to update here ATM; the output value is just the input value
}

#endif
