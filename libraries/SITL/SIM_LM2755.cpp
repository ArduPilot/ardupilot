#include "SIM_config.h"

#if AP_SIM_LM2755_ENABLED

#include "SIM_LM2755.h"

using namespace SITL;

#include <stdio.h>

void LM2755::init()
{
    add_register("GENERAL_PURPOSE", LM2755DevReg::GENERAL_PURPOSE, I2CRegisters::RegMode::WRONLY);
    add_register("TIME_STEP", LM2755DevReg::TIME_STEP, I2CRegisters::RegMode::WRONLY);

    add_register("D1_HIGH_LEVEL", LM2755DevReg::D1_HIGH_LEVEL, I2CRegisters::RegMode::WRONLY);
    add_register("D1_LOW_LEVEL", LM2755DevReg::D1_LOW_LEVEL, I2CRegisters::RegMode::WRONLY);
    add_register("D1_DELAY", LM2755DevReg::D1_DELAY, I2CRegisters::RegMode::WRONLY);
    add_register("D1_RAMP_UP_STEP_TIME", LM2755DevReg::D1_RAMP_UP_STEP_TIME, I2CRegisters::RegMode::WRONLY);
    add_register("D1_TIME_HIGH", LM2755DevReg::D1_TIME_HIGH, I2CRegisters::RegMode::WRONLY);
    add_register("D1_RAMP_DOWN_STEP_TIME", LM2755DevReg::D1_RAMP_DOWN_STEP_TIME, I2CRegisters::RegMode::WRONLY);
    add_register("D1_TIMING", LM2755DevReg::D1_TIMING, I2CRegisters::RegMode::WRONLY);

    add_register("D2_HIGH_LEVEL", LM2755DevReg::D2_HIGH_LEVEL, I2CRegisters::RegMode::WRONLY);
    add_register("D2_LOW_LEVEL", LM2755DevReg::D2_LOW_LEVEL, I2CRegisters::RegMode::WRONLY);
    add_register("D2_DELAY", LM2755DevReg::D2_DELAY, I2CRegisters::RegMode::WRONLY);
    add_register("D2_RAMP_UP_STEP_TIME", LM2755DevReg::D2_RAMP_UP_STEP_TIME, I2CRegisters::RegMode::WRONLY);
    add_register("D2_TIME_HIGH", LM2755DevReg::D2_TIME_HIGH, I2CRegisters::RegMode::WRONLY);
    add_register("D2_RAMP_DOWN_STEP_TIME", LM2755DevReg::D2_RAMP_DOWN_STEP_TIME, I2CRegisters::RegMode::WRONLY);
    add_register("D2_TIMING", LM2755DevReg::D2_TIMING, I2CRegisters::RegMode::WRONLY);

    add_register("D3_HIGH_LEVEL", LM2755DevReg::D3_HIGH_LEVEL, I2CRegisters::RegMode::WRONLY);
    add_register("D3_LOW_LEVEL", LM2755DevReg::D3_LOW_LEVEL, I2CRegisters::RegMode::WRONLY);
    add_register("D3_DELAY", LM2755DevReg::D3_DELAY, I2CRegisters::RegMode::WRONLY);
    add_register("D3_RAMP_UP_STEP_TIME", LM2755DevReg::D3_RAMP_UP_STEP_TIME, I2CRegisters::RegMode::WRONLY);
    add_register("D3_TIME_HIGH", LM2755DevReg::D3_TIME_HIGH, I2CRegisters::RegMode::WRONLY);
    add_register("D3_RAMP_DOWN_STEP_TIME", LM2755DevReg::D3_RAMP_DOWN_STEP_TIME, I2CRegisters::RegMode::WRONLY);
    add_register("D3_TIMING", LM2755DevReg::D3_TIMING, I2CRegisters::RegMode::WRONLY);


    // set startup register values.  Note that a lot of these are zero
    // in the datasheet and are zero when we allocate this object, so
    // code is commented out here

    // set_register(LM2755DevReg::GENERAL_PURPOSE, 0b00000000);
    set_register(LM2755DevReg::TIME_STEP, (uint8_t)0b10001000);

    set_register(LM2755DevReg::D1_HIGH_LEVEL, (uint8_t)0b11100000);
    set_register(LM2755DevReg::D1_LOW_LEVEL, (uint8_t)0b11100000);
    // set_register(LM2755DevReg::D1_DELAY, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D1_RAMP_UP_STEP_TIME, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D1_TIME_HIGH, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D1_RAMP_DOWN_STEP_TIME, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D1_TIMING, (uint8_t)0b00000000);

    set_register(LM2755DevReg::D2_HIGH_LEVEL, (uint8_t)0b11100000);
    set_register(LM2755DevReg::D2_LOW_LEVEL, (uint8_t)0b11100000);
    // set_register(LM2755DevReg::D2_DELAY, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D2_RAMP_UP_STEP_TIME, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D2_TIME_HIGH, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D2_RAMP_DOWN_STEP_TIME, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D2_TIMING, (uint8_t)0b00000000);

    set_register(LM2755DevReg::D3_HIGH_LEVEL, (uint8_t)0b11100000);
    set_register(LM2755DevReg::D3_LOW_LEVEL, (uint8_t)0b11100000);
    // set_register(LM2755DevReg::D3_DELAY, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D3_RAMP_UP_STEP_TIME, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D3_TIME_HIGH, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D3_RAMP_DOWN_STEP_TIME, (uint8_t)0b00000000);
    // set_register(LM2755DevReg::D3_TIMING, (uint8_t)0b00000000);

    // create objects for each of the channels to handle the dynamics
    // of updating each.  The channel gets references to the relevant
    // configuration bytes.
    d1 = NEW_NOTHROW LEDChannel(
        byte[(uint8_t)LM2755DevReg::D1_HIGH_LEVEL],
        byte[(uint8_t)LM2755DevReg::D1_LOW_LEVEL],
        byte[(uint8_t)LM2755DevReg::D1_DELAY],
        byte[(uint8_t)LM2755DevReg::D1_RAMP_UP_STEP_TIME],
        byte[(uint8_t)LM2755DevReg::D1_TIME_HIGH],
        byte[(uint8_t)LM2755DevReg::D1_RAMP_DOWN_STEP_TIME],
        byte[(uint8_t)LM2755DevReg::D1_TIMING]
        );
    d2 = NEW_NOTHROW LEDChannel(
        byte[(uint8_t)LM2755DevReg::D2_HIGH_LEVEL],
        byte[(uint8_t)LM2755DevReg::D2_LOW_LEVEL],
        byte[(uint8_t)LM2755DevReg::D2_DELAY],
        byte[(uint8_t)LM2755DevReg::D2_RAMP_UP_STEP_TIME],
        byte[(uint8_t)LM2755DevReg::D2_TIME_HIGH],
        byte[(uint8_t)LM2755DevReg::D2_RAMP_DOWN_STEP_TIME],
        byte[(uint8_t)LM2755DevReg::D2_TIMING]
        );
    d3 = NEW_NOTHROW LEDChannel(
        byte[(uint8_t)LM2755DevReg::D3_HIGH_LEVEL],
        byte[(uint8_t)LM2755DevReg::D3_LOW_LEVEL],
        byte[(uint8_t)LM2755DevReg::D3_DELAY],
        byte[(uint8_t)LM2755DevReg::D3_RAMP_UP_STEP_TIME],
        byte[(uint8_t)LM2755DevReg::D3_TIME_HIGH],
        byte[(uint8_t)LM2755DevReg::D3_RAMP_DOWN_STEP_TIME],
        byte[(uint8_t)LM2755DevReg::D3_TIMING]
        );

    rgbled.init();
}

void LM2755::update(const class Aircraft &aircraft)
{
    union {
        struct {
            uint8_t d1_high_current : 1;
            uint8_t d2_high_current : 1;
            uint8_t d3_high_current : 1;
            uint8_t d1_dimming : 1;
            uint8_t d2_dimming : 1;
            uint8_t d3_dimming : 1;
            uint8_t external_clock : 1;
            uint8_t charge_pump : 1;
        } bits;
        uint8_t byte;
    } gp;
    gp.byte = get_register(LM2755DevReg::GENERAL_PURPOSE);

    if (gp.byte & 0x7) {
        // disallow high current
        AP_HAL::panic("High current bits set");
    }
    if (gp.bits.charge_pump) {
        AP_HAL::panic("Charge pump set");
    }
    if (gp.bits.external_clock) {
        AP_HAL::panic("external clock set");
    }

    // update each channel.
    if (gp.bits.d1_dimming) {
        d1->update();
    }
    if (gp.bits.d2_dimming) {
        d2->update();
    }
    if (gp.bits.d3_dimming) {
        d3->update();
    }

    rgbled.set_colours(
        d1->current_value(),
        d2->current_value(),
        d3->current_value()
        );
}

void LM2755::LEDChannel::update()
{
    // calculate the brightness for this LED:

    // this is an awful simulation, which assumes that ArduPilot only
    // step-changtes the input for this device.  AP could do better in
    // terms of passing through the "throbber" logic down to the
    // device itself - at which point a *lot* of these sanity checks
    // will fail.

    if (high_level != low_level) {
        // we should not be dimming when high != low
        AP_HAL::panic("high != low");
    }
    if (high_level > 31) {
        AP_HAL::panic("Invalid high value %u", high_level);
    }
    output_value = high_level * 8;  // OK, so this is 0-248.  So Sue me.
}

#endif
