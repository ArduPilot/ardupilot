#include "SIM_config.h"

#if AP_SIM_IS31FL3195_ENABLED

#include "SIM_IS31FL3195.h"

using namespace SITL;

#include <stdio.h>

void IS31FL3195::init()
{
    add_register("PRODUCT_ID", IS31FL3195DevReg::PRODUCT_ID, I2CRegisters::RegMode::RDONLY);
    add_register("SHUTDOWN_CONTROL", IS31FL3195DevReg::SHUTDOWN_CONTROL, I2CRegisters::RegMode::RDWR);

    add_register("P1_STATE", IS31FL3195DevReg::P1_STATE, I2CRegisters::RegMode::RDONLY);
    add_register("P2_STATE", IS31FL3195DevReg::P2_STATE, I2CRegisters::RegMode::RDWR);
    add_register("P3_STATE", IS31FL3195DevReg::P3_STATE, I2CRegisters::RegMode::RDWR);
    add_register("P4_STATE", IS31FL3195DevReg::P4_STATE, I2CRegisters::RegMode::RDWR);
    add_register("COLOUR_UPDATE", IS31FL3195DevReg::COLOUR_UPDATE, I2CRegisters::RegMode::WRONLY);

    add_register("OUT1", IS31FL3195DevReg::OUT1, I2CRegisters::RegMode::WRONLY);
    add_register("OUT2", IS31FL3195DevReg::OUT2, I2CRegisters::RegMode::WRONLY);
    add_register("OUT3", IS31FL3195DevReg::OUT3, I2CRegisters::RegMode::WRONLY);
    add_register("OUT4", IS31FL3195DevReg::OUT4, I2CRegisters::RegMode::WRONLY);

    add_register("RESET_REGISTER", IS31FL3195DevReg::RESET_REGISTER, I2CRegisters::RegMode::WRONLY);

    // set_register(IS31FL3195DevReg::GENERAL_PURPOSE, 0b00000000);

    reset_registers();

    rgbled.init();
}

void IS31FL3195::rdwr_store_register_value(uint8_t reg, uint8_t value)
{
    if (reg == IS31FL3195DevReg::COLOUR_UPDATE && value == 0xc5) {
        colour_update_register_poked = true;
    }
    I2CRegisters_8Bit::rdwr_store_register_value(reg, value);
}

int IS31FL3195::rdwr(I2C::i2c_rdwr_ioctl_data *&data) {
    return I2CRegisters_8Bit::rdwr(data);
}

void IS31FL3195::set_product_id(uint8_t product_id)
{
    set_register(IS31FL3195DevReg::PRODUCT_ID, uint8_t(product_id << 1U));
}


void IS31FL3195::reset_registers()
{
    // see page 11 of datasheet for reset states
    set_register(IS31FL3195DevReg::SHUTDOWN_CONTROL, (uint8_t)0b11110000);

    set_register(IS31FL3195DevReg::P1_STATE, (uint8_t)0);
    set_register(IS31FL3195DevReg::P2_STATE, (uint8_t)0);
    set_register(IS31FL3195DevReg::P3_STATE, (uint8_t)0);
    set_register(IS31FL3195DevReg::P4_STATE, (uint8_t)0);

    set_register(IS31FL3195DevReg::OUT1, (uint8_t)0);
    set_register(IS31FL3195DevReg::OUT2, (uint8_t)0);
    set_register(IS31FL3195DevReg::OUT3, (uint8_t)0);
    set_register(IS31FL3195DevReg::OUT4, (uint8_t)0);

    set_register(IS31FL3195DevReg::RESET_REGISTER, (uint8_t)0);
}

void IS31FL3195::update(const class Aircraft &aircraft)
{
    if (get_register(IS31FL3195DevReg::RESET_REGISTER) == 0xC5) {
        reset_registers();
    }
    if (get_register(IS31FL3195DevReg::SHUTDOWN_CONTROL) == 0xf0) {
        // in shutdown
        return;
    }

    if (!colour_update_register_poked) {
        return;
    }
    colour_update_register_poked = false;

    const uint8_t red = get_register(IS31FL3195DevReg::OUT1);
    const uint8_t green = get_register(IS31FL3195DevReg::OUT2);
    const uint8_t blue = get_register(IS31FL3195DevReg::OUT3);

    rgbled.set_colours(red, green, blue);
}

#endif
