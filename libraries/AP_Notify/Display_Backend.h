#pragma once

#include "Display.h"

#if  CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define OLED_I2C_BUS 2
#else
#define OLED_I2C_BUS 1
#endif

class Display_Backend {

public:
    virtual bool hw_init() = 0;
    virtual bool hw_update() = 0;
    virtual bool set_pixel(uint16_t x, uint16_t y) = 0;
    virtual bool clear_pixel(uint16_t x, uint16_t y) = 0;
    virtual bool clear_screen() = 0;
};
