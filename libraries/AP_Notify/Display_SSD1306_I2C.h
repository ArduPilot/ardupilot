/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "Display.h"
#include <AP_HAL/I2CDevice.h>

#define SSD1306_ROWS 128		    // display rows
#define SSD1306_COLUMNS 64		    // display columns
#define SSD1306_COLUMNS_PER_PAGE 8

class Display_SSD1306_I2C: public Display {
public:
    virtual bool hw_init();
    virtual bool hw_update();
    virtual bool set_pixel(uint16_t x, uint16_t y);
    virtual bool clear_pixel(uint16_t x, uint16_t y);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _displaybuffer[SSD1306_ROWS * SSD1306_COLUMNS_PER_PAGE];
};
