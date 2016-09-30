#pragma once

#include "Display.h"
#include "Display_OLED_I2C.h"
#include <AP_HAL/I2CDevice.h>

#define SH1106_COLUMNS 132		// display columns
#define SH1106_ROWS 64		    // display rows
#define SH1106_ROWS_PER_PAGE 8

class Display_SH1106_I2C: public Display_OLED_I2C {
public:
    static bool hw_autodetect() { return true; }

protected:
    virtual bool hw_init();
    virtual bool hw_update();
    virtual bool set_pixel(uint16_t x, uint16_t y);
    virtual bool clear_pixel(uint16_t x, uint16_t y);
    virtual bool clear_screen();

    virtual bool _update_timer();

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _displaybuffer[SH1106_COLUMNS * SH1106_ROWS_PER_PAGE];
    bool _need_hw_update;
};
