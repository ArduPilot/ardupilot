#pragma once

#include "Display.h"

#if  CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define OLED_I2C_BUS 2
#else
#define OLED_I2C_BUS 1
#endif

class Display_OLED_I2C: public Display {

protected:
    virtual bool hw_init() { return hasInstance() && getInstance()->hw_init(); }
    virtual bool hw_update() { return hasInstance() && getInstance()->hw_update(); }
    virtual bool set_pixel(uint16_t x, uint16_t y) { return hasInstance() && getInstance()->set_pixel(x, y); }
    virtual bool clear_pixel(uint16_t x, uint16_t y) { return hasInstance() && getInstance()->clear_pixel(x, y); }
    virtual bool clear_screen() { return hasInstance() && getInstance()->clear_screen(); }

    virtual bool _update_timer() { return true; };

private:
    bool hasInstance() { return getInstance() != nullptr; }

    AP_HAL::OwnPtr<Display_OLED_I2C> _instance;
    Display_OLED_I2C* getInstance();
};
