#pragma once

#include "Display.h"
#include "Display_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define SH1106_COLUMNS 132		// display columns
#define SH1106_ROWS 64		    // display rows
#define SH1106_ROWS_PER_PAGE 8

class Display_SH1106_I2C: public Display_Backend {

public:

    static Display_SH1106_I2C *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    void hw_update() override;
    void set_pixel(uint16_t x, uint16_t y) override;
    void clear_pixel(uint16_t x, uint16_t y) override;
    void clear_screen() override;

protected:

    Display_SH1106_I2C(AP_HAL::OwnPtr<AP_HAL::Device> dev);
    ~Display_SH1106_I2C() override;

private:

    bool hw_init() override;

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    uint8_t _displaybuffer[SH1106_COLUMNS * SH1106_ROWS_PER_PAGE];
    AP_HAL::Semaphore *_displaybuffer_sem;
    bool _need_hw_update;

};
