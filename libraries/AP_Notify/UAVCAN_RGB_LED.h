#pragma once

#include "RGBLed.h"

class UAVCAN_RGB_LED: public RGBLed {
public:
    UAVCAN_RGB_LED(uint8_t led_index, uint8_t led_off, uint8_t led_full,
                   uint8_t led_medium, uint8_t led_dim);
    UAVCAN_RGB_LED(uint8_t led_index);

protected:
    bool hw_init() override;
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override;

private:
    uint8_t _led_index;
};

