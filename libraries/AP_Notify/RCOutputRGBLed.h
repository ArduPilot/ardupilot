#pragma once

#include "RGBLed.h"

class RCOutputRGBLed: public RGBLed {
public:
    RCOutputRGBLed(uint8_t red_channel, uint8_t green_channel,
                   uint8_t blue_channel, uint8_t led_off, uint8_t led_full,
                   uint8_t led_medium, uint8_t led_dim);
    RCOutputRGBLed(uint8_t red_channel, uint8_t green_channel,
                   uint8_t blue_channel);
    bool hw_init();
    bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

private:
    uint8_t _red_channel;
    uint8_t _green_channel;
    uint8_t _blue_channel;
};
