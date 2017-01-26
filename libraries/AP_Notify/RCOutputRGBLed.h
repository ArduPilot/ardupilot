#pragma once

#include "RGBLed.h"

class RCOutputRGBLed: public RGBLed {
public:
    RCOutputRGBLed(uint8_t red_channel, uint8_t green_channel,
                   uint8_t blue_channel, uint8_t led_off, uint8_t led_full,
                   uint8_t led_medium, uint8_t led_dim);
    RCOutputRGBLed(uint8_t red_channel, uint8_t green_channel,
                   uint8_t blue_channel);

protected:
    bool hw_init() override;
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override;

private:
    uint8_t _red_channel;
    uint8_t _green_channel;
    uint8_t _blue_channel;
};

class RCOutputRGBLedOff : public RCOutputRGBLed {
public:
    RCOutputRGBLedOff(uint8_t red_channel, uint8_t green_channel,
                      uint8_t blue_channel, uint8_t led_off)
        : RCOutputRGBLed(red_channel, green_channel, blue_channel,
                         led_off, led_off, led_off, led_off)
    { }

    /* Override the hw_set_rgb method to turn leds off regardless of the
     * values passed */
    bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override
    {
        return RCOutputRGBLed::hw_set_rgb(_led_off, _led_off, _led_off);
    }
};
