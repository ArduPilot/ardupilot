#pragma once

#include "AP_HAL_SITL.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

class HALSITL::GPIO : public AP_HAL::GPIO {
public:
    explicit GPIO(SITL_State *sitlState): _sitlState(sitlState) {}
    void init() override;
    void pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void write(uint8_t pin, uint8_t value) override;
    void toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool usb_connected(void) override;

    bool valid_pin(uint8_t pin) const override { return pin < 16; }
    
private:
    SITL_State *_sitlState;

    uint8_t pin_mode_is_write;
};

class HALSITL::DigitalSource : public AP_HAL::DigitalSource {
public:
    explicit DigitalSource(uint8_t pin);
    void mode(uint8_t output) override;
    uint8_t read() override;
    void write(uint8_t value) override;
    void toggle() override;

private:
    uint8_t _pin;
};
#endif
