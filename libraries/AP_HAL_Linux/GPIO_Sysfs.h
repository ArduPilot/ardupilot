#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/AP_HAL.h>

#include "GPIO.h"

class Linux::DigitalSource_Sysfs : public AP_HAL::DigitalSource {
    friend class Linux::GPIO_Sysfs;
public:
    ~DigitalSource_Sysfs();
    uint8_t read();
    void write(uint8_t value);
    void mode(uint8_t output);
    void toggle();
private:
    /* Only GPIO_Sysfs will be able to instantiate */
    DigitalSource_Sysfs(unsigned pin, int value_fd);
    int _value_fd;
    unsigned _pin;
};

/**
 * Generic implementation of AP_HAL::GPIO for Linux based boards.
 */
class Linux::GPIO_Sysfs : public AP_HAL::GPIO {
    friend class Linux::DigitalSource_Sysfs;
public:
    /* Fill this table with the real pin numbers. */
    static const unsigned pin_table[];
    static const uint8_t n_pins;

    static GPIO_Sysfs *from(AP_HAL::GPIO *gpio) {
        return static_cast<GPIO_Sysfs*>(gpio);
    }

    void init();

    void pinMode(uint8_t vpin, uint8_t output) override;
    uint8_t read(uint8_t vpin) override;
    void write(uint8_t vpin, uint8_t value) override;
    void toggle(uint8_t vpin) override;

    /*
     * Export pin, instantiate a new DigitalSource_Sysfs and return its
     * pointer.
     */
    AP_HAL::DigitalSource *channel(uint16_t vpin) override;

    /*
     * Currently this function always returns -1.
     */
    int8_t analogPinToDigitalPin(uint8_t vpin) override;

    /*
     * Currently this function always returns false.
     */
    bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) override;

    /*
     * Currently this function always returns false.
     */
    bool usb_connected() override;

protected:
    void _pinMode(unsigned int pin, uint8_t output);
    int _open_pin_value(unsigned int pin, int flags);

    /*
     * Make pin available for use. This function should be called before
     * calling functions that use the pin number as parameter.
     *
     * Returns true if pin is exported successfully and false otherwise.
     *
     * Note: the pin is ignored if already exported.
     */
    static bool _export_pin(uint8_t vpin);
};
