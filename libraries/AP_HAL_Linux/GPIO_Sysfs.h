#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_HAL_Linux.h"

#include "GPIO.h"

class Linux::LinuxDigitalSource_Sysfs : public AP_HAL::DigitalSource {
    friend class Linux::LinuxGPIO_Sysfs;
public:
    ~LinuxDigitalSource_Sysfs();
    uint8_t read();
    void write(uint8_t value);
    void mode(uint8_t output);
    void toggle();
private:
    /* Only LinuxGPIO_Sysfs will be able to instantiate */
    LinuxDigitalSource_Sysfs(unsigned pin, int value_fd);
    int _value_fd;
    unsigned _pin;
};

/**
 * Generic implementation of AP_HAL::GPIO for Linux based boards.
 */
class Linux::LinuxGPIO_Sysfs : public AP_HAL::GPIO {
    friend class Linux::LinuxDigitalSource_Sysfs;
public:
    /* Fill this table with the real pin numbers. */
    static const unsigned pin_table[];

    /* Disabled APIs without VirtualPin */
    void pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void write(uint8_t pin, uint8_t value);
    void toggle(uint8_t pin);
    AP_HAL::DigitalSource* channel(uint16_t n);

    void init();

    void pinMode(enum VirtualPin pin, uint8_t output);
    uint8_t read(enum VirtualPin pin);
    void write(enum VirtualPin pin, uint8_t value);
    void toggle(enum VirtualPin pin);

    /*
     * Export pin n, instantiate a new LinuxDigitalSource_Sysfs and return its
     * pointer.
     */
    AP_HAL::DigitalSource* channel(enum VirtualPin vpin);

    /*
     * Currently this function always returns -1.
     */
    int8_t analogPinToDigitalPin(uint8_t pin);

    /*
     * Currently this function always returns false.
     */
    bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                          uint8_t mode);
    /*
     * Currently this function always returns false.
     */
    bool usb_connected(void);

    /*
     * Make pin available for use. This function should be called before
     * calling functions that use the pin number as parameter.
     *
     * Returns true if pin is exported successfully and false otherwise.
     *
     * Note: the pin is ignored if already exported.
     */
    static bool export_pin(enum VirtualPin pin);

    /*
     * Make pins available for use. This function should be called before
     * calling functions that use pin number as parameter.
     *
     * If all pins are exported successfully, true is returned. If there is an
     * error for one of them, false is returned.
     *
     * Note: pins already exported are ignored.
     */
    static bool export_pins(enum VirtualPin pins[], size_t num_pins);

protected:
    void _pinMode(unsigned int pin, uint8_t output);
    int _open_pin_value(unsigned int pin, int flags);
};

#endif
