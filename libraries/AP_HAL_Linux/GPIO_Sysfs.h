#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "GPIO.h"

namespace Linux {

class DigitalSource_Sysfs : public AP_HAL::DigitalSource {
    friend class GPIO_Sysfs;
public:
    ~DigitalSource_Sysfs();
    uint8_t read() override;
    void write(uint8_t value) override;
    void mode(uint8_t output) override;
    void toggle() override;
private:
    /* Only GPIO_Sysfs will be able to instantiate */
    DigitalSource_Sysfs(unsigned pin, int value_fd);
    int _value_fd;
    unsigned _pin;
};

/**
 * Generic implementation of AP_HAL::GPIO for Linux based boards.
 */
class GPIO_Sysfs : public AP_HAL::GPIO {
    friend class DigitalSource_Sysfs;
public:
    /* Fill this table with the real pin numbers. */
    static const unsigned pin_table[];
    static const uint8_t n_pins;

    static GPIO_Sysfs *from(AP_HAL::GPIO *gpio) {
        return static_cast<GPIO_Sysfs*>(gpio);
    }

    void init() override;

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

#ifdef HAL_GPIO_SCRIPT
    /*
      support for calling external scripts based on GPIO writes
     */
    void _gpio_script_write(uint8_t vpin, uint8_t value);

    /*
      thread to run scripts
     */
    void _gpio_script_thread(void);

    /*
      control structures for _gpio_script_write
     */
    typedef struct {
        uint8_t pin;
        uint8_t value;
    } pin_value_t;

    struct {
        bool thread_created;
        ObjectBuffer<pin_value_t> pending{10};
    } _script;
#endif // HAL_GPIO_SCRIPT
};

}
