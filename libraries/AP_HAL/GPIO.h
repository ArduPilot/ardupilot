#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_ALT    2

class AP_HAL::DigitalSource {
public:
    virtual void    mode(uint8_t output) = 0;
    virtual uint8_t read() = 0;
    virtual void    write(uint8_t value) = 0;
    virtual void    toggle() = 0;
};

class AP_HAL::PWMSource {
public:

    // Destructor detaches interrupt
    ~PWMSource();

    bool set_pin(int16_t new_pin, const char *subsystem);
    int16_t pin() const { return _pin; }  // returns pin this is attached to

    uint16_t get_pwm_us();            // return last measured PWM input
    uint16_t get_pwm_avg_us();        // return average PWM since last call to get_pwm_avg_us

private:
    uint16_t _irq_value_us;         // last calculated pwm value (irq copy)
    uint32_t _pulse_start_us;       // system time of start of pulse
    int16_t _pin = -1;

    uint32_t _irq_value_us_sum;     // for get_pwm_avg_us
    uint32_t _irq_value_us_count;   // for get_pwm_avg_us

    bool interrupt_attached;

    // PWM input handling
    void irq_handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);
};

class AP_HAL::GPIO {
public:
    GPIO() {}
    virtual void    init() = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;

    // optional interface on some boards
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    virtual uint8_t read(uint8_t pin) = 0;
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    virtual void    toggle(uint8_t pin) = 0;
    virtual bool    valid_pin(uint8_t pin) const { return true; }

    // return servo channel associated with GPIO pin.  Returns true on success and fills in servo_ch argument
    // servo_ch uses zero-based indexing
    virtual bool    pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const { return false; }

    // allow for save and restore of pin settings
    virtual bool    get_mode(uint8_t pin, uint32_t &mode) { return false; }
    virtual void    set_mode(uint8_t pin, uint32_t mode) {}

    /* Alternative interface: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    enum INTERRUPT_TRIGGER_TYPE {
        INTERRUPT_NONE,
        INTERRUPT_FALLING,
        INTERRUPT_RISING,
        INTERRUPT_BOTH,
    };

    /* Interrupt interface: */
    //                                ret , pin    , state,timestamp
    // where:
    //    ret indicates the functor must return void
    //    pin is the pin which has triggered the interrupt
    //    state is the new state of the pin
    //    timestamp is the time in microseconds the interrupt occured
    FUNCTOR_TYPEDEF(irq_handler_fn_t, void, uint8_t, bool, uint32_t);
    virtual bool    attach_interrupt(uint8_t pin,
                                     irq_handler_fn_t fn,
                                     INTERRUPT_TRIGGER_TYPE mode) {
        return false;
    }

    virtual bool    attach_interrupt(uint8_t pin,
                                     AP_HAL::Proc proc,
                                     INTERRUPT_TRIGGER_TYPE mode) {
        return false;
    }
    bool detach_interrupt(uint8_t pin) {
        if (attach_interrupt(pin, (irq_handler_fn_t)nullptr, AP_HAL::GPIO::INTERRUPT_NONE)) {
            return true;
        }
        return attach_interrupt(pin, (AP_HAL::Proc)nullptr, AP_HAL::GPIO::INTERRUPT_NONE);
    }

    /*
      block waiting for a pin to change. A timeout of 0 means wait
      forever. Return true on pin change, false on timeout
     */
    virtual bool wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us) { return false; }

    /* return true if USB cable is connected */
    virtual bool    usb_connected(void) = 0;

    // optional timer tick
    virtual void timer_tick(void) {};
};
