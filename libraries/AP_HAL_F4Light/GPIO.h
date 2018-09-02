
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_F4Light_Namespace.h"
#include "handler.h"
#include <boards.h>
#include <exti.h>

#ifndef HIGH
 #define HIGH 0x1
#endif

#ifndef LOW
 #define LOW  0x0
#endif


#ifndef HAL_GPIO_LED_ON
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif



typedef enum HALPinMode {
    INPUT = GPIO_INPUT_FLOATING, /**< Basic digital input. The pin voltage is sampled; when
              it is closer to 3.3v (Vcc) the pin status is high, and
              when it is closer to 0v (ground) it is low. If no
              external circuit is pulling the pin voltage to high or
              low, it will tend to randomly oscillate and be very
              sensitive to noise (e.g., a breath of air across the pin
              might cause the state to flip). */

    OUTPUT = GPIO_OUTPUT_PP, /* Basic digital output: when the pin is HIGH, the
               voltage is held at +3.3v (Vcc) and when it is LOW, it
               is pulled down to ground. */
    
    OUTPUT_ALT = GPIO_AF_OUTPUT_PP, /* basic alternate function mode */
    
// more complex modes

//    INPUT_FLOATING = GPIO_INPUT_FLOATING, /**< Synonym for INPUT. */

    INPUT_ANALOG = GPIO_INPUT_ANALOG, /**< This is a special mode for when the pin will be
                     used for analog (not digital) reads.  Enables ADC
                     conversion to be performed on the voltage at the
                     pin. */

    INPUT_PULLDOWN = GPIO_INPUT_PD, /**< The state of the pin in this mode is reported
                       the same way as with INPUT, but the pin voltage
                       is gently "pulled down" towards 0v. This means
                       the state will be low unless an external device
                       is specifically pulling the pin up to 3.3v, in
                       which case the "gentle" pull down will not
                       affect the state of the input. */

    INPUT_PULLUP = GPIO_INPUT_PU, /**< The state of the pin in this mode is reported
                     the same way as with INPUT, but the pin voltage
                     is gently "pulled up" towards +3.3v. This means
                     the state will be high unless an external device
                     is specifically pulling the pin down to ground,
                     in which case the "gentle" pull up will not
                     affect the state of the input. */


    OUTPUT_OPEN_DRAIN = GPIO_OUTPUT_OD, /**< In open drain mode, the pin indicates
                          "low" by accepting current flow to ground
                          and "high" by providing increased
                          impedance. An example use would be to
                          connect a pin to a bus line (which is pulled
                          up to a positive voltage by a separate
                          supply through a large resistor). When the
                          pin is high, not much current flows through
                          to ground and the line stays at positive
                          voltage; when the pin is low, the bus
                          "drains" to ground with a small amount of
                          current constantly flowing through the large
                          resistor from the external supply. In this
                          mode, no current is ever actually sourced
                          from the pin. */



    OUTPUT_OPEN_DRAIN_PU = GPIO_OUTPUT_OD_PU, /**< open drain mode with pull-up */

    PWM = GPIO_PIN_MODE_LAST, /**< This is a special mode for when the pin will be used for
                            PWM output (a special case of digital output). */

    PWM_OPEN_DRAIN, /**< Like PWM, except that instead of alternating
                       cycles of LOW and HIGH, the voltage on the pin
                       consists of alternating cycles of LOW and
                       floating (disconnected). */
} HAL_PinMode;


// HAL_GPIO_INTERRUPT_ not used anywhere

typedef enum ExtIntTriggerMode {
    RISING  = (uint8_t)EXTI_RISING, /**< To trigger an interrupt when the pin transitions LOW to HIGH */
    FALLING = (uint8_t)EXTI_FALLING, /**< To trigger an interrupt when the pin transitions   HIGH to LOW */
    CHANGE  = (uint8_t)EXTI_RISING_FALLING/**< To trigger an interrupt when the pin transitions from LOW to HIGH or HIGH to LOW (i.e., when the pin changes). */
} ExtIntTriggerMode;


class F4Light::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(const gpio_dev *device, uint8_t bit): _device(device), _bit(bit){ }

    void    mode(uint8_t output);

    uint8_t read() {                 return _read(); }
    void    write(uint8_t value) {   _write(value); }

    inline uint8_t _read() {                 return gpio_read_bit(_device, _bit) ? HIGH : LOW; }
    inline void    _write(uint8_t value) {   gpio_write_bit(_device, _bit, value); }

    inline void    toggle() {               gpio_toggle_bit(_device, _bit); }

private:
    const gpio_dev *_device;
    uint8_t _bit;
};


class F4Light::GPIO : public AP_HAL::GPIO {
public:
    GPIO() {};
    void    init() override {   gpio_init_all(); }
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    static bool    _attach_interrupt(uint8_t pin, Handler h, uint8_t mode, uint8_t priority);

    inline bool    attach_interrupt(uint8_t pin, AP_HAL::MemberProc p, uint8_t mode) { 
        Revo_handler h = { .mp = p  };  
        return _attach_interrupt(pin, h.h, mode, GPIO_INT_PRIORITY);   
    }

    inline bool    attach_interrupt(uint8_t pin, AP_HAL::Proc p, uint8_t mode) {       
        Revo_handler h = { .hp = p  };  
        return _attach_interrupt(pin, h.h, mode, GPIO_INT_PRIORITY);   
    }
    
    void           detach_interrupt(uint8_t pin);
    static inline void enable_interrupt(uint8_t pin, bool e) { exti_enable_interrupt((afio_exti_num)(PIN_MAP[pin].gpio_bit), e); }

    /* return true if USB cable is connected */
    inline bool    usb_connected(void) override {
        return gpio_read_bit(PIN_MAP[BOARD_USB_SENSE].gpio_device,PIN_MAP[BOARD_USB_SENSE].gpio_bit);
    }
    
// internal usage static versions
    static void           _pinMode(uint8_t pin, uint8_t output);
    static inline uint8_t _read(uint8_t pin) { const stm32_pin_info &pp = PIN_MAP[pin];   return gpio_read_bit( pp.gpio_device, pp.gpio_bit) ? HIGH : LOW; }
    static inline void    _write(uint8_t pin, uint8_t value) { const stm32_pin_info &pp = PIN_MAP[pin]; gpio_write_bit(pp.gpio_device, pp.gpio_bit, value); }
    static inline void    _toggle(uint8_t pin) {               const stm32_pin_info &p = PIN_MAP[pin];  gpio_toggle_bit(p.gpio_device, p.gpio_bit);  }
    
    static inline void    _setSpeed(uint8_t pin, GPIOSpeed_t gpio_speed) { const stm32_pin_info &pp = PIN_MAP[pin]; gpio_set_speed(pp.gpio_device, pp.gpio_bit, gpio_speed);}

    static inline DigitalSource* get_channel(uint16_t pin) { const stm32_pin_info &pp = PIN_MAP[pin]; return new DigitalSource(pp.gpio_device, pp.gpio_bit); }
};


static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode) { 
    return (exti_trigger_mode)mode;
}

extern "C" {
    void digitalWrite(uint8_t pin, uint8_t value);
    uint8_t digitalRead(uint8_t pin);
    void digitalToggle(uint8_t pin);
}


