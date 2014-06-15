
#ifndef __AP_HAL_MPNG_GPIO_H__
#define __AP_HAL_MPNG_GPIO_H__

#include <AP_HAL.h>
#include "AP_HAL_MPNG_Namespace.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_MPNG
# define HAL_GPIO_A_LED_PIN        13
# define HAL_GPIO_B_LED_PIN        31
# define HAL_GPIO_C_LED_PIN        30
# define HAL_GPIO_LED_ON           HIGH
# define HAL_GPIO_LED_OFF          LOW
# define HAL_GPIO_USB_MUX_PIN	    -1
#endif

class MPNG::AVRDigitalSource : public AP_HAL::DigitalSource {
public:
  AVRDigitalSource(uint8_t bit, uint8_t port) : _bit(bit), _port(port) {}
  void    mode(uint8_t output);
  uint8_t read();
  void    write(uint8_t value);
  void    toggle();

private:
  const uint8_t _bit;
  const uint8_t _port;
};

class MPNG::AVRGPIO : public AP_HAL::GPIO {
public:
    AVRGPIO() {}
    void    init() {}
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);
    AP_HAL::DigitalSource* channel(uint16_t);
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc proc,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);

/* private-ish: only to be used from the appropriate interrupt */
    static AP_HAL::Proc _interrupt_6;
};

#endif // __AP_HAL_MPNG_GPIO_H__

