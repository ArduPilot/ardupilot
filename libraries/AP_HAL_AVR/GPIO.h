
#ifndef __AP_HAL_AVR_GPIO_H__
#define __AP_HAL_AVR_GPIO_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRDigitalSource : public AP_HAL::DigitalSource {
public:
  AVRDigitalSource(uint8_t bit, uint8_t port) : _bit(bit), _port(port) {}
  void    mode(uint8_t output);
  uint8_t read();
  void    write(uint8_t value);

private:
  const uint8_t _bit;
  const uint8_t _port;
};

class AP_HAL_AVR::AVRGPIO : public AP_HAL::GPIO {
public:
    AVRGPIO() {}
    void    init() {}
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    AP_HAL::DigitalSource* channel(uint16_t);
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc proc,
            uint8_t mode);
/* private-ish: only to be used from the appropriate interrupt */
    static AP_HAL::Proc _interrupt_6;
};

#endif // __AP_HAL_AVR_GPIO_H__

