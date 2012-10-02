
#ifndef __AP_HAL_AVR_GPIO_H__
#define __AP_HAL_AVR_GPIO_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::ArduinoDigitalSource : public AP_HAL::DigitalSource {
public:
  ArduinoDigitalSource(int pin) : _pin(pin) {}
  void    mode(uint8_t output);
  uint8_t read();
  void    write(uint8_t value);

  static ArduinoGPIO* parent;
private:
  int _pin;
};

class AP_HAL_AVR::ArduinoGPIO : public AP_HAL::GPIO {
public:
    ArduinoGPIO() {}
    void    init() {}
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    AP_HAL::DigitalSource* channel(int);
};

#endif // __AP_HAL_AVR_GPIO_H__

