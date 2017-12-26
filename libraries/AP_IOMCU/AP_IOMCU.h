/*
  implement protocol for controlling an IO microcontroller

  For bootstrapping this will initially implement the px4io protocol,
  but will later move to an ArduPilot specific protocol
 */

#include <AP_HAL/AP_HAL.h>

class AP_IOMCU {
public:
    AP_IOMCU(AP_HAL::UARTDriver &uart);

    void write_channel(uint8_t chan, uint16_t pwm);
private:
    AP_HAL::UARTDriver &uart;
};
