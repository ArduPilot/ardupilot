
#ifndef __AP_HAL_ARDUINO_SPI_DRIVER_H__
#define __AP_HAL_ARDUINO_SPI_DRIVER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::ArduinoSPIDriver : public AP_HAL::SPIDriver {
public:
    ArduinoSPIDriver() {}
    void init(void* machtnichts);
    void set_freq(uint32_t freq_hz);
    uint8_t transfer(uint8_t data);
private:
    uint8_t _divider_bits(uint8_t divider);
    void _set_clock_divider_bits(uint8_t b);
};

#endif // __AP_HAL_ARDUINO_SPI_DRIVER_H__

