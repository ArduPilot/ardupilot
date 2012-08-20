
#ifndef __AP_HAL_ARDUINO_SPI_DRIVER_H__
#define __AP_HAL_ARDUINO_SPI_DRIVER_H__

#include <AP_HAL.h>

class AP_HAL::ArduinoSPIDriver : public AP_HAL::SPIDriver {
public:
    ArduinoSPIDriver() : _init(0) {}
    void init() { _init = 1; }
private:
    int _init;
};

#endif // __AP_HAL_ARDUINO_SPI_DRIVER_H__

