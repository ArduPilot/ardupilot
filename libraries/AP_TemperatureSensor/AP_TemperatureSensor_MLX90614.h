#pragma once

#include "AP_TemperatureSensor_Backend.h"

#if AP_TEMPERATURE_SENSOR_MLX90614_ENABLED


class AP_TemperatureSensor_MLX90614 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;
public:

    __INITFUNC__ void init(void) override;

    void update() override {};

private:
   
    // update the temperature, called at 20Hz
    void _timer(void);

    uint16_t read_data(uint8_t cmd);
    uint16_t read_eeprom(uint8_t address) {return read_data(address | 0x20);};
};
#endif  // AP_TEMPERATURE_SENSOR_MLX90614_ENABLED
