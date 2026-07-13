#pragma once

#include "AP_TemperatureSensor_Backend.h"

class AP_TemperatureSensor_MCP9808 : public AP_TemperatureSensor_Backend {

public:

    AP_TemperatureSensor_MCP9808(
        AP_TemperatureSensor &frontend,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    static AP_TemperatureSensor_Backend *probe(
        AP_TemperatureSensor &frontend,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void update() override;

private:

    bool init();

    bool read_temperature(float &temperature);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

};