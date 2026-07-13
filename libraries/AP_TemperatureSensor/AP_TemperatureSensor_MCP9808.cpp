#include "AP_TemperatureSensor_MCP9808.h"

extern const AP_HAL::HAL& hal;


AP_TemperatureSensor_MCP9808::AP_TemperatureSensor_MCP9808(
    AP_TemperatureSensor &frontend,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
    AP_TemperatureSensor_Backend(frontend),
    _dev(std::move(dev))
{

}


AP_TemperatureSensor_Backend *
AP_TemperatureSensor_MCP9808::probe(
    AP_TemperatureSensor &frontend,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_TemperatureSensor_MCP9808 *sensor =
        new AP_TemperatureSensor_MCP9808(frontend, std::move(dev));

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_TemperatureSensor_MCP9808::init()
{
    // Check MCP9808 ID registers here

    return true;
}


void AP_TemperatureSensor_MCP9808::update()
{
    float temp;

    if (read_temperature(temp)) {
        set_temperature(temp);
    }
}


bool AP_TemperatureSensor_MCP9808::read_temperature(float &temperature)
{
    // Read temperature register here

    return true;
}