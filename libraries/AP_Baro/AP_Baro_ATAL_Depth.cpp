#include "AP_Baro_ATAL_Depth.h"

#if AP_BARO_ATAL_DEPTH_ENABLED

AP_Baro_ATAL_Depth::AP_Baro_ATAL_Depth(AP_Baro &baro, AP_HAL::UARTDriver *_uart,
                                       uint8_t serial_port) :
    AP_Baro_Backend(baro),
    uart(_uart)
{
    instance = _frontend.register_sensor();
    set_bus_id(instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,
                                                     serial_port, 0, DEVTYPE_BARO_ATAL_DEPTH));
    _frontend.set_type(instance, AP_Baro::BARO_TYPE_WATER);
}

void AP_Baro_ATAL_Depth::update()
{
    uint32_t available = MIN(uart->available(), 512u);
    while (available-- > 0) {
        uint8_t byte;
        if (!uart->read(byte)) {
            break;
        }
        AP_Baro_ATAL_Depth_Parser::Sample sample;
        if (parser.feed(byte, sample) && pressure_ok(sample.pressure_pa)) {
            _copy_to_frontend(instance, sample.pressure_pa, sample.temperature_c);
        }
    }
}

#endif // AP_BARO_ATAL_DEPTH_ENABLED
