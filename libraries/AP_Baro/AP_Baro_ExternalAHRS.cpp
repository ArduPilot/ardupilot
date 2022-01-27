#include "AP_Baro_ExternalAHRS.h"

#if AP_BARO_EXTERNALAHRS_ENABLED

AP_Baro_ExternalAHRS::AP_Baro_ExternalAHRS(AP_Baro &baro, uint8_t port) :
    AP_Baro_Backend(baro)
{
    instance = _frontend.register_sensor();
    set_bus_id(instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,port,0,0));
}

// Read the sensor
void AP_Baro_ExternalAHRS::update(void)
{
    if (count) {
        WITH_SEMAPHORE(_sem);
        _copy_to_frontend(instance, sum_pressure/count, sum_temp/count);
        sum_pressure = sum_temp = 0;
        count = 0;
    }
}

void AP_Baro_ExternalAHRS::handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt)
{
    if (pkt.instance != 0) {
        // not for us
        return;
    }
    WITH_SEMAPHORE(_sem);
    sum_pressure += pkt.pressure_pa;
    sum_temp += pkt.temperature;
    count++;
}

#endif // AP_BARO_EXTERNALAHRS_ENABLED
