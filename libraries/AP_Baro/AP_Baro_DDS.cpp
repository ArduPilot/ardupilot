#include "AP_Baro_DDS.h"

#if AP_BARO_DDS_ENABLED

//! @todo(srmainwaring) remove debug info
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "Baro"

AP_Baro_DDS::AP_Baro_DDS(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();

    auto devid = AP_HAL::Device::make_bus_id(
        AP_HAL::Device::BUS_TYPE_DDS, 0, _instance, DEVTYPE_BARO_DDS);
    set_bus_id(_instance, devid);

    //! @todo(srmainwaring) remove debug info
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Baro_DDS: instance: %d, devid: %d",
        _instance, devid);
}

AP_Baro_DDS::~AP_Baro_DDS() = default;

void AP_Baro_DDS::update_healthy_flag(uint8_t instance)
{
    _frontend.sensors[instance].healthy = healthy(instance);

    //! @todo(srmainwaring) - find alternative
    // Force calibration true. Baro calibration fails for DDS because the
    // DDS client is not created until after the baro calibration has run. 
    _frontend.sensors[instance].calibrated = true;
};

bool AP_Baro_DDS::healthy(uint8_t instance) 
{
    return _last_sample_time != 0;
}

void AP_Baro_DDS::update(void)
{
    if (_count) {
        //! @todo(srmainwaring) remove debug info
        // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Baro_DDS: count: %d, pressure: %f, temperature: %f",
        //     _count, _sum_pressure/_count, _sum_temp/_count);

        WITH_SEMAPHORE(_sem);
        _copy_to_frontend(_instance, _sum_pressure/_count, _sum_temp/_count);
        _sum_pressure = 0;
        _sum_temp = 0;
        _count = 0;
    }
}

void AP_Baro_DDS::handle_external(
    const AP_ExternalAHRS::baro_data_message_t &pkt)
{
    if (pkt.instance != 0) {
        return;
    }

    _last_sample_time = AP_HAL::millis();

    WITH_SEMAPHORE(_sem);
    _sum_pressure += pkt.pressure_pa;
    _sum_temp += pkt.temperature;
    _count++;
}

#endif // AP_BARO_DDS_ENABLED
