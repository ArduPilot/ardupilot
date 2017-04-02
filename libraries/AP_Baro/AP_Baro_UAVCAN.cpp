#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Baro_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

extern const AP_HAL::HAL& hal;

#define debug_baro_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

// There is limitation to use only one UAVCAN barometer now.

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_UAVCAN::AP_Baro_UAVCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            // Give time to receive some packets from CAN if baro sensor is present
            // This way it will get calibrated correctly
            hal.scheduler->delay(1000);
            ap_uavcan->register_baro_listener(this, 1);

            debug_baro_uavcan(2, "AP_Baro_UAVCAN loaded\n\r");
        }
    }

    _sem_baro = hal.util->new_semaphore();
}

AP_Baro_UAVCAN::~AP_Baro_UAVCAN()
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            ap_uavcan->remove_baro_listener(this);
            debug_baro_uavcan(2, "AP_Baro_UAVCAN destructed\n\r");
        }
    }
}

// Read the sensor
void AP_Baro_UAVCAN::update(void)
{
    if (_sem_baro->take(0)) {
        _copy_to_frontend(_instance, _pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        _sem_baro->give();
    }
}

void AP_Baro_UAVCAN::handle_baro_msg(float pressure, float temperature)
{
    if (_sem_baro->take(0)) {
        _pressure = pressure;
        _temperature = temperature - 273.15f;
        _last_timestamp = AP_HAL::micros64();
        _sem_baro->give();
    }
}

#endif // HAL_WITH_UAVCAN

