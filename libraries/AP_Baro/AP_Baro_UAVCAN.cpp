#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Baro_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#if HAL_OS_POSIX_IO
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

extern const AP_HAL::HAL& hal;

#define debug_baro_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_UAVCAN::AP_Baro_UAVCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _sem_baro = hal.util->new_semaphore();
}

AP_Baro_UAVCAN::~AP_Baro_UAVCAN()
{
    if (!_initialized) {
        return;
    }
    
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(_manager);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    ap_uavcan->remove_baro_listener(this);
    delete _sem_baro;
    
    debug_baro_uavcan(2, "AP_Baro_UAVCAN destructed\n\r");
}

AP_Baro_Backend *AP_Baro_UAVCAN::probe(AP_Baro &baro)
{

    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return nullptr;
    }

    AP_Baro_UAVCAN *sensor;
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        
        uint8_t freebaro = ap_uavcan->find_smallest_free_baro_node();
        if (freebaro == UINT8_MAX) {
            continue;
        }
        sensor = new AP_Baro_UAVCAN(baro);
        if (sensor->register_uavcan_baro(i, freebaro)) {
            debug_baro_uavcan(2, "AP_Baro_UAVCAN probed, drv: %d, node: %d\n\r", i, freebaro);
            return sensor;
        } else {
            delete sensor;
        }
    }

    return nullptr;
}

// Read the sensor
void AP_Baro_UAVCAN::update(void)
{
    if (_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _copy_to_frontend(_instance, _pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        _sem_baro->give();
    }
}

void AP_Baro_UAVCAN::handle_baro_msg(float pressure, float temperature)
{
    if (_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _pressure = pressure;
        _temperature = temperature - C_TO_KELVIN;
        _last_timestamp = AP_HAL::micros64();
        _sem_baro->give();
    }
}

bool AP_Baro_UAVCAN::register_uavcan_baro(uint8_t mgr, uint8_t node)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return false;
    }
    _manager = mgr;

    if (ap_uavcan->register_baro_listener_to_node(this, node)) {
        _instance = _frontend.register_sensor();
        debug_baro_uavcan(2, "AP_Baro_UAVCAN loaded\n\r");

        _initialized = true;

        return true;
    }

    return false;
}

#endif // HAL_WITH_UAVCAN

