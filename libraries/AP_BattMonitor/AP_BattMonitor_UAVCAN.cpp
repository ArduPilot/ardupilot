#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

#define debug_bm_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type) :
    AP_BattMonitor_Backend(mon, mon_state),
    _type(type)
{
    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    switch (_type) {
                        case UAVCAN_CIRCUIT_STATUS:
                            if(uavcan->register_BM_cs_listener_to_id(this, _mon._serial_numbers[_state.instance])) {
                                debug_bm_uavcan(2, "UAVCAN BattMonitor[%d] CircuitStatus registered id: %d\n\r", _state.instance+1, _mon._serial_numbers[_state.instance]);
                            }
                            break;
                        case UAVCAN_BATTERY_INFO:
                            if (uavcan->register_BM_bi_listener_to_id(this, _mon._serial_numbers[_state.instance])) {
                                debug_bm_uavcan(2, "UAVCAN BattMonitor[%d] BatteryInfo registered id: %d\n\r", _state.instance+1, _mon._serial_numbers[_state.instance]);
                            }
                            break;
//OW
                        case UAVCAN_GENERIC_BATTERY_INFO:
                            if (uavcan->register_gbi_listener_to_id(this, _mon._serial_numbers[_state.instance])) {
                                debug_bm_uavcan(2, "UAVCAN BattMonitor[%d] GenericBatteryInfo registered id: %d\n\r", _state.instance+1, _mon._serial_numbers[_state.instance]);
                            }
                            break;
//OWEND
                    }
                }
            }
        }
    }
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS) {
        _state.healthy = false;
    }
}

//OW
void AP_BattMonitor_UAVCAN::handle_gbi_msg(float voltage, float current, float charge)
{
//    handle_cs_msg(voltage, current);

//XX    if( charge == NAN ){
    if (uavcan::isNaN(charge)) {
        handle_cs_msg(voltage, current);
        return;
    }

    _state.voltage = voltage;
    _state.current_amps = current;
    _state.current_total_mah = charge;

    // record time, so that we can fall back to handle_cs_msg() whenever needed
    // and to avoid timeout
    uint32_t tnow = AP_HAL::micros();
    _state.last_time_micros = tnow;

    _state.healthy = true;
}
//OWEND

void AP_BattMonitor_UAVCAN::handle_bi_msg(float voltage, float current, float temperature)
{
    _state.temperature = temperature;
    handle_cs_msg(voltage, current);
}

void AP_BattMonitor_UAVCAN::handle_cs_msg(float voltage, float current)
{
    _state.voltage = voltage;
    _state.current_amps = current;

    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _state.last_time_micros;

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000.0f) {
        // .0002778 is 1/3600 (conversion to hours)
        _state.current_total_mah += _state.current_amps * dt * 0.0000002778f;
    }

    // record time
    _state.last_time_micros = tnow;

    _state.healthy = true;
}


#endif
