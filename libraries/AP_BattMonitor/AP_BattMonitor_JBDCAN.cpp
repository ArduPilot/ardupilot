#include "AP_BattMonitor_JBDCAN.h"
#include <GCS_MAVLink/GCS.h>
#include <cstring>  // memset, memcpy
#include <cstdio>
extern const AP_HAL::HAL& hal;

// Constructor: lưu con trỏ đến state và param
AP_BattMonitor_JBDCAN::AP_BattMonitor_JBDCAN(AP_BattMonitor& mon,
                                             AP_BattMonitor::BattMonitor_State& mon_state,
                                             AP_BattMonitor_Params& params) :
    AP_BattMonitor_Backend(mon, mon_state, params)


{}

// Lấy CAN interface (mặc định là CAN1 = hal.can[0])
void AP_BattMonitor_JBDCAN::init() {
    _iface = hal.can[0];
    if (_iface == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "JBDCAN: CAN interface not available");
        return;
    }
    // Khởi tạo CAN: 500kbps, chế độ Normal
    if (!_iface->init(500000, AP_HAL::CANIface::NormalMode)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "JBDCAN: Failed to initialize CAN interface");
        return;
    }
    if (!_iface->register_frame_callback(
        FUNCTOR_BIND_MEMBER(&AP_BattMonitor_JBDCAN::handle_frame_callback, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags),
        _callback_id)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "JBDCAN: Failed to register frame callback");
        return;
    }
    _last_update_us = AP_HAL::micros64();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JBDCAN: Interface ready");
    

}




void AP_BattMonitor_JBDCAN::read()
{
    if (_iface == nullptr) return;

    uint64_t now = AP_HAL::micros();
    if (now - _last_update_us < 200000) return;

    uint16_t id = 0x100;
    AP_HAL::CANFrame rtr_frame;
    rtr_frame.id = id | AP_HAL::CANFrame::FlagRTR;
    rtr_frame.dlc = 0;
    rtr_frame.setCanFD(false);

    int16_t ret = _iface->send(rtr_frame, now + 100000, 0);
    if (ret > 0) {
        _last_update_us = now;
    } else if (ret < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "JBDCAN: Failed to send RTR frame");
    }
}

void AP_BattMonitor_JBDCAN::handle_frame_callback(uint8_t iface_num, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags)
{
    // Chỉ xử lý frame có ID phù hợp (response từ JBD BMS)
    if ((frame.id & 0xFFFF) == 0x100) {
        handle_frame(frame);
    }
}

    
void AP_BattMonitor_JBDCAN::handle_frame(const AP_HAL::CANFrame &frame)
{   
    
    if (frame.dlc < 6 || frame.dlc > 8) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "JBDCAN: Frame length invalid");
        return;
    }

    const uint8_t *data = frame.data;

    uint16_t raw_voltage = (data[0] << 8) | data[1];
    float voltage = raw_voltage * 0.01f;

    int16_t raw_current = (data[2] << 8) | data[3];
    float current = raw_current * 0.01f;

    uint16_t raw_capacity = (data[4] << 8) | data[5];


    float remaining_mah = raw_capacity * 10.0f;
    // Tính consumed capacity nếu có thông tin pack capacity
    if (_params._pack_capacity > 0) {
        _state.consumed_mah = 25000-remaining_mah;
    }


    _state.voltage = voltage;
    _state.current_amps  = current;
    _state.healthy = true;
    _last_update_us = AP_HAL::micros();

    if (voltage > 0) {
        _state.consumed_wh = _state.consumed_mah * voltage * 0.001f;
    }

    uint64_t now = AP_HAL::micros64();
    static uint64_t last_log_us = 0;
    if (now - last_log_us > 5000000) {  // 5 giây
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JBDCAN: V=%.2fV I=%.2fA Remain=%.0fmAh Consumed=%.0fmAh", 
                      _state.voltage, _state.current_amps, remaining_mah, _state.consumed_mah);
        last_log_us = now;
    }
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JBDCAN: V=%.2fV I=%.2fA C=%.2fmAh", _state.voltage, _state.current_amps, _state.consumed_mah);
}



bool AP_BattMonitor_JBDCAN::capacity_remaining_pct(uint8_t &percentage) const {
if (_params._pack_capacity > 0) {
    percentage = 100 * (25000-_state.consumed_mah ) / 25000;
}
return true;
}
