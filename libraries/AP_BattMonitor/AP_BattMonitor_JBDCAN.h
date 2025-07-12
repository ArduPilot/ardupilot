#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_HAL/AP_HAL.h>

// Timeout để đợi phản hồi từ BMS JBD (5 giây)
#define AP_BATTMONITOR_JBDCAN_TIMEOUT_MICROS 5000000

class AP_BattMonitor_JBDCAN : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_JBDCAN(AP_BattMonitor& mon,
                          AP_BattMonitor::BattMonitor_State& mon_state,
                          AP_BattMonitor_Params& params);


                          

    void init() override;

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    //bool has_temperature() const override{ return true; };  
    bool has_current() const override { return true; }

    // Always have consumed energy, either directly from BatteryInfoAux msg or by cumulative current draw
    bool has_consumed_energy() const override { return true; }



private:
    AP_HAL::CANIface* _iface = nullptr; // con trỏ tới CAN interface đang dùng

    uint8_t _callback_id;


    uint64_t _last_update_us;      // để kiểm soát timeout hoặc polling

    
    static const uint16_t query_ids;

    void handle_frame_callback(uint8_t iface_num, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags);
        
    void handle_frame(const AP_HAL::CANFrame &frame);

    static const uint16_t JBD_REQUEST_ID = 0x100;
    static const uint16_t JBD_RESPONSE_ID = 0x100;
    static const uint32_t UPDATE_INTERVAL_US = 200000;  // 200ms
    static const uint32_t TIMEOUT_US = 100000;   
};
