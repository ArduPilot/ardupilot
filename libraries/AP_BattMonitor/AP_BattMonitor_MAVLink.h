#pragma once
#if AP_BATTERY_MAVLINK_ENABLED
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#define AP_BATTMONITOR_MAVLINK_TIMEOUT_MICROS 5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

// Base SUI class
class AP_BattMonitor_MAVLink : public AP_BattMonitor_Backend
{
public:
    // Constructor
    AP_BattMonitor_MAVLink(AP_BattMonitor &mon,
                           AP_BattMonitor::BattMonitor_State &mon_state,
                           AP_BattMonitor_Params &params);

    void handle_msg(const mavlink_message_t &msg) override;

    // read the battery voltage and current. Should be called at 10hz
    void read() override;

    // returns true if battery monitor provides current info
    bool has_current() const override { return _have_info; }

    // returns true if battery monitor provides individual cell voltage
    bool has_cell_voltages() const override { return _have_info; }

    // returns true if battery monitor provides temperature
    bool has_temperature() const override { return _have_info; };

    // capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override WARN_IF_UNUSED;

private:
    bool _have_info;        // flag is true if battery MAVlink message is received
    uint8_t _remaining_pct; // battery remaining capacity in percentage
    void update_health();   // function to update battery health status based on the last message received from the battery
};

#endif // AP_BATTERY_MAVLINK_ENABLED
