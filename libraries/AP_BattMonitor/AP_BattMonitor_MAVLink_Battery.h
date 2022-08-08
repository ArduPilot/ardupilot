#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#define CELLS_PER_BATTERY_MAVLINK_MSG_MAX       10
#define MAX_MAVLINK_SMART_BATTERIES             AP_BATT_MONITOR_MAX_INSTANCES
#define MAVLINK_SMART_BATTERY_TIMEOUT_MICROS    (AP_BATT_MONITOR_TIMEOUT * 1000)

class AP_BattMonitor_MAVLink_Battery : public AP_BattMonitor_Backend
{
public:
    AP_BattMonitor_MAVLink_Battery(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    virtual ~AP_BattMonitor_MAVLink_Battery(void) {};

    void init(void) override {}

    /* read latest values received from mavlink */
    void read(void) override;

    /* returns true if battery monitor instance provides consumed energy info */
	bool has_consumed_energy() const override { return true; }

	/* returns true if battery monitor instance provides current info */
	bool has_current() const override { return true; }

	/* returns true if battery monitor provides individual cell voltages */
	bool has_cell_voltages() const override { return true; }

    /* returns true if battery monitor instance provides time remaining info */
    bool has_time_remaining() const override { return _state.has_time_remaining; }

	/* capacity_remaining_pct - returns the % battery capacity if provided */
	bool capacity_remaining_pct(uint8_t &cap_remain_pct) const override;

    bool has_temperature() const override { return have_temp; }

    bool get_cycle_count(uint16_t &cycles) const override;

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    uint32_t get_mavlink_fault_bitmask() const override { return fault_bitmask; }

    /* don't allow reset of remaining capacity */
    bool reset_remaining(float percentage) override { return false; }

    // handle mavlink msg 
    void handle_mavlink_battery_message(const mavlink_message_t &msg) override;
    
    /* process mavlink msg */
	void process_info_msg(const mavlink_message_t &msg);
	void process_status_msg(const mavlink_message_t &msg);
    
    /* handle incoming MAVLink messages */
	void handle_battery_info_msg(const mavlink_message_t &msg);

    /* check msg sys/comp id & configured for mavlink batteries */
	bool accept_mavlink_battery_message(const mavlink_message_t &msg);
    bool accept_sys_comp_id(const mavlink_message_t &msg);
    bool check_driver_type(const mavlink_message_t &msg); 

private:

    bool                        have_temp;
	int8_t                      capacity_rem_pct;  
    uint16_t                    cycle_count;
    uint32_t                    fault_bitmask;    
};
