    #include "AP_BattMonitor_JBDCAN.h"
    #include <AP_HAL/AP_HAL.h>
#if AP_BATTERY_JBDCAN_ENABLED 

    extern const AP_HAL::HAL& hal;

    const AP_Param::GroupInfo AP_BattMonitor_JBDCAN::var_info[] = { AP_GROUPEND };

    AP_BattMonitor_JBDCAN::AP_BattMonitor_JBDCAN(AP_BattMonitor &mon,
                                                AP_BattMonitor::BattMonitor_State &st,
                                                AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, st, params) {
        AP_Param::setup_object_defaults(this, var_info);
        _state.var_info = var_info;
        _state.healthy = false;
        hal.console->printf("[JBDCAN] init OK\n");
    }

    void AP_BattMonitor_JBDCAN::init() {
        hal.console->printf("[JBDCAN] init() called\n");
    }   

    void AP_BattMonitor_JBDCAN::read() {
        hal.console->printf("[JBDCAN] read() test\n");
        _state.voltage = 12.3f;
        _state.current_amps = 1.5f;
        _state.healthy = true;
        _state.last_time_micros = AP_HAL::micros();
    }

    bool AP_BattMonitor_JBDCAN::capacity_remaining_pct(uint8_t &percentage) const {
        percentage = 75;
        return true;
    }

#endif 