#include "AP_BattMonitor_JBDCAN.h"
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;
const AP_Param::GroupInfo AP_BattMonitor_JBDCAN::var_info[] = {
    AP_GROUPEND
};
AP_BattMonitor_JBDCAN::AP_BattMonitor_JBDCAN(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    _state.healthy = false;  // mặc định chưa có dữ liệu
    hal.console->printf("[JBDCAN] init full OK (instance=%d)\n", mon_state.instance);
}


void AP_BattMonitor_JBDCAN::read()
{
    hal.console->printf("[JBDCAN] read() chạy test mỗi vòng\n");

    _state.voltage = 12.4f;
    _state.current_amps = 1.6f;
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();
}