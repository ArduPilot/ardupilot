#include "AP_BattMonitor_Backend.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// Write BAT data packet(s)
void AP_BattMonitor_Backend::Log_Write_BAT(const uint8_t instance, const uint64_t time_us) const
{
    bool has_curr = has_current();

    const struct log_BAT pkt{
        LOG_PACKET_HEADER_INIT(LOG_BAT_MSG),
        time_us             : time_us,
        instance            : instance,
        voltage             : _state.voltage,
        voltage_resting     : _state.voltage_resting_estimate,
        current_amps        : has_curr ? _state.current_amps : AP::logger().quiet_nanf(),
        current_total       : has_curr ? _state.consumed_mah : AP::logger().quiet_nanf(),
        consumed_wh         : has_curr ? _state.consumed_wh : AP::logger().quiet_nanf(),
        temperature         : (int16_t) ( has_temperature() ? _state.temperature * 100 : 0),
        resistance          : _state.resistance
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write BCL data packet if has_cell_voltages
void AP_BattMonitor_Backend::Log_Write_BCL(const uint8_t instance, const uint64_t time_us) const
{
    if (!has_cell_voltages()) {
        return;
    }
    
    struct log_BCL cell_pkt{
        LOG_PACKET_HEADER_INIT(LOG_BCL_MSG),
        time_us             : time_us,
        instance            : instance,
        voltage             : _state.voltage
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(_state.cell_voltages.cells); i++) {
        cell_pkt.cell_voltages[i] = _state.cell_voltages.cells[i] + 1;
    }
    AP::logger().WriteBlock(&cell_pkt, sizeof(cell_pkt));

    // check battery structure can hold all cells  
    static_assert(ARRAY_SIZE(_state.cell_voltages.cells) == ARRAY_SIZE(cell_pkt.cell_voltages),
                    "Battery cell number doesn't match in library and log structure");
}
