#include "AP_BattMonitor_config.h"
#include <AP_Logger/AP_Logger_config.h>

#if AP_BATTERY_ENABLED && HAL_LOGGING_ENABLED

#include "AP_BattMonitor_Backend.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// Write BAT data packet(s)
void AP_BattMonitor_Backend::Log_Write_BAT(const uint8_t instance, const uint64_t time_us) const
{
    bool has_curr = has_current();
    uint8_t percent = -1;
    IGNORE_RETURN(capacity_remaining_pct(percent));

    float temperature;
    int16_t temperature_cd = 0;
    if (get_temperature(temperature)) {
        temperature_cd = temperature * 100.0;
    }

    uint8_t soh_pct = 0;
    IGNORE_RETURN(get_state_of_health_pct(soh_pct));

    const struct log_BAT pkt{
        LOG_PACKET_HEADER_INIT(LOG_BAT_MSG),
        time_us             : time_us,
        instance            : instance,
        voltage             : _state.voltage,
        voltage_resting     : _state.voltage_resting_estimate,
        current_amps        : has_curr ? _state.current_amps : AP_Logger::quiet_nanf(),
        current_total       : has_curr ? _state.consumed_mah : AP_Logger::quiet_nanf(),
        consumed_wh         : has_curr ? _state.consumed_wh : AP_Logger::quiet_nanf(),
        temperature         : temperature_cd,
        resistance          : _state.resistance,
        rem_percent         : percent,
        health              : _state.healthy,
        state_of_health_pct : soh_pct
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

    // we pack the entire BCL message - we must have at least that
    // many supported cells or the loop below will over-read
    static_assert(ARRAY_SIZE(_state.cell_voltages.cells) >= ARRAY_SIZE(cell_pkt.cell_voltages), "must have at least ARRAY_SIZE(log_BCL.cell_voltages) cells");

    for (uint8_t i = 0; i < ARRAY_SIZE(cell_pkt.cell_voltages); i++) {
        cell_pkt.cell_voltages[i] = _state.cell_voltages.cells[i] + 1; // add 1mv
    }
    AP::logger().WriteBlock(&cell_pkt, sizeof(cell_pkt));

#if AP_BATT_MONITOR_CELLS_MAX > 12
    if (_state.cell_voltages.cells[12] != UINT16_MAX || _state.cell_voltages.cells[13] != UINT16_MAX)
    {
// @LoggerMessage: BCL2
// @Description: Battery cell voltage information
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: V13: thirteenth cell voltage
// @Field: V14: fourteenth cell voltage
        AP::logger().WriteStreaming(
            "BCL2",
            "TimeUS,Instance,V13,V14",
            "s#vv",
            "F-CC",
            "QBHH",
            time_us,
            instance,
            _state.cell_voltages.cells[ARRAY_SIZE(cell_pkt.cell_voltages)+0] + 1, // add 1mv
            _state.cell_voltages.cells[ARRAY_SIZE(cell_pkt.cell_voltages)+1] + 1  // add 1mv
            );
    }
#endif
}

#endif  // AP_BATTERY_ENABLED && HAL_LOGGING_ENABLED
