#include "AP_BattMonitor_Backend.h"
#include <AP_Logger/AP_Logger.h>

#if HAL_SMART_BATTERY_INFO_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

// Write BAT data packet(s)
void AP_BattMonitor_Backend::Log_Write_BAT(const uint8_t instance, const uint64_t time_us) const
{
    bool has_curr = has_current();
    uint8_t percent = -1;
    IGNORE_RETURN(capacity_remaining_pct(percent));

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
        resistance          : _state.resistance,
        rem_percent         : percent,
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

// Write BATI data, returns false if cells_in_series is unavailable 
bool AP_BattMonitor_Backend::Log_Write_BATI(const uint8_t instance) const
{
#if HAL_SMART_BATTERY_INFO_ENABLED
#if HAL_LOGGING_ENABLED

    Smart_Batt_Info smart_batt_info;
    if (!get_smart_batt_info(smart_batt_info)) {
        return false;
    }

    // @LoggerMessage: BATI
    // @Vehicles: All
    // @Description: Smart Battery Information
    // @Field: TimeUS: Time since system startup
    // @Field: id: battery monitor instance number
    // @Field: func: battery function
    // @Field: type: battery type (chemistry)
    // @Field: nSer: number of battery cells in series
    // @Field: capD: design capacity when full according to manufacturer, -1: field not provided.
    // @Field: capF: capacity when full (accounting for battery degradation), -1: field not provided.
    // @Field: Vmax: maximum charging voltage, -1: field not provided.
    // @Field: cyc: charge/discharge cycle count. UINT16_MAX: field not provided.
    // @Field: ser: serial number in ASCII characters, 0 terminated. First Char \0: field not provided.
    // @Field: date: manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. First Char \0: field not provided.
    // @Field: name: product name encoded as 'device name'_'manufacturer'. First Char \0: field not provided

    AP::logger().Write(
        "BATI",
        "TimeUS,id,func,type,nSer,capD,Vmax,capF,cyc,name,ser,date",
        "s#--?ivi?---",
        "F0000/0/0---",
        "QBBBBfffHZNN",
        AP_HAL::micros64(),
        instance,
        MAV_BATTERY_FUNCTION_UNKNOWN,
        MAV_BATTERY_TYPE_UNKNOWN,
        smart_batt_info.cells_in_series,
        smart_batt_info.got_capacity_design ? smart_batt_info.capacity_design_mah * 0.001 : -1.0,
        smart_batt_info.got_design_voltage ? smart_batt_info.design_voltage / smart_batt_info.cells_in_series : -1.0,
        smart_batt_info.got_capacity_full ? smart_batt_info.capacity_full_mah * 0.001 : -1.0,
        (uint16_t) (smart_batt_info.got_cycle_count ? smart_batt_info.cycles : UINT16_MAX),
        smart_batt_info.product_name,
        smart_batt_info.serial_number,
        smart_batt_info.manufacture_date);

    return true;
#endif  // HAL_LOGGING_ENABLED
#endif  // HAL_SMART_BATTERY_INFO_ENABLED
    return false;
}

// Send the mavlink message for the smart_battery_info message
// returns false only if an instance didn't have enough space available on the link
bool AP_BattMonitor_Backend::send_mavlink_smart_battery_info(const uint8_t instance, const mavlink_channel_t chan) const
{
#if HAL_SMART_BATTERY_INFO_ENABLED
#if HAL_GCS_ENABLED

    if (!HAVE_PAYLOAD_SPACE(chan, SMART_BATTERY_INFO)) {
        return false;
    }

    Smart_Batt_Info smart_batt_info {};

    // Only can return false if space is not available on the link thus return true here
    if (!get_smart_batt_info(smart_batt_info)) {
        return true;
    }

    mavlink_msg_smart_battery_info_send(
        chan,
        instance,                           // Battery ID
        MAV_BATTERY_FUNCTION_UNKNOWN,       // (not provided) Function of the battery
        MAV_BATTERY_TYPE_UNKNOWN,           // (not provided) Type (chemistry) of the battery
        smart_batt_info.got_capacity_design ? smart_batt_info.capacity_design_mah: -1, // [mAh] Design capacity when full according to manufacturer, -1: field not provided.
        smart_batt_info.got_capacity_full ? smart_batt_info.capacity_full_mah: -1,     // [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.
        smart_batt_info.got_cycle_count ? smart_batt_info.cycles : UINT16_MAX,                // Charge/discharge cycle count. UINT16_MAX: field not provided.
        smart_batt_info.serial_number,      // Serial number in ASCII characters, 0 terminated. First Char \0: field not provided
        smart_batt_info.product_name,       // Static device name. Encode as manufacturer and product names separated using an underscore. First Char \0: field not provided
        0,                                  // (not provided) [g] Battery weight. 0: field not provided.
        UINT16_MAX,                         // (not provided) [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
        UINT16_MAX,                         // (not provided) [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
        UINT16_MAX,                         // (not provided) [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
        smart_batt_info.got_design_voltage ? (smart_batt_info.design_voltage / smart_batt_info.cells_in_series * 1000.0) : 0, // [mV] Maximum per-cell voltage when charged. 0: field not provided.
        smart_batt_info.cells_in_series,    // Number of battery cells in series. 0: field not provided.
        0,                                  // (not provided) [mA] Maximum pack discharge current. 0: field not provided.
        0,                                  // (not provided) [mA] Maximum pack discharge burst current. 0: field not provided.
        smart_batt_info.manufacture_date);  // Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. First Char \0: field not provided

    return true;
#endif // HAL_GCS_ENABLED
#endif // HAL_SMART_BATTERY_INFO_ENABLED

    return false;
}

// Smart battery info. helper to gather data for logging and mavlink
// returns false if cells_in_series is not available
bool AP_BattMonitor_Backend::get_smart_batt_info(Smart_Batt_Info &smart_batt_info) const
{
#if HAL_SMART_BATTERY_INFO_ENABLED

    smart_batt_info.got_cells_in_series = get_cells_in_series(smart_batt_info.cells_in_series);

    // get_cells_in_series only returns false if cells_in_series is not available
    if (!smart_batt_info.got_cells_in_series) {
       return false;
    }

    smart_batt_info.got_cycle_count = get_cycle_count(smart_batt_info.cycles);
    smart_batt_info.got_capacity_design = get_design_capacity_mah(smart_batt_info.capacity_design_mah);
    smart_batt_info.got_capacity_full = get_full_charge_capacity_mah(smart_batt_info.capacity_full_mah);
    smart_batt_info.got_design_voltage = get_design_voltage(smart_batt_info.design_voltage);
    get_product_name(smart_batt_info.product_name, ARRAY_SIZE(smart_batt_info.product_name));
    get_manufacture_date(smart_batt_info.manufacture_date,  ARRAY_SIZE(smart_batt_info.manufacture_date));

    if (_params._serial_number != -1) {
        snprintf(smart_batt_info.serial_number, ARRAY_SIZE(smart_batt_info.serial_number), "%i", (signed) _params._serial_number);
    } else {
        // ensure null termination when data is unavailable
        if (ARRAY_SIZE(smart_batt_info.serial_number) > 0) {
            smart_batt_info.serial_number[0] = '\0';
        }
    }

    return true;
#else
    return false;
#endif // HAL_SMART_BATTERY_INFO_ENABLED
}
