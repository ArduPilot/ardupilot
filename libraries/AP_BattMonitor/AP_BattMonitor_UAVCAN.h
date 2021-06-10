#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class BattInfoCb;
#if HAL_BATTMONITOR_UAVCAN_CBAT_ENABLED
class CBattCb;
#endif

class AP_BattMonitor_UAVCAN : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_UAVCAN_Type {
        UAVCAN_BATTERY_INFO = 0,
#if HAL_BATTMONITOR_UAVCAN_CBAT_ENABLED
        UAVCAN_CBAT         = 1
#endif
    };

    /// Constructor
    AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params);

    void init() override {}

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    uint8_t capacity_remaining_pct() const override;

    /// cycle_count - return true if cycle count can be provided and fills in cycles argument
    bool get_cycle_count(uint16_t &cycles) const override;

    bool has_temperature() const override { return _has_temperature; }

    bool has_current() const override { return true; }

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    bool has_time_remaining() const override { return _has_time_remaining; }

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_BattMonitor_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id, AP_BattMonitor::Type type);
    static void handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb);
#if HAL_BATTMONITOR_UAVCAN_CBAT_ENABLED
    static void handle_cbat_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const CBattCb &cb);
#endif

private:
    void handle_battery_info(const BattInfoCb &cb);
#if HAL_BATTMONITOR_UAVCAN_CBAT_ENABLED
    void handle_cbat(const CBattCb &cb);
#endif

    static bool match_battery_id(uint8_t instance, uint8_t battery_id) {
        // when serial number is negative, all batteries are accepted. Else, it must match
        return (AP::battery().get_serial_number(instance) < 0) || (AP::battery().get_serial_number(instance) == (int32_t)battery_id);
    }

    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_UAVCAN_Type _type;

    HAL_Semaphore _sem_battmon;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    uint8_t _soc;
    uint16_t _cycles;
    uint8_t _cell_count;
    bool _has_temperature;
    bool _has_cell_voltages;
    bool _has_time_remaining;
};
