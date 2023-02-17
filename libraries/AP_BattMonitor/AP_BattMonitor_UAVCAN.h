#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>
#include <mppt/OutputEnable.hpp>

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

#ifndef AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
#define AP_BATTMONITOR_UAVCAN_MPPT_DEBUG 0
#endif

class BattInfoCb;
class BattInfoAuxCb;
class MpptStreamCb;

class AP_BattMonitor_UAVCAN : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_UAVCAN_Type {
        UAVCAN_BATTERY_INFO = 0
    };

    /// Constructor
    AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    void init() override {}

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    bool has_temperature() const override { return _has_temperature; }

    bool has_current() const override { return true; }

    bool has_consumed_energy() const override { return _has_consumed_energy; }

    bool has_time_remaining() const override { return _has_time_remaining; }

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    bool get_cycle_count(uint16_t &cycles) const override;

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    uint32_t get_mavlink_fault_bitmask() const override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_BattMonitor_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id);
    static void handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb);
    static void handle_battery_info_aux_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoAuxCb &cb);
    static void handle_mppt_stream_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MpptStreamCb &cb);
    void handle_outputEnable_response(const uint8_t nodeId, const bool enabled);

    void mppt_set_powered_state(bool power_on) override;

private:
    void handle_battery_info(const BattInfoCb &cb);
    void handle_battery_info_aux(const BattInfoAuxCb &cb);
    void update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc);

    static bool match_battery_id(uint8_t instance, uint8_t battery_id) {
        // when serial number is negative, all batteries are accepted. Else, it must match
        return (AP::battery().get_serial_number(instance) < 0) || (AP::battery().get_serial_number(instance) == (int32_t)battery_id);
    }

    // MPPT related enums and methods
    enum class MPPT_FaultFlags : uint8_t {
        OVER_VOLTAGE        = (1U<<0),
        UNDER_VOLTAGE       = (1U<<1),
        OVER_CURRENT        = (1U<<2),
        OVER_TEMPERATURE    = (1U<<3),
    };
    void handle_mppt_stream(const MpptStreamCb &cb);
    void mppt_check_powered_state();

#if AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
    static void mppt_report_faults(const uint8_t instance, const uint8_t fault_flags);
    static const char* mppt_fault_string(const MPPT_FaultFlags fault);
#endif

    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_UAVCAN_Type _type;

    HAL_Semaphore _sem_battmon;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _soc;
    uint8_t _node_id;
    uint16_t _cycle_count;
    float _remaining_capacity_wh;
    float _full_charge_capacity_wh;
    bool _has_temperature;
    bool _has_cell_voltages;
    bool _has_time_remaining;
    bool _has_consumed_energy;
    bool _has_battery_info_aux;
    uint8_t _instance;                  // instance of this battery monitor
    uavcan::Node<0> *_node;             // UAVCAN node id
    AP_Float _curr_mult;                 // scaling multiplier applied to current reports for adjustment
    // MPPT variables
    struct {
        bool is_detected;               // true if this UAVCAN device is a Packet Digital MPPT
        bool powered_state;             // true if the mppt is powered on, false if powered off
        bool vehicle_armed_last;        // latest vehicle armed state. used to detect changes and power on/off MPPT board
        uint8_t fault_flags;            // bits holding fault flags
        uint32_t powered_state_remote_ms; // timestamp of when request was sent, zeroed on response. Used to retry
    } _mppt;
};
