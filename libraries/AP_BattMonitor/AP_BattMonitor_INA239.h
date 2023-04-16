#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_INA239_ENABLED

class AP_BattMonitor_INA239 : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_INA239(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    bool has_cell_voltages() const override { return false; }
    bool has_temperature() const override { return false; }
    bool has_current() const override { return true; }
    bool reset_remaining(float percentage) override { return false; }
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    void init(void) override;
    void read() override;

protected:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    void configure(void);
    bool read_word(const uint8_t reg, int16_t& data) const;
    bool write_word(const uint8_t reg, const uint16_t data) const;
    void timer(void);

    bool configured;
    bool callback_registered;
    uint32_t failed_reads;
    uint32_t last_configure_ms;

    struct {
        uint16_t count;
        float volt_sum;
        float current_sum;
        HAL_Semaphore sem;
    } accumulate;
    float current_LSB;
    float voltage_LSB;
};

#endif // AP_BATTERY_INA239_ENABLED
