#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA3221_ENABLED

/*
 *
 * Datasheet: https://www.ti.com/lit/ds/symlink/ina3221.pdf?ts=1597369254046
 */

// The INA3221 takes two measurements for each channel: one for shunt voltage
// and one for bus voltage. Each measurement can be independently or
// sequentially measured, based on the mode setting (bits 2-0 in the
// Configuration register). When the INA3221 is in normal operating mode
// (that is, the MODE bits of the Configuration register are set to 111), the
// device continuously converts a shunt-voltage reading followed by a
// bus-voltage reading. This procedure converts one channel, and then continues
// to the shunt voltage reading of the next enabled channel, followed by the
// bus-voltage reading for that channel, and so on, until all enabled channels
// have been measured. The programmed Configuration register mode setting
// applies to all channels. Any channels that are not enabled are bypassed in
// the measurement sequence, regardless of mode setting.


// 8.3.3 Software Reset
// The INA3221 features a software reset that reinitializes the device and
// register settings to default power-up values without having to cycle power
// to the device. Use bit 15 (RST) of the Configuration register to perform a
// software reset. Setting RST reinitializes all registers and settings to the
// default power state with the exception of the power-valid output state. If a
// software reset is issued, the INA3221 holds the output of the PV pin until
// the power-valid detection sequence completes. The Power-Valid UpperLimit and
// Power-Valid Lowerlimit registers return to the default state when the
// software reset has been issued. Therefore, any reprogrammed limit registers
// are reset, resulting in the original power-valid thresholds validating the
// power-valid conditions. This architecture prevents interruption to circuitry
// connected to the powervalid output during a software reset event.

// The INA3221 has programmable conversion times for both the shunt- and
// bus-voltage measurements. The selectable conversion times for these
// measurements range from 140μs to 8.244ms.


#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#ifndef HAL_BATTMON_INA3221_MAX_DEVICES
#define HAL_BATTMON_INA3221_MAX_DEVICES 1
#endif

class AP_BattMonitor_INA3221 : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_INA3221(AP_BattMonitor &mon,
                           AP_BattMonitor::BattMonitor_State &mon_state,
                           AP_BattMonitor_Params &params);

    void init() override;

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    bool has_current() const override {
        return true;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8 i2c_bus;
    AP_Int8 i2c_address;
    AP_Int8 channel;

    static struct AddressDriver {
        bool read_register(uint8_t addr, uint16_t &ret);
        bool write_register(uint8_t addr, uint16_t val);
        bool write_config(void);
        void timer(void);
        void register_timer();

        AP_HAL::I2CDevice *dev;
        uint8_t bus;
        uint8_t address;
        uint8_t channel_mask;
        uint8_t dev_channel_mask;

        struct StateList {
            struct StateList *next;
            HAL_Semaphore sem;
            uint8_t channel;

            bool healthy;
            float voltage;
            float current_amps;
            float delta_mah;
            float delta_wh;
            uint32_t last_time_micros;
        };
        StateList *statelist;

    } address_driver[HAL_BATTMON_INA3221_MAX_DEVICES];
    static uint8_t address_driver_count;

    AddressDriver::StateList *address_driver_state;
};

#endif  // AP_BATTERY_INA3221_ENABLED
