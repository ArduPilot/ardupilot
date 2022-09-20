#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#define AP_BATTMONITOR_SMBUS_BUS_INTERNAL           0
#define AP_BATTMONITOR_SMBUS_BUS_EXTERNAL           1
#define AP_BATTMONITOR_SMBUS_I2C_ADDR               0x0B
#define AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS         5000000         // sensor becomes unhealthy if no successful readings for 5 seconds
#define AP_BATTMONITOR_SMBUS_READ_BLOCK_MAXIMUM_TRANSFER 0x20       // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.

class AP_BattMonitor_SMBus : public AP_BattMonitor_Backend
{
public:

    // Smart Battery Data Specification Revision 1.1
    enum BATTMONITOR_SMBUS {
        BATTMONITOR_SMBUS_TEMP = 0x08,                 // Temperature
        BATTMONITOR_SMBUS_VOLTAGE = 0x09,              // Voltage
        BATTMONITOR_SMBUS_CURRENT = 0x0A,              // Current
        BATTMONITOR_SMBUS_REMAINING_CAPACITY = 0x0F,   // Remaining Capacity
        BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY = 0x10, // Full Charge Capacity
        BATTMONITOR_SMBUS_CYCLE_COUNT = 0x17,          // Cycle Count
        BATTMONITOR_SMBUS_SPECIFICATION_INFO = 0x1A,   // Specification Info
        BATTMONITOR_SMBUS_SERIAL = 0x1C,               // Serial Number
        BATTMONITOR_SMBUS_MANUFACTURE_NAME = 0x20,     // Manufacture Name
        BATTMONITOR_SMBUS_MANUFACTURE_DATA = 0x23,     // Manufacture Data
    };

    /// Constructor
    AP_BattMonitor_SMBus(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_BattMonitor_Params &params,
                    uint8_t i2c_bus);

    // virtual destructor to reduce compiler warnings
    virtual ~AP_BattMonitor_SMBus() {}

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    bool has_temperature() const override { return _has_temperature; }

    // all smart batteries are expected to provide current
    bool has_current() const override { return true; }

    // don't allow reset of remaining capacity for SMBus
    bool reset_remaining(float percentage) override { return false; }

    // return true if cycle count can be provided and fills in cycles argument
    bool get_cycle_count(uint16_t &cycles) const override;

    virtual void init(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    void read(void) override;

    // reads the pack full charge capacity
    void read_full_charge_capacity(void);

    // reads the remaining capacity
    // which will only be read if we know the full charge capacity (accounting for battery degradation)
    void read_remaining_capacity(void);

    // return a scaler that should be multiplied by the battery's reported capacity numbers to arrive at the actual capacity in mAh
    virtual uint16_t get_capacity_scaler() const { return 1; }

    // reads the temperature word from the battery
    virtual void read_temp(void);

    // reads the temperature word from the battery
    virtual void read_current(void);

    // reads the serial number if it's not already known
    // returns if the serial number was already known
    void read_serial_number(void);

    // reads the battery's cycle count
    void read_cycle_count();

     // read word from register
     // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, uint8_t len) const;

    // get_PEC - calculate PEC for a read or write from the battery
    // buff is the data that was read or will be written
    uint8_t get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool _pec_supported; // true if PEC is supported

    int32_t _serial_number = -1;    // battery serial number
    uint16_t _full_charge_capacity; // full charge capacity, used to stash the value before setting the parameter
    bool _has_cell_voltages;        // smbus backends flag this as true once they have received a valid cell voltage report
    uint16_t _cycle_count = 0;      // number of cycles the battery has experienced. An amount of discharge approximately equal to the value of DesignCapacity.
    bool _has_cycle_count;          // true if cycle count has been retrieved from the battery
    bool _has_temperature;

    virtual void timer(void) = 0;   // timer function to read from the battery

    AP_HAL::Device::PeriodicHandle timer_handle;

    // Parameters
    AP_Int8  _bus;          // I2C bus number
    AP_Int8  _address;      // I2C address

};
