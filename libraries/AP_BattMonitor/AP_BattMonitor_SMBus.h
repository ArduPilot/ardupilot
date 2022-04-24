#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"

#define AP_BATTMONITOR_SMBUS_BUS_INTERNAL           0
#define AP_BATTMONITOR_SMBUS_BUS_EXTERNAL           1
#define AP_BATTMONITOR_SMBUS_I2C_ADDR               0x0B
#define AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds
#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER           0x20 // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.

class AP_BattMonitor_SMBus : public AP_BattMonitor_Backend
{
public:

    // Smart Battery Data Specification Revision 1.1
    enum BATTMONITOR_SMBUS {
        BATTMONITOR_SMBUS_TEMP = 0x08,                 // Temperature
        BATTMONITOR_SMBUS_VOLTAGE = 0x09,              // Voltage
        BATTMONITOR_SMBUS_CURRENT = 0x0A,              // Current
        BATTMONITOR_SMBUS_REMAINING_CAPACITY = 0x0F,   // Remaining Capacity
        BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY = 0x10, // Full Charge Capacity (accounting for battery degradation)
        BATTMONITOR_SMBUS_CYCLE_COUNT = 0x17,          // Cycle Count
        BATTMONITOR_SMBUS_DESIGN_CAPACITY = 0x18,      // Design capacity (capacity when newly manufactured)
        BATTMONITOR_SMBUS_DESIGN_VOLTAGE = 0x19,       // Design voltage
        BATTMONITOR_SMBUS_SPECIFICATION_INFO = 0x1A,   // Specification Info
        BATTMONITOR_SMBUS_MANUFACTURE_DATE = 0x1B,     // Manufacture date
        BATTMONITOR_SMBUS_SERIAL = 0x1C,               // Serial Number
        BATTMONITOR_SMBUS_MANUFACTURE_NAME = 0x20,     // Manufacture Name
        BATTMONITOR_SMBUS_DEVICE_NAME = 0x21,          // Device Name
        BATTMONITOR_SMBUS_DEVICE_CHEMISTRY = 0x22,     // Battery chemistry type
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

#if HAL_SMART_BATTERY_INFO_ENABLED
    // returns true if product name can be filled in from either manufacturer name and device name
    bool get_product_name(char *product_name, uint8_t buflen) const override;

    // returns true if design_capacity in mAh (capacity when newly manufactured) can be provided and fills it in
    bool get_design_capacity_mah(float &design_capacity) const override;

    // returns true if the full charge capacity in mAh (accounting for battery degradation) can be provided and fills it in
    bool get_full_charge_capacity_mah(float &full_charge_capacity) const override;

    // returns true if the design voltage in volts (maximum charging voltage) can be provided and fill it in
    bool get_design_voltage(float &design_voltage) const override;

    // returns true if the manufacture date can be provided and fills it in
    bool get_manufacture_date(char *manufacture_date, uint8_t buflen) const override;

    // returns true if the number of cells in series can be provided and fills it in
    virtual bool get_cells_in_series(uint8_t &cells_in_series) const override { return false; }
#endif

    virtual void init(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    void read(void) override;

    // reads the pack full charge capacity (accounting for battery degradation)
    // returns true if the read was successful, or if we already knew the pack capacity
    void read_full_charge_capacity(void);

    // reads the remaining capacity
    // which will only be read if we know the full charge capacity (accounting for battery degradation)
    void read_remaining_capacity(void);

    // return a scaler that should be multiplied by the battery's reported capacity numbers to arrive at the actual capacity in mAh
    virtual uint16_t get_capacity_scaler() const { return 1; }

    // reads the temperature word from the battery
    virtual void read_temp(void);

    // reads the serial number if it's not already known
    // returns if the serial number was already known
    void read_serial_number(void);

    // reads the battery's cycle count
    void read_cycle_count();

    // reads the battery's pack design voltage (maximum charging voltage)
    void read_design_voltage(void);

    // reads the pack design capacity (capacity when newly manufactured)
    void read_design_capacity(void);

    // returns number of characters written to name_out if the given register read of a character array was successful else returns 0
    uint8_t read_name(AP_BattMonitor_SMBus::BATTMONITOR_SMBUS reg_name,  char * name_out, uint8_t buflen);

    // return the manufacturer name and return if already known
    void read_manufacturer_name(void);

    // read the decvice name and return if already known
    void read_device_name(void);

    // read the manufacture date and return if already known
    void read_manufacture_date(void);

     // read word from register
     // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, bool append_zero) const;

    // get_PEC - calculate PEC for a read or write from the battery
    // buff is the data that was read or will be written
    uint8_t get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool _pec_supported; // true if PEC is supported

    int32_t _serial_number = -1;    // battery serial number
    uint16_t _full_charge_capacity_mah; // full charge capacity, used to stash the value before setting the parameter
    bool _has_cell_voltages;        // smbus backends flag this as true once they have received a valid cell voltage report
    uint16_t _cycle_count;          // number of cycles the battery has experienced. An amount of discharge approximately equal to the value of DesignCapacity.
    bool _has_cycle_count;          // true if cycle count has been retrieved from the battery
    bool _has_temperature;

#if HAL_SMART_BATTERY_INFO_ENABLED
    uint16_t _design_voltage_mv;    // battery pack design voltage in  mV (maximum charging voltage)
    uint16_t _design_capacity_mah;  // battery pack design capacity in mAh (capacity when newly manufactured)

    uint8_t _device_name_len;
    char _device_name[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER+1];
    uint8_t _manufacturer_name_len;
    char _manufacturer_name[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER+1];

    bool _has_manufacture_date;
    uint8_t _manufacture_date[3];   // manufacturer date {day, month, year since 1980}
#endif

    virtual void timer(void) = 0;   // timer function to read from the battery

    AP_HAL::Device::PeriodicHandle timer_handle;

    // Parameters
    AP_Int8  _bus;          // I2C bus number
    AP_Int8  _address;      // I2C address

};
