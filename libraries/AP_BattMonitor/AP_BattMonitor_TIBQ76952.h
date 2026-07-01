#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_TIBQ76952_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Param/AP_Param.h>
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_TIBQ76952 : public AP_BattMonitor_Backend
{
public:
    AP_BattMonitor_TIBQ76952(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    // initialise
    void init() override;
   
    // read the latest battery voltage
    void read() override;

    // battery capabilities
    bool has_cell_voltages() const override { return true; }
    bool has_temperature() const override { return true; }
    bool has_current() const override { return true; }
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    // set desired powered state (enabled/disabled) by enabling/disabling discharge FET
    void set_powered_state(bool power_on) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Subcommand operation types
    enum class CommandType {
        READ = 0,    // Read operation (R)
        WRITE = 1,   // Write operation with 1 byte data (W)
    };

    // periodic timer callback
    void timer();

    // configure device
    // this includes delays so it should only be called during startup configuration
    bool configure();

    // compare the current configuration against the desired settings
    // returns true if the current device configuration matches, false otherwise
    bool check_configuration_ok() const;

    // returns true if the device is configured and responding to commands
    bool healthy() const;

    // read voltage, current and temperature from BMS device, returns true on success
    bool read_voltage_current_temperature();

    // read charging state (e.g. idle, charging, discharging)
    void read_charging_state();

    // check if the BMS should sleep
    void check_sleep_timeout();

    // read bytes from a register. returns true on success
    bool read_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t len) const;

    // write a single byte to consecutive registers. returns true on success
    bool write_register(uint8_t reg_addr, const uint8_t *reg_data, uint8_t len) const;

    // read or write a direct command. returns true on success
    // direct commands are send using a 7-bit command address and may trigger an action or read/write a datavalue
    bool direct_command(uint16_t command, uint16_t data, CommandType type, uint8_t *rx_data = nullptr, uint8_t rx_len = 2) const;

    // send a direct command to read 1 or 2 bytes
    uint8_t direct_command_read_1byte(uint16_t reg) const;
    uint16_t direct_command_read_2bytes(uint16_t reg) const;

    // send a direct command to write 1byte
    bool direct_command_write_1byte(uint16_t reg, uint8_t data) const;

    // write 1, 2, or 4 bytes to a RAM data memory register (0x9xxx addresses)
    // this includes delays so it should only be called during startup configuration
    void set_register(uint16_t reg_addr, uint32_t reg_data, uint8_t len) const;

    // send a command-only subcommand (no data payload, no checksum)
    bool sub_command(uint16_t command) const;

    // send a subcommand with read or write capability
    // this includes delays so it should only be called during startup configuration
    bool sub_command(uint16_t command, uint16_t data, CommandType type, uint8_t *rx_data = nullptr, uint8_t rx_len = 32) const;

    // subcommands are sent using a 16 bit command address and support block data transfers
    // this includes delays so it should only be called during startup configuration
    uint32_t sub_command_read_4bytes(uint16_t reg) const;

    // calculate checksum for given data buffer and length
    uint8_t calculate_checksum(const uint8_t* data, uint8_t len) const;

    // enum for CFG_UPDATE parameter
    enum class ConfigUpdateType : int8_t {
        DISABLED = 0,
        WRITE_ONCE = 1,
        CHECK_AND_UPDATE = 2
    };

    // parameters
    AP_Enum<ConfigUpdateType> cfg_update;   // config update (0:disabled, 1:write once, 2:check and update)
    AP_Int16 sleep_timeout_sec;             // battery sleep timeout in seconds

    // internal variables
    AP_HAL::I2CDevice *dev; // I2C device
    bool configured;        // true once device has been configured

    // configuration settings to write during setup
    static const struct ConfigurationSetting {
        uint16_t reg_addr;
        uint32_t reg_data;
        uint8_t len;
    } config_settings[]; 

    struct {
        uint16_t count;     // number of readings, values below should be divided by this number
        float voltage;      // battery voltage in volts
        uint32_t cell_voltages_mv[AP_BATT_MONITOR_CELLS_MAX];   // individual cell voltages in mv
        float current;      // battery current in amps
        float temp;         // battery temperature in degrees Celsius
    } accumulate;
    HAL_Semaphore accumulate_sem;   // semaphore for accumulate structure
    uint32_t last_read_time_ms;     // timestamp of last read
    bool bms_fault;         // true if BMS reports some kind of failure or fault
    uint32_t activity_timer_ms; // timestamp of last activity, used to determine if sleep mode
};

#endif // AP_BATTERY_TIBQ76952_ENABLED
