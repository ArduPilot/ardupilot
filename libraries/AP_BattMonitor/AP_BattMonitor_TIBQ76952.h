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

    // set battery BMS sleep timeout in seconds
    // set to zero to disable sleep
    void set_sleep_timeout(uint16_t timeout_sec) override { sleep_timeout_sec = timeout_sec; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // periodic timer callback
    void timer();

    // configure device
    // this includes delays so it should only be called during startup configuration
    bool configure();

    // wait for the device to enter (or exit) CONFIG_UPDATE mode, returns true on success
    bool wait_for_cfgupdate(bool in_cfgupdate) const;

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

    // estimate state of charge (0-100%) from the average cell voltage
    // this is only accurate when the battery is at rest
    // returns true on success
    bool estimate_soc_from_cell_voltage(float &soc_pct) const;

    // read bytes from a register. returns true on success
    bool read_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t len) const;

    // write a single byte to consecutive registers. returns true on success
    bool write_register(uint8_t reg_addr, const uint8_t *reg_data, uint8_t len) const;

    // send a direct command to read 2 bytes, returns true on success
    bool direct_command_read_2bytes(uint16_t reg, uint16_t &value) const;

    // send a direct command to write 1byte
    bool direct_command_write_1byte(uint16_t reg, uint8_t data) const;

    // send a command with no data payload and no checksum (e.g. ALL_FETS_ON, SLEEP_DISABLE)
    bool indirect_send_command(uint16_t command) const;

    // write 1, 2 or 4 bytes to a Data Memory address (0x9xxx) or Subcommand with data
    // this includes delays so it should only be called during startup configuration
    bool indirect_write(uint16_t addr, uint32_t data, uint8_t len) const;

    // read up to 32 bytes from a Data Memory address (0x9xxx) or Subcommand response
    // returns true if the response is ready and its checksum is correct
    // this includes delays so it should only be called during startup configuration
    bool indirect_read(uint16_t addr, uint8_t *rx_data, uint8_t len) const;

    // read 4 bytes via the indirect mechanism (e.g. DEVICE_NUMBER, FW_VERSION, HW_VERSION)
    // this includes delays so it should only be called during startup configuration
    uint32_t indirect_read_4bytes(uint16_t addr) const;

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

    // internal variables
    AP_HAL::I2CDevice *dev; // I2C device
    bool configured;        // true once device has been configured
    bool soc_initialised;   // true once consumed capacity has been seeded from cell voltages

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

    struct {
        bool on;            // true if user has requested the battery be powered on (discharge FET enabled)
        bool pending;       // true if the requested state has not yet been sent to the TIBQ device
    } power_state_req;      // user requested power state

    uint32_t last_read_time_ms;     // timestamp of last read
    bool bms_fault;         // true if BMS reports some kind of failure or fault
    uint16_t sleep_timeout_sec = 30;    // battery BMS sleep timeout in seconds
    bool sleep_timeout_extended;    // true if MCU remained powered after TIBQ device was put into deep sleep, sleep timeout is increased
    uint32_t activity_timer_ms; // timestamp of last activity, used to determine if sleep mode
    uint32_t deep_sleep_req_ms; // system time TIBQ device was commanded into deep sleep.  0 if not requested
};

#endif // AP_BATTERY_TIBQ76952_ENABLED
