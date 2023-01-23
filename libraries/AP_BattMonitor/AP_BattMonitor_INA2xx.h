/****************************************************************************************
@file:AP_BattMonitor_INA2xx.h
@brief:this library is based on INA series power monitor sensors (INA226,INA260 and INA219).
       All three of the sensor's are I2C based.
*****************************************************************************************
*/

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <AP_Param/AP_Param.h>
#include <utility>

#ifndef HAL_BATTMON_INA2XX_ENABLED
#define HAL_BATTMON_INA2XX_ENABLED (BOARD_FLASH_SIZE>1024)
#endif

#if HAL_BATTMON_INA2XX_ENABLED

//enable or disable only one of the sensor at a given time (enable at least one of the following sensor)
#define HAL_BATTMON_INA260_ENABLED  1                   
#define HAL_BATTMON_INA219_ENABLED  0
#define HAL_BATTMON_INA226_ENABLED  0

class AP_BattMonitor_INA2xx: public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_INA2xx(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);
    
    //set default of class methods for the specified sensor
    bool has_cell_voltages() const override { return false; }
    bool has_temperature() const override { return false; }
    bool has_current() const override { return true; }
    bool reset_remaining(float percentage) override { return false; }
    bool get_cycle_count(uint16_t &cycles) const override { return false; }
    bool has_consumed_energy() const override {return true;}

    void init(void) override;
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    void configure(void);
    bool read_word(const uint8_t reg, int16_t& data) const;
    bool write_word(const uint8_t reg, const uint16_t data) const;
    void timer(void);

    AP_Int8 _i2c_bus;
    AP_Int8 _i2c_address;
    bool _configured;
    uint32_t _failed_reads;
    uint32_t _last_configure_ms;

    struct {
        uint16_t _count;
        float _voltage;
        float _current;
        HAL_Semaphore sem;
    } accumulate;

#if HAL_BATTMON_INA260_ENABLED
    /*Current and Voltage LSB variables for INA260*/
    static constexpr float _current_LSB = 0.00125;   //1.25mA
    static constexpr float _voltage_LSB = 0.00125;   //1.25mV
#endif

#if HAL_BATTMON_INA219_ENABLED
    /*Current and Voltage LSB variables for INA219*/
    float _current_LSB;
    static constexpr float _voltage_LSB = 0.004;     //4mV for bus voltage
#endif

#if HAL_BATTMON_INA226_ENABLED
    /*Current and Voltage LSB variables for INA226*/
    float _current_LSB;
    static constexpr float _voltage_LSB = 0.00125;     //1.25mV for bus voltage
#endif

};

#endif // HAL_BATTMON_INA2XX_ENABLED
