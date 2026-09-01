/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_BattMonitor_ACS37800.h"

#if AP_BATTERY_ACS37800_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_BattMonitor_ACS37800.h"

extern const AP_HAL::HAL& hal;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t vcodes : 16;
      uint32_t icodes : 16;
    } bits;
  } data;
} acs378002a_t;

#define RESIST_MULT 202.5

#define REG_ACCESS_CODE 0x2F // access_code
#define CUSTOMER_CODE 0x4F70656E
#define REG_C_EEPROM 0x0C
#define REG_F_SHADOW 0x1F
#define REG_F_EEPROM 0x0F
#define CONFIG_C_MASK 0x0000000F
#define CONFIG_F 0x01FFC000 // bypass_N_en=1, n=1023

#define REG_VRMS_IRMS 0x20 
#define REG_VRMS_IRMS_ONESEC 0x26
#define REG_VCODES_ICODES 0x2A
#define REG_PINSTANT 0x2C

#ifndef HAL_BATTMON_ACS37800_BUS
#define HAL_BATTMON_ACS37800_BUS 0
#endif
#ifndef HAL_BATTMON_ACS37800_ADDR
#define HAL_BATTMON_ACS37800_ADDR 96
#endif

// ACS37800-I2C provides voltage, current, and power measurements, however, it does not provide any means for calibration.
// Thus, added V_FINE_M and C_FINE_M to adjust the voltage and current readings.
const AP_Param::GroupInfo AP_BattMonitor_ACS37800::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 4
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 138, AP_BattMonitor_ACS37800, i2c_bus, HAL_BATTMON_ACS37800_BUS),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 139, AP_BattMonitor_ACS37800, i2c_address, HAL_BATTMON_ACS37800_ADDR),

    // @Param: V_FINE_M
    // @DisplayName: Voltage fine adjustment
    // @Description: Fine-adjust the voltage reading. Calculate dividing the indicated measured voltage by the known applied voltage.
    // @User: Advanced
    AP_GROUPINFO("V_FINE_M", 8, AP_BattMonitor_ACS37800, volt_fine_mult, 1.0),

    // @Param: C_FINE_M
    // @DisplayName: Current fine adjustment
    // @Description: Fine-adjust the current reading. Calculate dividing the indicated measured current by the known current draw.
    // @User: Advanced
    AP_GROUPINFO("C_FINE_M", 9, AP_BattMonitor_ACS37800, curr_fine_mult, 1.0),

    AP_GROUPEND
};

AP_BattMonitor_ACS37800::AP_BattMonitor_ACS37800(AP_BattMonitor &mon, 
                            AP_BattMonitor::BattMonitor_State &mon_state, 
                            AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_ACS37800::init(void)
{
    _dev = hal.i2c_mgr->get_device_ptr(i2c_bus, i2c_address, 100000, false, 20);
    if (!_dev) {
        return;
    }
    //Device found
    WITH_SEMAPHORE(_dev->get_semaphore());
    uint32_t f_config;
    read_word(REG_F_SHADOW, f_config);
    // if sensor is already configured, no need to write
    if (f_config != CONFIG_F) { 
        // first time using the sensor -> write config to EEPROM
        uint32_t c_config;
        // enter "customer mode" by writing the access code - so EEPROM is writeable
        write_word(REG_ACCESS_CODE, CUSTOMER_CODE);
        hal.scheduler->delay(50);
        // write configuration for DC operation 
        write_word(REG_F_EEPROM, CONFIG_F); 
        hal.scheduler->delay(90);
        read_word(REG_C_EEPROM, c_config);
        c_config |= CONFIG_C_MASK;
        write_word(REG_C_EEPROM, c_config);
        hal.scheduler->delay(100);
    } 

    _dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_ACS37800::update, void));
}

void AP_BattMonitor_ACS37800::read()
{
    WITH_SEMAPHORE(accumulate.sem);
    _state.healthy = accumulate.count > 0;
    if (!_state.healthy) {
        return;
    }

    _state.voltage = accumulate.volt_sum / accumulate.count;
    _state.current_amps = accumulate.current_sum / accumulate.count;
    accumulate.volt_sum = 0;
    accumulate.current_sum = 0;
    accumulate.count = 0;

    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;
    
    // update total current drawn since startup
    update_consumed(_state, dt_us);
    _state.last_time_micros = tnow;
}

void AP_BattMonitor_ACS37800::update()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    acs378002a_t immediate;
    if (read_word(REG_VCODES_ICODES, immediate.data.all)) {     
        WITH_SEMAPHORE(accumulate.sem);
        accumulate.volt_sum += convert_voltage(immediate.data.bits.vcodes);
        accumulate.current_sum += convert_current(immediate.data.bits.icodes);
        accumulate.count++;
    }
}

float AP_BattMonitor_ACS37800::convert_voltage(int16_t vcode)
{
    return (((((float) vcode)/27536.13)*250)/1000)*RESIST_MULT*volt_fine_mult;
}

float AP_BattMonitor_ACS37800::convert_current(int16_t icode)
{
    return (((float) icode)/32768.0)*90.0*curr_fine_mult;
}

bool AP_BattMonitor_ACS37800::write_word(const uint8_t reg, const uint32_t data) const
{
    const uint8_t b[5] { reg,  uint8_t (data&0xff), uint8_t(data >> 8), uint8_t(data >> 16), uint8_t(data >> 24) };
    return _dev->transfer(b, sizeof(b), nullptr, 0);
}

bool AP_BattMonitor_ACS37800::read_word(const uint8_t reg, uint32_t& data) const
{
    return _dev->read_registers(reg, (uint8_t *)&data, sizeof(data));
}

#endif
