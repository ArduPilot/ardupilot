#include "AP_BattMonitor_INA2xx.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#if HAL_BATTMON_INA2XX_ENABLED

extern const AP_HAL::HAL& hal;

/*******************register definitions***************/

#if HAL_BATTMON_INA260_ENABLED
    /*INA260 registers*/
    #define REG_CONFIG            0x00
    #define REG_CURRENT           0x01
    #define REG_BUS_VOLTAGE       0x02
    #define REG_POWER             0x03
    #define REG_MASK              0x06
    #define REG_ALERT             0x07
    #define REG_MANUF_ID          0xFE
    #define REG_DIE_ID            0xFF

    #define REG_CONFIG_RESET      0x8000
    #define REG_CONFIG_DEFAULT    0x6127                      
    #define REG_MANUF_ID_NUMBER   0x5449
    #define REG_DEVICE_ID_NUMBER  0x227

#endif

#if HAL_BATTMON_INA219_ENABLED
    /*INA219 registers*/
    #define REG_CONFIG            0x00
    #define REG_SHUNT_VOLTAGE     0x01
    #define REG_BUS_VOLTAGE       0x02
    #define REG_POWER             0x03
    #define REG_CURRENT           0x04
    #define REG_CALIBRATION       0x05

    #define REG_CONFIG_RESET      0x8000
    #define REG_CONFIG_DEFAULT    0x399F                          

    #define MAX_AMPS 3.0
#endif

#if HAL_BATTMON_INA226_ENABLED
    /*INA226 registers*/
    #define REG_CONFIG            0x00
    #define REG_SHUNT_VOLTAGE     0x01
    #define REG_BUS_VOLTAGE       0x02
    #define REG_CURRENT           0x04
    #define REG_CALIBRATION       0x05
    #define REG_MASK              0x06
    #define REG_ALERT             0x07
    #define REG_MANUF_ID          0xFE
    #define REG_DIE_ID            0xFF

    #define REG_CONFIG_RESET      0x8000
    #define REG_CONFIG_DEFAULT    0x4127
    #define REG_MANUF_ID_NUMBER   0x5449
    #define REG_DEVICE_ID_NUMBER  0x226

    // this should become a parameter in future
    #define MAX_AMPS 90.0

#endif


/*I2C bus and address*/
#ifndef HAL_BATTMON_INA2XX_BUS
#define HAL_BATTMON_INA2XX_BUS  0                          
#endif
#ifndef HAL_BATTMON_INA2XX_ADDR
#define HAL_BATTMON_INA2XX_ADDR 64                         //address 0x40 = 64 when A1 and A0 are GND
#endif

const AP_Param::GroupInfo AP_BattMonitor_INA2XX::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 25, AP_BattMonitor_INA2XX, _i2c_bus, HAL_BATTMON_INA2XX_BUS),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 26, AP_BattMonitor_INA2XX, _i2c_address, HAL_BATTMON_INA2XX_ADDR),

    AP_GROUPEND
};

AP_BattMonitor_INA2XX::AP_BattMonitor_INA2XX(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

/*
@brief  : backend init function, used to create a device handle
          as well as setup a timer function which runs at the interval programmed.
@param  : none
@retval : none
*/
void AP_BattMonitor_INA2XX::init(void)
{
    dev = hal.i2c_mgr->get_device(_i2c_bus, _i2c_address, 100000, false, 20);        //100kHz freq. and 20ms timeout
    if (!dev) {
        return;
    }
    // register now and configure in the timer callbacks
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA2xx::timer, void));
}

/*
@brief  : this function is responsible for basic configuration of the INA sensor selected
@param  : none
@retval : none
*/
void AP_BattMonitor_INA2XX::configure(void)
{
    uint16_t config = 0;
    //setup configuration register
    if (!write_word(REG_CONFIG, REG_CONFIG_RESET) ||
        !write_word(REG_CONFIG, REG_CONFIG_DEFAULT) ||
        !read_word_unsigned(REG_CONFIG, config) ||
        config != REG_CONFIG_DEFAULT) {
        return;
//         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "not configured");      //only for debugging
    }

    /*For INA260 based sensor*/
#if HAL_BATTMON_INA260_ENABLED
    //check manufacturer id and device id details
    int16_t manuf_id = 0;
    int16_t die_id = 0;
    if(!read_word(REG_MANUF_ID,manuf_id) || !read_word(REG_DIE_ID,die_id)){
        return;
    }else{
        if((manuf_id!=REG_MANUF_ID_NUMBER) || ((die_id>>4)!=REG_DEVICE_ID_NUMBER)){
            return;
        }
    }
#endif

    /*For INA219 based sensor*/
#if HAL_BATTMON_INA219_ENABLED

    // configure for MAX_AMPS
    const uint16_t conf = (1<<13) | (0x3<<11) | (0x3<<7) | (0x3<<3) | 0x7;
    const float rShunt = 0.1;
    _current_LSB = MAX_AMPS / 32768.0;
    const uint16_t cal = uint16_t(0.04096 / (_current_LSB * rShunt));
    if (!write_word(REG_CALIBRATION, cal) || !write_word(REG_CONFIG, conf)){
        return;
    }
#endif

    /*For INA226 based sensor*/
#if HAL_BATTMON_INA226_ENABLED    
    //check manufacturer id and device id details
    int16_t manuf_id = 0;
    int16_t die_id = 0;
    if(!read_word(REG_MANUF_ID,manuf_id) || !read_word(REG_DIE_ID,die_id)){
        return;
    }else{
        if((manuf_id!=REG_MANUF_ID_NUMBER) || ((die_id>>4)!=REG_DEVICE_ID_NUMBER)){
            return;
        }
    }

    // configure for MAX_AMPS
    const uint16_t conf = (0x2<<9) | (0x5<<6) | (0x5<<3) | 0x7; // 2ms conv time, 16x sampling
    const float rShunt = 0.0005;
    _current_LSB = MAX_AMPS / 32768.0;
    const uint16_t cal = uint16_t(0.00512 / (_current_LSB * rShunt));
    if (!write_word(REG_CONFIG, REG_CONFIG_RESET) || // reset
        !write_word(REG_CONFIG, conf) ||
        !write_word(REG_CALIBRATION, cal)) {
        return;
    }    
#endif

    _configured = true;
}

/*
@brief  : this function is responsible for reading the battery_voltage and current, should be called at 10hz
@param  : none
@retval : none
*/
void AP_BattMonitor_INA2XX::read(void)
{
    if(_device_detected){
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"device detected");   //only for debugging
        
        WITH_SEMAPHORE(accumulate.sem);
       _state.healthy = accumulate._count > 0;
       if (!_state.healthy) {
        return;
       }

       _state.voltage = accumulate._volt_sum/ accumulate._count;
       _state.current_amps = accumulate._current_sum/ accumulate._count;          
       accumulate._volt_sum = 0;
       accumulate._current_sum = 0;
       accumulate._count = 0;

       const uint32_t tnow = AP_HAL::micros();
       const uint32_t dt_us = tnow - _state.last_time_micros;
    
       // update total current drawn since startup
       update_consumed(_state, dt_us);

       _state.last_time_micros = tnow;
    }
    else{
        _state.healthy = false;
    }
}

/*
@brief  : this function is responsible for reading word from register 
@param  : register to read from and signed integer variable to store the register value
@retval : returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA2xx::read_word_signed(const uint8_t reg, int16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = int16_t(be16toh(uint16_t(data)));

    return true;
}

/*
@brief  : this function is responsible for reading word from voltage register 
@param  : register to read from and unsigned integer variable to store the register value
@retval : returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA2xx::read_word_unsigned(const uint8_t reg, uint16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = uint16_t(be16toh(uint16_t(data)));

    return true;
}

/*
@brief  : this function is responsible for writing word to register 
@param  : register to write to and data to be written
@retval : returns true if write was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { reg, uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

/*
@brief  : configuration status is checked at first.
          reads voltage and current then updates the variable. 
@param  : none
@retval : none
*/
void AP_BattMonitor_INA2XX::timer(void)
{
    // allow for power-on after boot
   if (!_configured) {
        uint32_t now = AP_HAL::millis();
        if (now - _last_configure_ms > 200) {
            // try contacting the device at 5Hz
            _last_configure_ms = now;
            configure();
        }
        if (!_configured) {
            _device_detected = false;
            // waiting for the device to respond
            return;
        }
    }else{
        _device_detected = true;
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "configured");               //only for debugging
    }

    if(_device_detected){
        uint16_t bus_voltage;
        uint16_t power;
        int16_t  current;

        if (!read_word_unsigned(REG_BUS_VOLTAGE, bus_voltage) || 
            !read_word_signed(REG_CURRENT, current) ||
            !read_word_unsigned(REG_POWER,power)) {
                _failed_reads++;
                if (_failed_reads > 10) {
                // device has disconnected, we need to reconfigure it
                _configured = false;
                _failed_reads = 0;
                return;
                }        
        }

        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"timer loop");
        WITH_SEMAPHORE(accumulate.sem);
#if (HAL_BATTMON_INA260_ENABLED || HAL_BATTMON_INA226_ENABLED)
        accumulate._volt_sum += (bus_voltage*_voltage_LSB);            //mVolts
#endif

#if HAL_BATTMON_INA219_ENABLED
        accumulate._volt_sum += ((bus_voltage>>3)*_voltage_LSB);       //mVolts
#endif
        accumulate._current_sum += (current*_current_LSB);             //mAmperes
        accumulate._count++;
    }
}

#endif // HAL_BATTMON_INA2XX_ENABLED
