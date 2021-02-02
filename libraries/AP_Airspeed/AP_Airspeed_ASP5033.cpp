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

/*
  backend driver for airspeed from www.qio-tek.com a I2C ASP5033 sensor
 */

#include "AP_Airspeed_ASP5033.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define ASP5033_I2C_ADDR 0x6D
#define SYS_CONFIG       0xA5
#define CMD              0x30
#define CMD_MEASURE      0x0A
#define PRESS_DATA       0x06
#define TEMP_DATA        0x09
#define WAI_REG          0x01
#define WAI_SET          0XA4
#define DEVICE_ID        0X66
#define SENSOR_READY     0x08


AP_Airspeed_ASP5033::AP_Airspeed_ASP5033(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

// probe and initialise the sensor
bool AP_Airspeed_ASP5033::init()
{
    uint8_t SysID;
    uint8_t whoami;
    _dev = hal.i2c_mgr->get_device(get_bus(), ASP5033_I2C_ADDR);
    if (!_dev) {
        return false;      
    }
    _dev->get_semaphore()->take_blocking();
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->write_register(WAI_SET, DEVICE_ID); //Set Whoami ID. 0XA4 is a OPT WhoamI Part ID register, also can be read from address 0x01.
    //check whoami, set the PartID register to 0X66 first, them Read the PartID register.
    if (!_dev->read_registers(WAI_REG, &whoami, 1) || //Read the whoami ID
        whoami != DEVICE_ID){
        _dev->set_retries(2);   
        return false;        
    }
    _dev->get_semaphore()->give();
    if(!_dev->read_registers(SYS_CONFIG, &SysID, 1)|| //Read Sensor_config
        SysID != (SysID&0xFD)){ //Read the 0xA5 register value, put the read binary value "and" on "11111111101" then write to 0xA5.
    _dev->write_register(SYS_CONFIG, SysID);     //Set Sensor_config
    }
    _dev->register_periodic_callback(1000000UL/80U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_ASP5033::timer, void));
    return true;
}         

// read the data from the sensor
void AP_Airspeed_ASP5033::timer()
{
    uint8_t SoC; //Start of conversion flag
    uint8_t pressraw_bytes[2]; //pressure data AD value
    uint8_t tempraw_bytes[2]; //temperature data AD value
    _dev->write_register(CMD, CMD_MEASURE); //once temperature conversion immediately followed by once sensor signal conversion
    _dev->read_registers(CMD, &SoC, 1); //Judge whether Data collection is over
    if ((SoC & SENSOR_READY) != 1){   // Read the 0x30 register address. If Sco bit is 0,signify the acquisition end, the data can be read.
        if(_dev->read_registers(PRESS_DATA, pressraw_bytes, 2)){
        //Read ADC output data of pressure
        //Read 0x06, 0x07, 0x08 register address data to form a 24-bit AD value (pressure data AD value).
        pressraw = (pressraw_bytes[0]<<24) + (pressraw_bytes[1]<<16); 
        //Effective value converting 24 bit integer to 32 bit integer 
        pressraw >>=15;
        }        
        //Read ADC output data of temperature
        //Read 0x09, 0x10, register address data to form a 16-bit AD value (temperature data AD value,the highest is the symbol bit.).
        if(_dev->read_registers(TEMP_DATA, tempraw_bytes, 2)){
        tempraw = (tempraw_bytes[0]<<8) + tempraw_bytes[1];
        //Right-shift signed integer back to get correct measurement value
        tempraw >>=8;
        }            
        WITH_SEMAPHORE(sem); 
        pressure_sum += pressraw;
        temperature_sum += tempraw;
        press_count++;
        temp_count++;
        last_sample_time_ms = AP_HAL::millis();
        }  
}

 

// return the current differential_pressure in Pascal
 bool AP_Airspeed_ASP5033::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }

    _pressure = pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_ASP5033::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (temp_count > 0) {
        temperature = temperature_sum / temp_count;
        temp_count = 0;
        temperature_sum = 0;
    }

    _temperature = temperature;
    return true;
}