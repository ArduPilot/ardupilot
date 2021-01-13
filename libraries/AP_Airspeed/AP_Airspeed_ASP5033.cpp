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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define ASP5033_I2C_ADDR 0x6D
#define Sys_config 0xA5
#define CMD 0x30
#define CMD_measure 0x0A
#define Press_MSB 0x06
#define Press_CSB 0x07
#define Press_LSB 0x08
#define Temp_MSB 0x09
#define Temp_LSB 0x0A
#define Press_neg 0x800000
#define Press_Rng 0x1000000
#define Press_RngCal 0x80
#define Temp_neg 0x8000
#define Temp_Rng 0x10000
#define Temp_RngCal 0x100

AP_Airspeed_ASP5033::AP_Airspeed_ASP5033(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

// probe and initialise the sensor
bool AP_Airspeed_ASP5033::init()
{
    _dev = hal.i2c_mgr->get_device(get_bus(), ASP5033_I2C_ADDR);
    if (!_dev) {
        return false;
        printf("ASP5033: sensor no found\n");
    }
    _dev->get_semaphore()->take_blocking();
    _dev->set_retries(2);
    _dev->get_semaphore()->give();
    _dev->register_periodic_callback(1000000UL/80U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_ASP5033::timer, void));
    printf("ASP5033: sensor found\n");
    return true;
}

// read the values from the sensor
void AP_Airspeed_ASP5033::timer()
{
    uint8_t SysID;
    uint8_t Sco;
    uint8_t pressraw_bytes[3];
    uint8_t tempraw_bytes[2];
    _dev->read_registers(Sys_config, &SysID, 1); //Read Sensor_config
     SysID = (SysID&0xFD);
    _dev->write_register(Sys_config, SysID); //Set Sensor_config
    _dev->write_register(CMD, CMD_measure); //once temperature conversion immediately followed by once sensor signal conversion
    _dev->read_registers(CMD, &Sco, 1); //Judge whether Data collection is over
       while ((Sco & 0x08) > 0){ 
       _dev->read_registers(CMD, &Sco, 1);
       }
       //Read ADC output data of pressure
        _dev->read_registers(Press_MSB, &pressraw_bytes[0], 1);  
        _dev->read_registers(Press_CSB, &pressraw_bytes[1], 1);
        _dev->read_registers(Press_LSB, &pressraw_bytes[2], 1); 
        pressraw = (pressraw_bytes[0]<<16) + (pressraw_bytes[1]<<8)+ pressraw_bytes[2];
        //Read ADC output data of temperature
        _dev->read_registers(Temp_MSB, &tempraw_bytes[0], 1);
        _dev->read_registers(Temp_LSB, &tempraw_bytes[1], 1);
        tempraw = (tempraw_bytes[0]<<8 )+tempraw_bytes[1];
        //Judge pressure data direction
        if (pressraw> Press_neg){
        pressraw = (pressraw - Press_Rng);
        pressure_sum += pressraw / Press_RngCal; 
        }
        else{
        pressure_sum += pressraw / Press_RngCal;
        }
        //Temperature positive and negative processing
        if(tempraw >Temp_neg){
        tempraw = tempraw - Temp_Rng; 
        temperature_sum += tempraw/Temp_RngCal;  
      }
        else {
        temperature_sum += tempraw/Temp_RngCal;}
        WITH_SEMAPHORE(sem); 
        press_count++;
        temp_count++;   
    last_sample_time_ms = AP_HAL::millis();
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