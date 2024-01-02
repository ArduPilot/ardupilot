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



#include "AP_AngleSensor_AS5048B.h"

#if AP_ANGLESENSOR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/definitions.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AS5048B_POLL_RATE_HZ 20 

#define AS5048B_DIAGNOSTIC_REG   0xFB  // Magnitude, least significant byte
#define AS5048B_MAGMSB_REG       0xFC  // Magnitude, most significant byte
#define AS5048B_MAGLSB_REG       0xFD  // Magnitude, least significant byte
#define AS5048B_ANGLEMSB_REG     0xFE  // Angle, most significant byte
#define AS5048B_ANGLELSB_REG     0xFF  // Angle, least significant byte

// We will read 5 registers, starting from the diagnostic register
#define AS5048B_DIAGNOSTIC_OFS   0
#define AS5048B_MAGMSB_OFS       (AS5048B_MAGMSB_REG - AS5048B_DIAGNOSTIC_REG)
#define AS5048B_MAGLSB_OFS       (AS5048B_MAGLSB_REG - AS5048B_DIAGNOSTIC_REG)
#define AS5048B_ANGLEMSB_OFS     (AS5048B_ANGLEMSB_REG - AS5048B_DIAGNOSTIC_REG)
#define AS5048B_ANGLELSB_OFS     (AS5048B_ANGLELSB_REG - AS5048B_DIAGNOSTIC_REG)

#define AS5048B_COUNTS 16384

AP_AngleSensor_AS5048B::AP_AngleSensor_AS5048B(
    AP_AngleSensor &frontend, 
    uint8_t instance, 
    AP_AngleSensor::AngleSensor_State &state) :
    AP_AngleSensor_Backend(frontend, instance, state)
{
    init();
}

bool AP_AngleSensor_AS5048B::init(void)
{
    uint8_t bus = _frontend._params[_state.instance]._bus;
    uint8_t address = _frontend._params[_state.instance]._addr;

    _dev = hal.i2c_mgr->get_device(bus, address);
    if (!_dev) {
        hal.console->printf("AP_AngleSensor: Could not open I2C Device on Bus %u at Address 0x%02x \n", bus, address);
        return false;
    }

    hal.console->printf("AP_AngleSensor: Opened Device for Bus %u at Address 0x%02x \n", bus, address);   
    
    // call timer() at 20Hz
    _dev->register_periodic_callback(1E6/AS5048B_POLL_RATE_HZ ,
                                     FUNCTOR_BIND_MEMBER(&AP_AngleSensor_AS5048B::timer, void));

    return true;
}

void AP_AngleSensor_AS5048B::update(void)
{
    // nothing to do - its all done in the timer()
}


void AP_AngleSensor_AS5048B::timer(void)
{
    
    uint8_t buf[5];

    uint32_t time = AP_HAL::millis();
    _dev->get_semaphore()->take_blocking();
    bool success = _dev->read_registers(AS5048B_DIAGNOSTIC_REG, buf, sizeof(buf));
    _dev->get_semaphore()->give();

    if (!success){
        copy_state_to_frontend(0,0,time);
        return;
    }

    bool data_invalid = buf[AS5048B_DIAGNOSTIC_OFS] & 0b10;  // CORDIC Overflow bit
    
    if (data_invalid) {
        copy_state_to_frontend(0,0,time);
        return;
    }

    uint8_t angle_msb = buf[AS5048B_ANGLEMSB_OFS];
    uint8_t angle_lsb = buf[AS5048B_ANGLELSB_OFS];
    uint16_t angle_raw = ((uint16_t) angle_msb << 6) | (angle_lsb & 0x3f);
    float angle_radians = (angle_raw / (float) AS5048B_COUNTS) * M_2PI;

    uint8_t mag_msb = buf[AS5048B_MAGMSB_OFS];
    uint8_t mag_lsb = buf[AS5048B_MAGLSB_OFS];
    uint16_t mag_raw = ((uint16_t) mag_msb << 6) | (mag_lsb & 0x3f);
    uint8_t quality = (100 * mag_raw)/ AS5048B_COUNTS ;
    
    copy_state_to_frontend(angle_radians,quality,time);

}


#endif  // AP_ANGLESENSOR_ENABLED
