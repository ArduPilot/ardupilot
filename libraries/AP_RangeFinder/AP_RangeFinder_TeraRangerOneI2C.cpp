// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_RangeFinder_TeraRangerOneI2C.cpp
 *       Code by Philippe Crochat. www.anemos-technologies.com
 *
 *       datasheet: http://www.teraranger.com/wp-content/uploads/2016/06/TR_One_Manual_FW5.0.Version1_0_0.pdf
 *
 *       Sensor should be connected to the I2C port
 */

#include "AP_RangeFinder_TeraRangerOneI2C.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

static const uint8_t crc_table[] = {  
    // This table is used by the CRC8 calculation function  
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe, 
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4, 
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10, 
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7, 
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83, 
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 
    0xfa, 0xfd, 0xf4, 0xf3 
}; 

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_TerarangerI2C::AP_RangeFinder_TerarangerI2C(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/* 
   detect if a Teraranger rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a whoami result the sensor is
   there.
*/
bool AP_RangeFinder_TerarangerI2C::detect(RangeFinder &_ranger, uint8_t instance)
{
    uint8_t buff[1];
	
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();
    
    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }
    
    uint8_t tosend[1] = 
        { AP_RANGE_FINDER_TERARANGERI2C_COMMAND_WHOAMI };

    // send command to take reading
    if (hal.i2c->write(AP_RANGE_FINDER_TERARANGERI2C_DEFAULT_ADDR,
                       1, tosend) != 0) {
        i2c_sem->give();
        return false;
    }
    
    i2c_sem->give();
    
    if (!start_reading()) {
        return false;
    }
    
    // give time for the sensor to process the request
    hal.scheduler->delay(50);
    
    if(hal.i2c->read(AP_RANGE_FINDER_TERARANGERI2C_DEFAULT_ADDR, 1, buff)) {
    	i2c_sem->give();
        return false;    
    }
    
    return (buff[0]==0xA1);
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_TerarangerI2C::start_reading()
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    uint8_t tosend[1] = 
        { AP_RANGE_FINDER_TERARANGERI2C_COMMAND_TAKE_RANGE_READING };

    // send command to take reading
    if (hal.i2c->write(AP_RANGE_FINDER_TERARANGERI2C_DEFAULT_ADDR,
                       1, tosend) != 0) {
        i2c_sem->give();
        return false;
    }

    // return semaphore
    i2c_sem->give();

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_TerarangerI2C::get_reading(uint16_t &reading_cm)
{
    uint8_t buff[3];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    // take range reading and read back results
    if (hal.i2c->read(AP_RANGE_FINDER_TERARANGERI2C_DEFAULT_ADDR, 3, buff) != 0) {
        i2c_sem->give();
        return false;
    }
    i2c_sem->give();

    // combine results into distance
    reading_cm = (((uint16_t)buff[0]) << 8 | buff[1])/10;

    // trigger a new reading
    start_reading();

    if(AP_RangeFinder_TerarangerI2C_crc8(buff,2)!=buff[2])
    	    return false;
    
    return true;
}


/* 
   update the state of the sensor
*/
void AP_RangeFinder_TerarangerI2C::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

uint8_t AP_RangeFinder_TerarangerI2C_crc8(uint8_t *p, uint8_t len)
{
	// p is a buffer of type uint8_t and of length len  
	uint16_t i; 
        uint16_t crc = 0x0; 
        
        while (len--) {
        	i = (crc ^ *p++) & 0xFF;
        	crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
        }
        
        return crc & 0xFF;
}


