/*
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/* feature added by Anemos Technologies : Philippe Crochat (pcrochat@anemos-technologies.com) */

#include "RCInput_Aero.h"
#include <stdio.h>

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

using namespace Linux;

#define DEBUG 0

#define DEVICE_NAME "aeroio"

#define CHANNEL_MIN_VALUE 970
#define CHANNEL_MAX_VALUE 2000
#define MAX_DIFF_VALUE 300
#define MAX_RETRY 10
#define CLOCK_REG 0x4a
#define CHANNEL_REG 0x4b
#define COMPLEMENTARY_FILTER 0.93

#if DEBUG
#define debug(fmt, args...) ::printf(fmt "\n", ##args)
#else
#define debug(fmt, args...)
#endif

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();


RCInput_Aero::RCInput_Aero()
{
	
}

//initialize Aero RC Input 
void RCInput_Aero::init()
{
	//get the clock speed of the FPGA in MHz
	const uint8_t tx_buffer[2]={CLOCK_REG, 0x00};
	uint8_t rx_buffer[2];
	
	_spi = hal.spi->get_device(DEVICE_NAME);
	
	if (!_spi) {
        AP_HAL::panic("Could not initialize Aero RCInput");
    }
	
    if (!_spi->get_semaphore()->take_nonblocking()) {
        return;
    }
    
    if (!_spi->transfer_fullduplex(tx_buffer, rx_buffer, 2)) {
    	AP_HAL::panic("Could not get FPGA frequency");
    }
    
    _spi->get_semaphore()->give();
    
    _frequency=(uint8_t)rx_buffer[1];
    
    if (!_frequency) {
    	AP_HAL::panic("Aero RCInput : Panic no valid FPGA frequency read for device name");
    }
}


//reads all channels in a raw from the SPI bus
void RCInput_Aero::_timer_tick(void)
{
	uint8_t i;
	const uint8_t buffer_size=CHANNEL_NUMBER*2+1;
	
	uint16_t channels[CHANNEL_NUMBER];
	uint8_t tx_buffer[buffer_size];
	uint8_t rx_buffer[buffer_size];
	
	bool good_value;
	int8_t retry=-1;
	
	if (!_spi->get_semaphore()->take_nonblocking()) {
        return;
    }
	
	do {
		good_value=true;
		for(i=1;i<buffer_size;i++) {
			tx_buffer[i]=0x00;
		}
		
		tx_buffer[0]=CHANNEL_REG; //first CHANNEL SPI reg address
		
		if (!_spi->transfer_fullduplex(tx_buffer, rx_buffer, buffer_size)) {
			printf("Could not get rc input value\n");
		}
		
		for(i=0;i<CHANNEL_NUMBER;i++) {
			channels[i] = ((rx_buffer[(i<<1)+1]<<8) + rx_buffer[(i<<1)+2])/_frequency; //converts counter result into microseconds
			
			if (channels[i]<CHANNEL_MIN_VALUE || channels[i]>CHANNEL_MAX_VALUE) {
				good_value=false;
			}
		}
		
		retry++;
	} while(good_value==false && retry<MAX_RETRY);
    
    _spi->get_semaphore()->give();
	
    if (good_value==true) {
    	for(i=0;i<CHANNEL_NUMBER;i++) {
    		if (abs(channels[i]-_old_channels[i]) > MAX_DIFF_VALUE) {
    			channels[i] = (uint16_t) (COMPLEMENTARY_FILTER*(float)_old_channels[i] + (1.0-COMPLEMENTARY_FILTER)*(float)channels[i]);
    		}
    		
    		_old_channels[i]=channels[i];
    	}
    	
    	_update_periods(channels, CHANNEL_NUMBER);
    }
}
