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

#define DEBUG 1

#if DEBUG
#define debug(fmt, args...) ::printf(fmt "\n", ##args)
#else
#define debug(fmt, args...)
#endif

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();


RCInput_Aero::RCInput_Aero()
{
	
}

void RCInput_Aero::init()
{
	const uint8_t tx_buffer[2]={CLOCK_REG, 0x00};
	uint8_t rx_buffer[2];
	
	_spi = hal.spi->get_device(DEVICE_NAME);
	
	if (!_spi) {
        AP_HAL::panic("Could not initialize Aero RCInput");
    }
	
    if (!_spi->transfer_fullduplex(tx_buffer, rx_buffer, 2)) {
    	AP_HAL::panic("Could not get FPGA frequency");
    }
    
    _frequency=(uint8_t)rx_buffer[1];
    
    if (!_frequency) {
    	AP_HAL::panic("Aero RCInput : Panic no valid FPGA frequency read for device name");
    }
}

void RCInput_Aero::_timer_tick(void)
{
	uint8_t i;
	const uint8_t buffer_size=CHANNEL_NUMBER*2+1;
	
	uint16_t channels[CHANNEL_NUMBER];
	uint8_t tx_buffer[buffer_size];
	uint8_t rx_buffer[buffer_size];
	
	bool good_value;
	int8_t retry=-1;
	
	do {
		good_value=true;
		for(i=1;i<buffer_size;i++) {
			tx_buffer[i]=0x00;
		}
		
		tx_buffer[0]=CHANNEL_REG;
		
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
    
    
    if (good_value==true) {
    	for(i=0;i<CHANNEL_NUMBER;i++) {
    		if (abs(channels[i]-_old_channels[i]) > MAX_DIFF_VALUE) {
    			printf("\nOC C NC : %d	%d	%d\n", _old_channels[i], channels[i], (uint16_t) (COMPLEMENTARY_FILTER*(float)channels[i] + (1.0-COMPLEMENTARY_FILTER)*(float)_old_channels[i]));
    			channels[i] = (uint16_t) (COMPLEMENTARY_FILTER*(float)_old_channels[i] + (1.0-COMPLEMENTARY_FILTER)*(float)channels[i]);
    		}
    		
    		_old_channels[i]=channels[i];
    	}
    	
    	_update_periods(channels, CHANNEL_NUMBER);
    } else {
    	_update_periods(_old_channels, CHANNEL_NUMBER);
    }
}

uint16_t RCInput_Aero::_hw_read(uint16_t address)
{
    struct PACKED {
        uint8_t prefix;
        be16_t addr;
    } tx;
    struct PACKED {
        uint8_t ignored[2];
        be16_t val;
    } rx;

    address = RADDRESS(address);

    // Write in the SPI buffer the address value
    tx.prefix = WRITE_PREFIX;
    tx.addr = htobe16(address);
    if (!_spi->transfer((uint8_t *)&tx, sizeof(tx), nullptr, 0)) {
        return 0;
    }

    return be16toh(rx.val);
}