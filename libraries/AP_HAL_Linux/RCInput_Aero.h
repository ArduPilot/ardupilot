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

#pragma once

#define DEVICE_NAME "aeroio"
#define CHANNEL_NUMBER 8

#define CHANNEL_MIN_VALUE 970
#define CHANNEL_MAX_VALUE 2000
#define MAX_DIFF_VALUE 300
#define MAX_RETRY 10
#define CLOCK_REG 0x4a
#define CHANNEL_REG 0x4b
#define COMPLEMENTARY_FILTER 0.93

#define RADDRESS(x) ((x) & 0x7FFF)

// Variables to perform ongoing tests
#define READ_PREFIX 0x80
#define WRITE_PREFIX 0x40

#include <unistd.h>
#include "AP_HAL_Linux.h"

#include "RCInput.h"

namespace Linux {

class RCInput_Aero : public RCInput
{
public:
    RCInput_Aero();

    void init();
    void _timer_tick();

private:
	AP_HAL::OwnPtr<AP_HAL::SPIDevice> _spi;
	uint8_t _frequency;
	uint16_t _hw_read(uint16_t address);
	uint16_t _old_channels[CHANNEL_NUMBER];
};

}
