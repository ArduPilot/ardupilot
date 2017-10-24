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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
 * Baud-Rate: 115.200 bits per second
 * Number of Data bits: 8
 * Number of Stop bits: 1
 * Parity: None
 * Half/Full Duplex: Half Duplex
 *
 * Volz Command and Response are all 6 bytes
 *
 * Command
 * byte	|	Communication Type
 * 1		Command Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 * byte	|	Communication Type
 * 1		Response Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 */

#define VOLZ_SCALE_VALUE 					(uint16_t)(VOLZ_EXTENDED_POSITION_MAX - VOLZ_EXTENDED_POSITION_MIN)	// Extended Position Data Format defines 100 as 0x0F80, which results in 1920 steps for +100 deg and 1920 steps for -100 degs meaning if you take movement a scaled between -1 ... 1 and multiply by 1920 you get the travel from center
#define VOLZ_SET_EXTENDED_POSITION_CMD 		0xDC
#define VOLZ_SET_EXTENDED_POSITION_RSP 		0x2C
#define VOLZ_DATA_FRAME_SIZE		 		4

#define VOLZ_EXTENDED_POSITION_MIN 			0x0080	// Extended Position Data Format defines -100 as 0x0080 decimal 128
#define VOLZ_EXTENDED_POSITION_CENTER 		0x0800	// Extended Position Data Format defines 0 as 0x0800 - decimal 2048
#define VOLZ_EXTENDED_POSITION_MAX 			0x0F80	// Extended Position Data Format defines +100 as 0x0F80 decimal 3968 -> full range decimal 3840

class AP_Volz_Protocol{

public:
	static AP_Volz_Protocol create();

    constexpr AP_Volz_Protocol(AP_Volz_Protocol &&other) = default;

    /* Do not allow copies */
    AP_Volz_Protocol(const AP_Volz_Protocol &other) = delete;
    AP_Volz_Protocol &operator=(const AP_Volz_Protocol&) = delete;

    // Startup initialisation.
    void init(const AP_SerialManager& serial_manager);

    // Update servo output
    void update(void);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Volz_Protocol();
    AP_Int16         chan_bitmask;
    AP_Int16         min_pwm;
    AP_Int16         max_pwm;

    int16_t		_min_pwm;
    int16_t		_max_pwm;
    uint16_t		_pwm_range;
    int16_t		_chan_bitmask;
	AP_HAL::UARTDriver *_port;
	bool _initialized = false;
	void send_command(uint8_t* data);
};
