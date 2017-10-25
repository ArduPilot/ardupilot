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

#include <AP_HAL/AP_HAL.h>
#include "AP_Volz_Protocol.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Volz_Protocol::var_info[] = {

    // @Param: CHAN_BITMASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of volz servo protocol to specific channels
	// @Values: 0,1....
    // @User: Standard
    AP_GROUPINFO("BITMASK", 0, AP_Volz_Protocol, chan_bitmask,  0),


    AP_GROUPEND
};

// Constructor
AP_Volz_Protocol::AP_Volz_Protocol()
{
	AP_Param::setup_object_defaults(this, var_info);
}

AP_Volz_Protocol AP_Volz_Protocol::create(){
	return AP_Volz_Protocol{};
}

void AP_Volz_Protocol::init(const AP_SerialManager& serial_manager){
	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
}

void AP_Volz_Protocol::update(){
	if(_port == nullptr)
		return;

	SRV_Channel::ServoChannelData servo_data[NUM_SERVO_CHANNELS];

	if(!SRV_Channels::get_all_outputs_pwm(servo_data,NUM_SERVO_CHANNELS)){
		gcs().send_text(MAV_SEVERITY_CRITICAL, "Volz Protocol failed to get pwm values, don't fly.");
		return;
	}

	uint8_t i;


	// loop for all 16 channels
	for (i=0;i<NUM_SERVO_CHANNELS;i++){
		// check if current channel is needed for Volz protocol
		if((chan_bitmask >> i) & 0x0001){
			uint16_t value;
			// check if current channel PWM is within range
			if(servo_data[i].pwm - servo_data[i].min_pwm < 0)
				value = 0;
			else
				value = servo_data[i].pwm - servo_data[i].min_pwm;

			// scale the PWM value to Volz value
			value = value + VOLZ_EXTENDED_POSITION_MIN;
			value = value * VOLZ_SCALE_VALUE / (servo_data[i].max_pwm - servo_data[i].min_pwm);

			// make sure value stays in range
			if(value > VOLZ_EXTENDED_POSITION_MAX)
				value = VOLZ_EXTENDED_POSITION_MAX;

			// prepare Volz protocol data.
			uint8_t data[VOLZ_DATA_FRAME_SIZE];

			data[0] = VOLZ_SET_EXTENDED_POSITION_CMD;
			data[1] = i + 1;		// send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
			data[2] = HIGHBYTE(value);
			data[3] = LOWBYTE(value);

			send_command(data);
		}// end if
	}// end for

}

void AP_Volz_Protocol::send_command(uint8_t data[VOLZ_DATA_FRAME_SIZE]){
	uint8_t i,j;
	uint16_t crc = 0xFFFF;

	// calculate Volz CRC value according to protocol definition
	for(i=0;i<4;i++){

		// take input data into message that will be transmitted.
		crc = ((data[i] << 8) ^ crc);

		for(j=0;j<8;j++){

			if(crc & 0x8000)
				crc = (crc << 1) ^ 0x8005;
			else
				crc = crc << 1;
		}
	}

	// add CRC result to the message
	data[4] = HIGHBYTE(crc);
	data[5] = LOWBYTE(crc);
	_port->write(data, VOLZ_DATA_FRAME_SIZE);
}

