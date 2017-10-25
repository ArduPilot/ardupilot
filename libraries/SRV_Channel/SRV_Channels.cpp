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
  SRV_Channel.cpp - object to separate input and output channel
  ranges, trim and reversal
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "SRV_Channel.h"

extern const AP_HAL::HAL& hal;

SRV_Channel *SRV_Channels::channels;
SRV_Channels *SRV_Channels::instance;
bool SRV_Channels::disabled_passthrough;
bool SRV_Channels::initialised;
Bitmask SRV_Channels::function_mask{SRV_Channel::k_nr_aux_servo_functions};
SRV_Channels::srv_function SRV_Channels::functions[SRV_Channel::k_nr_aux_servo_functions];

const AP_Param::GroupInfo SRV_Channels::var_info[] = {
    // @Group: 1_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, SRV_Channels, SRV_Channel),

    // @Group: 2_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, SRV_Channels, SRV_Channel),

    // @Group: 3_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, SRV_Channels, SRV_Channel),

    // @Group: 4_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, SRV_Channels, SRV_Channel),

    // @Group: 5_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, SRV_Channels, SRV_Channel),

    // @Group: 6_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, SRV_Channels, SRV_Channel),

    // @Group: 7_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, SRV_Channels, SRV_Channel),

    // @Group: 8_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, SRV_Channels, SRV_Channel),

    // @Group: 9_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, SRV_Channels, SRV_Channel),

    // @Group: 10_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_",  10, SRV_Channels, SRV_Channel),

    // @Group: 11_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_",  11, SRV_Channels, SRV_Channel),

    // @Group: 12_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_",  12, SRV_Channels, SRV_Channel),

    // @Group: 13_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_",  13, SRV_Channels, SRV_Channel),

    // @Group: 14_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_",  14, SRV_Channels, SRV_Channel),

    // @Group: 15_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_",  15, SRV_Channels, SRV_Channel),

    // @Group: 16_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_",  16, SRV_Channels, SRV_Channel),

    // @Param: _AUTO_TRIM
    // @DisplayName: Automatic servo trim
    // @Description: This enables automatic servo trim in flight. Servos will be trimed in stabilized flight modes when the aircraft is close to level. Changes to servo trim will be saved every 10 seconds and will persist between flights.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FRAME("_AUTO_TRIM",  17, SRV_Channels, auto_trim, 0, AP_PARAM_FRAME_PLANE),

    // @Param: _RATE
    // @DisplayName: Servo default output rate
    // @Description: This sets the default output rate in Hz for all outputs.
    // @Range: 25 400
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("_RATE",  18, SRV_Channels, default_rate, 50),

    // @Param: CHAN_BITMASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of volz servo protocol to specific channels
	// @Values: 0,1....
    // @User: Standard
    AP_GROUPINFO("_VOLZ_MASK", 19, SRV_Channels, volz_chan_bitmask,  0),

    AP_GROUPEND
};

/*
  constructor
 */
SRV_Channels::SRV_Channels(void)
{
    instance = this;
    channels = obj_channels;

    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    // setup ch_num on channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].ch_num = i;
    }
}

/*
  save adjusted trims
 */
void SRV_Channels::save_trim(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (trimmed_mask & (1U<<i)) {
            channels[i].servo_trim.set_and_save(channels[i].servo_trim.get());
        }
    }
    trimmed_mask = 0;
}

void SRV_Channels::output_trim_all(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].set_output_pwm(channels[i].servo_trim);
    }
}

void SRV_Channels::setup_failsafe_trim_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        hal.rcout->set_failsafe_pwm(1U<<channels[i].ch_num, channels[i].servo_trim);
    }
}

// get pwm output for all channels
bool SRV_Channels::get_all_outputs_pwm(SRV_Channel::ServoChannelData values[], uint8_t chan_num){
	if(chan_num <= 0 || chan_num > NUM_SERVO_CHANNELS)
		return false;

	for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
		values[i].pwm = channels[i].output_pwm;
		values[i].min_pwm = channels[i].servo_min;
		values[i].max_pwm = channels[i].servo_max;
	}

	return true;
}

/*
  run calc_pwm for all channels
 */
void SRV_Channels::calc_pwm(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].calc_pwm(functions[channels[i].function].output_scaled);
    }
}

// set output value for a specific function channel as a pwm value
void SRV_Channels::set_output_pwm_chan(uint8_t chan, uint16_t value)
{
    if (chan < NUM_SERVO_CHANNELS) {
        channels[chan].set_output_pwm(value);
    }
}

// init the serial port for volz servos
void SRV_Channels::init_serial(const AP_SerialManager& serial_manager){
	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
}

// calculate and send set extended position command to volz servos over serial port
void SRV_Channels::update_volz(){
	if(_port == nullptr)
		return;

	SRV_Channel::ServoChannelData servo_data[NUM_SERVO_CHANNELS];

	if(!SRV_Channels::get_all_outputs_pwm(servo_data,NUM_SERVO_CHANNELS)){
//		gcs().send_text(MAV_SEVERITY_CRITICAL, "Volz Protocol failed to get pwm values, don't fly.");
		return;
	}

	uint8_t i;


	// loop for all 16 channels
	for (i=0;i<NUM_SERVO_CHANNELS;i++){
		// check if current channel is needed for Volz protocol
		if((volz_chan_bitmask >> i) & 0x0001){
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

// calculate CRC for volz serial protocol and send the data.
void SRV_Channels::send_command(uint8_t data[VOLZ_DATA_FRAME_SIZE]){
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

