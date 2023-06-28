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

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LD19_ENABLED

#include "AP_Proximity_LD19.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// CRC calculation according to the datasheet

static const uint8_t CrcTable[256] ={
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

uint8_t AP_Proximity_LD19::CalCRC8(uint8_t *p, uint8_t len){
    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++){
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

// update the state of the sensor
void AP_Proximity_LD19::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_LD19_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_LD19::distance_max() const
{
    return 12.0f;
}

float AP_Proximity_LD19::distance_min() const
{
    return 0.20f;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_LD19::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t message_count 	= 0;
    int16_t nbytes = _uart->available();
    //int16_t uart_buf_bytes = nbytes;

    uint16_t bad_crc 		= 0;
    uint16_t good_crc 		= 0;
    uint16_t pkg_count 		= 0;
    uint16_t low_conf 		= 0;
 
    //hal.console->printf("\nLD19 uart bytes: %d\n",nbytes);

    while (nbytes-- > 0) {

        int16_t c = _uart->read();

        if (c==-1) {
            return false;
        }

        if (char(c) == PROXIMITY_LD19_HEADER && post_header == 0) {
            buffer_count = 0;
            post_header = 1;
	    pkg_count++;
 	    //hal.console->printf("LD19 packet header\n");
        }

        buffer[buffer_count++] = c;

        // we should always read 47 bytes
        if (buffer_count >= 47){
	    //hal.console->printf("LD19 %d bytes found\n",buffer_count);
            buffer_count = 0;
            post_header = 0;
	    uint8_t payload_len = 46;

            // check if message has right CRC
            if (CalCRC8(buffer,payload_len) == buffer[46]){
		good_crc++;

                //hal.console->printf("LD19 Data received, CRC OK.\n");
		//uint16_t test = UINT16_VALUE(buffer[3],  buffer[2]);
		//hal.console->printf("Hz: %f\n",(float)(test/360));

		uint16_t start_angle = UINT16_VALUE(buffer[5],  buffer[4]);

		//hal.console->printf("Start: %f\n",(float)(start_angle*0.01));

		uint16_t end_angle = UINT16_VALUE(buffer[43],  buffer[42]);

                //hal.console->printf("End: %f\n",(float)(end_angle*0.01));

		if(start_angle > 36000 || end_angle > 36000) {
			hal.console->printf("Insane start/stop_angle. start: %f stop: %f\n",(float)(start_angle*0.01),(float)(end_angle*0.01));
		}

		uint16_t final_distance = INT16_MAX;
		uint16_t final_angle	= 0;

		for(uint8_t i=0;i<=11;i++) {
			// FIXME. need to find a better way to deal with angle rollover
			if(end_angle < start_angle) {
				end_angle = 36000;
			}
			uint16_t step = (end_angle - start_angle)/(12-1);
			uint16_t angle = start_angle + step*i;
 			uint16_t distance = UINT16_VALUE(buffer[7+i*3], buffer[6+i*3]);
                        uint8_t confidence = buffer[8+i*3];
			if(confidence > 190) {
				if(distance < final_distance) {
					final_distance = distance;
					final_angle = angle*0.01;
				}	
			} else {
				low_conf++;
			}
		}

	 	// update the shortest value extracted from 12 value block		
		//hal.console->printf("ang: %d dist: %d\n", final_angle,final_distance);
	        update_sector_data(final_angle,final_distance);
		message_count++;	

            } else {

		bad_crc++;
	    }
        }
    }
    //hal.console->printf("ubuf: %d gcrc: %d bcrc: %d lco: %d pkg: %d msg: %d\n",uart_buf_bytes,good_crc,bad_crc,low_conf,pkg_count,message_count);
    return (message_count > 0);
}

// process reply
void AP_Proximity_LD19::update_sector_data(int16_t angle_deg, uint16_t distance_mm)
{
    // Get location on 3-D boundary based on angle to the object
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
    if ((distance_mm != 0xffff) && !ignore_reading(angle_deg, distance_mm * 0.001f, false)) {
        frontend.boundary.set_face_attributes(face, angle_deg, ((float) distance_mm) / 1000, state.instance);
        // update OA database
        database_push(angle_deg, ((float) distance_mm) / 1000);
    } else {
        frontend.boundary.reset_face(face, state.instance);
    }
    _last_distance_received_ms = AP_HAL::millis();
}

#endif // AP_PROXIMITY_LD19_ENABLED
