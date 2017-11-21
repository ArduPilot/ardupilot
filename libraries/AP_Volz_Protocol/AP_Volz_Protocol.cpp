/*
 * AP_VOLZ_PROTOCOL.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: guy
 */
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_Volz_Protocol.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Volz_Protocol::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of volz servo protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_Volz_Protocol, bitmask, 0),

    AP_GROUPEND
};

// constructor
AP_Volz_Protocol::AP_Volz_Protocol(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Volz_Protocol::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
    update_volz_bitmask(bitmask);
}

void AP_Volz_Protocol::update()
{
    if (!initialised) {
        initialised = true;
        init();
    }
    
    if (port == nullptr) {
        return;
    }

    if (last_used_bitmask != uint32_t(bitmask.get())) {
        update_volz_bitmask(bitmask);
    }

    uint32_t now = AP_HAL::micros();
    if (now - last_volz_update_time < volz_time_frame_micros ||
        port->txspace() < VOLZ_DATA_FRAME_SIZE) {
        return;
    }

    last_volz_update_time = now;

    uint8_t i;
    uint16_t value;

    // loop for all 16 channels
    for (i=0; i<NUM_SERVO_CHANNELS; i++) {
        // check if current channel is needed for Volz protocol
        if (last_used_bitmask & (1U<<i)) {

            SRV_Channel *ch = SRV_Channels::srv_channel(i);
            if (ch == nullptr) {
                continue;
            }
            
            // check if current channel PWM is within range
            if (ch->get_output_pwm() < ch->get_output_min()) {
                value = 0;
            } else {
                value = ch->get_output_pwm() - ch->get_output_min();
            }

            // scale the PWM value to Volz value
            value = value + VOLZ_EXTENDED_POSITION_MIN;
            value = value * VOLZ_SCALE_VALUE / (ch->get_output_max() - ch->get_output_min());

            // make sure value stays in range
            if (value > VOLZ_EXTENDED_POSITION_MAX) {
                value = VOLZ_EXTENDED_POSITION_MAX;
            }

            // prepare Volz protocol data.
            uint8_t data[VOLZ_DATA_FRAME_SIZE];

            data[0] = VOLZ_SET_EXTENDED_POSITION_CMD;
            data[1] = i + 1;		// send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
            data[2] = HIGHBYTE(value);
            data[3] = LOWBYTE(value);

            send_command(data);
        }
    }
}

// calculate CRC for volz serial protocol and send the data.
void AP_Volz_Protocol::send_command(uint8_t data[VOLZ_DATA_FRAME_SIZE])
{
    uint8_t i,j;
    uint16_t crc = 0xFFFF;

    // calculate Volz CRC value according to protocol definition
    for(i=0; i<4; i++) {
        // take input data into message that will be transmitted.
        crc = ((data[i] << 8) ^ crc);

        for(j=0; j<8; j++) {

            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc = crc << 1;
            }
        }
    }

    // add CRC result to the message
    data[4] = HIGHBYTE(crc);
    data[5] = LOWBYTE(crc);
    port->write(data, VOLZ_DATA_FRAME_SIZE);
}

void AP_Volz_Protocol::update_volz_bitmask(uint32_t new_bitmask)
{
    uint8_t count = 0;
    last_used_bitmask = new_bitmask;

    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (new_bitmask & (1U<<i)) {
            count++;
        }
    }

    // have a safety margin of 20% to allow for not having full uart
    // utilisation. We really don't want to start filling the uart
    // buffer or we'll end up with servo lag
    const float safety = 1.3;

    // each channel take about 425.347us to transmit so total time will be ~ number of channels * 450us
    // rounded to 450 to make sure we don't go over the baud rate.
    uint32_t channels_micros = count * 450 * safety;

    // limit the minimum to 2500 will result a max refresh frequency of 400hz.
    if (channels_micros < 2500) {
        channels_micros = 2500;
    }

    volz_time_frame_micros = channels_micros;
}
