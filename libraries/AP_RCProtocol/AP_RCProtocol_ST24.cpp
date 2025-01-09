/*
  SUMD decoder, based on PX4Firmware/src/rc/lib/rc/sumd.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
 */
/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file sumd.h
 *
 * RC protocol definition for Graupner HoTT transmitter (SUMD/SUMH Protocol)
 *
 * @author Marco Bauer <marco@wtns.de>
 */
#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_ST24_ENABLED

#include "AP_RCProtocol_ST24.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <math.h>
#include <stdio.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>







// #define SUMD_DEBUG
extern const AP_HAL::HAL& hal;


uint8_t AP_RCProtocol_ST24::st24_crc8(uint8_t *ptr, uint8_t len)
{
    uint8_t i, crc ;
    crc = 0;

    while (len--) {
        for (i = 0x80; i != 0; i >>= 1) {
            if ((crc & 0x80) != 0) {
                crc <<= 1;
                crc ^= 0x07;

            } else {
                crc <<= 1;
            }

            if ((*ptr & i) != 0) {
                crc ^= 0x07;
            }
        }

        ptr++;
    }

    return (crc);
}


void AP_RCProtocol_ST24::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(b);
    }
}

void AP_RCProtocol_ST24::_process_byte(uint8_t byte)
{
    switch (_decode_state) {
    case ST24_DECODE_STATE_UNSYNCED:
        if (byte == ST24_STX1) {
            _decode_state = ST24_DECODE_STATE_GOT_STX1;

        }

        break;

    case ST24_DECODE_STATE_GOT_STX1:
        if (byte == ST24_STX2) {
            _decode_state = ST24_DECODE_STATE_GOT_STX2;

        } else {
            _decode_state = ST24_DECODE_STATE_UNSYNCED;
        }

        break;

    case ST24_DECODE_STATE_GOT_STX2:

        /* ensure no data overflow failure or hack is possible */
        if (byte > 8 && (unsigned)byte <= sizeof(_rxpacket.length) + sizeof(_rxpacket.type) + sizeof(_rxpacket.st24_data)) {
            _rxpacket.length = byte;
            _rxlen = 0;
            _decode_state = ST24_DECODE_STATE_GOT_LEN;

        } else {
            _decode_state = ST24_DECODE_STATE_UNSYNCED;
        }

        break;

    case ST24_DECODE_STATE_GOT_LEN:
        _rxpacket.type = byte;
        _rxlen++;
        _decode_state = ST24_DECODE_STATE_GOT_TYPE;
        break;

    case ST24_DECODE_STATE_GOT_TYPE:
        _rxpacket.st24_data[_rxlen - 1] = byte;
        _rxlen++;

        if (_rxlen == (_rxpacket.length - 1)) {
            _decode_state = ST24_DECODE_STATE_GOT_DATA;
        }

        break;

    case ST24_DECODE_STATE_GOT_DATA:
        _rxpacket.crc8 = byte;
        _rxlen++;

        log_data(AP_RCProtocol::ST24, AP_HAL::micros(), (const uint8_t *)&_rxpacket, _rxlen+3);

        if (st24_crc8((uint8_t *) & (_rxpacket.length), _rxlen) == _rxpacket.crc8) {

            /* decode the actual packet */
            bool bfailsafe = false;
            
            switch (_rxpacket.type) {

            case ST24_PACKET_TYPE_CHANNELDATA12: {
                uint16_t values[12];
                uint8_t num_values;
                ChannelData12 *d = (ChannelData12 *)_rxpacket.st24_data;
                //TBD: add support for RSSI
                // *rssi = d->rssi;
                //*rx_count = d->packet_count;

                /* this can lead to rounding of the strides */
                num_values = (MAX_RCIN_CHANNELS < 12) ? MAX_RCIN_CHANNELS : 12;

                unsigned stride_count = (num_values * 3) / 2;
                unsigned chan_index = 0;

                for (unsigned i = 0; i < stride_count; i += 3) {
                    values[chan_index] = ((uint16_t)d->channel[i] << 4);
                    values[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
                    /* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
                    values[chan_index] = (uint16_t)(values[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
                    chan_index++;

                    values[chan_index] = ((uint16_t)d->channel[i + 2]);
                    values[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
                    /* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
                    values[chan_index] = (uint16_t)(values[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
                    chan_index++;
                }
                if(d->packet_count > ST24_MAX_DROPCOUNT){
                    bfailsafe = true;
                }
                add_input(num_values, values, bfailsafe, d->rssi);//AP_RCProtocol: Fix the issue of ST24 receiver not working
                
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
                //_TelemetryData
                if(_count < 3){//Reduce CPU usage.
                    _count++;
                }
                else{
                    UpdateSendtoFCinfo();
                    sendstatetoRC(_sendMsg,41);
                    _count = 0;
                }
#endif
            }
            break;

            case ST24_PACKET_TYPE_CHANNELDATA24: {
                uint16_t values[24];
                uint8_t num_values;
                ChannelData24 *d = (ChannelData24 *)&_rxpacket.st24_data;

                //*rssi = d->rssi;
                //*rx_count = d->packet_count;

                /* this can lead to rounding of the strides */
                num_values = (MAX_RCIN_CHANNELS < 24) ? MAX_RCIN_CHANNELS : 24;

                unsigned stride_count = (num_values * 3) / 2;
                unsigned chan_index = 0;

                for (unsigned i = 0; i < stride_count; i += 3) {
                    values[chan_index] = ((uint16_t)d->channel[i] << 4);
                    values[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
                    /* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
                    values[chan_index] = (uint16_t)(values[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
                    chan_index++;

                    values[chan_index] = ((uint16_t)d->channel[i + 2]);
                    values[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
                    /* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
                    values[chan_index] = (uint16_t)(values[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
                    chan_index++;
                }
                if(d->packet_count > ST24_MAX_DROPCOUNT){
                    bfailsafe = true;
                }
                add_input(num_values, values, bfailsafe, d->rssi);//AP_RCProtocol: Fix the issue of ST24 receiver not working

#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
                //_TelemetryData
                if(_count < 3){//Reduce CPU usage.
                    _count++;
                }
                else{
                    UpdateSendtoFCinfo();
                    sendstatetoRC(_sendMsg,41);
                    _count = 0;
                }
#endif
            }
            break;

            case ST24_PACKET_TYPE_TRANSMITTERGPSDATA: {

                // ReceiverFcPacket* d = (ReceiverFcPacket*)&_rxpacket.st24_data;
                /* we silently ignore this data for now, as it is unused */
            }
            break;

            default:
                break;
            }

        } else {
            /* decoding failed */
        }

        _decode_state = ST24_DECODE_STATE_UNSYNCED;
        break;
    }
}

void AP_RCProtocol_ST24::process_byte(uint8_t byte, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(byte);
}

void AP_RCProtocol_ST24::sendstatetoRC(uint8_t *msg, uint8_t lenth)
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    
    _uart = get_available_UART();
    
    if (_uart == NULL) {
        return;
    }
    _uart->write(msg, lenth);
    _uart->flush();
#endif
return;
}    

void AP_RCProtocol_ST24::UpdateSendtoFCinfo()
{ 
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    const Location &loc = AP::gps().location();  
    uint16_t voltage = (roundf(AP::battery().voltage(0) * 10.0f));

    if(voltage > 305){//RC only supports 0-30V 
       voltage = 305;
    } 
    
    if(voltage< 50) {
       voltage = 0;
    }
    else {
       voltage = voltage  - 50;//voltage = 5 + X ,Remote control increases from 5V~30
    }
   
 
    memset((uint8_t*)&_TelemetryData,0,32);
    Update_ahrs_info();
    
    _TelemetryData.lat = htole32(loc.lat);
    _TelemetryData.lon = htole32(loc.lng);
    
    //FCinfotemp.vx = 0;//No use
    //FCinfotemp.vy = 0;//No use
    //FCinfotemp.vz = 0;//No use
    _TelemetryData.nsat = AP::gps().num_sats();
    _TelemetryData.voltage = (uint8_t)voltage;
    //FCinfotemp.current = (roundf(current * 10.0f));
      
    _TelemetryData.motorStatus = 0xff;
    _TelemetryData.imuStatus = 0xff;
    _TelemetryData.pressCompassStatus = 0x11;

    memset(_sendMsg,0,41);
    _sendMsg[0] = 0x55;
    _sendMsg[1] = 0x55;
    _sendMsg[2] = 0x26;
    _sendMsg[3] = 0x02;
    //_sendMsg[4] = 0x00;//No use
    //_sendMsg[5] = 0x00;//No use
    memcpy(&_sendMsg[4], (uint8_t*)&_TelemetryData, 32);
    //_sendMsg[36] = 0x00;//Todo I don't know the purpose of this field, it may be status or alarm
    //_sendMsg[37] = 0x00;//Todo 
    //_sendMsg[38] = 0x00;//Todo
    //_sendMsg[39] = 0x00;//Todo
    _sendMsg[40]  =  st24_crc8(&_sendMsg[2],38);   
#endif
   return;
}
//todo:The receiver will not send any signal before binding, 
//so we cannot know which serial port the receiver is connected to. Currently, This function is currently unavailable.
//(If you first use an already bound receiver and then replace it with an unbound receiver, the binding function can take effect)
void AP_RCProtocol_ST24::start_bind(void)
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    AP_HAL::UARTDriver *tuart = get_available_UART();
    
    if (tuart == NULL) {
        return;
    }
    uint8_t bindMsg[11]= {0x55, 0x55, 0x08, 0x04, 0x00, 0x00, 0x42, 0x49, 0x4E, 0x44, 0xB0};
    tuart->write(bindMsg, 11);
    tuart->flush();
#endif
    return;
}

void AP_RCProtocol_ST24::Update_ahrs_info()
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)

    AP_AHRS &_ahrs = AP::ahrs();    
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    float tAlt = 0;
    _ahrs.get_relative_position_D_home(tAlt);
    //The remote control has a bug, exceeding the value may cause the remote control to crash
    _TelemetryData.alt = htole32(-constrain_int32(roundf(tAlt*100.00f),-20000,20000));
     _TelemetryData.roll = htole16(constrain_int16(roundf(ToDeg(wrap_PI(_ahrs.roll))*100.00f),-18000,18000));
    _TelemetryData.pitch = htole16(-constrain_int16(roundf(ToDeg(wrap_PI(_ahrs.pitch))*100.00f),-18000,18000));
     //The remote control will decrease by 90 degrees, which needs to be added here
    _TelemetryData.yaw = htole16(-constrain_int16(roundf(ToDeg(wrap_PI(_ahrs.yaw))*100.00f),-18000,18000) + 9000);

#endif
     return;
}

#endif  // AP_RCPROTOCOL_ST24_ENABLED
