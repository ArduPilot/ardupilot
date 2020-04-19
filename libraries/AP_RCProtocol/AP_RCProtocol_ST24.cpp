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
#include "AP_RCProtocol_ST24.h"

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

        if (st24_crc8((uint8_t *) & (_rxpacket.length), _rxlen) == _rxpacket.crc8) {

            /* decode the actual packet */

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
