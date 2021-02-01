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
#include "AP_RCProtocol_SUMD.h"
#include <AP_Math/crc.h>

#define SUMD_HEADER_LENGTH	3
#define SUMD_HEADER_ID		0xA8
#define SUMD_ID_SUMH		0x00
#define SUMD_ID_SUMD		0x01
#define SUMD_ID_FAILSAFE	0x81

/* define range mapping here, -+100% -> 1000..2000 */
#define SUMD_RANGE_MIN 0.0f
#define SUMD_RANGE_MAX 4096.0f

#define SUMD_TARGET_MIN 1000.0f
#define SUMD_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SUMD_SCALE_FACTOR ((SUMD_TARGET_MAX - SUMD_TARGET_MIN) / (SUMD_RANGE_MAX - SUMD_RANGE_MIN))
#define SUMD_SCALE_OFFSET (int)(SUMD_TARGET_MIN - (SUMD_SCALE_FACTOR * SUMD_RANGE_MIN + 0.5f))

// #define SUMD_DEBUG
extern const AP_HAL::HAL& hal;

uint8_t AP_RCProtocol_SUMD::sumd_crc8(uint8_t crc, uint8_t value)
{
    crc += value;
    return crc;
}

void AP_RCProtocol_SUMD::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

void AP_RCProtocol_SUMD::_process_byte(uint32_t timestamp_us, uint8_t byte)
{
    if (timestamp_us - last_packet_us > 5000U) {
        _decode_state = SUMD_DECODE_STATE_UNSYNCED;
    }
    switch (_decode_state) {
    case SUMD_DECODE_STATE_UNSYNCED:
#ifdef SUMD_DEBUG
        hal.console->printf(" SUMD_DECODE_STATE_UNSYNCED \n") ;
#endif
        if (byte == SUMD_HEADER_ID) {
            _rxpacket.header = byte;
            _sumd = true;
            _rxlen = 0;
            _crc16 = 0x0000;
            _crc8 = 0x00;
            _crcOK = false;
            _crc16 = crc_xmodem_update(_crc16, byte);
            _crc8 = sumd_crc8(_crc8, byte);
            _decode_state = SUMD_DECODE_STATE_GOT_HEADER;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_HEADER: %x \n", byte) ;
#endif
            last_packet_us = timestamp_us;
        }
        break;

    case SUMD_DECODE_STATE_GOT_HEADER:
        if (byte == SUMD_ID_SUMD || byte == SUMD_ID_SUMH) {
            _rxpacket.status = byte;

            if (byte == SUMD_ID_SUMH) {
                _sumd = false;
            }

            if (_sumd) {
                _crc16 = crc_xmodem_update(_crc16, byte);

            } else {
                _crc8 = sumd_crc8(_crc8, byte);
            }

            _decode_state = SUMD_DECODE_STATE_GOT_STATE;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_STATE: %x \n", byte) ;
#endif

        } else {
            _decode_state = SUMD_DECODE_STATE_UNSYNCED;
        }

        break;

    case SUMD_DECODE_STATE_GOT_STATE:
        if (byte >= 2 && byte <= SUMD_MAX_CHANNELS) {
            _rxpacket.length = byte;

            if (_sumd) {
                _crc16 = crc_xmodem_update(_crc16, byte);

            } else {
                _crc8 = sumd_crc8(_crc8, byte);
            }

            _rxlen++;
            _decode_state = SUMD_DECODE_STATE_GOT_LEN;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_LEN: %x (%d) \n", byte, byte) ;
#endif

        } else {
            _decode_state = SUMD_DECODE_STATE_UNSYNCED;
        }

        break;

    case SUMD_DECODE_STATE_GOT_LEN:
        _rxpacket.sumd_data[_rxlen] = byte;

        if (_sumd) {
            _crc16 = crc_xmodem_update(_crc16, byte);

        } else {
            _crc8 = sumd_crc8(_crc8, byte);
        }

        _rxlen++;

        if (_rxlen <= ((_rxpacket.length * 2))) {
#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_DATA[%d]: %x\n", _rxlen - 2, byte) ;
#endif

        } else {
            _decode_state = SUMD_DECODE_STATE_GOT_DATA;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_DATA -- finish --\n") ;
#endif

        }

        break;

    case SUMD_DECODE_STATE_GOT_DATA:
        _rxpacket.crc16_high = byte;

#ifdef SUMD_DEBUG
        hal.console->printf(" SUMD_DECODE_STATE_GOT_CRC16[1]: %x   [%x]\n", byte, ((_crc16 >> 8) & 0xff)) ;
#endif

        if (_sumd) {
            _decode_state = SUMD_DECODE_STATE_GOT_CRC;

        } else {
            _decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_1;
        }

        break;

    case SUMD_DECODE_STATE_GOT_CRC16_BYTE_1:
        _rxpacket.crc16_low = byte;

#ifdef SUMD_DEBUG
        hal.console->printf(" SUMD_DECODE_STATE_GOT_CRC16[2]: %x   [%x]\n", byte, (_crc16 & 0xff)) ;
#endif

        _decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_2;

        break;

    case SUMD_DECODE_STATE_GOT_CRC16_BYTE_2:
        _rxpacket.telemetry = byte;

#ifdef SUMD_DEBUG
        hal.console->printf(" SUMD_DECODE_STATE_GOT_SUMH_TELEMETRY: %x\n", byte) ;
#endif

        _decode_state = SUMD_DECODE_STATE_GOT_CRC;

        break;

    case SUMD_DECODE_STATE_GOT_CRC:
        if (_sumd) {
            _rxpacket.crc16_low = byte;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_CRC[2]: %x   [%x]\n\n", byte, (_crc16 & 0xff)) ;
#endif

            if (_crc16 == (uint16_t)(_rxpacket.crc16_high << 8) + _rxpacket.crc16_low) {
                _crcOK = true;
            }

        } else {
            _rxpacket.crc8 = byte;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_CRC8_SUMH: %x   [%x]\n\n", byte, _crc8) ;
#endif

            if (_crc8 == _rxpacket.crc8) {
                _crcOK = true;
            }
        }

        if (_crcOK) {
#ifdef SUMD_DEBUG
            hal.console->printf(" CRC - OK \n") ;
#endif
            if (_sumd) {
#ifdef SUMD_DEBUG
                hal.console->printf(" Got valid SUMD Packet\n") ;
#endif
            } else {
#ifdef SUMD_DEBUG
                hal.console->printf(" Got valid SUMH Packet\n") ;
#endif

            }

#ifdef SUMD_DEBUG
            hal.console->printf(" RXLEN: %d  [Chans: %d] \n\n", _rxlen - 1, (_rxlen - 1) / 2) ;
#endif


            unsigned i;
            uint8_t num_values;
            uint16_t values[SUMD_MAX_CHANNELS];

            /* received Channels */
            if ((uint16_t)_rxpacket.length > SUMD_MAX_CHANNELS) {
                _rxpacket.length = (uint8_t) SUMD_MAX_CHANNELS;
            }

            num_values = (uint16_t)_rxpacket.length;

            /* decode the actual packet */
            /* reorder first 4 channels */

            /* ch1 = roll -> sumd = ch2 */
            values[0] = (uint16_t)((_rxpacket.sumd_data[1 * 2 + 1] << 8) | _rxpacket.sumd_data[1 * 2 + 2]) >> 3;
            /* ch2 = pitch -> sumd = ch2 */
            values[1] = (uint16_t)((_rxpacket.sumd_data[2 * 2 + 1] << 8) | _rxpacket.sumd_data[2 * 2 + 2]) >> 3;
            /* ch3 = throttle -> sumd = ch2 */
            values[2] = (uint16_t)((_rxpacket.sumd_data[0 * 2 + 1] << 8) | _rxpacket.sumd_data[0 * 2 + 2]) >> 3;
            /* ch4 = yaw -> sumd = ch2 */
            values[3] = (uint16_t)((_rxpacket.sumd_data[3 * 2 + 1] << 8) | _rxpacket.sumd_data[3 * 2 + 2]) >> 3;

            /* we start at channel 5(index 4) */
            unsigned chan_index = 4;

            for (i = 4; i < _rxpacket.length; i++) {
#ifdef SUMD_DEBUG
                hal.console->printf("ch[%u] : %x %x [ %x    %d ]\n", i + 1, _rxpacket.sumd_data[i * 2 + 1], _rxpacket.sumd_data[i * 2 + 2],
                                    ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3,
                                    ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3);
#endif

                values[chan_index] = (uint16_t)((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3;
                /* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
                //channels[chan_index] = (uint16_t)(channels[chan_index] * SUMD_SCALE_FACTOR + .5f) + SUMD_SCALE_OFFSET;

                chan_index++;
            }
            if (_rxpacket.status == 0x01) {
                add_input(num_values, values, false);
            } else if (_rxpacket.status == 0x81) {
                add_input(num_values, values, true);
            }
        } else {
#ifdef SUMD_DEBUG
            hal.console->printf(" CRC - fail 0x%X 0x%X\n", _crc16, (uint16_t)(_rxpacket.crc16_high << 8) + _rxpacket.crc16_low);
#endif
        }

        _decode_state = SUMD_DECODE_STATE_UNSYNCED;
        break;
    }
}

void AP_RCProtocol_SUMD::process_byte(uint8_t byte, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), byte);
}
