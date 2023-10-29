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

#if AP_RCPROTOCOL_SUMD_ENABLED

#include "AP_RCProtocol_SUMD.h"

#include <AP_Math/crc.h>

#define SUMD_HEADER_LENGTH	3
#define SUMD_HEADER_ID		0xA8
#define SUMD_ID_SUMH		0x00
#define SUMD_ID_SUMD		0x01
#define SUMD_ID_FAILSAFE	0x81

// #define SUMD_DEBUG
extern const AP_HAL::HAL& hal;

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
            _rxlen = 0;
            _crc16 = crc_xmodem_update(0, byte);
            _decode_state = SUMD_DECODE_STATE_GOT_HEADER;

#ifdef SUMD_DEBUG
            hal.console->printf(" SUMD_DECODE_STATE_GOT_HEADER: %x \n", byte) ;
#endif
            last_packet_us = timestamp_us;
        }
        break;

    case SUMD_DECODE_STATE_GOT_HEADER:
        if ((byte == SUMD_ID_SUMD) || (byte == SUMD_ID_FAILSAFE)) {
            _rxpacket.status = byte;
            _crc16 = crc_xmodem_update(_crc16, byte);
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
            _crc16 = crc_xmodem_update(_crc16, byte);
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
        _crc16 = crc_xmodem_update(_crc16, byte);
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
        _decode_state = SUMD_DECODE_STATE_GOT_CRC;
#ifdef SUMD_DEBUG
        hal.console->printf(" SUMD_DECODE_STATE_GOT_CRC16[1]: %x   [%x]\n", byte, ((_crc16 >> 8) & 0xff)) ;
#endif
        break;

    case SUMD_DECODE_STATE_GOT_CRC:
#ifdef SUMD_DEBUG
        hal.console->printf(" SUMD_DECODE_STATE_GOT_CRC[2]: %x   [%x]\n\n", byte, (_crc16 & 0xff)) ;
#endif

        log_data(AP_RCProtocol::SUMD, timestamp_us, _rxpacket.sumd_data, _rxlen);

        if (_crc16 == (uint16_t)(_rxpacket.crc16_high << 8) + byte) {
#ifdef SUMD_DEBUG
            hal.console->printf(" CRC - OK \n") ;
            hal.console->printf(" Got valid SUMD Packet\n") ;
            hal.console->printf(" RXLEN: %d  [Chans: %d] \n\n", _rxlen - 1, (_rxlen - 1) / 2) ;
#endif

            uint16_t values[SUMD_MAX_CHANNELS];

            /* received Channels */
            if ((uint16_t)_rxpacket.length > SUMD_MAX_CHANNELS) {
                _rxpacket.length = (uint8_t) SUMD_MAX_CHANNELS;
            }

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
            for (uint8_t i = 4; i < _rxpacket.length; i++) {
#ifdef SUMD_DEBUG
                hal.console->printf("ch[%u] : %x %x [ %x    %d ]\n", i + 1, _rxpacket.sumd_data[i * 2 + 1], _rxpacket.sumd_data[i * 2 + 2],
                                    ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3,
                                    ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3);
#endif

                values[i] = (uint16_t)((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3;
            }
            add_input(_rxpacket.length, values, (_rxpacket.status == SUMD_ID_FAILSAFE));
#ifdef SUMD_DEBUG
        } else {
            hal.console->printf(" CRC - fail 0x%X 0x%X\n", _crc16, (uint16_t)(_rxpacket.crc16_high << 8) + byte);
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

#endif  // AP_RCPROTOCOL_SUMD_ENABLED
