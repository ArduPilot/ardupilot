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

#include <stdio.h>
#include <stdint.h>
#include <AP_Math/crc.h>
#include "sumd.h"

#define SUMD_MAX_CHANNELS	32
#define SUMD_HEADER_LENGTH	3
#define SUMD_HEADER_ID		0xA8
#define SUMD_ID_SUMH		0x00
#define SUMD_ID_SUMD		0x01
#define SUMD_ID_FAILSAFE	0x81


#pragma pack(push, 1)
typedef struct {
	uint8_t	header;							///< 0xA8 for a valid packet
	uint8_t	status;							///< 0x01 valid and live SUMD data frame / 0x00 = SUMH / 0x81 = Failsafe
	uint8_t	length;							///< Channels
	uint8_t	sumd_data[SUMD_MAX_CHANNELS * 2];	///< ChannelData (High Byte/ Low Byte)
	uint8_t	crc16_high;						///< High Byte of 16 Bit CRC
	uint8_t	crc16_low;						///< Low Byte of 16 Bit CRC
	uint8_t	telemetry;						///< Telemetry request
	uint8_t	crc8;							///< SUMH CRC8
} ReceiverFcPacketHoTT;
#pragma pack(pop)


enum SUMD_DECODE_STATE {
	SUMD_DECODE_STATE_UNSYNCED = 0,
	SUMD_DECODE_STATE_GOT_HEADER,
	SUMD_DECODE_STATE_GOT_STATE,
	SUMD_DECODE_STATE_GOT_LEN,
	SUMD_DECODE_STATE_GOT_DATA,
	SUMD_DECODE_STATE_GOT_CRC,
	SUMD_DECODE_STATE_GOT_CRC16_BYTE_1,
	SUMD_DECODE_STATE_GOT_CRC16_BYTE_2
};

/*
const char *decode_states[] = {"UNSYNCED",
			       "GOT_HEADER",
			       "GOT_STATE",
			       "GOT_LEN",
			       "GOT_DATA",
			       "GOT_CRC",
			       "GOT_CRC16_BYTE_1",
			       "GOT_CRC16_BYTE_2"
			      };
*/

static uint8_t 	_crc8 	= 0x00;
static uint16_t 	_crc16 = 0x0000;
static bool 		_sumd	= true;
static bool		_crcOK	= false;
static bool		_debug	= false;


/* define range mapping here, -+100% -> 1000..2000 */
#define SUMD_RANGE_MIN 0.0f
#define SUMD_RANGE_MAX 4096.0f

#define SUMD_TARGET_MIN 1000.0f
#define SUMD_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SUMD_SCALE_FACTOR ((SUMD_TARGET_MAX - SUMD_TARGET_MIN) / (SUMD_RANGE_MAX - SUMD_RANGE_MIN))
#define SUMD_SCALE_OFFSET (int)(SUMD_TARGET_MIN - (SUMD_SCALE_FACTOR * SUMD_RANGE_MIN + 0.5f))

static enum SUMD_DECODE_STATE _decode_state = SUMD_DECODE_STATE_UNSYNCED;
static uint8_t _rxlen;

static ReceiverFcPacketHoTT _rxpacket;


static uint8_t sumd_crc8(uint8_t crc, uint8_t value)
{
	crc += value;
	return crc;
}

int sumd_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count, uint16_t *channels,
		uint16_t max_chan_count)
{

	int ret = 1;

	switch (_decode_state) {
	case SUMD_DECODE_STATE_UNSYNCED:
		if (_debug) {
			printf(" SUMD_DECODE_STATE_UNSYNCED \n") ;
		}

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

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_HEADER: %x \n", byte) ;
			}

		} else {
			ret = 3;
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

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_STATE: %x \n", byte) ;
			}

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

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_LEN: %x (%d) \n", byte, byte) ;
			}

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
			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_DATA[%d]: %x\n", _rxlen - 2, byte) ;
			}

		} else {
			_decode_state = SUMD_DECODE_STATE_GOT_DATA;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_DATA -- finish --\n") ;
			}

		}

		break;

	case SUMD_DECODE_STATE_GOT_DATA:
		_rxpacket.crc16_high = byte;

		if (_debug) {
			printf(" SUMD_DECODE_STATE_GOT_CRC16[1]: %x   [%x]\n", byte, ((_crc16 >> 8) & 0xff)) ;
		}

		if (_sumd) {
			_decode_state = SUMD_DECODE_STATE_GOT_CRC;

		} else {
			_decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_1;
		}

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_1:
		_rxpacket.crc16_low = byte;

		if (_debug) {
			printf(" SUMD_DECODE_STATE_GOT_CRC16[2]: %x   [%x]\n", byte, (_crc16 & 0xff)) ;
		}

		_decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_2;

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_2:
		_rxpacket.telemetry = byte;

		if (_debug) {
			printf(" SUMD_DECODE_STATE_GOT_SUMH_TELEMETRY: %x\n", byte) ;
		}

		_decode_state = SUMD_DECODE_STATE_GOT_CRC;

		break;

	case SUMD_DECODE_STATE_GOT_CRC:
		if (_sumd) {
			_rxpacket.crc16_low = byte;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_CRC[2]: %x   [%x]\n\n", byte, (_crc16 & 0xff)) ;
			}

			if (_crc16 == (uint16_t)(_rxpacket.crc16_high << 8) + _rxpacket.crc16_low) {
				_crcOK = true;
			}

		} else {
			_rxpacket.crc8 = byte;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_CRC8_SUMH: %x   [%x]\n\n", byte, _crc8) ;
			}

			if (_crc8 == _rxpacket.crc8) {
				_crcOK = true;
			}
		}

		if (_crcOK) {
			if (_debug) {
				printf(" CRC - OK \n") ;
			}

			if (_sumd) {
				if (_debug) {
					printf(" Got valid SUMD Packet\n") ;
				}

			} else {
				if (_debug) {
					printf(" Got valid SUMH Packet\n") ;
				}

			}

			if (_debug) {
				printf(" RXLEN: %d  [Chans: %d] \n\n", _rxlen - 1, (_rxlen - 1) / 2) ;
			}

			ret = 0;
			unsigned i;
			uint8_t _cnt = *rx_count + 1;
			*rx_count = _cnt;

			*rssi = 100;

			/* received Channels */
			if ((uint16_t)_rxpacket.length > max_chan_count) {
				_rxpacket.length = (uint8_t) max_chan_count;
			}

			*channel_count = (uint16_t)_rxpacket.length;

			/* decode the actual packet */
			/* reorder first 4 channels */

			/* ch1 = roll -> sumd = ch2 */
			channels[0] = (uint16_t)((_rxpacket.sumd_data[1 * 2 + 1] << 8) | _rxpacket.sumd_data[1 * 2 + 2]) >> 3;
			/* ch2 = pitch -> sumd = ch2 */
			channels[1] = (uint16_t)((_rxpacket.sumd_data[2 * 2 + 1] << 8) | _rxpacket.sumd_data[2 * 2 + 2]) >> 3;
			/* ch3 = throttle -> sumd = ch2 */
			channels[2] = (uint16_t)((_rxpacket.sumd_data[0 * 2 + 1] << 8) | _rxpacket.sumd_data[0 * 2 + 2]) >> 3;
			/* ch4 = yaw -> sumd = ch2 */
			channels[3] = (uint16_t)((_rxpacket.sumd_data[3 * 2 + 1] << 8) | _rxpacket.sumd_data[3 * 2 + 2]) >> 3;

			/* we start at channel 5(index 4) */
			unsigned chan_index = 4;

			for (i = 4; i < _rxpacket.length; i++) {
				if (_debug) {
					printf("ch[%u] : %x %x [ %x    %d ]\n", i + 1, _rxpacket.sumd_data[i * 2 + 1], _rxpacket.sumd_data[i * 2 + 2],
					       ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3,
					       ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3);
				}

				channels[chan_index] = (uint16_t)((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3;
				/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
				//channels[chan_index] = (uint16_t)(channels[chan_index] * SUMD_SCALE_FACTOR + .5f) + SUMD_SCALE_OFFSET;

				chan_index++;
			}

		} else {
			/* decoding failed */
			ret = 4;

			if (_debug) {
				printf(" CRC - fail \n") ;
			}

		}

		_decode_state = SUMD_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}
