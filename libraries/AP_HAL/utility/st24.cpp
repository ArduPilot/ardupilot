/*
  st24 decoder, based on PX4Firmware/src/rc/lib/rc/st24.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
 */
/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file st24.cpp
 *
 * RC protocol implementation for Yuneec ST24 transmitter.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <stdio.h>
#include <stdint.h>
#include "st24.h"

#define ST24_DATA_LEN_MAX	64
#define ST24_STX1		0x55
#define ST24_STX2		0x55

enum ST24_PACKET_TYPE {
	ST24_PACKET_TYPE_CHANNELDATA12 = 0,
	ST24_PACKET_TYPE_CHANNELDATA24,
	ST24_PACKET_TYPE_TRANSMITTERGPSDATA
};

#pragma pack(push, 1)
typedef struct {
	uint8_t	header1;			///< 0x55 for a valid packet
	uint8_t	header2;			///< 0x55 for a valid packet
	uint8_t	length;				///< length includes type, data, and crc = sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
	uint8_t	type;				///< from enum ST24_PACKET_TYPE
	uint8_t	st24_data[ST24_DATA_LEN_MAX];
	uint8_t	crc8;				///< crc8 checksum, calculated by st24_common_crc8 and including fields length, type and st24_data
} ReceiverFcPacket;

/**
 * RC Channel data (12 channels).
 *
 * This is incoming from the ST24
 */
typedef struct {
	uint16_t t;			///< packet counter or clock
	uint8_t	rssi;			///< signal strength
	uint8_t	packet_count;		///< Number of UART packets sent since reception of last RF frame (this tells something about age / rate)
	uint8_t	channel[18];		///< channel data, 12 channels (12 bit numbers)
} ChannelData12;

/**
 * RC Channel data (12 channels).
 *
 */
typedef struct {
	uint16_t t;			///< packet counter or clock
	uint8_t	rssi;			///< signal strength
	uint8_t	packet_count;		///< Number of UART packets sent since reception of last RF frame (this tells something about age / rate)
	uint8_t	channel[36];		///< channel data, 24 channels (12 bit numbers)
} ChannelData24;

/**
 * Telemetry packet
 *
 * This is outgoing to the ST24
 *
 * imuStatus:
 * 8 bit total
 * bits 0-2 for status
 * - value 0 is FAILED
 * - value 1 is INITIALIZING
 * - value 2 is RUNNING
 * - values 3 through 7 are reserved
 * bits 3-7 are status for sensors (0 or 1)
 * - mpu6050
 * - accelerometer
 * - primary gyro x
 * - primary gyro y
 * - primary gyro z
 *
 * pressCompassStatus
 * 8 bit total
 * bits 0-3 for compass status
 * - value 0 is FAILED
 * - value 1 is INITIALIZING
 * - value 2 is RUNNING
 * - value 3 - 15 are reserved
 * bits 4-7 for pressure status
 * - value 0 is FAILED
 * - value 1 is INITIALIZING
 * - value 2 is RUNNING
 * - value 3 - 15 are reserved
 *
 */
typedef struct {
	uint16_t t;			///< packet counter or clock
	int32_t	lat;			///< lattitude (degrees)	+/- 90 deg
	int32_t	lon;			///< longitude (degrees)	+/- 180 deg
	int32_t	alt;			///< 0.01m resolution, altitude (meters)
	int16_t	vx, vy, vz; 		///< velocity 0.01m res, +/-320.00 North-East- Down
	uint8_t	nsat;			///<number of satellites
	uint8_t	voltage; 		///< 25.4V	voltage = 5 + 255*0.1 = 30.5V, min=5V
	uint8_t	current; 		///< 0.5A resolution
	int16_t	roll, pitch, yaw;	///< 0.01 degree resolution
	uint8_t	motorStatus;		///< 1 bit per motor for status 1=good, 0= fail
	uint8_t	imuStatus;		///< inertial measurement unit status
	uint8_t	pressCompassStatus;	///< baro / compass status
} TelemetryData;

#pragma pack(pop)

enum ST24_DECODE_STATE {
	ST24_DECODE_STATE_UNSYNCED = 0,
	ST24_DECODE_STATE_GOT_STX1,
	ST24_DECODE_STATE_GOT_STX2,
	ST24_DECODE_STATE_GOT_LEN,
	ST24_DECODE_STATE_GOT_TYPE,
	ST24_DECODE_STATE_GOT_DATA
};

/* define range mapping here, -+100% -> 1000..2000 */
#define ST24_RANGE_MIN 0.0f
#define ST24_RANGE_MAX 4096.0f

#define ST24_TARGET_MIN 1000.0f
#define ST24_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define ST24_SCALE_FACTOR ((ST24_TARGET_MAX - ST24_TARGET_MIN) / (ST24_RANGE_MAX - ST24_RANGE_MIN))
#define ST24_SCALE_OFFSET (int)(ST24_TARGET_MIN - (ST24_SCALE_FACTOR * ST24_RANGE_MIN + 0.5f))

static enum ST24_DECODE_STATE _decode_state = ST24_DECODE_STATE_UNSYNCED;
static uint8_t _rxlen;

static ReceiverFcPacket _rxpacket;

static uint8_t st24_common_crc8(uint8_t *ptr, uint8_t len)
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


int st24_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count, uint16_t *channels,
		uint16_t max_chan_count)
{

	int ret = 1;

	switch (_decode_state) {
	case ST24_DECODE_STATE_UNSYNCED:
		if (byte == ST24_STX1) {
			_decode_state = ST24_DECODE_STATE_GOT_STX1;

		} else {
			ret = 3;
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
		if ((unsigned)byte <= sizeof(_rxpacket.length) + sizeof(_rxpacket.type) + sizeof(_rxpacket.st24_data)) {
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

		if (st24_common_crc8((uint8_t *) & (_rxpacket.length), _rxlen) == _rxpacket.crc8) {

			ret = 0;

			/* decode the actual packet */

			switch (_rxpacket.type) {

			case ST24_PACKET_TYPE_CHANNELDATA12: {
					ChannelData12 *d = (ChannelData12 *)_rxpacket.st24_data;

					*rssi = d->rssi;
					*rx_count = d->packet_count;

					/* this can lead to rounding of the strides */
					*channel_count = (max_chan_count < 12) ? max_chan_count : 12;

					unsigned stride_count = (*channel_count * 3) / 2;
					unsigned chan_index = 0;

					for (unsigned i = 0; i < stride_count; i += 3) {
						channels[chan_index] = ((uint16_t)d->channel[i] << 4);
						channels[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;

						channels[chan_index] = ((uint16_t)d->channel[i + 2]);
						channels[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;
					}
				}
				break;

			case ST24_PACKET_TYPE_CHANNELDATA24: {
					ChannelData24 *d = (ChannelData24 *)&_rxpacket.st24_data;

					*rssi = d->rssi;
					*rx_count = d->packet_count;

					/* this can lead to rounding of the strides */
					*channel_count = (max_chan_count < 24) ? max_chan_count : 24;

					unsigned stride_count = (*channel_count * 3) / 2;
					unsigned chan_index = 0;

					for (unsigned i = 0; i < stride_count; i += 3) {
						channels[chan_index] = ((uint16_t)d->channel[i] << 4);
						channels[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;

						channels[chan_index] = ((uint16_t)d->channel[i + 2]);
						channels[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;
					}
				}
				break;

			case ST24_PACKET_TYPE_TRANSMITTERGPSDATA: {

					// ReceiverFcPacket* d = (ReceiverFcPacket*)&_rxpacket.st24_data;
					/* we silently ignore this data for now, as it is unused */
					ret = 2;
				}
				break;

			default:
				ret = 2;
				break;
			}

		} else {
			/* decoding failed */
			ret = 4;
		}

		_decode_state = ST24_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}
