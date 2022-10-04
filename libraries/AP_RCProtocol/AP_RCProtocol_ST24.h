/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

#define ST24_DATA_LEN_MAX	64
#define ST24_MAX_FRAMELEN   70
#define ST24_STX1		0x55
#define ST24_STX2		0x55

/* define range mapping here, -+100% -> 1000..2000 */
#define ST24_RANGE_MIN 0.0f
#define ST24_RANGE_MAX 4096.0f

#define ST24_TARGET_MIN 1000.0f
#define ST24_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define ST24_SCALE_FACTOR ((ST24_TARGET_MAX - ST24_TARGET_MIN) / (ST24_RANGE_MAX - ST24_RANGE_MIN))
#define ST24_SCALE_OFFSET (int)(ST24_TARGET_MIN - (ST24_SCALE_FACTOR * ST24_RANGE_MIN + 0.5f))

class AP_RCProtocol_ST24 : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_ST24(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    void process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    void _process_byte(uint8_t byte);
    static uint8_t st24_crc8(uint8_t *ptr, uint8_t len);
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

    enum ST24_DECODE_STATE _decode_state = ST24_DECODE_STATE_UNSYNCED;
    uint8_t _rxlen;

    ReceiverFcPacket _rxpacket;
};
