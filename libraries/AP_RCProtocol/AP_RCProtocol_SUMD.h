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

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SUMD_ENABLED

#include "AP_RCProtocol_Backend.h"
#include "SoftSerial.h"

#define SUMD_MAX_CHANNELS	32
#define SUMD_FRAME_MAXLEN   40
class AP_RCProtocol_SUMD : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_SUMD(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);

    typedef struct {
        uint8_t	status;							///< 0x01 valid and live SUMD data frame / 0x00 = SUMH / 0x81 = Failsafe
        uint8_t	length;							///< Channels
        uint8_t	sumd_data[(SUMD_MAX_CHANNELS+1) * 2];	///< ChannelData (High Byte/ Low Byte)
        uint8_t	crc16_high;						///< High Byte of 16 Bit CRC
    } ReceiverFcPacketHoTT;


    enum SUMD_DECODE_STATE {
        SUMD_DECODE_STATE_UNSYNCED = 0,
        SUMD_DECODE_STATE_GOT_HEADER,
        SUMD_DECODE_STATE_GOT_STATE,
        SUMD_DECODE_STATE_GOT_LEN,
        SUMD_DECODE_STATE_GOT_DATA,
        SUMD_DECODE_STATE_GOT_CRC,
    };

    enum SUMD_DECODE_STATE _decode_state = SUMD_DECODE_STATE_UNSYNCED;
    uint8_t _rxlen;
    ReceiverFcPacketHoTT _rxpacket;
    uint16_t 	_crc16;
    uint32_t last_packet_us;

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};

#endif  // AP_RCPROTOCOL_SUMD_ENABLED
