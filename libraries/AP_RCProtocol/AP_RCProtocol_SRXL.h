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

#define SRXL_MIN_FRAMESPACE_US 8000U    /* Minumum space between srxl frames in us (applies to all variants)  */
#define SRXL_MAX_CHANNELS 20U           /* Maximum number of channels from srxl datastream  */

/* Variant specific SRXL datastream characteristics */
/* Framelength in byte */
#define SRXL_FRAMELEN_V1    27U      /* Framelength with header in byte for:  Mpx SRXLv1 or XBUS Mode B */
#define SRXL_FRAMELEN_V2    35U      /* Framelength with header in byte for:  Mpx SRXLv2 */
#define SRXL_FRAMELEN_V5    18U      /* Framelength with header in byte for  Spk AR7700 etc. */
#define SRXL_FRAMELEN_MAX   35U      /* maximum possible framelengh  */

/* Headerbyte */
#define SRXL_HEADER_V1          0xA1U    /* Headerbyte for:  Mpx SRXLv1 or XBUS Mode B */
#define SRXL_HEADER_V2          0xA2U    /* Headerbyte for:  Mpx SRXLv2  */
#define SRXL_HEADER_V5          0xA5U    /* Headerbyte for:  Spk AR7700 etc. */
#define SRXL_HEADER_NOT_IMPL    0xFFU    /* Headerbyte for non impemented srxl header*/

class AP_RCProtocol_SRXL : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_SRXL(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    int srxl_channels_get_v1v2(uint16_t max_values, uint8_t *num_values, uint16_t *values, bool *failsafe_state);
    int srxl_channels_get_v5(uint16_t max_values, uint8_t *num_values, uint16_t *values, bool *failsafe_state);
    uint8_t buffer[SRXL_FRAMELEN_MAX];       /* buffer for raw srxl frame data in correct order --> buffer[0]=byte0  buffer[1]=byte1  */
    uint8_t buflen;                          /* length in number of bytes of received srxl dataframe in buffer  */
    uint32_t last_data_us;                   /* timespan since last received data in us   */
    uint16_t channels[SRXL_MAX_CHANNELS] = {0};    /* buffer for extracted RC channel data as pulsewidth in microseconds */
    uint16_t max_channels = 0;
    enum {
        STATE_IDLE,                          /* do nothing */
        STATE_NEW,                           /* get header of frame + prepare for frame reception + begin new crc cycle   */
        STATE_COLLECT                        /* collect RC channel data from frame + concurrently calc crc over payload data + extract channel information */
    };
    uint8_t frame_header = 0U;                   /* Frame header from SRXL datastream    */
    uint8_t frame_len_full = 0U;                 /* Length in number of bytes of full srxl datastream */
    uint8_t decode_state = STATE_IDLE;           /* Current state of SRXL frame decoding */
    uint8_t decode_state_next = STATE_IDLE;      /* State of frame decoding thatwill be applied when the next byte from dataframe drops in  */
    uint16_t crc_fmu = 0U;                       /* CRC calculated over payload from srxl datastream on this machine */
    uint16_t crc_receiver = 0U;                  /* CRC extracted from srxl datastream  */

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};
