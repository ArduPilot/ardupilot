/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SRXL protocol decoder, tested against AR7700 SRXL port
  Andrew Tridgell, September 2016

  Co author: Roman Kirchner, September 2016
   - 2016.10.23: SRXL variant V1 sucessfully (Testbench and Pixhawk/MissionPlanner) tested with RX-9-DR M-LINK (SW v1.26)
 */

#include "AP_RCProtocol_SRXL.h"
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>

// #define SUMD_DEBUG
extern const AP_HAL::HAL& hal;

void AP_RCProtocol_SRXL::process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id)
{
    uint8_t b;
    if (ss_default.process_pulse(width_s0, width_s1, pulse_id, b)) {
        _process_byte(ss_default.get_byte_timestamp_us(), b);
    }
}


/**
 * Get RC channel information as microsecond pulsewidth representation from srxl version 1 and 2
 *
 * This function extracts RC channel information from srxl dataframe. The function expects the whole dataframe
 * in correct order in static array "buffer[SRXL_FRAMELEN_MAX]". After extracting all RC channel information, the data
 * is transferred to "values" array from parameter list. If the pixhawk does not support all channels from srxl datastream,
 * only supported number of channels will be refreshed.
 *
 * IMPORTANT SAFETY NOTICE: This function shall only be used after CRC has been successful.
 *
 * Structure of SRXL v1 dataframe --> 12 channels, 12 Bit per channel
 * Byte0: Header 0xA1
 * Byte1: Bits7-4:Empty Bits3-0:Channel1 MSB
 * Byte2: Bits7-0: Channel1 LSB
 * (....)
 * Byte23: Bits7-4:Empty Bits3-0:Channel12 MSB
 * Byte24: Bits7-0: Channel12 LSB
 * Byte25: CRC16 MSB
 * Byte26: CRC16 LSB
 *
 * Structure of SRXL v2 dataframe --> 16 channels, 12 Bit per channel
 * Byte0: Header 0xA2
 * Byte1: Bits7-4:Empty Bits3-0:Channel1 MSB
 * Byte2: Bits7-0: Channel1 LSB
 * (....)
 * Byte31: Bits7-4:Empty Bits3-0:Channel16 MSB
 * Byte32: Bits7-0: Channel16 LSB
 * Byte33: CRC16 MSB
 * Byte34: CRC16 LSB
 *
 * @param[in]   max_values - maximum number of values supported by the pixhawk
 * @param[out]  num_values - number of RC channels extracted from srxl frame
 * @param[out]  values - array of RC channels with refreshed information as pulsewidth in microseconds Range: 800us - 2200us
 * @param[out]  failsafe_state - true: RC-receiver is in failsafe state, false: RC-receiver is not in failsafe state
 * @retval 0 success
 */
int AP_RCProtocol_SRXL::srxl_channels_get_v1v2(uint16_t max_values, uint8_t *num_values, uint16_t *values, bool *failsafe_state)
{
    uint8_t loop;
    uint32_t channel_raw_value;

    *num_values = (uint8_t)((frame_len_full - 3U)/2U);
    *failsafe_state = 0U;    /* this protocol version does not support failsafe information */

    /* get data channel data from frame */
    for (loop=0U; loop < *num_values; loop++) {
        channel_raw_value = ((((uint32_t)buffer[loop*2U+1U])& 0x0000000FU) << 8U)  | ((uint32_t)(buffer[loop*2U+2U]));   /* get 12bit channel raw value from srxl datastream (mask out unused bits with 0x0000000F)  */
        channels[loop] = (uint16_t)(((channel_raw_value * (uint32_t)1400U) >> 12U) + (uint32_t)800U);                    /* convert raw value to servo/esc signal pulsewidth in us */
    }

    /* provide channel data to FMU */
    if ((uint16_t)*num_values > max_values) {
        *num_values = (uint8_t)max_values;
    }
    memcpy(values, channels, (*num_values)*2);

    return 0;   /* for srxl protocol version 1 and 2 it is not expected, that any error happen during decode process  */
}

/**
 * Get RC channel information as microsecond pulsewidth representation from srxl version 5
 *
 * This function extracts RC channel information from srxl dataframe. The function expects the whole dataframe
 * in correct order in static array "buffer[SRXL_FRAMELEN_MAX]". After extracting all RC channel information, the data
 * is transferred to "values" array from parameter list. If the pixhawk does not support all channels from srxl datastream,
 * only supported number of channels will be refreshed.
 *
 * IMPORTANT SAFETY NOTICE: This function shall only be used after CRC has been successful.
 *
 * Structure of SRXL v5 dataframe
 * Byte0: Header 0xA5
 * Byte1 - Byte16: Payload
 * Byte17: CRC16 MSB
 * Byte18: CRC16 LSB
 *
 * @param[in]  max_values - maximum number of values supported by the pixhawk
 * @param[out] num_values - number of RC channels extracted from srxl frame
 * @param[out] values - array of RC channels with refreshed information as pulsewidth in microseconds Range: 800us - 2200us
 * @param[out] failsafe_state - true: RC-receiver is in failsafe state, false: RC-receiver is not in failsafe state
 * @retval 0 success
 */
int AP_RCProtocol_SRXL::srxl_channels_get_v5(uint16_t max_values, uint8_t *num_values, uint16_t *values, bool *failsafe_state)
{
    // up to 7 channel values per packet. Each channel value is 16
    // bits, with 11 bits of data and 4 bits of channel number. The
    // top bit indicates a special X-Plus channel
    for (uint8_t i=0; i<7; i++) {
        uint16_t b = buffer[i*2+2] << 8 | buffer[i*2+3];
        uint16_t c = b >> 11; // channel number
        int32_t v = b & 0x7FF;
        if (b & 0x8000) {
            continue;
        }
        if (c == 12) {
            // special handling for channel 12
            // see http://www.deviationtx.com/forum/protocol-development/2088-18-channels-for-dsm2-dsmx?start=40
            //printf("c12: 0x%x %02x %02x\n", (unsigned)(b>>9), (unsigned)buffer[0], (unsigned)buffer[1]);
            v = (b & 0x1FF) << 2;
            c = 10 + ((b >> 9) & 0x7);
            if (buffer[1] & 1) {
                c += 4;
            }
        } else if (c > 12) {
            // invalid
            v = 0;
        }

        // if channel number if greater than 16 then it is a X-Plus
        // channel. We don't yet know how to decode those. There is some information here:
        // http://www.deviationtx.com/forum/protocol-development/2088-18-channels-for-dsm2-dsmx?start=40
        // but we really need some sample data to confirm
        if (c < SRXL_MAX_CHANNELS) {
            v = (((v - 0x400) * 500) / 876) + 1500;
            channels[c] = v;
            if (c >= max_channels) {
                max_channels = c+1;
            }
        }

        //printf("%u:%u ", (unsigned)c, (unsigned)v);
    }
    //printf("\n");

    *num_values = max_channels;
    if (*num_values > max_values) {
        *num_values = max_values;
    }
    memcpy(values, channels, (*num_values)*2);

    // check failsafe bit, this goes low when connection to the
    // transmitter is lost
    *failsafe_state = ((buffer[1] & 2) == 0);

    // success
    return 0;
}

void AP_RCProtocol_SRXL::_process_byte(uint32_t timestamp_us, uint8_t byte)
{
    /*----------------------------------------distinguish different srxl variants at the beginning of each frame---------------------------------------------- */
    /* Check if we have a new begin of a frame --> indicators: Time gap in datastream + SRXL header 0xA<VARIANT>*/
    if ((timestamp_us - last_data_us) >= SRXL_MIN_FRAMESPACE_US) {
        /* Now detect SRXL variant based on header */
        switch (byte) {
        case SRXL_HEADER_V1:
            frame_len_full = SRXL_FRAMELEN_V1;
            frame_header = SRXL_HEADER_V1;
            decode_state = STATE_NEW;
            break;
        case SRXL_HEADER_V2:
            frame_len_full = SRXL_FRAMELEN_V2;
            frame_header = SRXL_HEADER_V2;
            decode_state = STATE_NEW;
            break;
        case SRXL_HEADER_V5:
            frame_len_full = SRXL_FRAMELEN_V5;
            frame_header = SRXL_HEADER_V5;
            decode_state = STATE_NEW;
            break;
        default:
            frame_len_full = 0U;
            frame_header = SRXL_HEADER_NOT_IMPL;
            decode_state = STATE_IDLE;
            buflen = 0;
            return; /* protocol version not implemented --> no channel data --> unknown packet  */
        }
    }


    /*--------------------------------------------collect all data from stream and decode-------------------------------------------------------*/
    switch (decode_state) {
    case STATE_NEW:   /* buffer header byte and prepare for frame reception and decoding */
        buffer[0U]=byte;
        crc_fmu = crc_xmodem_update(0U,byte);
        buflen = 1U;
        decode_state_next = STATE_COLLECT;
        break;

    case STATE_COLLECT: /* receive all bytes. After reception decode frame and provide rc channel information to FMU   */
        if (buflen >= frame_len_full) {
            // a logic bug in the state machine, this shouldn't happen
            decode_state = STATE_IDLE;
            buflen = 0;
            frame_len_full = 0;
            frame_header = SRXL_HEADER_NOT_IMPL;
            return;
        }
        buffer[buflen] = byte;
        buflen++;
        /* CRC not over last 2 frame bytes as these bytes inhabitate the crc */
        if (buflen <= (frame_len_full-2)) {
            crc_fmu = crc_xmodem_update(crc_fmu,byte);
        }
        if (buflen == frame_len_full) {
            log_data(AP_RCProtocol::SRXL, timestamp_us, buffer, buflen);
            /* CRC check here */
            crc_receiver = ((uint16_t)buffer[buflen-2] << 8U) | ((uint16_t)buffer[buflen-1]);
             if (crc_receiver == crc_fmu) {
                 /* at this point buffer contains all frame data and crc is valid --> extract channel info according to SRXL variant */
                 const uint8_t max_values = MIN((unsigned)SRXL_MAX_CHANNELS,(unsigned)MAX_RCIN_CHANNELS);
                 uint16_t values[max_values];
                 uint8_t num_values;
                 bool failsafe_state;
                 switch (frame_header) {
                 case SRXL_HEADER_V1:
                     srxl_channels_get_v1v2(max_values, &num_values, values, &failsafe_state);
                     add_input(num_values, values, failsafe_state);
                     break;
                 case SRXL_HEADER_V2:
                     srxl_channels_get_v1v2(max_values, &num_values, values, &failsafe_state);
                     add_input(num_values, values, failsafe_state);
                     break;
                 case SRXL_HEADER_V5:
                     srxl_channels_get_v5(max_values, &num_values, values, &failsafe_state);
                     add_input(num_values, values, failsafe_state);
                     break;
                 default:
                     break;
                 }
             }
             decode_state_next = STATE_IDLE; /* frame data buffering and decoding finished --> statemachine not in use until new header drops is */
        } else {
            /* frame not completely received --> frame data buffering still ongoing  */
            decode_state_next = STATE_COLLECT;
        }
        break;

    default:
        break;
    } /* switch (decode_state) */

    decode_state = decode_state_next;
    last_data_us = timestamp_us;
}

/*
  process a byte provided by a uart
 */
void AP_RCProtocol_SRXL::process_byte(uint8_t byte, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), byte);
}
