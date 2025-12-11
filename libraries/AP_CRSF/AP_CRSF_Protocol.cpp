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
 * Code by Andy Piper <github@andypiper.com>
 */

/*
 * AP_CRSF_Protocol.cpp - a stateless parser and encoder for the CRSF wire protocol
 */

#include "AP_CRSF_config.h"

#if AP_CRSF_ENABLED

#pragma GCC optimize("O2")

#include "AP_CRSF_Protocol.h"
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_FWVersion.h>
#include <string.h>

// Defines for CRSFv3 subset RC frame packing/unpacking
#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS        5
#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF

extern const AP_HAL::HAL& hal;

//#define CRSF_PROTOCOL_DEBUG
#if defined(CRSF_PROTOCOL_DEBUG)
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

// get printable name for frame type (for debug)
const char* AP_CRSF_Protocol::get_frame_type(uint8_t byte, uint8_t subtype)
{
    switch(byte) {
    case CRSF_FRAMETYPE_GPS:
        return "GPS";
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        return "BATTERY";
    case CRSF_FRAMETYPE_HEARTBEAT:
        return "HEARTBEAT";
    case CRSF_FRAMETYPE_VTX:
        return "VTX";
    case CRSF_FRAMETYPE_VTX_TELEM:
        return "VTX_TELEM";
    case CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        return "PING";
    case CRSF_FRAMETYPE_COMMAND:
        return "COMMAND";
    case CRSF_FRAMETYPE_ATTITUDE:
        return "ATTITUDE";
    case CRSF_FRAMETYPE_FLIGHT_MODE:
        return "FLIGHT_MODE";
    case CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
        return "DEVICE_INFO";
    case CRSF_FRAMETYPE_PARAMETER_READ:
        return "PARAM_READ";
    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        return "SETTINGS_ENTRY";
    case CRSF_FRAMETYPE_LINK_STATISTICS:
        return "LINK_STATS";
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        return "RC";
    case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
        return "RCv3";
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED_11BIT:
        return "RCv3_11BIT";
    case CRSF_FRAMETYPE_LINK_STATISTICS_RX:
        return "LINK_STATSv3_RX";
    case CRSF_FRAMETYPE_LINK_STATISTICS_TX:
        return "LINK_STATSv3_TX";
    case CRSF_FRAMETYPE_PARAMETER_WRITE:
        return "PARAM_WRITE";
    case CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY:
    case CRSF_FRAMETYPE_AP_CUSTOM_TELEM:
        switch (subtype) {
        case CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH:
            return "AP_CUSTOM_SINGLE";
        case CRSF_AP_CUSTOM_TELEM_STATUS_TEXT:
            return "AP_CUSTOM_TEXT";
        case CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH:
            return "AP_CUSTOM_MULTI";
        }
        return "AP_CUSTOM";
    }
    return "UNKNOWN";
}

// unpack channels from a CRSFv2 11-bit fixed-width frame payload
void AP_CRSF_Protocol::decode_11bit_channels(const uint8_t* payload, uint8_t nchannels, uint16_t *values)
{
    // CRSFv2 channels are packed 11 bits each, 16 channels in 22 bytes.
    // This is the same packing as SBUS.
    // Explicitly cast to uint32_t to avoid narrowing conversion warnings.
    const uint32_t raw_channels[16] = {
        (( (uint32_t)payload[0]    | (uint32_t)payload[1] << 8) & 0x07FF),
        ((( (uint32_t)payload[1] >> 3 | (uint32_t)payload[2] << 5) & 0x07FF)),
        ((( (uint32_t)payload[2] >> 6 | (uint32_t)payload[3] << 2 | (uint32_t)payload[4] << 10) & 0x07FF)),
        ((( (uint32_t)payload[4] >> 1 | (uint32_t)payload[5] << 7) & 0x07FF)),
        ((( (uint32_t)payload[5] >> 4 | (uint32_t)payload[6] << 4) & 0x07FF)),
        ((( (uint32_t)payload[6] >> 7 | (uint32_t)payload[7] << 1 | (uint32_t)payload[8] << 9) & 0x07FF)),
        ((( (uint32_t)payload[8] >> 2 | (uint32_t)payload[9] << 6) & 0x07FF)),
        ((( (uint32_t)payload[9] >> 5 | (uint32_t)payload[10] << 3) & 0x07FF)),
        (( (uint32_t)payload[11]   | (uint32_t)payload[12] << 8) & 0x07FF),
        ((( (uint32_t)payload[12] >> 3 | (uint32_t)payload[13] << 5) & 0x07FF)),
        ((( (uint32_t)payload[13] >> 6 | (uint32_t)payload[14] << 2 | (uint32_t)payload[15] << 10) & 0x07FF)),
        ((( (uint32_t)payload[15] >> 1 | (uint32_t)payload[16] << 7) & 0x07FF)),
        ((( (uint32_t)payload[16] >> 4 | (uint32_t)payload[17] << 4) & 0x07FF)),
        ((( (uint32_t)payload[17] >> 7 | (uint32_t)payload[18] << 1 | (uint32_t)payload[19] << 9) & 0x07FF)),
        ((( (uint32_t)payload[18] >> 2 | (uint32_t)payload[19] << 6) & 0x07FF)),
        ((( (uint32_t)payload[20] >> 5 | (uint32_t)payload[21] << 3) & 0x07FF))
    };

    const uint8_t num_to_decode = MIN(nchannels, (uint8_t)16);
    for (uint8_t i = 0; i < num_to_decode; i++) {
        // scale from CRSFv2's native 172-1811 range to 1000-2000us PWM
        values[i] = (uint16_t)(((raw_channels[i] - 992) * 5 / 8) + 1500);
    }
}

// unpack channels from a CRSFv3 variable bit length frame payload
void AP_CRSF_Protocol::decode_variable_bit_channels(const uint8_t* payload, uint8_t frame_length, uint8_t nchannels, uint16_t *values)
{
    const AP_RCProtocol_CRSF::SubsetChannelsFrame* channel_data = (const AP_RCProtocol_CRSF::SubsetChannelsFrame*)payload;
    // We currently only support 11-bit resolution
    const uint8_t channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
    const uint16_t channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
    const float channelScale = 0.5f;

    // calculate the number of channels packed
    const uint8_t numOfChannels = MIN(uint8_t(((frame_length - 2) * 8 - CRSF_SUBSET_RC_STARTING_CHANNEL_BITS) / channelBits), (uint8_t)CRSF_MAX_CHANNELS);

    // unpack the channel data
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 1;

    for (uint8_t n = 0; n < numOfChannels; n++) {
        while (bitsMerged < channelBits) {
            if (readByteIndex >= CRSF_FRAME_PAYLOAD_MAX) {
                return;
            }
            uint8_t readByte = payload[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        if (uint8_t(channel_data->starting_channel + n) >= nchannels) {
            return;
        }
        values[channel_data->starting_channel + n] =
            uint16_t(channelScale * float(uint16_t(readValue & channelMask)) + 988);
        readValue >>= channelBits;
        bitsMerged -= channelBits;
    }
}

// encode 16 channels of PWM values into a CRSFv3 variable bit length frame payload
uint8_t AP_CRSF_Protocol::encode_variable_bit_channels(uint8_t *payload, const uint16_t *values, uint8_t nchannels, uint8_t start_chan)
{
    // We will use 11-bit resolution which is standard for CRSFv3
    const uint8_t channelBits = CRSF_SUBSET_RC_RES_BITS_11B;

    AP_RCProtocol_CRSF::SubsetChannelsFrame* channel_data = (AP_RCProtocol_CRSF::SubsetChannelsFrame*)payload;
    memset(payload, 0, 23); // 1 byte header + 22 bytes for 16 channels

    channel_data->starting_channel = start_chan;
    channel_data->res_configuration = CRSF_SUBSET_RC_RES_CONF_11B;

    uint32_t writeValue = 0;
    uint8_t bitsMerged = 0;
    uint8_t writeByteIndex = 1;
    const uint8_t num_to_send = MIN(nchannels, (uint8_t)16);

    for (uint8_t n = 0; n < num_to_send; n++) {
        uint16_t channel_value = constrain_int16(lroundf((values[n] - 988) / 0.5f), 0, 2047);
        writeValue |= ((uint32_t)channel_value) << bitsMerged;
        bitsMerged += channelBits;
        while (bitsMerged >= 8) {
            if (writeByteIndex < 23) {
                payload[writeByteIndex++] = writeValue & 0xFF;
            }
            writeValue >>= 8;
            bitsMerged -= 8;
        }
    }
    if (bitsMerged > 0 && writeByteIndex < 23) {
        payload[writeByteIndex++] = writeValue & 0xFF;
    }

    return writeByteIndex;
}

// encode a ping frame
void AP_CRSF_Protocol::encode_ping_frame(Frame& frame, DeviceAddress destination, DeviceAddress origin)
{
    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = CRSF_FRAMETYPE_PARAM_DEVICE_PING;

    // Command payload buffer: dest(1), origin(1)
    uint8_t command_data[2];

    // Construct the inner command frame header
    ParameterPingFrame* cmd_header = (ParameterPingFrame*)command_data;
    cmd_header->destination = destination;
    cmd_header->origin = origin;

    // Calculate the inner CRC (poly 0xBA) over the 2-byte command data

    const uint8_t inner_payload_len = 2;
    uint8_t inner_crc = crc8_dvb_update_generic(0, command_data, inner_payload_len, 0xBA);

    // Copy the command data and the inner CRC into the final frame payload
    memcpy(frame.payload, command_data, inner_payload_len);
    frame.payload[inner_payload_len] = inner_crc;

    // Set the outer frame length.
    // It is the length of the payload (type + inner command frame + outer CRC)
    // inner command frame = command_data(2) + inner_crc(1) = 3 bytes
    // Total length = type(1) + inner command frame(3) + outer CRC(1) = 5 bytes
    frame.length = 5;
}

// send a baudrate proposal
void AP_CRSF_Protocol::encode_speed_proposal(uint32_t baudrate, Frame& frame, DeviceAddress destination, DeviceAddress origin)
{
    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = CRSF_FRAMETYPE_COMMAND;

    // Command payload buffer: dest(1), origin(1), cmd_id(1), sub_cmd(1), port_id(1), baud(4)
    uint8_t command_data[9];

    // Construct the inner command frame header
    CommandFrame* cmd_header = (CommandFrame*)command_data;
    cmd_header->destination = destination;
    cmd_header->origin = origin;
    cmd_header->command_id = CRSF_COMMAND_GENERAL;

    // Construct the sub-command payload
    uint8_t* sub_payload = cmd_header->payload;
    sub_payload[0] = CRSF_COMMAND_GENERAL_CRSF_SPEED_PROPOSAL;
    sub_payload[1] = 1; // port ID, 0 for UART
    uint32_t baud_be = htobe32(baudrate);
    memcpy(&sub_payload[2], &baud_be, sizeof(baud_be));

    // Copy the command data into the final frame payload
    const uint8_t inner_payload_len = 9;
    memcpy(frame.payload, command_data, inner_payload_len);

    // Calculate the inner CRC (poly 0xBA) over the 9-byte command data + command id
    frame.payload[inner_payload_len] = crc8_dvb_update_generic(0, (uint8_t*)(&frame.type), inner_payload_len + 1, 0xBA);

    // It is the length of the payload (type + inner command frame + outer CRC)
    // inner command frame = command data(9) + inner crc(1) = 10 bytes
    // total length = type(1) + inner command frame(10) + outer CRC(1) = 12 bytes
    frame.length = 12;
}

// request for device info
bool AP_CRSF_Protocol::process_device_info_frame(ParameterDeviceInfoFrame* info, VersionInfo* version, bool fakerx)
{
    const uint8_t origin = fakerx ? CRSF_ADDRESS_FLIGHT_CONTROLLER : CRSF_ADDRESS_CRSF_RECEIVER;

    if ((fakerx && info->destination != 0 && info->destination != CRSF_ADDRESS_CRSF_RECEIVER && info->destination != CRSF_ADDRESS_RADIO_TRANSMITTER)
        || (!fakerx && info->destination != 0 && info->destination != CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
        debug("process_device_info_frame(): rejected destination 0x%x -> %s", info->destination, info->payload);
        return false; // request was not for us
    }

    // we are only interested in RC device info for firmware version detection
    if (info->origin != 0 && info->origin != origin) {
        debug("process_device_info_frame(0x%x != 0x%x): rejected origin %s", info->origin, origin, info->payload);
        return false;
    }

    /*
        Payload size is 58:
        char[] Device name ( Null-terminated string, max len is 42 )
        uint32_t Serial number
        uint32_t Hardware ID
        uint32_t Firmware ID (0x00:0x00:0xAA:0xBB AA=major, BB=minor)
        uint8_t Parameters count
        uint8_t Parameter version number
    */
    // get the terminator of the device name string
    const uint8_t offset = strnlen((char*)info->payload,42U);
    if (strncmp((char*)info->payload, "Tracer", 6) == 0) {
        version->protocol = ProtocolType::PROTOCOL_TRACER;
    } else if (strncmp((char*)&info->payload[offset+1], "ELRS", 4) == 0) {
        // ELRS magic number is ELRS encoded in the serial number
        // 0x45 'E' 0x4C 'L' 0x52 'R' 0x53 'S'
        version->protocol = ProtocolType::PROTOCOL_ELRS;
    }

    if (version->protocol != ProtocolType::PROTOCOL_ELRS) {
        if (strncmp((char*)info->payload, "Betaflight", 10) == 0) {
            version->major = info->payload[11] - '0';
            version->minor = info->payload[13] - '0';
            version->is_betaflight = true;
        } else {
            /*
                fw major ver = offset + terminator (8bits) + serial (32bits) + hw id (32bits) + 3rd byte of sw id = 11bytes
                fw minor ver = offset + terminator (8bits) + serial (32bits) + hw id (32bits) + 4th byte of sw id = 12bytes
            */
            version->major = info->payload[offset+11];
            version->minor = info->payload[offset+12];
        }
    } else {
        // ELRS does not populate the version field so cook up something sensible
        version->major = 1;
        version->minor = 0;
    }

    // should we use rf_mode reported by link statistics?
    if (version->protocol == ProtocolType::PROTOCOL_ELRS
        || (version->protocol != ProtocolType::PROTOCOL_TRACER
            && (version->major > 3 || (version->major == 3 && version->minor >= 72)))) {
        version->use_rf_mode = true;
    }

    debug("process_device_info_frame(): %u %u %u", version->major, version->minor, (unsigned)version->protocol);

    return true;
}

#if 0
bool AP_CRSF_Protocol::process_ping_frame(ParameterPingFrame* info, bool fakerx)
{
    const uint8_t destination = fakerx ? AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER : AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    const uint8_t origin = fakerx ? AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER : AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER;

    if (info->destination != 0 && info->destination != destination) {
        return false; // request was not for us
    }

    // we are only interested in RC device info for firmware version detection
    if (info->origin != 0 && info->origin != origin) {
        return false;
    }
}
#endif

// encode a ping frame
void AP_CRSF_Protocol::encode_device_info_frame(Frame& frame, DeviceAddress destination, DeviceAddress origin)
{
    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = CRSF_FRAMETYPE_PARAM_DEVICE_INFO;

    ParameterDeviceInfoFrame info;

    // Construct the inner command frame header
    info.destination = destination;
    info.origin = origin;

    const uint8_t inner_payload_len = encode_device_info(info, 0);
    // Calculate the inner CRC (poly 0xBA) over the 2-byte command data
    uint8_t inner_crc = crc8_dvb_update_generic(0, (uint8_t*)&info, inner_payload_len, 0xBA);

    // Copy the command data and the inner CRC into the final frame payload
    memcpy(frame.payload, (uint8_t*)&info, inner_payload_len);
    frame.payload[inner_payload_len] = inner_crc;

    // Set the outer frame length.
    // It is the length of the payload (Type + Inner Command Frame)
    // Inner Command Frame = data + crc
    frame.length = inner_payload_len + 3;
}

// encode a link stats frame
void AP_CRSF_Protocol::encode_link_stats_tx_frame(uint32_t fps, Frame& frame, DeviceAddress destination, DeviceAddress origin)
{
    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = CRSF_FRAMETYPE_LINK_STATISTICS_TX;

    LinkStatisticsTXFrame* stats = (LinkStatisticsTXFrame*)&frame.payload;
    stats->fps = fps/10;
    stats->rssi_db = 0;
    stats->rssi_percent = 100;
    stats->link_quality = fps;
    stats->snr = 30;
    stats->rf_power_db = 14;    // 25mW

    // It is the length of the payload + type + crc
    frame.length = sizeof(LinkStatisticsTXFrame) + 2;
}

// return device information about ArduPilot
uint32_t AP_CRSF_Protocol::encode_device_info(ParameterDeviceInfoFrame& info, uint8_t num_params)
{
    // char[] Device name ( Null-terminated string )
    // uint32_t Serial number
    // uint32_t Hardware ID
    // uint32_t Firmware ID
    // uint8_t Parameters count
    // uint8_t Parameter version number

    const AP_FWVersion &fwver = AP::fwversion();
    // write out the name with version, max width is 60 - 18 = the meaning of life
    uint32_t n = strlen(fwver.fw_short_string);
    strncpy((char*)info.payload, fwver.fw_short_string, 41);
    n = MIN(n + 1, 42U);

    put_be32_ptr(&info.payload[n], fwver.os_sw_version);   // serial number
    n += 4;

    put_be32_ptr(&info.payload[n], // hardware id
        uint32_t(fwver.vehicle_type) << 24 | uint32_t(fwver.board_type) << 16 | uint32_t(fwver.board_subtype));
    n += 4;

    put_be32_ptr(&info.payload[n], // firmware id, major/minor should be 3rd and 4th byte
        uint32_t(fwver.major) << 8 | uint32_t(fwver.minor) | uint32_t(fwver.patch) << 24 | uint32_t(fwver.fw_type) << 16);
    n += 4;

    info.payload[n++] = num_params;
    info.payload[n++] = 0;   // param version

    return n;
}

// prepare qos data - mandatory frame that must be sent periodically
void AP_CRSF_Protocol::encode_heartbeat_frame(Frame& frame, DeviceAddress origin)
{
    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = CRSF_FRAMETYPE_HEARTBEAT;

    HeartbeatFrame* hb = (HeartbeatFrame*)&frame.payload;
    hb->origin = origin;

    // It is the length of the payload + type + crc
    frame.length = sizeof(HeartbeatFrame) + 2;
}

#endif // AP_CRSF_ENABLED
