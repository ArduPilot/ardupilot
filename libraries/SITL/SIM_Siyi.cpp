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
  Base class for for Siyi camera simulaors
*/

#include "SIM_config.h"

#if AP_SIM_SIYI_ENABLED

#include "SIM_Siyi.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <errno.h>

using namespace SITL;

void Siyi::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<buflen; i++) {
        if ((uint8_t)msg.buffer[i] == HEADER1) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(msg.buffer, &msg.buffer[i], buflen-i);
    buflen = buflen - i;
}

uint32_t Siyi::expected_message_length(Siyi::CommandID id) const
{
    switch (id) {
    case CommandID::ACQUIRE_FIRMWARE_VERSION:
        return sizeof(FirmwareVersionRequest);
    case CommandID::HARDWARE_ID:
        return sizeof(HardwareIDRequest);
       case CommandID::ACQUIRE_GIMBAL_ATTITUDE:
        return sizeof(GimbalAttitudeRequest);
    case CommandID::EXTERNAL_ATTITUDE:
        return sizeof(ExternalAttitude);
    case CommandID::SET_CAMERA_IMAGE_TYPE:
        return sizeof(SetCameraImageType);
    case CommandID::GET_TEMP_FULL_IMAGE:
        return sizeof(GetTempFullImageRequest);
    case CommandID::READ_RANGEFINDER:
        return sizeof(ReadRangefinderRequest);
    case CommandID::PHOTO:
        return sizeof(Photo);
    case CommandID::ACQUIRE_GIMBAL_CONFIG_INFO:
        return sizeof(AcquireGimbalConfigInfo);
    case CommandID::GIMBAL_ROTATION:
        return sizeof(GimbalRotation);
    case CommandID::POSITION_DATA:
        return sizeof(PositionData);
    case CommandID::SET_TIME:
        return sizeof(SetTime);
    }
    AP_HAL::panic("Unknown command received (0x%02x)", unsigned(CommandID(msg.packed_empty.msg.cmdid)));
}

void Siyi::handle_received_message()
{
    const CommandID id { CommandID(msg.packed_empty.msg.cmdid) };

    const uint16_t want_datalen = expected_message_length(id) - 1;
    const uint16_t got_datalen = le16toh(msg.packed_empty.datalen);
    if (want_datalen != got_datalen) {
        if (strict_parsing) {
            AP_HAL::panic("Bad datalen for %02x, got=%u want=%u", (unsigned)id, got_datalen, want_datalen);
        }
        return;
    }

    switch (id) {
    case CommandID::ACQUIRE_FIRMWARE_VERSION:
        handle_message(msg.packed_firmwareversionrequest);
        break;
    case CommandID::HARDWARE_ID:
        handle_message(msg.packed_hardwareidrequest);
        break;
    case CommandID::ACQUIRE_GIMBAL_ATTITUDE:
        handle_message(msg.packed_gimbalattituderequest);
        break;
    case CommandID::EXTERNAL_ATTITUDE:
        handle_message(msg.packed_externalattitude);
        break;
    case CommandID::SET_CAMERA_IMAGE_TYPE:
        handle_message(msg.packed_setcameraimgetype);
        break;
    case CommandID::PHOTO:
        handle_message(msg.packed_photo);
        break;
    case CommandID::ACQUIRE_GIMBAL_CONFIG_INFO:
        handle_message(msg.packed_acquiregimbalconfiginfo);
        break;
    case CommandID::GET_TEMP_FULL_IMAGE:
        handle_message(msg.packed_gettempfullimage);
        break;
    case CommandID::READ_RANGEFINDER:
        handle_message(msg.packed_readrangefinder);
        break;
    case CommandID::GIMBAL_ROTATION:
        handle_message(msg.packed_gimbalrotation);
        break;
    case CommandID::POSITION_DATA:
        handle_message(msg.packed_positiondata);
        break;
    case CommandID::SET_TIME:
        handle_message(msg.packed_settime);
        break;
    default:
        if (strict_parsing) {
            AP_HAL::panic("Unknown command received (%02x)", unsigned(CommandID(msg.packed_empty.msg.cmdid)));
        }
    }
}

void Siyi::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&msg.buffer[buflen], ARRAY_SIZE(msg.buffer) - buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }

    switch (buflen) {
    default:
    case 7:  // validate sequence number
    case 6:
        FALLTHROUGH;
    case 5:  // validate data length
        if (strict_parsing) {
            const uint16_t received_length = le16toh(msg.packed_empty.datalen);
            if (received_length > 60) {
                // the number is arbitrary and based off our current
                // driver; if you're adding support for longer messages
                // just change it.
                AP_HAL::panic("Bad length %u", received_length);
            }
        }
        FALLTHROUGH;
    case 4:
    case 3:  // validate control field
        if (msg.packed_empty.ctrl != 1) {
            if (strict_parsing) {
                AP_HAL::panic("Bad control");
            }
            move_preamble_in_buffer(3);
            return;
        }
        FALLTHROUGH;
    case 2:  // validate HEADER2
        if (msg.packed_empty.header2 != HEADER2) {
            if (strict_parsing) {
                AP_HAL::panic("bad HEADER2");
            }
            move_preamble_in_buffer(1);
            return;
        }
        FALLTHROUGH;
    case 1:  // validate HEADER1
        if (msg.packed_empty.header1 != HEADER1) {
            if (strict_parsing) {
                AP_HAL::panic("bad HEADER1");
            }
            move_preamble_in_buffer(1);
            return;
        }
        break;
    case 0:
        return;
    }

    // this sanity check so we don't use the length fields when
    // they're not valid:
    if (buflen < sizeof(msg.packed_empty)) {
        return;
    }

    const uint16_t datalen = le16toh(msg.packed_empty.datalen);
    const uint16_t totallen = sizeof(msg.packed_empty) + datalen;
    if (totallen > sizeof(msg.buffer)) {
        if (strict_parsing) {
            AP_HAL::panic("bad datalen");
        }
        move_preamble_in_buffer(1);
        return;
    }

    if (buflen < totallen) {
        if (strict_parsing) {
            AP_HAL::panic("bad datalen");
        }
        move_preamble_in_buffer(1);
        return;
    }

    const uint16_t calculated_checksum = msg.packed_empty.calculate_checksum(totallen-2);  // -2 to omit checksum bytes
    const uint16_t received_checksum = UINT16_VALUE(msg.buffer[totallen-1], msg.buffer[totallen-2]);
    if (calculated_checksum != received_checksum) {
        if (strict_parsing) {
            AP_HAL::panic("bad checksum");
        }
        move_preamble_in_buffer(1);
        return;
    }

    if (strict_parsing) {
        const uint16_t received_seq = le16toh(msg.packed_empty.seq);
        // printf("received_seq=%u\n", unsigned(received_seq));
        if (received_seq != expected_seq) {
            AP_HAL::panic("Bad sequence (got=%u vs want=%u)", msg.packed_empty.seq, expected_seq+1);
        }
        expected_seq = received_seq + 1;
    }

    handle_received_message();

    move_preamble_in_buffer(totallen);
}

void Siyi::update(const class Aircraft &aircraft)
{
    update_input();
    // update_output(location);
}

#endif  // AP_SIM_SIYI_ENABLED
