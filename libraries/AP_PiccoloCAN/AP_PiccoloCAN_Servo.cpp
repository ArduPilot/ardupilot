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
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#include "AP_PiccoloCAN_Servo.h"

#if HAL_PICCOLO_CAN_ENABLE

/*
 * Decode a recevied CAN frame.
 * It is assumed at this point that the received frame is intended for *this* servo
 */
bool AP_PiccoloCAN_Servo::handle_can_frame(AP_HAL::CANFrame &frame)
{
    bool result = true;

    if (decodeServo_StatusAPacketStructure(&frame, &status.statusA)) {
        newTelemetry = true;
    } else if (decodeServo_StatusBPacketStructure(&frame, &status.statusB)) {
        newTelemetry = true;
    } else if (decodeServo_FirmwarePacketStructure(&frame, &settings.firmware)) {
    } else if (decodeServo_AddressPacketStructure(&frame, &settings.address)) {
    } else if (decodeServo_SettingsInfoPacketStructure(&frame, &settings.settings)) {
    } else {
        // Incoming frame did not match any packet decoding routine
        result = false;
    }

    if (result) {
        reset_rx_timestamp();
    }

    return result;
}


/* Piccolo Glue Logic
 * The following functions are required by the auto-generated protogen code.
 */

//! \return the packet data pointer from the packet
uint8_t* getServoPacketData(void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (uint8_t*) frame->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getServoPacketDataConst(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (const uint8_t*) frame->data;
}

//! Complete a packet after the data have been encoded
void finishServoPacket(void* pkt, int size, uint32_t packetID)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    if (size > AP_HAL::CANFrame::MaxDataLen) {
        size = AP_HAL::CANFrame::MaxDataLen;
    }

    frame->dlc = size;

    /* Encode the CAN ID
     * 0x07mm20dd
     * - 07 = ACTUATOR group ID
     * - mm = Message ID
     * - 00 = Servo actuator type
     * - dd = Device ID
     *
     * Note: The Device ID (lower 8 bits of the frame ID) will have to be inserted later
     */

    uint32_t id = (((uint8_t) PiccoloCAN_MessageGroup::ACTUATOR) << 24) |   // CAN Group ID
                  ((packetID & 0xFF) << 16) |                               // Message ID
                  (((uint8_t) PiccoloCAN_DeviceType::SERVO) << 8);          // Actuator type

    // Extended frame format
    id |= AP_HAL::CANFrame::FlagEFF;

    frame->id = id;
}

//! \return the size of a packet from the packet header
int getServoPacketSize(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (int) frame->dlc;
}

//! \return the ID of a packet from the packet header
uint32_t getServoPacketID(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    // Extract the message ID field from the 29-bit ID
    return (uint32_t) ((frame->id >> 16) & 0xFF);
}

#endif // HAL_PICCOLO_CAN_ENABLE