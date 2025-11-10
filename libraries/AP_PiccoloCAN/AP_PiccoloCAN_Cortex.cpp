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

#include "AP_PiccoloCAN_Cortex.h"

#if HAL_PICCOLO_CAN_ENABLE

// Protocol files for the Cortex controller
#include <AP_PiccoloCAN/piccolo_protocol/CortexDefines.h>
#include <AP_PiccoloCAN/piccolo_protocol/CortexPackets.h>
#include <AP_PiccoloCAN/piccolo_protocol/CortexProtocol.h>

/**
 * PiccoloCAN glue logic
 */

//! \return the packet data pointer from the packet
uint8_t* getCortexPacketData(void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (uint8_t*) frame->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getCortexPacketDataConst(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (const uint8_t*) frame->data;
}

//! Complete a packet after the data have been encoded
void finishCortexPacket(void* pkt, int size, uint32_t packetID)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    if (size > AP_HAL::CANFrame::MaxDataLen) {
        size = AP_HAL::CANFrame::MaxDataLen;
    }

    frame->dlc = size;

    /* Encode the CAN ID
     * 0x1Ammddii
     * - 1A = Battery group ID
     * - mm = Message ID
     * - dd = Cortex device type
     * - ii = Device ID
     *
     * Note: The Device type and node ID (lower 16 bits of the frame ID) will be inserted later
     */

    uint32_t id = (((uint8_t) PiccoloCAN_MessageGroup::BATTERY) << 24) |    // CAN Group ID
                  ((packetID & 0xFF) << 16);                                // Message ID

    // Extended frame format
    id |= AP_HAL::CANFrame::FlagEFF;

    frame->id = id;
}

//! \return the size of a packet from the packet header
int getCortexPacketSize(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (int) frame->dlc;
}

//! \return the ID of a packet from the packet header
uint32_t getCortexPacketID(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    // Extract the message ID field from the 29-bit ID
    return (uint32_t) ((frame->id >> 16) & 0xFF);
}

#endif // HAL_PICCOLO_CAN_ENABLE
