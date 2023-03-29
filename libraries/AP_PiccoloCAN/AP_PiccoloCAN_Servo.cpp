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

#include <AP_HAL/AP_HAL.h>

#include "AP_PiccoloCAN.h"
#include "AP_PiccoloCAN_Servo.h"

#if HAL_PICCOLO_CAN_ENABLE

AP_PiccoloCAN_Servo::AP_PiccoloCAN_Servo() : AP_PiccoloCAN_Device()
{
}


/**
 * Decode an incoming CAN frame within the ACTUATOR / Servo message space
 */
bool AP_PiccoloCAN_Servo::decode_servo_packet(AP_HAL::CANFrame &frame, uint64_t timestamp)
{
    if (decodeServo_StatusAPacketStructure(&frame, &statusA)) {
        newTelemetryAvailable = true;
    } else if (decodeServo_StatusBPacketStructure(&frame, &statusB)) {
        newTelemetryAvailable = true;
    } else if (decodeServo_FirmwarePacketStructure(&frame, &firmware)) {
    } else if (decodeServo_AddressPacketStructure(&frame, &address)) {
    } else if (decodeServo_SettingsInfoPacketStructure(&frame, &settings)) {
    } else if (decodeServo_TelemetryConfigPacketStructure(&frame, &telemetry)) {
    } else {
        // No packet matches
        return false;
    }

    last_rx_msg_timestamp = timestamp;

    return true;
}


void AP_PiccoloCAN_Servo::set_output_command(uint16_t cmd)
{
    command = cmd;
    newCommand = true;
}


bool AP_PiccoloCAN_Servo::is_enabled(void) const
{
    return statusA.status.enabled;
}


bool AP_PiccoloCAN_Servo::add_log_data(AP_Logger *logger, uint64_t timestamp, uint8_t id)
{
    if (logger && newTelemetryAvailable) {

        logger->Write_ServoStatus(
            timestamp,
            id,
            (float) statusA.position,         // Servo position (represented in microsecond units)
            (float) statusB.current / 100.0f, // Servo force (actually servo current, 0.01A per bit)
            (float) statusB.speed,            // Servo speed (degrees per second)
            (uint8_t) abs(statusB.dutyCycle)  // Servo duty cycle (absolute value as it can be +/- 100%)
        );

        newTelemetryAvailable = false;

        return true;
    } else {
        return false;
    }
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
     * 0x07mm00dd
     * - 07 = ACTUATOR group ID
     * - mm = Message ID
     * - 00 = Servo actuator type
     * - dd = Device ID
     *
     * Note: The Device ID (lower 8 bits of the frame ID) will have to be inserted later
     */

    uint32_t id = (((uint8_t) AP_PiccoloCAN::MessageGroup::ACTUATOR) << 24) |       // CAN Group ID
                  ((packetID & 0xFF) << 16) |                                       // Message ID
                  (((uint8_t) AP_PiccoloCAN::DeviceType::SERVO) << 8);            // Actuator type

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
