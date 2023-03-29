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
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_PiccoloCAN.h"
#include "AP_PiccoloCAN_ESC.h"

#if HAL_PICCOLO_CAN_ENABLE

AP_PiccoloCAN_ESC::AP_PiccoloCAN_ESC() : AP_PiccoloCAN_Device()
{
}


/**
 * Decode an incoming CAN frame with the ACTUATOR / ESC message space
 */
bool AP_PiccoloCAN_ESC::decode_esc_packet(AP_HAL::CANFrame &frame, uint8_t index, uint64_t timestamp)
{
    // Temporary decode variable
    uint8_t tmp;

    if (decodeESC_StatusAPacketStructure(&frame, &statusA)) {
        telemAvailableStatusA = true;
        update_rpm(index, rpm());
    } else if (decodeESC_StatusBPacketStructure(&frame, &statusB)) {
        telemAvailableStatusB = true;

        AP_ESC_Telem_Backend::TelemetryData t = {};

        t.temperature_cdeg = int16_t(temperature() * 100);
        t.motor_temp_cdeg = int16_t(motor_temperature() * 100);
        t.voltage = voltage();
        t.current = current();

        update_telem_data(index, t,
            AP_ESC_Telem_Backend::CURRENT |
            AP_ESC_Telem_Backend::VOLTAGE |
            AP_ESC_Telem_Backend::TEMPERATURE |
            AP_ESC_Telem_Backend::MOTOR_TEMPERATURE);

    } else if (decodeESC_StatusCPacketStructure(&frame, &statusC)) {
        telemAvailableStatusC = true;

        AP_ESC_Telem_Backend::TelemetryData t = {};

        t.temperature_cdeg = int16_t(temperature() * 100);
        update_telem_data(index, t, AP_ESC_Telem_Backend::TEMPERATURE);

    } else if (decodeESC_WarningErrorStatusPacket(&frame, &warningBits, &errorBits, &tmp, &warningBits, &errorBits)) {
        telemAvailableWarningErrorStatus = true;
    } else if (decodeESC_MotorStatusFlagsPacketStructure(&frame, &motorStatusFlags)) {
        telemAvailableMotorStatusFlags = true;
    } else if (decodeESC_TelltaleValuesPacketStructure(&frame, &telltale)) {
        telemAvailableTelltale = true;
    } else if (decodeESC_FirmwarePacketStructure(&frame, &firmware)) {
    } else if (decodeESC_AddressPacketStructure(&frame, &address)) {
    } else if (decodeESC_EEPROMSettingsPacketStructure(&frame, &eeprom)) {
    } else {
        // No packet matches
        return false;
    }

    last_rx_msg_timestamp = timestamp;

    return true;
}


void AP_PiccoloCAN_ESC::set_output_command(uint16_t cmd)
{
    command = cmd;
    newCommand = true;
}


int16_t AP_PiccoloCAN_ESC::rpm(void) const
{
    return statusA.rpm;
}


/*
 * Return the ESC voltage reading, in Volts
 * Note that the voltage is reported from the ESC in 100mV units
 */
float AP_PiccoloCAN_ESC::voltage(void) const
{
    return (float) statusB.voltage * 0.1f;
}


/*
 * Return the ESC DC current reading, in Amps 
 * Note that the current is reported from the ESC in 100mA units
 */
float AP_PiccoloCAN_ESC::current(void) const
{
    return (float) statusB.current * 0.1f;
}


/*
 * Return the ESC internal temperature, in degrees
 */
float AP_PiccoloCAN_ESC::temperature(void) const
{
    if (statusB.escTemperature > statusC.fetTemperature) {
        return (float) statusB.escTemperature;
    } else {
        return (float) statusC.fetTemperature;
    }
}


/*
 * Return the ESC motor temperature, in degrees
 * Note: The ESC must have a configured temperature sensor connected
 */
float AP_PiccoloCAN_ESC::motor_temperature(void) const
{
    return (float) statusB.motorTemperature;
}


/**
 * Returns true if the ESC is enabled (ready to receive commands)
 */
bool AP_PiccoloCAN_ESC::is_enabled(void) const
{
    return is_hardware_enabled() && is_software_enabled();
}


/**
 * Check if the ESC is hardware enabled.
 * Note the status bit is inverted, to represent "inhibit"
 */
bool AP_PiccoloCAN_ESC::is_hardware_enabled(void) const
{
    return !statusA.status.hwInhibit;
}

/**
 * Check if the ESC is software enabled.
 * Note the status bit is inverted, to represent "inhibit"
 */
bool AP_PiccoloCAN_ESC::is_software_enabled(void) const
{
    return !statusA.status.swInhibit;
}

/* Piccolo Glue Logic
 * The following functions are required by the auto-generated protogen code.
 */

//! \return the packet data pointer from the packet
uint8_t* getESCVelocityPacketData(void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (uint8_t*) frame->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getESCVelocityPacketDataConst(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (const uint8_t*) frame->data;
}

//! Complete a packet after the data have been encoded
void finishESCVelocityPacket(void* pkt, int size, uint32_t packetID)
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
     * - 20 = ESC actuator type
     * - dd = Device ID
     *
     * Note: The Device ID (lower 8 bits of the frame ID) will have to be inserted later
     */

    uint32_t id = (((uint8_t) AP_PiccoloCAN::MessageGroup::ACTUATOR) << 24) |       // CAN Group ID
                  ((packetID & 0xFF) << 16) |                                       // Message ID
                  (((uint8_t) AP_PiccoloCAN::DeviceType::ESC) << 8);              // Actuator type

    // Extended frame format
    id |= AP_HAL::CANFrame::FlagEFF;

    frame->id = id;
}

//! \return the size of a packet from the packet header
int getESCVelocityPacketSize(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (int) frame->dlc;
}

//! \return the ID of a packet from the packet header
uint32_t getESCVelocityPacketID(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    // Extract the message ID field from the 29-bit ID
    return (uint32_t) ((frame->id >> 16) & 0xFF);
}

#endif // HAL_PICCOLO_CAN_ENABLE
