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
 * Author: Oliver Walters
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_PiccoloCAN.h"

#if HAL_PICCOLO_CAN_ENABLE

#include <uavcan/uavcan.hpp>
#include <uavcan/driver/can.hpp>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#include <stdio.h>

#include <AP_PiccoloCAN/piccolo_protocol/ESCVelocityProtocol.h>
#include <AP_PiccoloCAN/piccolo_protocol/ESCPackets.h>


extern const AP_HAL::HAL& hal;

static const uint8_t CAN_IFACE_INDEX = 0;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

AP_PiccoloCAN::AP_PiccoloCAN()
{
    debug_can(2, "PiccoloCAN: constructed\n\r");
}

AP_PiccoloCAN *AP_PiccoloCAN::get_pcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_PiccoloCAN) {
        return nullptr;
    }

    return static_cast<AP_PiccoloCAN*>(AP::can().get_driver(driver_index));
}

// initialize PiccoloCAN bus
void AP_PiccoloCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(2, "PiccoloCAN: starting init\n\r");

    if (_initialized) {
        debug_can(1, "PiccoloCAN: already initialized\n\r");
        return;
    }

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "PiccoloCAN: no mgr for this driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "PiccoloCAN: mgr not initialized\n\r");
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "PiccoloCAN: no CAN driver\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PiccoloCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(1, "PiccoloCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    snprintf(_thread_name, sizeof(_thread_name), "PiccoloCAN_%u", driver_index);

    debug_can(2, "PiccoloCAN: init done\n\r");
}

// loop to send output to CAN devices in background thread
void AP_PiccoloCAN::loop()
{
    uavcan::CanFrame txFrame;
    uavcan::CanFrame rxFrame;

    // How often to transmit CAN messages (milliseconds)
#define CMD_TX_PERIOD 10

    uint16_t txCounter = 0;

    // CAN Frame ID components
    uint8_t frame_id_group;     // Piccolo message group
    uint16_t frame_id_device;   // Device identifier

    uavcan::MonotonicTime timeout;

    while (true) {

        if (!_initialized) {
            debug_can(2, "PiccoloCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 250);

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1 * 1000);

        // Transmit CAN commands at regular intervals
        if (txCounter++ > CMD_TX_PERIOD) {

            txCounter = 0;

            // Transmit ESC commands
            send_esc_messages();
        }

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {
            frame_id_group = (rxFrame.id >> 24) & 0x1F;
            frame_id_device = (rxFrame.id >> 8) & 0xFF;

            // Only accept extended messages
            if ((rxFrame.id & uavcan::CanFrame::FlagEFF) == 0) {
                continue;
            }

            switch (MessageGroup(frame_id_group)) {
            // ESC messages exist in the ACTUATOR group
            case MessageGroup::ACTUATOR:

                switch (ActuatorType(frame_id_device)) {
                case ActuatorType::ESC:
                    if (handle_esc_message(rxFrame)) {
                        // Returns true if the message was successfully decoded
                    }
                    break;
                default:
                    // Unknown actuator type
                    break;
                }

                break;
            default:
                break;
            }
        }
    }
}

// write frame on CAN bus, returns true on success
bool AP_PiccoloCAN::write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout)
{
    if (!_initialized) {
        debug_can(1, "PiccoloCAN: Driver not initialized for write_frame\n\r");
        return false;
    }

    // wait for space in buffer to send command
    uavcan::CanSelectMasks inout_mask;

    do {
        inout_mask.read = 0;
        inout_mask.write = (1 << CAN_IFACE_INDEX);
        _select_frames[CAN_IFACE_INDEX] = &out_frame;
        _can_driver->select(inout_mask, _select_frames, timeout);

        if (!inout_mask.write) {
            hal.scheduler->delay_microseconds(50);
        }
    } while (!inout_mask.write);

    return (_can_driver->getIface(CAN_IFACE_INDEX)->send(out_frame, timeout, uavcan::CanIOFlagAbortOnError) == 1);
}

// read frame on CAN bus, returns true on succses
bool AP_PiccoloCAN::read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout)
{
    if (!_initialized) {
        debug_can(1, "PiccoloCAN: Driver not initialized for read_frame\n\r");
        return false;
    }

    uavcan::CanSelectMasks inout_mask;
    inout_mask.read = 1 << CAN_IFACE_INDEX;
    inout_mask.write = 0;

    _select_frames[CAN_IFACE_INDEX] = &recv_frame;
    _can_driver->select(inout_mask, _select_frames, timeout);

    if (!inout_mask.read) {
        // No frame available
        return false;
    }

    uavcan::MonotonicTime time;
    uavcan::UtcTime utc_time;
    uavcan::CanIOFlags flags {};

    return (_can_driver->getIface(CAN_IFACE_INDEX)->receive(recv_frame, time, utc_time, flags) == 1);
}

// called from SRV_Channels
void AP_PiccoloCAN::update()
{
    uint64_t timestamp = AP_HAL::micros64();

    /* Read out the ESC commands from the channel mixer */
    for (uint8_t i = 0; i < PICCOLO_CAN_MAX_NUM_ESC; i++) {

        // Check each channel to determine if a motor function is assigned
        SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(i);

        if (SRV_Channels::function_assigned(motor_function)) {

            uint16_t output = 0;

            if (SRV_Channels::get_output_pwm(motor_function, output)) {

                _esc_info[i].command = output;
                _esc_info[i].newCommand = true;
            }
        }
    }

    AP_Logger *logger = AP_Logger::get_singleton();

    // Push received telemtry data into the logging system
    if (logger && logger->logging_enabled()) {

        WITH_SEMAPHORE(_telem_sem);

        for (uint8_t i = 0; i < PICCOLO_CAN_MAX_NUM_ESC; i++) {

            PiccoloESC_Info_t &esc = _esc_info[i];

            if (esc.newTelemetry) {

                logger->Write_ESC(i, timestamp,
                                  (int32_t) esc.statusA.rpm * 100,
                                  esc.statusB.voltage,
                                  esc.statusB.current,
                                  (int16_t) esc.statusB.escTemperature,
                                  0,  // TODO - Accumulated current
                                  (int16_t) esc.statusB.motorTemperature);

                esc.newTelemetry = false;
            }
        }
    }
}

// send ESC telemetry messages over MAVLink
void AP_PiccoloCAN::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
    // Arrays to store ESC telemetry data
    uint8_t temperature[4] {};
    uint16_t voltage[4] {};
    uint16_t rpm[4] {};
    uint16_t count[4] {};
    uint16_t current[4] {};
    uint16_t totalcurrent[4] {};

    bool dataAvailable = false;

    uint8_t idx = 0;

    WITH_SEMAPHORE(_telem_sem);

    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_ESC; ii++) {

        // Calculate index within storage array
        idx = (ii % 4);

        PiccoloESC_Info_t &esc = _esc_info[idx];

        // Has the ESC been heard from recently?
        if (is_esc_present(ii)) {
            dataAvailable = true;

            temperature[idx] = esc.statusB.escTemperature;
            voltage[idx] = esc.statusB.voltage;
            current[idx] = esc.statusB.current;
            totalcurrent[idx] = 0;
            rpm[idx] = esc.statusA.rpm;
            count[idx] = 0;
        } else {
            temperature[idx] = 0;
            voltage[idx] = 0;
            current[idx] = 0;
            totalcurrent[idx] = 0;
            rpm[idx] = 0;
            count[idx] = 0;
        }

        // Send ESC telemetry in groups of 4
        if ((ii % 4) == 3) {

            if (dataAvailable) {
                if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t) mav_chan, ESC_TELEMETRY_1_TO_4)) {
                    continue;
                }

                switch (ii) {
                case 3:
                    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                case 7:
                    mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                case 11:
                    mavlink_msg_esc_telemetry_9_to_12_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                default:
                    break;
                }
            }

            dataAvailable = false;
        }
    }
}


// send ESC messages over CAN
void AP_PiccoloCAN::send_esc_messages(void)
{
    uavcan::CanFrame txFrame;

    uavcan::MonotonicTime timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 250);

    // TODO - How to buffer CAN messages properly?
    // Sending more than 2 messages at each loop instance means that sometimes messages are dropped

    if (hal.util->get_soft_armed()) {

        bool send_cmd = false;
        int16_t cmd[4] {};
        uint8_t idx;

        // Transmit bulk command packets to 4x ESC simultaneously
        for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_GROUP_ESC; ii++) {

            send_cmd = false;

            for (uint8_t jj = 0; jj < 4; jj++) {

                idx = (ii * 4) + jj;

                /* Check if the ESC is software-inhibited.
                 * If so, send a message to enable it.
                 */
                if (is_esc_present(idx) && !is_esc_enabled(idx)) {
                    encodeESC_EnablePacket(&txFrame);
                    txFrame.id |= (idx + 1);
                    write_frame(txFrame, timeout);
                }
                else if (_esc_info[idx].newCommand) {
                    send_cmd = true;
                    cmd[jj] = _esc_info[idx].command;
                    _esc_info[idx].newCommand = false;
                } else {
                    // A command of 0xFFFF is 'out of range' and will be ignored by the corresponding ESC
                    cmd[jj] = 0xFFFF;
                }
            }

            if (send_cmd) {
                encodeESC_CommandMultipleESCsPacket(
                    &txFrame,
                    cmd[0],
                    cmd[1],
                    cmd[2],
                    cmd[3],
                    (PKT_ESC_SETPOINT_1 + ii)
                );

                // Broadcast the command to all ESCs
                txFrame.id |= 0xFF;

                write_frame(txFrame, timeout);
            }
        }

    } else {
        // System is NOT armed - send a "disable" message to all ESCs on the bus

        // Command all ESC into software disable mode
        encodeESC_DisablePacket(&txFrame);

        // Set the ESC address to the broadcast ID (0xFF)
        txFrame.id |= 0xFF;

        write_frame(txFrame, timeout);
    }
}


// interpret an ESC message received over CAN
bool AP_PiccoloCAN::handle_esc_message(uavcan::CanFrame &frame)
{
    uint64_t timestamp = AP_HAL::micros64();

    // The ESC address is the lower byte of the address
    uint8_t addr = frame.id & 0xFF;

    // Ignore any ESC with node ID of zero
    if (addr == 0x00) {
        return false;
    }

    // Subtract to get the address in memory
    addr -= 1;

    // Maximum number of ESCs allowed
    if (addr >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    PiccoloESC_Info_t &esc = _esc_info[addr];

    bool result = true;

    // Throw the packet against each decoding routine
    if (decodeESC_StatusAPacketStructure(&frame, &esc.statusA)) {
        esc.newTelemetry = true;
    } else if (decodeESC_StatusBPacketStructure(&frame, &esc.statusB)) {

        esc.newTelemetry = true;
    } else if (decodeESC_FirmwarePacketStructure(&frame, &esc.firmware)) {
        // TODO
    } else if (decodeESC_AddressPacketStructure(&frame, &esc.address)) {
        // TODO
    } else if (decodeESC_EEPROMSettingsPacketStructure(&frame, &esc.eeprom)) {
        // TODO
    } else {
        result = false;
    }

    if (result) {
        // Reset the Rx timestamp
        esc.last_rx_msg_timestamp = timestamp;
    }

    return result;
}


bool AP_PiccoloCAN::is_esc_present(uint8_t chan, uint64_t timeout_ms)
{

    if (chan >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    PiccoloESC_Info_t &esc = _esc_info[chan];

    // No messages received from this ESC
    if (esc.last_rx_msg_timestamp == 0) {
        return false;
    }

    uint64_t now = AP_HAL::micros64();

    uint64_t timeout_us = timeout_ms * 1000;

    if (now > (esc.last_rx_msg_timestamp + timeout_us)) {
        return false;
    }

    return true;
}


bool AP_PiccoloCAN::is_esc_enabled(uint8_t chan)
{
    if (chan >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    // If the ESC is not present, we cannot determine if it is enabled or not
    if (!is_esc_present(chan)) {
        return false;
    }

    PiccoloESC_Info_t &esc = _esc_info[chan];

    if (esc.statusA.status.hwInhibit || esc.statusA.status.swInhibit) {
        return false;
    }

    // ESC is present, and enabled
    return true;

}


bool AP_PiccoloCAN::pre_arm_check(char* reason, uint8_t reason_len)
{
    // Check that each required ESC is present on the bus
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_ESC; ii++) {

        SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(ii);

        // There is a motor function assigned to this channel
        if (SRV_Channels::function_assigned(motor_function)) {

            if (!is_esc_present(ii)) {
                snprintf(reason, reason_len, "ESC %u not detected", ii + 1);
                return false;
            }

            PiccoloESC_Info_t &esc = _esc_info[ii];

            if (esc.statusA.status.hwInhibit) {
                snprintf(reason, reason_len, "ESC %u is hardware inhibited", (ii + 1));
                return false;
            }
        }
    }

    return true;
}


/* Piccolo Glue Logic
 * The following functions are required by the auto-generated protogen code.
 */

//! \return the packet data pointer from the packet
uint8_t* getESCVelocityPacketData(void* pkt)
{
    uavcan::CanFrame* frame = (uavcan::CanFrame*) pkt;

    return (uint8_t*) frame->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getESCVelocityPacketDataConst(const void* pkt)
{
    uavcan::CanFrame* frame = (uavcan::CanFrame*) pkt;

    return (const uint8_t*) frame->data;
}

//! Complete a packet after the data have been encoded
void finishESCVelocityPacket(void* pkt, int size, uint32_t packetID)
{
    uavcan::CanFrame* frame = (uavcan::CanFrame*) pkt;

    if (size > uavcan::CanFrame::MaxDataLen) {
        size = uavcan::CanFrame::MaxDataLen;
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
                  (((uint8_t) AP_PiccoloCAN::ActuatorType::ESC) << 8);              // Actuator type

    // Extended frame format
    id |= uavcan::CanFrame::FlagEFF;

    frame->id = id;
}

//! \return the size of a packet from the packet header
int getESCVelocityPacketSize(const void* pkt)
{
    uavcan::CanFrame* frame = (uavcan::CanFrame*) pkt;

    return (int) frame->dlc;
}

//! \return the ID of a packet from the packet header
uint32_t getESCVelocityPacketID(const void* pkt)
{
    uavcan::CanFrame* frame = (uavcan::CanFrame*) pkt;

    // Extract the message ID field from the 29-bit ID
    return (uint32_t) ((frame->id >> 16) & 0xFF);
}


#endif // HAL_PICCOLO_CAN_ENABLE

