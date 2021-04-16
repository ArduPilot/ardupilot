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

#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
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

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "PiccoloCAN", fmt, ##args); } while (0)

// table of user-configurable Piccolo CAN bus parameters
const AP_Param::GroupInfo AP_PiccoloCAN::var_info[] = {

    // @Param: ESC_BM
    // @DisplayName: ESC channels
    // @Description: Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 1, AP_PiccoloCAN, _esc_bm, 0xFFFF),

    // @Param: ESC_RT
    // @DisplayName: ESC output rate
    // @Description: Output rate of ESC command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("ESC_RT", 2, AP_PiccoloCAN, _esc_hz, PICCOLO_MSG_RATE_HZ_DEFAULT),

    AP_GROUPEND
};

AP_PiccoloCAN::AP_PiccoloCAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    debug_can(AP_CANManager::LOG_INFO, "PiccoloCAN: constructed\n\r");
}

AP_PiccoloCAN *AP_PiccoloCAN::get_pcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_PiccoloCAN) {
        return nullptr;
    }

    return static_cast<AP_PiccoloCAN*>(AP::can().get_driver(driver_index));
}

bool AP_PiccoloCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize PiccoloCAN bus
void AP_PiccoloCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "PiccoloCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: already initialized\n\r");
        return;
    }
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PiccoloCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    snprintf(_thread_name, sizeof(_thread_name), "PiccoloCAN_%u", driver_index);

    debug_can(AP_CANManager::LOG_DEBUG, "PiccoloCAN: init done\n\r");
}

// loop to send output to CAN devices in background thread
void AP_PiccoloCAN::loop()
{
    AP_HAL::CANFrame txFrame;
    AP_HAL::CANFrame rxFrame;

    uint16_t esc_tx_counter = 0;

    // CAN Frame ID components
    uint8_t frame_id_group;     // Piccolo message group
    uint16_t frame_id_device;   // Device identifier

    while (true) {

        _esc_hz = constrain_int16(_esc_hz, PICCOLO_MSG_RATE_HZ_MIN, PICCOLO_MSG_RATE_HZ_MAX);

        uint16_t escCmdRateMs = (uint16_t) ((float) 1000 / _esc_hz);

        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        uint64_t timeout = AP_HAL::micros64() + 250ULL;

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // Transmit ESC commands at regular intervals
        if (esc_tx_counter++ > escCmdRateMs) {
            esc_tx_counter = 0;

            send_esc_messages();
        }

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {

            // Extract group and device ID values from the frame identifier
            frame_id_group = (rxFrame.id >> 24) & 0x1F;
            frame_id_device = (rxFrame.id >> 8) & 0xFF;

            // Only accept extended messages
            if ((rxFrame.id & AP_HAL::CANFrame::FlagEFF) == 0) {
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
bool AP_PiccoloCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized for write_frame\n\r");
        return false;
    }

    bool read_select = false;
    bool write_select = true;
    
    bool ret =  _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on succses
bool AP_PiccoloCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized for read_frame\n\r");
        return false;
    }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from SRV_Channels
void AP_PiccoloCAN::update()
{
    /* Read out the ESC commands from the channel mixer */
    for (uint8_t i = 0; i < PICCOLO_CAN_MAX_NUM_ESC; i++) {

        if (is_esc_channel_active(i)) {

            uint16_t output = 0;
            
            SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(i);

            if (SRV_Channels::get_output_pwm(motor_function, output)) {

                _esc_info[i].command = output;
                _esc_info[i].newCommand = true;
            }
        }
    }
}

// send ESC messages over CAN
void AP_PiccoloCAN::send_esc_messages(void)
{
    AP_HAL::CANFrame txFrame;

    uint64_t timeout = AP_HAL::micros64() + 1000ULL;

    // No ESCs are selected? Don't send anything
    if (_esc_bm == 0x00) {
        return;
    }

    // System is armed - send out ESC commands
    if (hal.util->get_soft_armed()) {

        bool send_cmd = false;
        int16_t cmd[4] {};
        uint8_t idx;

        // Transmit bulk command packets to 4x ESC simultaneously
        for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_GROUP_ESC; ii++) {

            send_cmd = false;

            for (uint8_t jj = 0; jj < 4; jj++) {

                idx = (ii * 4) + jj;

                // Skip an ESC if the motor channel is not enabled
                if (!is_esc_channel_active(idx)) {
                    continue;
                }

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
bool AP_PiccoloCAN::handle_esc_message(AP_HAL::CANFrame &frame)
{
    bool result = true;

#if HAL_WITH_ESC_TELEM
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

    /*
     * The STATUS_A packet has slight variations between Gen-1 and Gen-2 ESCs.
     * We can differentiate between the different versions,
     * and coerce the "legacy" values into the modern values
     * Legacy STATUS_A packet variables
     */
    ESC_LegacyStatusBits_t legacyStatus;
    ESC_LegacyWarningBits_t legacyWarnings;
    ESC_LegacyErrorBits_t legacyErrors;

    // Throw the packet against each decoding routine
    if (decodeESC_StatusAPacket(&frame, &esc.mode, &esc.status, &esc.setpoint, &esc.rpm)) {
        esc.newTelemetry = true;
        update_rpm(addr, esc.rpm);
    } else if (decodeESC_LegacyStatusAPacket(&frame, &esc.mode, &legacyStatus, &legacyWarnings, &legacyErrors, &esc.setpoint, &esc.rpm)) {
        // The status / warning / error bits need to be converted to modern values
        // Note: Not *all* of the modern status bits are available in the Gen-1 packet
        esc.status.hwInhibit = legacyStatus.hwInhibit;
        esc.status.swInhibit = legacyStatus.swInhibit;
        esc.status.afwEnabled = legacyStatus.afwEnabled;
        esc.status.direction = legacyStatus.timeout;
        esc.status.timeout = legacyStatus.timeout;
        esc.status.starting = legacyStatus.starting;
        esc.status.commandSource = legacyStatus.commandSource;
        esc.status.running = legacyStatus.running;

        // Copy the legacy warning information across
        esc.warnings.overspeed = legacyWarnings.overspeed;
        esc.warnings.overcurrent = legacyWarnings.overcurrent;
        esc.warnings.escTemperature = legacyWarnings.escTemperature;
        esc.warnings.motorTemperature = legacyWarnings.motorTemperature;
        esc.warnings.undervoltage = legacyWarnings.undervoltage;
        esc.warnings.overvoltage = legacyWarnings.overvoltage;
        esc.warnings.invalidPWMsignal = legacyWarnings.invalidPWMsignal;
        esc.warnings.settingsChecksum = legacyErrors.settingsChecksum;

        // There are no common error bits between the Gen-1 and Gen-2 ICD
    } else if (decodeESC_StatusBPacket(&frame, &esc.voltage, &esc.current, &esc.dutyCycle, &esc.escTemperature, &esc.motorTemperature)) {
        TelemetryData t {
            .temperature_cdeg = int16_t(esc.escTemperature * 100),
            .voltage = float(esc.voltage) * 0.01f,
            .current = float(esc.current) * 0.01f,
        };
        update_telem_data(addr, t,
            AP_ESC_Telem_Backend::TelemetryType::CURRENT
                | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);

        esc.newTelemetry = true;
    } else if (decodeESC_StatusCPacket(&frame, &esc.fetTemperature, &esc.pwmFrequency, &esc.timingAdvance)) {
        TelemetryData t { };
        t.motor_temp_cdeg = int16_t(esc.fetTemperature * 100);
        update_telem_data(addr, t, AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE);

        esc.newTelemetry = true;
    } else if (decodeESC_WarningErrorStatusPacket(&frame, &esc.warnings, &esc.errors)) {
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
#endif

    return result;
}

/**
 * Check if a given ESC channel is "active" (has been configured correctly)
 */
bool AP_PiccoloCAN::is_esc_channel_active(uint8_t chan)
{
    // First check if the particular ESC channel is enabled in the channel mask
    if (((_esc_bm >> chan) & 0x01) == 0x00) {
        return false;
    }

    // Check if a motor function is assigned for this motor channel
    SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(chan);

    if (SRV_Channels::function_assigned(motor_function)) {
        return true;
    }

    return false;
}


/**
 * Determine if an ESC is present on the CAN bus (has telemetry data been received)
 */
bool AP_PiccoloCAN::is_esc_present(uint8_t chan, uint64_t timeout_ms) const
{
    if (chan >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    const PiccoloESC_Info_t &esc = _esc_info[chan];

    // No messages received from this ESC
    if (esc.last_rx_msg_timestamp == 0) {
        return false;
    }

    uint64_t now = AP_HAL::micros64();

    uint64_t timeout_us = timeout_ms * 1000ULL;

    if (now > (esc.last_rx_msg_timestamp + timeout_us)) {
        return false;
    }

    return true;
}


/**
 * Check if a given ESC is enabled (both hardware and software enable flags)
 */
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

    if (esc.status.hwInhibit || esc.status.swInhibit) {
        return false;
    }

    // ESC is present, and enabled
    return true;

}


bool AP_PiccoloCAN::pre_arm_check(char* reason, uint8_t reason_len)
{
    // Check that each required ESC is present on the bus
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_ESC; ii++) {

        // Skip any ESC channels where the motor channel is not enabled
        if (is_esc_channel_active(ii)) {

            if (!is_esc_present(ii)) {
                snprintf(reason, reason_len, "ESC %u not detected", ii + 1);
                return false;
            }

            PiccoloESC_Info_t &esc = _esc_info[ii];

            if (esc.status.hwInhibit) {
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
                  (((uint8_t) AP_PiccoloCAN::ActuatorType::ESC) << 8);              // Actuator type

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

