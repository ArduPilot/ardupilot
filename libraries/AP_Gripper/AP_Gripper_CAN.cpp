/*
 * AP_Gripper_CAN.cpp
 *
 * CAN-based gripper driver implementation
 */

#include "AP_Gripper_CAN.h"

// #if AP_GRIPPER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

// Constructor
AP_Gripper_CAN::AP_Gripper_CAN(struct AP_Gripper::Backend_Config &_config) :
    AP_Gripper_Backend(_config)  // Default board_ID, should be configurable via parameter
{
    _can_iface = nullptr;
    _can_driver_index = 0;  // CAN peripheral driver 1
    _state_changed = false;
    _last_send_ms = 0;
    _sequence_num = 0;
    _current_position = 0;
    _armed = false;
    _can_msg_id = 0x320;
}

// Initialize the CAN gripper
void AP_Gripper_CAN::init_gripper()
{
    // Get CAN interface directly from HAL
    // _can_driver_index represents which physical CAN peripheral to use (0=CAN1, 1=CAN2)
    _can_iface = hal.can[_can_driver_index];
    
    if (_can_iface == nullptr) {
        return;
    }
    
    // Initialize the CAN interface if not already initialized
    if (!_can_iface->is_initialized()) {
        return;
    }
}

// Grab - close the gripper (set to maximum position)
void AP_Gripper_CAN::grab()
{
    // Send position command to close gripper
    // Typically, close = high position value
    // ARM the System
    send_arm_command(true);
    
    // Send position command to close gripper
    if (send_position_command(POS_MAX)) {
        _state_changed = true;
        // Log the grab event. Please note Drop Mech has reverse logic than Gripper where grab is actually the open dropmech command
        AP::logger().Write_Event(LogEvent::GRIPPER_GRAB);
    }
    
}

// Release - open the gripper (set to minimum position)
void AP_Gripper_CAN::release()
{
    // First ARM the gripper if not already armed
    send_arm_command(true);

    // Send position command to open gripper
    // Typically, open = low position value
    if (send_position_command(POS_MIN)) {
        _state_changed = true;
        // Log the release event. Please note Drop Mech has reverse logic than Gripper where release is actually the close dropmech command
        AP::logger().Write_Event(LogEvent::GRIPPER_RELEASE);
    }
}

// Check if gripper is healthy
bool AP_Gripper_CAN::valid() const
{
    return (_can_iface != nullptr);
    // return true;
}

// Update gripper - called regularly from main loop
void AP_Gripper_CAN::update_gripper()
{
    // No implementation needed for now
}

// Build ARM state CAN frame (Message 1: 1 trunk)
void AP_Gripper_CAN::build_arm_frame(bool arm, AP_HAL::CANFrame& frame)
{
    frame.id = _can_msg_id;  // 0x320 + board_ID
    frame.dlc = 8;
    
    // Byte 0: Sequence number (bits 6-7) and Total trunks (bits 0-5)
    // Bits 6-7: sequence (0-3)
    // Bits 0-5: total_trunks = 1
    frame.data[0] = ((_sequence_num & 0x03) << 6) | (1 & 0x3F);
    
    // Byte 1: 0x0 (bits 6-7) and Message trunk number (bits 0-5)
    // Bits 6-7: 0x0
    // Bits 0-5: trunk_num = 0
    frame.data[1] = (0 << 6) | (0 & 0x3F);
    
    // Bytes 2-5: Reserved (0x0)
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    
    // Byte 6: ARM state
    // Bits 0-3: MCU-CAN state (ignored if from FC-CAN) - set to 0
    // Bits 4-7: FC-CAN state: 1=standby(disarm), 2=arm
    uint8_t fc_state = arm ? 2 : 1;  // 2=arm, 1=disarm
    frame.data[6] = (fc_state << 4) | 0;  // FC state in upper nibble
    
    // Byte 7: Reserved (0x0)
    frame.data[7] = 0x00;
}

// Utility function to print CAN frame (for debugging)
void print_frame(const AP_HAL::CANFrame& frame)
{
    hal.console->printf("CAN ID: 0x%03X DLC: %u Data:", static_cast<unsigned int>(frame.id), frame.dlc);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN ID: %03X DLC: %u Data:", static_cast<unsigned int>(frame.id), frame.dlc);
    for (uint8_t i = 0; i < frame.dlc; i++) {
        
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, " %02X", frame.data[i]);
    }
    hal.console->printf("\n");
}

// Send ARM/DISARM command (Message 1)
bool AP_Gripper_CAN::send_arm_command(bool arm)
{
    if (_can_iface == nullptr) {
        return false;
    }
    
    AP_HAL::CANFrame frame;
    build_arm_frame(arm, frame);
    
    // Send the frame
    uint64_t timeout = AP_HAL::micros64() + GRIPPER_CAN_TIMEOUT_MS * 1000ULL;
    int16_t res = _can_iface->send(frame, timeout, 0);

    if (res <= 0) {
        hal.console->printf("Gripper_CAN: Failed to send ARM command, res=%d\n", res);
        return false;
    }
    
    // Increment sequence number (wraps at 4)
    _sequence_num = (_sequence_num + 1) & 0x03;
    
    return true;
}

// Send position command (Message 2: 2 trunks)
bool AP_Gripper_CAN::send_position_command(uint8_t position)
{
    if (_can_iface == nullptr) {
        return false;
    }
    
    AP_HAL::CANFrame frame0, frame1;
    build_position_frames(position, frame0, frame1);
    // Debug Code to print frames before sending
    // print_frame(frame0);
    // print_frame(frame1);
    
    // Send trunk 0
    uint64_t timeout = AP_HAL::micros64() + GRIPPER_CAN_TIMEOUT_MS * 1000ULL;
    int16_t res = _can_iface->send(frame0, timeout, 0);
    
    if (res <= 0) {
        return false;
    }
    
    // Send trunk 1
    timeout = AP_HAL::micros64() + GRIPPER_CAN_TIMEOUT_MS * 1000ULL;
    res = _can_iface->send(frame1, timeout, 0);
    
    if (res <= 0) {
        return false;
    }
    
    // Increment sequence number (wraps at 4)
    _sequence_num = (_sequence_num + 1) & 0x03;
    _current_position = position;
    _last_send_ms = AP_HAL::millis();
    
    return true;
}



// Build position control CAN frames (Message 2: 2 trunks)
void AP_Gripper_CAN::build_position_frames(uint8_t position, AP_HAL::CANFrame& frame0, AP_HAL::CANFrame& frame1)
{
    // Get 64-bit timestamp (microseconds or milliseconds depending on your needs)
    // The spec shows 8 bytes split: Byte0-3 in trunk0, Byte4-7 in trunk1
    // uint64_t timestamp = AP_HAL::micros64();
    uint64_t timestamp = 0;
    
    // === TRUNK 0 (Message trunk number 0) ===
    frame0.id = _can_msg_id;  // 0x320 + board_ID
    frame0.dlc = 8;
    
    // Byte 0: Sequence number (bits 6-7) and Total trunks (bits 0-5)
    // Bits 6-7: sequence (0-3)
    // Bits 0-5: total_trunks = 2
    frame0.data[0] = ((_sequence_num & 0x03) << 6) | (2 & 0x3F);
    
    // Byte 1: 0x0 (bits 6-7) and Message trunk number (bits 0-5)
    // Bits 6-7: 0x0
    // Bits 0-5: trunk_num = 0
    frame0.data[1] = (0 << 6) | (0 & 0x3F);
    
    // Bytes 2-5: Command execution timestamp (little-endian, Byte0-3)
    frame0.data[2] = (timestamp >> 0) & 0xFF;   // Byte 0 of timestamp
    frame0.data[3] = (timestamp >> 8) & 0xFF;   // Byte 1 of timestamp
    frame0.data[4] = (timestamp >> 16) & 0xFF;  // Byte 2 of timestamp
    frame0.data[5] = (timestamp >> 24) & 0xFF;  // Byte 3 of timestamp
    
    // Byte 6: Reserved (0x0)
    frame0.data[6] = 0x00;
    
    // Byte 7: Set Position (1-255, 0=no change), valid only when DM is in arm state
    frame0.data[7] = position;
    
    // === TRUNK 1 (Message trunk number 1) ===
    frame1.id = _can_msg_id;  // Same message ID
    frame1.dlc = 8;
    
    // Byte 0: Sequence number (bits 6-7) and Total trunks (bits 0-5)
    // Bits 6-7: sequence (0-3) - SAME as trunk 0
    // Bits 0-5: total_trunks = 2
    frame1.data[0] = ((_sequence_num & 0x03) << 6) | (2 & 0x3F);
    
    // Byte 1: 0x0 (bits 6-7) and Message trunk number (bits 0-5)
    // Bits 6-7: 0x0
    // Bits 0-5: trunk_num = 1
    frame1.data[1] = (0 << 6) | (1 & 0x3F);
    
    // Bytes 2-5: Command execution timestamp (little-endian, Byte4-7)
    frame1.data[2] = (timestamp >> 32) & 0xFF;  // Byte 4 of timestamp
    frame1.data[3] = (timestamp >> 40) & 0xFF;  // Byte 5 of timestamp
    frame1.data[4] = (timestamp >> 48) & 0xFF;  // Byte 6 of timestamp
    frame1.data[5] = (timestamp >> 56) & 0xFF;  // Byte 7 of timestamp
    
    // Bytes 6-7: Reserved (0x0)
    frame1.data[6] = 0x00;
    frame1.data[7] = 0x00;
}

// #endif  // AP_GRIPPER_ENABLED