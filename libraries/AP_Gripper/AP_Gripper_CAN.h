/*
 * AP_Gripper_CAN.h
 *
 * CAN-based gripper driver for ArduPilot
 * Sends RC input from Ch8 over CAN bus to control gripper
 */

#pragma once

#include <AP_Gripper/AP_Gripper_Backend.h>
#include <AP_CANManager/AP_CANManager.h>
#define GRIPPER_CAN_TIMEOUT_MS 100  // Timeout for CAN communication

void print_frame(const AP_HAL::CANFrame&);

class AP_Gripper_CAN : public AP_Gripper_Backend {
public:
    AP_Gripper_CAN(struct AP_Gripper::Backend_Config &_config);
        // AP_Gripper_Backend(_config) { };

    // grab - send CAN message to initiate grabbing the cargo
    void grab() override;

    // release - send CAN message to initiate releasing the cargo
    void release() override;

    // grabbed - returns true if gripper in grabbed state
    bool grabbed() const override {
        return false;
    }

    // released - returns true if gripper in released state
    bool released() const override {
        return false;
    }

    // valid - returns true if the backend should be working
    bool valid() const override;
protected:
    // type-specific intiailisations:
    void init_gripper() override;

    // type-specific periodic updates:
    void update_gripper() override;
private:

    // Creating CAN message
    struct Gripper_CAN_Message {
        uint8_t sequence_num : 4;      // Bits 7-4: sequence number (0-3)
        uint8_t total_trunks : 4;      // Bits 3-0: total number of trunks (2)
        uint8_t trunk_num_byte1 : 4;   // Byte 1: Bits 7-4: 0x0
        uint8_t msg_trunk_num : 4;     // Byte 1: Bits 3-0: message trunk number
        uint32_t timestamp;            // Bytes 2-5: Command execution timestamp (little-endian)
        uint8_t reserved;              // Byte 6: Reserved (0x0)
        uint8_t set_pos;               // Byte 7: Set position (1-255, 0=no change)
    }__attribute__((packed));

    // Send CAN message to gripper
    bool send_can_message(const uint8_t set_pos);

    // Build CAN frame for gripper control
    void build_can_frame(uint8_t position, uint8_t trunk_num);

        // Send CAN message with gripper command
    bool send_position_command(uint8_t position);
    
    // Send ARM/DISARM state command
    bool send_arm_command(bool arm);
    
    // Build position control CAN frames (2 trunks)
    void build_position_frames(uint8_t position, AP_HAL::CANFrame& frame0, AP_HAL::CANFrame& frame1);
    
    // Build ARM state CAN frame (1 trunk)
    void build_arm_frame(bool arm, AP_HAL::CANFrame& frame);

    // CAN interface for gripper
    AP_HAL::CANIface* _can_iface;

    uint8_t _can_driver_index;

    // Gripper state tracking for CAN
    bool _state_changed;
    bool _armed;
    uint32_t _last_send_ms;
    uint8_t _sequence_num;
    uint8_t _current_position;
    uint32_t action_timestamp;

    // CAN message ID (0x320 + board ID)
    uint16_t _can_msg_id;

    // Gripper position limits
    static constexpr uint8_t POS_MIN = 1;
    static constexpr uint8_t POS_MAX = 255;
    static constexpr uint8_t POS_NEUTRAL = 0;  // 0 = no change

    bool has_state_can(const uint8_t state) const;

};