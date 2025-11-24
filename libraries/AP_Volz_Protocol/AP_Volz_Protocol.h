/*
 * AP_VOLZ_PROTOCOL.h
 *
 *  Created on: Oct 31, 2017
 *      Author: guy tzoler
 *
 * Baud-Rate: 115.200 bits per second
 * Number of Data bits: 8
 * Number of Stop bits: 1
 * Parity: None
 * Half/Full Duplex: Half Duplex
 *
 * Volz Command and Response are all 6 bytes
 *
 * Command
 * byte	|	Communication Type
 * 1		Command Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 * byte	|	Communication Type
 * 1		Response Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_VOLZ_ENABLED
#define AP_VOLZ_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 2048
#endif

#if AP_VOLZ_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include <AP_Servo_Telem/AP_Servo_Telem_config.h>

class AP_Volz_Protocol {
public:
    AP_Volz_Protocol();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Volz_Protocol);

    static const struct AP_Param::GroupInfo var_info[];

    void update();

private:

    // Command and response IDs
    enum class CMD_ID : uint8_t {
        SET_EXTENDED_POSITION = 0xDC,
        EXTENDED_POSITION_RESPONSE = 0x2C,
        READ_CURRENT = 0xB0,
        CURRENT_RESPONSE = 0x30,
        READ_VOLTAGE = 0xB1,
        VOLTAGE_RESPONSE = 0x31,
        READ_TEMPERATURE = 0xC0,
        TEMPERATURE_RESPONSE = 0x10,
    };

    // Command frame
    union CMD {
        struct PACKED {
            CMD_ID ID;
            uint8_t actuator_id; // actuator send to or receiving from
            uint8_t arg1; // CMD dependant argument 1
            uint8_t arg2; // CMD dependant argument 2
            uint8_t crc1;
            uint8_t crc2;
        };
        uint8_t data[6];
    };

    AP_HAL::UARTDriver *port;

    // Loop in thread to output to uart
    void loop();

    void init(void);

    // Return the crc for a given command packet
    uint16_t calculate_crc(const CMD &cmd) const;

    // calculate CRC for volz serial protocol and send the data.
    void send_command(CMD &cmd);

    // Incoming PWM commands from the servo lib
    uint16_t servo_pwm[NUM_SERVO_CHANNELS];

    // Send postion commands from PWM, cycle through each servo
    void send_position_cmd();
    uint8_t last_sent_index;

    AP_Int32 bitmask;
    AP_Int16 range;
    bool initialised;

#if AP_SERVO_TELEM_ENABLED
    // Request telem data, cycling through each servo and telem item
    void request_telem();

    // Return true if the given ID is a valid response
    bool is_response(uint8_t ID) const;

    // Reading of telem packets
    void read_telem();
    void process_response(const CMD &cmd);

    uint8_t sent_count;

    struct {
        CMD_ID types[3] {
            CMD_ID::READ_CURRENT,
            CMD_ID::READ_VOLTAGE,
            CMD_ID::READ_TEMPERATURE,
        };
        uint8_t actuator_id;
        uint8_t request_type;
        HAL_Semaphore sem;
        CMD cmd_buffer;
        uint8_t buffer_offset;
        struct {
            uint32_t last_response_ms;
            float desired_angle;
            float angle;
            float primary_current;
            float secondary_current;
            float primary_voltage;
            float secondary_voltage;
            int16_t motor_temp_deg;
            int16_t pcb_temp_deg;
        } data[NUM_SERVO_CHANNELS];
    } telem;
#endif

};

#endif  // AP_VOLZ_PROTOCOL
