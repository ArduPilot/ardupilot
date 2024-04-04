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
#define AP_VOLZ_ENABLED BOARD_FLASH_SIZE > 1024
#endif

#if AP_VOLZ_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>

class AP_Volz_Protocol {
public:
    AP_Volz_Protocol();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Volz_Protocol);

    static const struct AP_Param::GroupInfo var_info[];

    void update();

private:

    // Command frame
    union CMD {
        struct PACKED {
            uint8_t ID; // CMD ID
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
    uint8_t last_sent_index;

    void init(void);
    void send_command(CMD &cmd);

    // Incoming PWM commands from the servo lib
    uint16_t servo_pwm[NUM_SERVO_CHANNELS];

    AP_Int32 bitmask;
    AP_Int16 range;
    bool initialised;
};

#endif  // AP_VOLZ_PROTOCOL
