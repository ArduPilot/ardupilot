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

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>

//#include <GCS_MAVLink/GCS.h>

#define VOLZ_SCALE_VALUE 					(uint16_t)(VOLZ_EXTENDED_POSITION_MAX - VOLZ_EXTENDED_POSITION_MIN)	// Extended Position Data Format defines 100 as 0x0F80, which results in 1920 steps for +100 deg and 1920 steps for -100 degs meaning if you take movement a scaled between -1 ... 1 and multiply by 1920 you get the travel from center
#define VOLZ_SET_EXTENDED_POSITION_CMD 		0xDC
#define VOLZ_SET_EXTENDED_POSITION_RSP 		0x2C
#define VOLZ_DATA_FRAME_SIZE		 		6

#define VOLZ_EXTENDED_POSITION_MIN 			0x0080	// Extended Position Data Format defines -100 as 0x0080 decimal 128
#define VOLZ_EXTENDED_POSITION_CENTER 		0x0800	// Extended Position Data Format defines 0 as 0x0800 - decimal 2048
#define VOLZ_EXTENDED_POSITION_MAX 			0x0F80	// Extended Position Data Format defines +100 as 0x0F80 decimal 3968 -> full range decimal 3840

class AP_Volz_Protocol {
public:
    static const struct AP_Param::GroupInfo var_info[];

    static AP_Volz_Protocol create() {
        return AP_Volz_Protocol{};
    }

    constexpr AP_Volz_Protocol(AP_Volz_Protocol &&other) = default;

    /* Do not allow copies */
    AP_Volz_Protocol(const AP_Volz_Protocol &other) = delete;
    AP_Volz_Protocol &operator=(const AP_Volz_Protocol&) = delete;

    void update();

private:
    AP_Volz_Protocol();
    AP_HAL::UARTDriver *port;
    
    void init(void);
    void send_command(uint8_t data[VOLZ_DATA_FRAME_SIZE]);
    void update_volz_bitmask(uint32_t new_bitmask);

    uint32_t last_volz_update_time;
    uint32_t volz_time_frame_micros;
    uint32_t last_used_bitmask;

    AP_Int32 bitmask;
    bool initialised;
};
