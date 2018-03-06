/*
   USB Joypad Interface for URUS and Ardupilot.
   Copyright (c) 2017-2018 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This code was ported from the roginal source to work with
 * URUS SHAL and APM HAL, several parts from the original source
 * was modified, for more info please see external C file
 * there contains the links.
 * NOTE: External C file contains only the USB Vendor and Product
 * descriptors and headers licenses from original source.
 * I put it there because AVR GCC can't manage
 * wide char correctly in C++ when it is stored in progmem
 * section.
 *
 * Thanks to:
 * - Josh Kropf
 * - grunskis
 * - Toodles
 * - PJRC.COM
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if (CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM16U)

#include "AP_Joypad.h"
#include "AP_Joypad_Backend.h"
#include "external/avr/usb_gamepad_def_shared.h"

#pragma pack(push, 1)
#define USB_GAMEPAD_PRIVATE_INCLUDE

typedef struct __gamepad_state
{
	uint8_t id;
	// digital buttons, 0 = off, 1 = on
	uint8_t button_array[BUTTON_ARRAY_LENGTH];

	// digital direction, use the dir_* constants(enum)
	// 8 = center, 0 = up, 1 = up/right, 2 = right, 3 = right/down
	// 4 = down, 5 = down/left, 6 = left, 7 = left/up
	uint8_t direction;

	// left and right analog sticks, 0x000 left/up, 0x3E8 middle, 0xfff right/down
	int16_t l_x_axis;
	int16_t l_y_axis;
	int16_t r_x_axis;
	int16_t r_y_axis;
	int16_t x_3_axis;
	int16_t y_3_axis;

	// Gonna assume these are button analog values for the d-pad.
	// NOTE: NOT EVEN SURE THIS IS RIGHT, OR IN THE CORRECT ORDER
	uint8_t up_axis;
	uint8_t right_axis;
	uint8_t down_axis;
	uint8_t left_axis;

	// button axis, 0x00 = unpressed, 0xff = fully pressed
	uint8_t triangle_axis;
	uint8_t circle_axis;
	uint8_t cross_axis;
	uint8_t square_axis;

	uint8_t l1_axis;
	uint8_t r1_axis;
	uint8_t l2_axis;
	uint8_t r2_axis;

public:
    __gamepad_state():
        id(GAMEPAD_0_REPORT_ID),
        button_array{0,0,0,0},
        direction(0x08),
        l_x_axis(0x3E8),
        l_y_axis(0x3E8),
        r_x_axis(0x3E8),
        r_y_axis(0x3E8),
        x_3_axis(0x3E8),
        y_3_axis(0x3E8),
        up_axis(0x00),
        right_axis(0x00),
        down_axis(0x00),
        left_axis(0x00),
        triangle_axis(0x00),
        circle_axis(0x00),
        cross_axis(0x00),
        square_axis(0x00),
        l1_axis(0x00),
        r1_axis(0x00),
        l2_axis(0x00),
        r2_axis(0x00)
    {}

} gamepad_state;
#pragma pack(pop)

class AP_Joypad_USB : public AP_Joypad_Backend
{
public:
    ~AP_Joypad_USB();

    /** see process function on backend class
      */
    void process(AP_Joypad::ProcessMode process_mode) override;

    /** see update function on top class
      */
    void update() override;

    /** see configure function on top class
      */
    static AP_Joypad_Backend *configure(AP_Joypad &joypad);

    static void fire_isr_usb_genvect();
    static void fire_isr_usb_comvect();

private:
    AP_Joypad_USB(AP_Joypad &joypad);

    bool _configure();
    void _process_event();

    uint8_t usb_configure(void);
    uint8_t usb_configured(void);
    int8_t send_controller_data_to_usb(data_controller_t btnList, uint8_t playerID);
    int8_t usb_gamepad_send(void);

    static void usb_wait_in_ready(void);
    static void usb_send_in(void);
    static void usb_wait_receive_out(void);
    static void usb_ack_out(void);

    /** Get a 16 bit value off the serial port by doing two successive reads
      * Assumes that data is being transmitted high byte first
      */
    int16_t get_16bit_value(int serial_index);

    // Initializes the USART to receive and transmit,
    //  takes in a value you can find in the datasheet
    //  based on desired communication and clock speeds
    void usart_init(uint16_t baud_setting);

    // This reads the USART serial port, returning any data that's in the
    //  buffer, or a guaranteed zero if it took longer than timeout ms
    //  Input: uint_16 timeout - milliseconds to wait for data before timing out
    unsigned char serialRead( uint16_t timeout );

    // This sends out a byte of data via the USART.
    void serialWrite( unsigned char data );
    void flush_serialRead();

    gamepad_state usb_controller_state;
    data_controller_t controllerData1;
    data_controller_t controllerData2;

    uint32_t _now;
    bool _inproc_event;

    // zero when we are not configured, non-zero when enumerated
    static volatile uint8_t usb_configuration;
    static uint8_t gamepad_idle_config;

    // protocol setting from the host.  We use exactly the same report
    // either way, so this variable only stores the setting since we
    // are required to be able to report which setting is in use.
    static uint8_t gamepad_protocol;
    bool _auto_process = false;

};

#endif
