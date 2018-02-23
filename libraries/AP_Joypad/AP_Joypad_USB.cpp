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
 * This code was ported from C to work with URUS SHAL and APM HAL,
 * several parts from the original source was modified, for
 * original source code and info please see external/avr path C file,
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

#include <AP_HAL/AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM16U)

#include "AP_Joypad_USB.h"

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define TXLED 5

extern const usb_string_descriptor_struct string0 PROGMEM;
extern const usb_string_descriptor_struct string1 PROGMEM;
extern const usb_string_descriptor_struct string2 PROGMEM;

static const descriptor_list_struct descriptor_list[] PROGMEM = {
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
	{0x2100, GAMEPAD_INTERFACE, config1_descriptor+GAMEPAD_HID_DESC_OFFSET, 9},
	{0x2200, GAMEPAD_INTERFACE, gamepad_hid_report_desc, sizeof(gamepad_hid_report_desc)},
	{0x0300, 0x0000, (const uint8_t *)&string0, 4},
	{0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)}
};

#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(descriptor_list_struct))

volatile uint8_t AP_Joypad_USB::usb_configuration = 0;
uint8_t AP_Joypad_USB::gamepad_idle_config = 0;
uint8_t AP_Joypad_USB::gamepad_protocol = 1;

extern const AP_HAL::HAL& hal;

AP_Joypad_USB::AP_Joypad_USB(AP_Joypad &joypad) :
    AP_Joypad_Backend(joypad)
{
}

AP_Joypad_USB::~AP_Joypad_USB()
{}

void AP_Joypad_USB::process(AP_Joypad::ProcessMode process_mode)
{
    hal.gpio->pinMode(TXLED, HAL_GPIO_OUTPUT);
    hal.gpio->write(TXLED, 1);
    _now = AP_HAL::micros();

    switch (process_mode) {
    case AP_Joypad::loop_process:
        _auto_process = false;
        break;
    case AP_Joypad::auto_process:
        _auto_process = true;
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Joypad_USB::_process_event, void));
        break;
    default:
        break;
    }
    hal.scheduler->delay(100);
}

void AP_Joypad_USB::update()
{
    if (_auto_process) {
        return;
    }

    _process_event();
}

AP_Joypad_Backend *AP_Joypad_USB::configure(AP_Joypad &joypad)
{
    AP_Joypad_USB *joy = new AP_Joypad_USB(joypad);

    if (!joy || !joy->_configure()) {
        delete joy;
        return nullptr;
    }

    return joy;
}

void AP_Joypad_USB::usart_init(uint16_t baud_setting)
{
    // Set baud rate
    UBRR1 = baud_setting;
    // Enable receiver and transmitter
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);
    // Set frame format: 8data, 1stop bit
    UCSR1C = (1<<UCSZ10)|(1<<UCSZ11);
    UCSR1A = (1<<U2X1);
}

void AP_Joypad_USB::serialWrite(unsigned char data)
{
    // Wait for empty transmit buffer
    while (!( UCSR1A & (1<<UDRE1))){
    }
    // Put data into buffer, sends the data
    UDR1 = data;
}

void AP_Joypad_USB::flush_serialRead()
{
    unsigned char dummy;
    while ( UCSR1A & (1<<RXC1) ) {
        dummy = UDR1;
    }
    dummy++;
}

unsigned char AP_Joypad_USB::serialRead(uint16_t timeout)
{
    // Wait for data to be received
    while (!(UCSR1A & (1<<RXC1))) {
        hal.scheduler->delay(1);
        timeout--;
        if (timeout == 0) {
            return 0;
        }
    }
    // Get and return received data from buffer
    return UDR1;
    return 0;
}

void AP_Joypad_USB::_process_event()
{
    if (_inproc_event) {
        return;
    }

    if ((AP_HAL::micros() - _now) > 4000LU)
    {
        // We get our data from the remote by writing which byte we
        // want from the DataController_t, and then wait for the
        // remote to send that back to us.
        // The serialRead(number) function reads the serial port, and the
        // number is a timeout (in ms) so if there's a transmission error,
        // we don't stall forever.

        _inproc_event = true;
        _now = AP_HAL::micros();
		hal.gpio->write(TXLED, 1);
		flush_serialRead();
		int serialIndex = 0;

		// The buttons are held in an array, so we need to break it between the two controllers
		for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
			serialWrite(serialIndex);
			serialIndex++;
			controllerData1.button_array[i] = serialRead(25);
		}

		serialWrite(serialIndex);
		serialIndex++;
		uint8_t directionButtons1 = serialRead(25);
		controllerData1.dpad_left_on = 1 & (directionButtons1 >> 0);
		controllerData1.dpad_up_on = 1 & (directionButtons1 >> 1);
		controllerData1.dpad_right_on = 1 & (directionButtons1 >> 2);
		controllerData1.dpad_down_on = 1 & (directionButtons1 >> 3);

		// Assuming that 16 bit data gets sent high byte first
		controllerData1.left_stick_x = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData1.left_stick_y = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData1.right_stick_x = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData1.right_stick_y = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData1.stick3_x = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData1.stick3_y = get_16bit_value(serialIndex);
		serialIndex += 2;

		for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
			serialWrite(serialIndex);
			serialIndex++;
			controllerData2.button_array[i] = serialRead(25);
		}

		serialWrite(serialIndex);
		serialIndex++;
		uint8_t directionButtons2 = serialRead(25);
		controllerData2.dpad_left_on = 1 & (directionButtons2 >> 0);
		controllerData2.dpad_up_on = 1 & (directionButtons2 >> 1);
		controllerData2.dpad_right_on = 1 & (directionButtons2 >> 2);
		controllerData2.dpad_down_on = 1 & (directionButtons2 >> 3);

		controllerData2.left_stick_x = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData2.left_stick_y = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData2.right_stick_x = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData2.right_stick_y = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData2.stick3_x = get_16bit_value(serialIndex);
		serialIndex += 2;
		controllerData2.stick3_y = get_16bit_value(serialIndex);

		// Communication with the Arduino chip is over here
		hal.gpio->write(TXLED, 0);
        // Finally, we send the data out via the USB port
		send_controller_data_to_usb(controllerData1, 1);
		hal.scheduler->delay(2);
		send_controller_data_to_usb(controllerData2, 2);
		_inproc_event = false;
    }
}

int16_t AP_Joypad_USB::get_16bit_value(int serial_index)
{
    int16_t returnValue = 0;
    serialWrite(serial_index);
    serial_index++;
    returnValue = serialRead(25);

    serialWrite(serial_index);
    serial_index++;
    returnValue += serialRead(25) << 8;
    return returnValue;
}

bool AP_Joypad_USB::_configure()
{
    usart_init(16);
	// Configure our USB connection
	usb_configure();
    hal.gpio->pinMode(TXLED, HAL_GPIO_OUTPUT);
    hal.gpio->write(TXLED, 1);

	while (!usb_configured()) {
        hal.gpio->toggle(TXLED);
		hal.scheduler->delay(50);
	} // wait

	hal.scheduler->delay(500);

    return true;
}

// Configure USB
//  Returns 0 if configured,
//          1 if it timed out waiting for connection
uint8_t AP_Joypad_USB::usb_configure(void)
{
	HW_CONFIG();
	USB_FREEZE();				// enable USB
	PLL_CONFIG();				// config PLL

	// wait a certain amount of time for PLL lock
	unsigned long timeoutCounter = 0;

	while (!(PLLCSR & (1<<PLOCK))) {
		hal.scheduler->delay(1);
		timeoutCounter++;
		if (timeoutCounter >= USB_TIMEOUT)
			return 1;
	}

	USB_CONFIG();				// start USB clock
	UDCON = 0;				// enable attach resistor
	usb_configuration = 0;
	UDIEN = (1<<EORSTE) | (1<<SOFE);
	//sei();
	return 0;
}

// return 0 if the USB is not configured, or the configuration
// number selected by the HOST
uint8_t AP_Joypad_USB::usb_configured(void)
{
	return usb_configuration;
}

int8_t AP_Joypad_USB::send_controller_data_to_usb(data_controller_t btnList, uint8_t playerID)
{
	usb_controller_state.id = playerID;
	memcpy(usb_controller_state.button_array, btnList.button_array, BUTTON_ARRAY_LENGTH);

	// digital direction, use the dir_* constants(enum)
	// 8 = center, 0 = up, 1 = up/right, 2 = right, 3 = right/down
	// 4 = down, 5 = down/left, 6 = left, 7 = left/up
	usb_controller_state.direction = 8;
	if (btnList.dpad_up_on == 1) {
		if (btnList.dpad_left_on == 1) {
			usb_controller_state.direction = 7;
		}
		else if (btnList.dpad_right_on == 1) {
			usb_controller_state.direction = 1;
		}
		else
			usb_controller_state.direction = 0;
	}
	else if (btnList.dpad_down_on == 1) {
				if (btnList.dpad_left_on == 1) {
			usb_controller_state.direction = 5;
		}
		else if (btnList.dpad_right_on == 1) {
			usb_controller_state.direction = 3;
		}
		else
			usb_controller_state.direction = 4;
	}
	else if (btnList.dpad_left_on == 1) {
		usb_controller_state.direction = 6;
	}
	else if (btnList.dpad_right_on == 1) {
		usb_controller_state.direction = 2;
	}

	// Take care of the d-pad analog pressures separately,
	//  since the 'convert to hat switch' code is confusing
	if (btnList.dpad_up_on == 1)
		usb_controller_state.up_axis = 0xFF;
	else
		usb_controller_state.up_axis = 0;
	if (btnList.dpad_right_on == 1)
		usb_controller_state.right_axis = 0xFF;
	else
		usb_controller_state.right_axis = 0;
	if (btnList.dpad_down_on == 1)
		usb_controller_state.down_axis = 0xFF;
	else
		usb_controller_state.down_axis = 0;
	if (btnList.dpad_left_on == 1)
		usb_controller_state.left_axis = 0xFF;
	else
		usb_controller_state.left_axis = 0;

	// left and right analog sticks, 0x000 left/up, 0x3E8 middle, 0x7D0 right/down
	// Sanity check the inputs so we don't try and go out of bounds

	int16_t stickMin = 0;
	int16_t stickMax = 2000;
	if (btnList.left_stick_x < stickMin)
		btnList.left_stick_x = stickMin;
	if (btnList.left_stick_x > stickMax)
		btnList.left_stick_x = stickMax;
	if (btnList.left_stick_y < stickMin)
		btnList.left_stick_y = stickMin;
	if (btnList.left_stick_y > stickMax)
		btnList.left_stick_y = stickMax;

	if (btnList.right_stick_x < stickMin)
		btnList.right_stick_x = stickMin;
	if (btnList.right_stick_x > stickMax)
		btnList.right_stick_x = stickMax;
	if (btnList.right_stick_y < stickMin)
		btnList.right_stick_y = stickMin;
	if (btnList.right_stick_y > stickMax)
		btnList.right_stick_y = stickMax;

	if (btnList.stick3_x < stickMin)
		btnList.stick3_x = stickMin;
	if (btnList.stick3_x > stickMax)
		btnList.stick3_x = stickMax;
	if (btnList.stick3_y < stickMin)
		btnList.stick3_y = stickMin;
	if (btnList.stick3_y > stickMax)
		btnList.stick3_y = stickMax;

	usb_controller_state.l_x_axis = btnList.left_stick_x;
	usb_controller_state.l_y_axis = btnList.left_stick_y;
	usb_controller_state.r_x_axis = btnList.right_stick_x;
	usb_controller_state.r_y_axis = btnList.right_stick_y;
	usb_controller_state.x_3_axis = btnList.stick3_x;
	usb_controller_state.y_3_axis = btnList.stick3_y;

	// Send the data out via USB
	return usb_gamepad_send();
}

int8_t AP_Joypad_USB::usb_gamepad_send(void)
{
	uint8_t intr_state, timeout, i;

	if (!usb_configuration) return -1;
	intr_state = SREG;
	cli();
	UENUM = GAMEPAD_ENDPOINT;
	timeout = UDFNUML + 50;
	while (1) {
		// are we ready to transmit?
		if (UEINTX & (1<<RWAL)) break;
		SREG = intr_state;
		// has the USB gone offline?
		if (!usb_configuration) return -1;
		// have we waited too long?
		if (UDFNUML == timeout) return -1;
		// get ready to try checking again
		intr_state = SREG;
		cli();
		UENUM = GAMEPAD_ENDPOINT;
	}

	for (i=0; i<sizeof(gamepad_state); i++) {
		UEDATX = ((uint8_t*)&usb_controller_state)[i];
	}

	UEINTX = 0x3A;
	SREG = intr_state;
	return 0;
}

// Misc functions to wait for ready and send/receive packets
void AP_Joypad_USB::usb_wait_in_ready(void)
{
	while (!(UEINTX & (1<<TXINI))) ;
}

void AP_Joypad_USB::usb_send_in(void)
{
	UEINTX = ~(1<<TXINI);
}

void AP_Joypad_USB::usb_wait_receive_out(void)
{
	while (!(UEINTX & (1<<RXOUTI))) ;
}

void AP_Joypad_USB::usb_ack_out(void)
{
	UEINTX = ~(1<<RXOUTI);
}

inline void AP_Joypad_USB::fire_isr_usb_genvect()
{
	uint8_t intbits;

	intbits = UDINT;
	UDINT = 0;
	if (intbits & (1<<EORSTI)) {
		UENUM = 0;
		UECONX = 1;
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = (1<<RXSTPE);
		usb_configuration = 0;
	}
}

void AP_Joypad_USB::fire_isr_usb_comvect()
{
	uint8_t intbits;
	const uint8_t *list;
	const uint8_t *cfg;
	uint8_t i, n, len, en;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint16_t desc_val;
	const uint8_t *desc_addr;
	uint8_t	desc_length;

	UENUM = 0;
	intbits = UEINTX;
	if (intbits & (1<<RXSTPI)) {
		bmRequestType = UEDATX;
		bRequest = UEDATX;
		wValue = UEDATX;
		wValue |= (UEDATX << 8);
		wIndex = UEDATX;
		wIndex |= (UEDATX << 8);
		wLength = UEDATX;
		wLength |= (UEDATX << 8);
		UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
		if (bRequest == GET_DESCRIPTOR) {
			list = (const uint8_t *)descriptor_list;
			for (i=0; ; i++) {
				if (i >= NUM_DESC_LIST) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
					return;
				}
				desc_val = pgm_read_word(list);
				if (desc_val != wValue) {
					list += sizeof(descriptor_list_struct);
					continue;
				}
				list += 2;
				desc_val = pgm_read_word(list);
				if (desc_val != wIndex) {
					list += sizeof(descriptor_list_struct)-2;
					continue;
				}
				list += 2;
				desc_addr = (const uint8_t *)pgm_read_word(list);
				list += 2;
				desc_length = pgm_read_byte(list);
				break;
			}
			len = (wLength < 256) ? wLength : 255;
			if (len > desc_length) len = desc_length;
			do {
				// wait for host ready for IN packet
				do {
					i = UEINTX;
				} while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
				if (i & (1<<RXOUTI)) return;	// abort
				// send IN packet
				n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (i = n; i; i--) {
					UEDATX = pgm_read_byte(desc_addr++);
				}
				len -= n;
				usb_send_in();
			} while (len || n == ENDPOINT0_SIZE);
			return;
		}
		if (bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = wValue | (1<<ADDEN);
			return;
		}
		if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
			usb_configuration = wValue;
			usb_send_in();
			cfg = endpoint_config_table;
			for (i=1; i<5; i++) {
				UENUM = i;
				en = pgm_read_byte(cfg++);
				UECONX = en;
				if (en) {
					UECFG0X = pgm_read_byte(cfg++);
					UECFG1X = pgm_read_byte(cfg++);
				}
			}
			UERST = 0x1E;
			UERST = 0;
			return;
		}
		if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}

		if (bRequest == GET_STATUS) {
			usb_wait_in_ready();
			i = 0;
			#ifdef SUPPORT_ENDPOINT_HALT
			if (bmRequestType == 0x82) {
				UENUM = wIndex;
				if (UECONX & (1<<STALLRQ)) i = 1;
				UENUM = 0;
			}
			#endif
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
		#ifdef SUPPORT_ENDPOINT_HALT
		if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
		  && bmRequestType == 0x02 && wValue == 0) {
			i = wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				UENUM = i;
				if (bRequest == SET_FEATURE) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);
				} else {
					UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
					UERST = (1 << i);
					UERST = 0;
				}
				return;
			}
		}
		#endif
		if (wIndex == GAMEPAD_INTERFACE) {
			if (bmRequestType == 0xA1) {
				if (bRequest == HID_GET_REPORT) {
					usb_wait_in_ready();

					for (i=0; i<sizeof(magic_init_bytes); i++) {
						UEDATX = pgm_read_byte(&magic_init_bytes[i]);
					}

					usb_send_in();
					return;
				}
				if (bRequest == HID_GET_IDLE) {
					usb_wait_in_ready();
					UEDATX = gamepad_idle_config;
					usb_send_in();
					return;
				}
				if (bRequest == HID_GET_PROTOCOL) {
					usb_wait_in_ready();
					UEDATX = gamepad_protocol;
					usb_send_in();
					return;
				}
			}
			if (bmRequestType == 0x21) {
				if (bRequest == HID_SET_REPORT) {
					usb_wait_receive_out();
					usb_ack_out();
					usb_send_in();
					return;
				}
				if (bRequest == HID_SET_IDLE) {
					gamepad_idle_config = (wValue >> 8);
					usb_send_in();
					return;
				}
				if (bRequest == HID_SET_PROTOCOL) {
					gamepad_protocol = wValue;
					usb_send_in();
					return;
				}
			}
		}
	}
	UECONX = (1<<STALLRQ) | (1<<EPEN);	// stall
}

/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/
ISR(USB_GEN_vect)
{
    AP_Joypad_USB::fire_isr_usb_genvect();
}

ISR(USB_COM_vect)
{
    AP_Joypad_USB::fire_isr_usb_comvect();
}

#endif

