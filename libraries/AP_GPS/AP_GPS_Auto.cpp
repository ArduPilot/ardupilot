// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_GPS_Auto.cpp
/// @brief	Simple GPS auto-detection logic.

#include <AP_HAL.h>
#include <AP_Common.h>

#include "AP_GPS.h"             // includes AP_GPS_Auto.h

extern const AP_HAL::HAL& hal;

static const uint32_t baudrates[] PROGMEM = {38400U, 57600U, 9600U, 4800U};

const prog_char AP_GPS_Auto::_mtk_set_binary[]   PROGMEM = MTK_SET_BINARY;
const prog_char AP_GPS_Auto::_sirf_set_binary[]  PROGMEM = SIRF_SET_BINARY;


AP_GPS_Auto::AP_GPS_Auto(GPS **gps)  :
	GPS(),
    _gps(gps)
{
}

// Do nothing at init time - it may be too early to try detecting the GPS
//
void
AP_GPS_Auto::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
    _nav_setting = nav_setting;
}


// Called the first time that a client tries to kick the GPS to update.
//
// We detect the real GPS, then update the pointer we have been called through
// and return.
//
bool
AP_GPS_Auto::read(void)
{
	static uint32_t last_baud_change_ms;
	static uint8_t last_baud;
	GPS *gps;
	uint32_t now = hal.scheduler->millis();

	if (now - last_baud_change_ms > 1200) {
		// its been more than 1.2 seconds without detection on this
		// GPS - switch to another baud rate
		_baudrate = pgm_read_dword(&baudrates[last_baud]);
		//hal.console->printf_P(PSTR("Setting GPS baudrate %u\n"), (unsigned)_baudrate);
		_port->begin(_baudrate, 256, 16);		
		last_baud++;
		last_baud_change_ms = now;
		if (last_baud == sizeof(baudrates) / sizeof(baudrates[0])) {
			last_baud = 0;
		}
		// write config strings for the types of GPS we support
		_send_progstr(_port, _mtk_set_binary, sizeof(_mtk_set_binary));
		_send_progstr(_port, AP_GPS_UBLOX::_ublox_set_binary, AP_GPS_UBLOX::_ublox_set_binary_size);
		_send_progstr(_port, _sirf_set_binary, sizeof(_sirf_set_binary));
	}

	_update_progstr();

	if (NULL != (gps = _detect())) {
		// configure the detected GPS
		gps->init(_port, _nav_setting);
		hal.console->println_P(PSTR("OK"));
		*_gps = gps;
		return true;
    }
    return false;
}

//
// Perform one iteration of the auto-detection process.
//
GPS *
AP_GPS_Auto::_detect(void)
{
	static uint32_t detect_started_ms;
	GPS *new_gps = NULL;

	if (detect_started_ms == 0 && _port->available() > 0) {
		detect_started_ms = hal.scheduler->millis();
	}

	while (_port->available() > 0 && new_gps == NULL) {
		uint8_t data = _port->read();
		/*
		  running a uBlox at less than 38400 will lead to packet
		  corruption, as we can't receive the packets in the 200ms
		  window for 5Hz fixes. The NMEA startup message should force
		  the uBlox into 38400 no matter what rate it is configured
		  for.
		 */
		if (_baudrate >= 38400 && AP_GPS_UBLOX::_detect(data)) {
			hal.console->print_P(PSTR(" ublox "));
			new_gps = new AP_GPS_UBLOX();
		} 
		else if (AP_GPS_MTK19::_detect(data)) {
			hal.console->print_P(PSTR(" MTK19 "));
			new_gps = new AP_GPS_MTK19();
		} 
		else if (AP_GPS_MTK::_detect(data)) {
			hal.console->print_P(PSTR(" MTK "));
			new_gps = new AP_GPS_MTK();
		}
#if !defined( __AVR_ATmega1280__ )
		// save a bit of code space on a 1280
		else if (AP_GPS_SIRF::_detect(data)) {
			hal.console->print_P(PSTR(" SIRF "));
			new_gps = new AP_GPS_SIRF();
		}
		else if (hal.scheduler->millis() - detect_started_ms > 5000) {
			// prevent false detection of NMEA mode in
			// a MTK or UBLOX which has booted in NMEA mode
			if (AP_GPS_NMEA::_detect(data)) {
				hal.console->print_P(PSTR(" NMEA "));
				new_gps = new AP_GPS_NMEA();
			}
		}
#endif
	}

	if (new_gps != NULL) {
		new_gps->init(_port);
	}

	return new_gps;
}




