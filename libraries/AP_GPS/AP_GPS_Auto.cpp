// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_GPS_Auto.cpp
/// @brief	Simple GPS auto-detection logic.

#include <FastSerial.h>
#include <AP_Common.h>

#include "AP_GPS.h"             // includes AP_GPS_Auto.h

static const uint32_t baudrates[] PROGMEM = {38400U, 57600U, 9600U, 4800U};

const prog_char AP_GPS_Auto::_mtk_set_binary[]   PROGMEM = MTK_SET_BINARY;
const prog_char AP_GPS_Auto::_sirf_set_binary[]  PROGMEM = SIRF_SET_BINARY;


AP_GPS_Auto::AP_GPS_Auto(FastSerial *s, GPS **gps)  :
    GPS(s),
    _fs(s),
    _gps(gps)
{
}

// Do nothing at init time - it may be too early to try detecting the GPS
//
void
AP_GPS_Auto::init(enum GPS_Engine_Setting nav_setting)
{
    idleTimeout = 1200;
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
	uint32_t now = millis();

	if (now - last_baud_change_ms > 1200) {
		// its been more than 1.2 seconds without detection on this
		// GPS - switch to another baud rate
		_fs->begin(pgm_read_dword(&baudrates[last_baud]), 256, 16);		
		last_baud++;
		last_baud_change_ms = now;
		if (last_baud == sizeof(baudrates) / sizeof(baudrates[0])) {
			last_baud = 0;
		}
		// write config strings for the types of GPS we support
		_write_progstr_block(_fs, _mtk_set_binary, sizeof(_mtk_set_binary));
		_write_progstr_block(_fs, AP_GPS_UBLOX::_ublox_set_binary, AP_GPS_UBLOX::_ublox_set_binary_size);
		_write_progstr_block(_fs, _sirf_set_binary, sizeof(_sirf_set_binary));
	}

	if (NULL != (gps = _detect())) {
		// configure the detected GPS
		gps->init(_nav_setting);
		Serial.println_P(PSTR("OK"));
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

	if (detect_started_ms == 0) {
		detect_started_ms = millis();
	}

	while (_port->available() > 0) {
		uint8_t data = _port->read();
		if (AP_GPS_UBLOX::_detect(data)) {
			Serial.print_P(PSTR(" ublox "));
			return new AP_GPS_UBLOX(_port);
		}
		if (AP_GPS_MTK16::_detect(data)) {
			Serial.print_P(PSTR(" MTK16 "));
			return new AP_GPS_MTK16(_port);
		}
		if (AP_GPS_MTK::_detect(data)) {
			Serial.print_P(PSTR(" MTK "));
			return new AP_GPS_MTK(_port);
		}
		if (AP_GPS_SIRF::_detect(data)) {
			Serial.print_P(PSTR(" SIRF "));
			return new AP_GPS_SIRF(_port);
		}
		if (millis() - detect_started_ms > 5000) {
			// prevent false detection of NMEA mode in
			// a MTK or UBLOX which has booted in NMEA mode
			if (AP_GPS_NMEA::_detect(data)) {
				Serial.print_P(PSTR(" NMEA "));
				return new AP_GPS_NMEA(_port);
			}
		}
	}

	return NULL;
}

