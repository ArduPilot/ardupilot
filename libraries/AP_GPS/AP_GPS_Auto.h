// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Auto-detecting pseudo-GPS driver
//

#ifndef AP_GPS_Auto_h
#define AP_GPS_Auto_h

#include <GPS.h>
#include <FastSerial.h>

class AP_GPS_Auto
{
public:
	/// Constructor
	///
	/// @note The stream is expected to be set up and configured for the
	///       correct bitrate before ::init is called.
	///
	/// @param	port	Stream connected to the GPS module.
	/// @param	ptr		Pointer to a GPS * that will be fixed up by ::init
	///					when the GPS type has been detected.
	///
	AP_GPS_Auto(FastSerial *port = NULL) : _port(port)	{};

	/// Detect and initialise the attached GPS unit.  Returns a
	/// pointer to the allocated & initialised GPS driver.
	///
	GPS		*detect(void);

private:
	/// Serial port connected to the GPS.
	FastSerial	*_port;

	/// low-level auto-detect routine
	///
	GPS			*_detect(void);

	/// fetch a character from the port
	///
	int			_getc(void);
};
#endif
