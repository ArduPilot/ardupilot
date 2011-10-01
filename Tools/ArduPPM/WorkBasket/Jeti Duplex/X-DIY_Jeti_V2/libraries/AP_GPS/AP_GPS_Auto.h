// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Auto-detecting pseudo-GPS driver
//

#ifndef AP_GPS_Auto_h
#define AP_GPS_Auto_h

#include <GPS.h>
#include <FastSerial.h>

class AP_GPS_Auto : public GPS
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
	AP_GPS_Auto(FastSerial *port, GPS **gps) : 
		_port(port),
		_gps(gps)
	{};

	void		init(void);

	/// Detect and initialise the attached GPS unit.  Returns a
	/// pointer to the allocated & initialised GPS driver.
	///
	void		update(void);

private:
	/// Serial port connected to the GPS.
	FastSerial	*_port;

	/// global GPS driver pointer, updated by auto-detection
	///
	GPS			**_gps;

	/// low-level auto-detect routine
	///
	GPS			*_detect(void);

	/// fetch a character from the port
	///
	int			_getc(void);
};
#endif
