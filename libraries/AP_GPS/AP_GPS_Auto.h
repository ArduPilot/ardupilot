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
	AP_GPS_Auto(FastSerial *port, GPS **gps);

	/// Dummy init routine, does nothing
	virtual void		init(void);

	/// Detect and initialise the attached GPS unit.  Updates the
	/// pointer passed into the constructor when a GPS is detected.
	///
	virtual bool 	read(void);

private:
	/// Serial port connected to the GPS.
	FastSerial	*_FSport;

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
