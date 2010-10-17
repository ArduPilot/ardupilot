// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GPS.h
/// @brief	Interface definition for the various GPS drivers.

#ifndef GPS_h
#define GPS_h

#include <inttypes.h>
#include <Stream.h>

/// @class	GPS
/// @brief	Abstract base class for GPS receiver drivers.
class GPS
{
public:

	/// Constructor
	///
	/// @note The stream is expected to be set up and configured for the
	///       correct bitrate before ::init is called.
	///
	/// @param	s	Stream connected to the GPS module.  If NULL, assumed
	///				to be set up at ::init time.  Support for setting 
	///				the port in the ctor for backwards compatibility.
	///
	GPS(Stream *s = NULL) : _port(s) {};

	/// Startup initialisation.
	///
	/// This routine performs any one-off initialisation required to set the
	/// GPS up for use.
	///
	/// @param	s	Stream connected to the GPS module.  If NULL, assumed to
	///				have been set up at constructor time.
	///
	virtual void	init(void) = 0;

	/// Update GPS state based on possible bytes received from the module.
	///
	/// This routine must be called periodically to process incoming data.
	///
	virtual void	update(void) = 0;

	// Properties
	long	time;			///< GPS time in milliseconds from the start of the week
	long	latitude;		///< latitude in degrees * 10,000,000
	long	longitude;		///< longitude in degrees * 10,000,000
	long	altitude;		///< altitude in cm
	long	ground_speed;	///< ground speed in cm/sec
	long	ground_course;	///< ground course in 100ths of a degree
	long	speed_3d;		///< 3D speed in cm/sec (not always available)
	uint8_t num_sats;		///< Number of visible satelites
	bool	fix;			///< true if we have a position fix
	
	/// Set to true when new data arrives.  A client may set this
	/// to false in order to avoid processing data they have
	/// already seen.
	bool	new_data;
	bool	valid_read;

	int	status(void);		///< 0 = Not Connected, 1 = Receiving Valid Messages but No Lock, 2 = Locked

	bool	print_errors; 	///< if true, errors will be printed to stderr

protected:
	Stream	*_port;			///< stream port the GPS is attached to

	/// perform an endian swap on a long
	///
	/// @param	bytes		pointer to a buffer containing bytes representing a
	///						long in the wrong byte order
	/// @returns			endian-swapped value
	///
	long	_swapl(const void *bytes);

	unsigned long _lastTime;	///< Timer for lost connection

	void	_setTime(void);

	/// perform an endian swap on an int
	///
	/// @param	bytes		pointer to a buffer containing bytes representing an
	///						int in the wrong byte order
	///	@returns			endian-swapped value
	int		_swapi(const void *bytes);

	/// emit an error message
	///
	/// based on the value of print_errors, emits the printf-formatted message
	/// in msg,... to stderr
	///
	/// @param	fmt			printf-like format string
	///
	void	_error(const char *msg, ...);
	
};

inline long
GPS::_swapl(const void *bytes)
{
	const uint8_t	*b = (const uint8_t *)bytes;
	union {
		long	v;
		uint8_t b[4];
	} u;

	u.b[0] = b[3];
	u.b[1] = b[2];
	u.b[2] = b[1];
	u.b[3] = b[0];

	return(u.v);
}

inline int16_t
GPS::_swapi(const void *bytes)
{
	const uint8_t	*b = (const uint8_t *)bytes;
	union {
		int16_t	v;
		uint8_t b[2];
	} u;

	u.b[0] = b[1];
	u.b[1] = b[0];

	return(u.v);
}

#endif
