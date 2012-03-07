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

    /// Update GPS state based on possible bytes received from the module.
    ///
    /// This routine must be called periodically to process incoming data.
    ///
    /// GPS drivers should not override this function; they should implement
    /// ::read instead.
    ///
    void			update(void);

    void (*callback)(unsigned long t);

    /// GPS status codes
    ///
    /// \note Non-intuitive ordering for legacy reasons
    ///
    enum GPS_Status {
        NO_GPS = 0,		///< No GPS connected/detected
        NO_FIX = 1,		///< Receiving valid GPS messages but no lock
        GPS_OK = 2		///< Receiving valid messages and locked
    };

    /// Query GPS status
    ///
    /// The 'valid message' status indicates that a recognised message was
    /// received from the GPS within the last 500ms.
    ///
    /// @returns			Current GPS status
    ///
    GPS_Status		status(void) {
        return _status;
    }

    /// GPS time epoch codes
    ///
    enum	GPS_Time_Epoch {
        TIME_OF_DAY 	= 0,		///<
        TIME_OF_WEEK 	= 1,		///< Ublox
        TIME_OF_YEAR 	= 2,		///< MTK, NMEA
        UNIX_EPOCH	 	= 3			///< If available
    };									///< SIFR?


    /// Query GPS time epoch
    ///
    /// @returns			Current GPS time epoch code
    ///
    GPS_Time_Epoch		epoch(void) {
        return _epoch;
    }

    /// Startup initialisation.
    ///
    /// This routine performs any one-off initialisation required to set the
    /// GPS up for use.
    ///
    /// Must be implemented by the GPS driver.
    ///
    virtual void	init(void) = 0;

    // Properties
    long	time;			///< GPS time (milliseconds from epoch)
    long	date;			///< GPS date (FORMAT TBD)
    long	latitude;		///< latitude in degrees * 10,000,000
    long	longitude;		///< longitude in degrees * 10,000,000
    long	altitude;		///< altitude in cm
    long	ground_speed;	///< ground speed in cm/sec
    long	ground_course;	///< ground course in 100ths of a degree
    long	speed_3d;		///< 3D speed in cm/sec (not always available)
    int		hdop;			///< horizontal dilution of precision in cm
    uint8_t num_sats;		///< Number of visible satelites

    /// Set to true when new data arrives.  A client may set this
    /// to false in order to avoid processing data they have
    /// already seen.
    bool	new_data;

    // Deprecated properties
    bool	fix;			///< true if we have a position fix (use ::status instead)
    bool	valid_read;		///< true if we have seen data from the GPS (use ::status instead)

    // Debug support
    bool	print_errors; 	///< deprecated

    // HIL support
    virtual void setHIL(long time, float latitude, float longitude, float altitude,
                        float ground_speed, float ground_course, float speed_3d, uint8_t num_sats);

    /// Time in milliseconds after which we will assume the GPS is no longer
    /// sending us updates and attempt a re-init.
    ///
    /// 1200ms allows a small amount of slack over the worst-case 1Hz update
    /// rate.
    ///
    uint32_t	idleTimeout;

	// our approximate linear acceleration in m/s/s
	float acceleration(void) { return _acceleration; }

	// the time we got our last fix in system milliseconds
	uint32_t last_fix_time;

protected:
    Stream	*_port;			///< port the GPS is attached to

    /// Constructor
    ///
    /// @note The stream is expected to be set up and configured for the
    ///       correct bitrate before ::init is called.
    ///
    /// @param	s	Stream connected to the GPS module.
    ///
    GPS(Stream *s) : _port(s) {};

    /// read from the GPS stream and update properties
    ///
    /// Must be implemented by the GPS driver.
    ///
    /// @returns			true if a valid message was received from the GPS
    ///
    virtual bool	read(void) = 0;

    /// perform an endian swap on a long
    ///
    /// @param	bytes		pointer to a buffer containing bytes representing a
    ///						long in the wrong byte order
    /// @returns			endian-swapped value
    ///
    long			_swapl(const void *bytes);

    /// perform an endian swap on an int
    ///
    /// @param	bytes		pointer to a buffer containing bytes representing an
    ///						int in the wrong byte order
    ///	@returns			endian-swapped value
    int16_t				_swapi(const void *bytes);

    /// emit an error message
    ///
    /// based on the value of print_errors, emits the printf-formatted message
    /// in msg,... to stderr
    ///
    /// @param	fmt			printf-like format string
    ///
    /// @note deprecated as-is due to the difficulty of hooking up to a working
    ///       printf vs. the potential benefits
    ///
    void			_error(const char *msg);

    /// Time epoch code for the gps in use
    GPS_Time_Epoch				_epoch;

private:


    /// Last time that the GPS driver got a good packet from the GPS
    ///
    uint32_t 				_idleTimer;

    /// Our current status
    GPS_Status					_status;

	// previous ground speed in cm/s
    uint32_t _last_ground_speed;

	// smoothed estimate of our acceleration
	float _acceleration;
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
