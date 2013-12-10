// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GPS.h
/// @brief	Interface definition for the various GPS drivers.

#ifndef __GPS_H__
#define __GPS_H__

#include <AP_HAL.h>

#include <inttypes.h>
#include <AP_Progmem.h>
#include <AP_Math.h>

/// @class	GPS
/// @brief	Abstract base class for GPS receiver drivers.
class GPS
{
public:
	GPS();

    /// Update GPS state based on possible bytes received from the module.
    ///
    /// This routine must be called periodically to process incoming data.
    ///
    /// GPS drivers should not override this function; they should implement
    /// ::read instead.
    ///
    void                        update(void);

    /// GPS status codes
    ///
    /// \note Non-intuitive ordering for legacy reasons
    ///
    enum GPS_Status {
        NO_GPS = 0,             ///< No GPS connected/detected
        NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3       ///< Receiving valid messages and 3D lock
    };

    /// Fix status codes
    ///
    enum Fix_Status {
        FIX_NONE = 0,           ///< No fix
        FIX_2D = 2,             ///< 2d fix
        FIX_3D = 3,             ///< 3d fix
    };

    // GPS navigation engine settings. Not all GPS receivers support
    // this
    enum GPS_Engine_Setting {
        GPS_ENGINE_NONE        = -1,
        GPS_ENGINE_PORTABLE    = 0,
        GPS_ENGINE_STATIONARY  = 2,
        GPS_ENGINE_PEDESTRIAN  = 3,
        GPS_ENGINE_AUTOMOTIVE  = 4,
        GPS_ENGINE_SEA         = 5,
        GPS_ENGINE_AIRBORNE_1G = 6,
        GPS_ENGINE_AIRBORNE_2G = 7,
        GPS_ENGINE_AIRBORNE_4G = 8
    };

    /// Query GPS status
    ///
    /// The 'valid message' status indicates that a recognised message was
    /// received from the GPS within the last 500ms.
    ///
    /// @returns			Current GPS status
    ///
    GPS_Status          status(void) const {
        return _status;
    }

    /// Startup initialisation.
    ///
    /// This routine performs any one-off initialisation required to set the
    /// GPS up for use.
    ///
    /// Must be implemented by the GPS driver.
    ///
    virtual void        init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting engine_setting = GPS_ENGINE_NONE) = 0;

    // Properties
    uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
    uint16_t time_week;                 ///< GPS week number
    int32_t latitude;                   ///< latitude in degrees * 10,000,000
    int32_t longitude;                  ///< longitude in degrees * 10,000,000
    int32_t altitude_cm;                ///< altitude in cm
    uint32_t ground_speed_cm;           ///< ground speed in cm/sec
    int32_t ground_course_cd;           ///< ground course in 100ths of a degree
    int32_t speed_3d_cm;                ///< 3D speed in cm/sec (not always available)
    int16_t hdop;                       ///< horizontal dilution of precision in cm
    uint8_t num_sats;           ///< Number of visible satelites

    /// Set to true when new data arrives.  A client may set this
    /// to false in order to avoid processing data they have
    /// already seen.
    bool new_data;

    Fix_Status fix;                        ///< 0 if we have no fix, 2 for 2D fix, 3 for 3D fix
    bool valid_read;                    ///< true if we have seen data from the GPS (use ::status instead)

    // Debug support
    bool print_errors;          ///< deprecated

    // HIL support
    virtual void setHIL(uint64_t time_epoch_ms, float latitude, float longitude, float altitude,
                        float ground_speed, float ground_course, float speed_3d, uint8_t num_sats);

    // components of velocity in 2D, in m/s
    float velocity_north(void) const {
        return _status >= GPS_OK_FIX_2D ? _velocity_north : 0;
    }
    float velocity_east(void)  const {
        return _status >= GPS_OK_FIX_2D ? _velocity_east  : 0;
    }
    float velocity_down(void)  const {
        return _status >= GPS_OK_FIX_3D ? _velocity_down  : 0;
    }

    // GPS velocity vector as NED in m/s
    Vector3f velocity_vector(void) const {
        return Vector3f(_velocity_north, _velocity_east, _velocity_down);
    }

    // last ground speed in m/s. This can be used when we have no GPS
    // lock to return the last ground speed we had with lock
    float last_ground_speed(void) {
        return static_cast<float>(_last_ground_speed_cm) * 0.01f;
    }

    // the expected lag (in seconds) in the position and velocity readings from the gps
    virtual float get_lag() { return 1.0f; }

    // the time we got our last fix in system milliseconds
    uint32_t last_fix_time;

	// the time we last processed a message in milliseconds
	uint32_t last_message_time_ms(void) { return _idleTimer; }

    // return last fix time since the 1/1/1970 in microseconds
    uint64_t time_epoch_usec(void);

	// return true if the GPS supports raw velocity values


protected:
    AP_HAL::UARTDriver *_port;   ///< port the GPS is attached to

    /// read from the GPS stream and update properties
    ///
    /// Must be implemented by the GPS driver.
    ///
    /// @returns			true if a valid message was received from the GPS
    ///
    virtual bool        read(void) = 0;

    /// perform an endian swap on a long
    ///
    /// @param	bytes		pointer to a buffer containing bytes representing a
    ///						long in the wrong byte order
    /// @returns			endian-swapped value
    ///
    int32_t                             _swapl(const void *bytes) const;

    /// perform an endian swap on an int
    ///
    /// @param	bytes		pointer to a buffer containing bytes representing an
    ///						int in the wrong byte order
    ///	@returns			endian-swapped value
    int16_t                             _swapi(const void *bytes) const;

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
    void                        _error(const char *msg);

    enum GPS_Engine_Setting _nav_setting;

    void _write_progstr_block(AP_HAL::UARTDriver *_fs, const prog_char *pstr, uint8_t size);
    void _send_progstr(AP_HAL::UARTDriver *_fs, const prog_char *pstr, uint8_t size);
    void _update_progstr(void);

    // velocities in cm/s if available from the GPS
    int32_t _vel_north;
    int32_t _vel_east;
    int32_t _vel_down;

    // does this GPS support raw velocity numbers?
    bool _have_raw_velocity;

	// detected baudrate
	uint16_t _baudrate;

    // the time we got the last GPS timestamp
    uint32_t _last_gps_time;

    // return time in seconds since GPS epoch given time components
    void _make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);

private:


    /// Last time that the GPS driver got a good packet from the GPS
    ///
    uint32_t _idleTimer;

    /// Our current status
    GPS_Status _status;

    // previous ground speed in cm/s
    uint32_t _last_ground_speed_cm;

    // components of the velocity, in m/s
    float _velocity_north;
    float _velocity_east;
    float _velocity_down;
};

#endif // __GPS_H__
