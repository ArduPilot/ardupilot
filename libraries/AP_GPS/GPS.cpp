// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-



#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include "GPS.h"

extern const AP_HAL::HAL& hal;

#define GPS_DEBUGGING 0

#if GPS_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(0); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

GPS::GPS(void) :
	// ensure all the inherited fields are zeroed
	time_week_ms(0),
    time_week(0),
    latitude(0),
    longitude(0),
    altitude_cm(0),
    ground_speed_cm(0),
    ground_course_cd(0),
    speed_3d_cm(0),
    hdop(0),
	num_sats(0),
	new_data(false),
	fix(FIX_NONE),
	valid_read(false),
	last_fix_time(0),
	_have_raw_velocity(false),
    _last_gps_time(0),
    _secondary_gps(false),
	_idleTimer(0),
	_status(GPS::NO_FIX),
	_last_ground_speed_cm(0),
	_velocity_north(0),
	_velocity_east(0),
	_velocity_down(0)
{
}

void
GPS::update(void)
{
    bool result;
    uint32_t tnow;

    // call the GPS driver to process incoming data
    result = read();

    tnow = hal.scheduler->millis();

    // if we did not get a message, and the idle timer of 1.2 seconds has expired, re-init
    if (!result) {
        if ((tnow - _idleTimer) > 1200) {
            Debug("gps read timeout %lu %lu", (unsigned long)tnow, (unsigned long)_idleTimer);
            _status = NO_GPS;

            init(_port, _nav_setting);
            // reset the idle timer
            _idleTimer = tnow;
        }
    } else {
        // we got a message, update our status correspondingly
        if (fix == FIX_3D) {
            _status = GPS_OK_FIX_3D;
        }else if (fix == FIX_2D) {
            _status = GPS_OK_FIX_2D;
        }else{
            _status = NO_FIX;
        }

        valid_read = true;
        new_data = true;

        // reset the idle timer
        _idleTimer = tnow;

        if (_status >= GPS_OK_FIX_2D) {
            last_fix_time = _idleTimer;
            _last_ground_speed_cm = ground_speed_cm;

            if (_have_raw_velocity) {
                // the GPS is able to give us velocity numbers directly
                _velocity_north = _vel_north * 0.01f;
                _velocity_east  = _vel_east * 0.01f;
                _velocity_down  = _vel_down * 0.01f;
            } else {
                float gps_heading = ToRad(ground_course_cd * 0.01f);
                float gps_speed   = ground_speed_cm * 0.01f;
                float sin_heading, cos_heading;

                cos_heading = cosf(gps_heading);
                sin_heading = sinf(gps_heading);

                _velocity_north = gps_speed * cos_heading;
                _velocity_east  = gps_speed * sin_heading;

				// no good way to get descent rate
				_velocity_down  = 0;
            }
        }
    }

    if (!_secondary_gps) {
        // update notify with gps status
        AP_Notify::flags.gps_status = _status;
    }
}

void
GPS::setHIL(Fix_Status fix_status,
            uint64_t _time_epoch_ms, float _latitude, float _longitude, float _altitude,
            float _ground_speed, float _ground_course, float _speed_3d, uint8_t _num_sats)
{
}

// XXX this is probably the wrong way to do it, too
void
GPS::_error(const char *msg)
{
    hal.console->println(msg);
}

///
/// write a block of configuration data to a GPS
///
void GPS::_write_progstr_block(AP_HAL::UARTDriver *_fs, const prog_char *pstr, uint8_t size)
{
    while (size--) {
        _fs->write(pgm_read_byte(pstr++));
    }
}

/*
  a prog_char block queue, used to send out config commands to a GPS
  in 16 byte chunks. This saves us having to have a 128 byte GPS send
  buffer, while allowing us to avoid a long delay in sending GPS init
  strings while waiting for the GPS auto detection to happen
 */

// maximum number of pending progstrings
#define PROGSTR_QUEUE_SIZE 3

struct progstr_queue {
	const prog_char *pstr;
	uint8_t ofs, size;
};

static struct {
    AP_HAL::UARTDriver *fs;
	uint8_t queue_size;
	uint8_t idx, next_idx;
	struct progstr_queue queue[PROGSTR_QUEUE_SIZE];
} progstr_state;

void GPS::_send_progstr(AP_HAL::UARTDriver *_fs, const prog_char *pstr, uint8_t size)
{
	progstr_state.fs = _fs;
	struct progstr_queue *q = &progstr_state.queue[progstr_state.next_idx];
	q->pstr = pstr;
	q->size = size;
	q->ofs = 0;
	progstr_state.next_idx++;
	if (progstr_state.next_idx == PROGSTR_QUEUE_SIZE) {
		progstr_state.next_idx = 0;
	}
}

void GPS::_update_progstr(void)
{
	struct progstr_queue *q = &progstr_state.queue[progstr_state.idx];
	// quick return if nothing to do
	if (q->size == 0 || progstr_state.fs->tx_pending()) {
		return;
	}
	uint8_t nbytes = q->size - q->ofs;
	if (nbytes > 16) {
		nbytes = 16;
	}
	//hal.console->printf_P(PSTR("writing %u bytes\n"), (unsigned)nbytes);
	_write_progstr_block(progstr_state.fs, q->pstr+q->ofs, nbytes);
	q->ofs += nbytes;
	if (q->ofs == q->size) {
		q->size = 0;
		progstr_state.idx++;
		if (progstr_state.idx == PROGSTR_QUEUE_SIZE) {
			progstr_state.idx = 0;
		}
	}
}

int32_t GPS::_swapl(const void *bytes) const
{
    const uint8_t       *b = (const uint8_t *)bytes;
    union {
        int32_t v;
        uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return(u.v);
}

int16_t GPS::_swapi(const void *bytes) const
{
    const uint8_t       *b = (const uint8_t *)bytes;
    union {
        int16_t v;
        uint8_t b[2];
    } u;

    u.b[0] = b[1];
    u.b[1] = b[0];

    return(u.v);
}

/**
   current time since the unix epoch in microseconds

   This costs about 60 usec on AVR2560
 */
uint64_t GPS::time_epoch_usec(void)
{
    if (_last_gps_time == 0) {
        return 0;
    }
    const uint64_t ms_per_week = 7000ULL*86400ULL;
    const uint64_t unix_offset = 17000ULL*86400ULL + 52*10*7000ULL*86400ULL - 15000ULL;
    uint64_t fix_time_ms = unix_offset + time_week*ms_per_week + time_week_ms;
    // add in the milliseconds since the last fix
    return (fix_time_ms + (hal.scheduler->millis() - _last_gps_time)) * 1000ULL;
}


/**
   fill in time_week_ms and time_week from BCD date and time components
   assumes MTK19 millisecond form of bcd_time

   This function takes about 340 usec on the AVR2560
 */
void GPS::_make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds)
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;

    year = bcd_date % 100;
    mon  = (bcd_date / 100) % 100;
    day  = bcd_date / 10000;
    msec = bcd_milliseconds % 1000;

    uint32_t v = bcd_milliseconds;
    msec = v % 1000; v /= 1000;
    sec  = v % 100; v /= 100;
    min  = v % 100; v /= 100;
    hour = v % 100; v /= 100;

    int8_t rmon = mon - 2;
    if (0 >= rmon) {    
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    uint32_t ret = (year/4) - 15 + 367*rmon/12 + day;
    ret += year*365 + 10501;
    ret = ret*24 + hour;
    ret = ret*60 + min;
    ret = ret*60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    time_week = ret / (7*86400UL);
    time_week_ms = (ret % (7*86400UL)) * 1000;
    time_week_ms += msec;
}
