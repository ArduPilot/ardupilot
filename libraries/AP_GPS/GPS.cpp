// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <FastSerial.h>

#define GPS_DEBUGGING 0

#if GPS_DEBUGGING
 #include <FastSerial.h>
 # define Debug(fmt, args ...)  do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); delay(0); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

#include <AP_Common.h>
#include <AP_Math.h>
#include "GPS.h"
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

void
GPS::update(void)
{
    bool result;
    uint32_t tnow;

    // call the GPS driver to process incoming data
    result = read();

    tnow = millis();

    // if we did not get a message, and the idle timer has expired, re-init
    if (!result) {
        if ((tnow - _idleTimer) > idleTimeout) {
            Debug("gps read timeout %lu %lu", (unsigned long)tnow, (unsigned long)_idleTimer);
            _status = NO_GPS;

            init(_nav_setting);
            // reset the idle timer
            _idleTimer = tnow;
        }
    } else {
        // we got a message, update our status correspondingly
        _status = fix ? GPS_OK : NO_FIX;

        valid_read = true;
        new_data = true;

        // reset the idle timer
        _idleTimer = tnow;

        if (_status == GPS_OK) {
            last_fix_time = _idleTimer;
            _last_ground_speed_cm = ground_speed;

            if (_have_raw_velocity) {
                // the GPS is able to give us velocity numbers directly
                _velocity_north = _vel_north * 0.01;
                _velocity_east  = _vel_east * 0.01;
                _velocity_down  = _vel_down * 0.01;
            } else {
                float gps_heading = ToRad(ground_course * 0.01);
                float gps_speed   = ground_speed * 0.01;
                float sin_heading, cos_heading;

                cos_heading = cos(gps_heading);
                sin_heading = sin(gps_heading);

                _velocity_north = gps_speed * cos_heading;
                _velocity_east  = gps_speed * sin_heading;

				// no good way to get descent rate
				_velocity_down  = 0;
            }
        }
    }
}

void
GPS::setHIL(uint32_t _time, float _latitude, float _longitude, float _altitude,
            float _ground_speed, float _ground_course, float _speed_3d, uint8_t _num_sats)
{
}

// XXX this is probably the wrong way to do it, too
void
GPS::_error(const char *msg)
{
    Serial.println(msg);
}

///
/// write a block of configuration data to a GPS
///
void GPS::_write_progstr_block(Stream *_fs, const prog_char *pstr, uint8_t size)
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
    FastSerial *fs;
	uint8_t queue_size;
	uint8_t idx, next_idx;
	struct progstr_queue queue[PROGSTR_QUEUE_SIZE];
} progstr_state;

void GPS::_send_progstr(Stream *_fs, const prog_char *pstr, uint8_t size)
{
	progstr_state.fs = (FastSerial *)_fs;
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
