/*
  SITL handling

  This simulates a GPS on a serial port

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <SITL.h>
#include <AP_GPS.h>
#include <AP_GPS_UBLOX.h>
#include "desktop.h"
#include "util.h"

extern SITL sitl;

#define MAX_GPS_DELAY 100

struct gps_data {
	double latitude;
	double longitude;
	float altitude;
	double speedN;
	double speedE;
	bool have_lock;
} gps_data[MAX_GPS_DELAY];

static uint8_t next_gps_index;
static uint8_t gps_delay;

// state of GPS emulation
static struct {
	/* pipe emulating UBLOX GPS serial stream */
	int gps_fd, client_fd;
	uint32_t last_update; // milliseconds
} gps_state;

/*
  hook for reading from the GPS pipe
 */
ssize_t sitl_gps_read(int fd, void *buf, size_t count)
{
	return read(fd, buf, count);
}

/*
  setup GPS input pipe
 */
int sitl_gps_pipe(void)
{
	int fd[2];
	if (gps_state.client_fd != 0) {
		return gps_state.client_fd;
	}
	pipe(fd);
	gps_state.gps_fd    = fd[1];
	gps_state.client_fd = fd[0];
	gps_state.last_update = millis();
	set_nonblocking(gps_state.gps_fd);
	set_nonblocking(fd[0]);
	return gps_state.client_fd;
}


/*
  send a UBLOX GPS message
 */
static void gps_send(uint8_t msgid, uint8_t *buf, uint16_t size)
{
        const uint8_t PREAMBLE1 = 0xb5;
        const uint8_t PREAMBLE2 = 0x62;
        const uint8_t CLASS_NAV = 0x1;
	uint8_t hdr[6], chk[2];
	hdr[0] = PREAMBLE1;
	hdr[1] = PREAMBLE2;
	hdr[2] = CLASS_NAV;
	hdr[3] = msgid;
	hdr[4] = size & 0xFF;
	hdr[5] = size >> 8;
	chk[0] = chk[1] = hdr[2];
	chk[1] += (chk[0] += hdr[3]);
	chk[1] += (chk[0] += hdr[4]);
	chk[1] += (chk[0] += hdr[5]);
	for (uint8_t i=0; i<size; i++) {
		chk[1] += (chk[0] += buf[i]);
	}
	write(gps_state.gps_fd, hdr, sizeof(hdr));
	write(gps_state.gps_fd, buf, size);
	write(gps_state.gps_fd, chk, sizeof(chk));
}


/*
  possibly send a new GPS UBLOX packet
 */
void sitl_update_gps(double latitude, double longitude, float altitude,
		     double speedN, double speedE, bool have_lock)
{
	struct ubx_nav_posllh {
		uint32_t	time; // GPS msToW
		int32_t		longitude;
		int32_t		latitude;
		int32_t		altitude_ellipsoid;
		int32_t		altitude_msl;
		uint32_t	horizontal_accuracy;
		uint32_t	vertical_accuracy;
	} pos;
	struct ubx_nav_status {
		uint32_t	time;				// GPS msToW
		uint8_t		fix_type;
		uint8_t		fix_status;
		uint8_t		differential_status;
		uint8_t		res;
		uint32_t	time_to_first_fix;
		uint32_t	uptime;				// milliseconds
	} status;
	struct ubx_nav_velned {
		uint32_t	time;				// GPS msToW
		int32_t		ned_north;
		int32_t		ned_east;
		int32_t		ned_down;
		uint32_t	speed_3d;
		uint32_t	speed_2d;
		int32_t		heading_2d;
		uint32_t	speed_accuracy;
		uint32_t	heading_accuracy;
	} velned;
        const uint8_t MSG_POSLLH = 0x2;
	const uint8_t MSG_STATUS = 0x3;
	const uint8_t MSG_VELNED = 0x12;
	struct gps_data d;

	// 5Hz, to match the real UBlox config in APM
	if (millis() - gps_state.last_update < 200) {
		return;
	}
	gps_state.last_update = millis();

	d.latitude = latitude;
	d.longitude = longitude;
	d.altitude = altitude;
	d.speedN = speedN;
	d.speedE = speedE;
	d.have_lock = have_lock;

	// add in some GPS lag
	gps_data[next_gps_index++] = d;
	if (next_gps_index >= gps_delay) {
		next_gps_index = 0;
	}

	d = gps_data[next_gps_index];

	if (sitl.gps_delay != gps_delay) {
		// cope with updates to the delay control
		gps_delay = sitl.gps_delay;
		for (uint8_t i=0; i<gps_delay; i++) {
			gps_data[i] = d;
		}
	}

	pos.time = millis(); // FIX
	pos.longitude = d.longitude * 1.0e7;
	pos.latitude  = d.latitude * 1.0e7;
	pos.altitude_ellipsoid = d.altitude*1000.0;
	pos.altitude_msl = d.altitude*1000.0;
	pos.horizontal_accuracy = 5;
	pos.vertical_accuracy = 10;

	status.time = pos.time;
	status.fix_type = d.have_lock?3:0;
	status.fix_status = d.have_lock?1:0;
	status.differential_status = 0;
	status.res = 0;
	status.time_to_first_fix = 0;
	status.uptime = millis();

	velned.time = pos.time;
	velned.ned_north = 100.0 * d.speedN;
	velned.ned_east  = 100.0 * d.speedE;
	velned.ned_down  = 0;
#define sqr(x) ((x)*(x))
	velned.speed_2d = sqrt(sqr(d.speedN)+sqr(d.speedE)) * 100;
	velned.speed_3d = velned.speed_2d;
	velned.heading_2d = ToDeg(atan2(d.speedE, d.speedN)) * 100000.0;
	if (velned.heading_2d < 0.0) {
		velned.heading_2d += 360.0 * 100000.0;
	}
	velned.speed_accuracy = 2;
	velned.heading_accuracy = 4;

	if (gps_state.gps_fd == 0) {
		return;
	}

	gps_send(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
	gps_send(MSG_STATUS, (uint8_t*)&status, sizeof(status));
	gps_send(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
}
