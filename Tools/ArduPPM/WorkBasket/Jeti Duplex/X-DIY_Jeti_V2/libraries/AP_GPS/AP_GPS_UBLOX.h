// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  u-blox UBX GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
#ifndef AP_GPS_UBLOX_h
#define AP_GPS_UBLOX_h

#include <GPS.h>

#define UBLOX_SET_BINARY	"$PUBX,41,1,0003,0001,38400,0*26"

class AP_GPS_UBLOX : public GPS
{
public:
    // Methods
	AP_GPS_UBLOX(Stream *s = NULL);
	void		init(void);
	void		update();

private:
	// u-blox UBX protocol essentials
#pragma pack(1)
	struct ubx_nav_posllh {
		uint32_t	time;				// GPS msToW
		int32_t		longitude;
		int32_t		latitude;
		int32_t		altitude_ellipsoid;
		int32_t		altitude_msl;
		uint32_t	horizontal_accuracy;
		uint32_t	vertical_accuracy;
	};
	struct ubx_nav_status {
		uint32_t	time;				// GPS msToW
		uint8_t		fix_type;
		uint8_t		fix_status;
		uint8_t		differential_status;
		uint8_t		res;
		uint32_t	time_to_first_fix;
		uint32_t	uptime;				// milliseconds
	};
	struct ubx_nav_solution {
		uint32_t	time;
		int32_t		time_nsec;
		int16_t		week;
		uint8_t		fix_type;
		uint8_t		fix_status;
		int32_t		ecef_x;
		int32_t		ecef_y;
		int32_t		ecef_z;
		uint32_t	position_accuracy_3d;
		int32_t		ecef_x_velocity;
		int32_t		ecef_y_velocity;
		int32_t		ecef_z_velocity;
		uint32_t	speed_accuracy;
		uint16_t	position_DOP;
		uint8_t		res;
		uint8_t		satellites;
		uint32_t	res2;
	};
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
	};
#pragma pack(pop)
	enum ubs_protocol_bytes {
		PREAMBLE1 = 0xb5,
		PREAMBLE2 = 0x62,
		CLASS_NAV = 0x1,
		MSG_POSLLH = 0x2,
		MSG_STATUS = 0x3,
		MSG_SOL = 0x6,
		MSG_VELNED = 0x12
	};
	enum ubs_nav_fix_type {
		FIX_NONE = 0,
		FIX_DEAD_RECKONING = 1,
		FIX_2D = 2,
		FIX_3D = 3,
		FIX_GPS_DEAD_RECKONING = 4,
		FIX_TIME = 5
	};
	enum ubx_nav_status_bits {
		NAV_STATUS_FIX_VALID = 1
	};

	// Packet checksum accumulators
	uint8_t		_ck_a;
	uint8_t		_ck_b;

	// State machine state
	uint8_t		_step;
	uint8_t		_msg_id;
	bool   		_gather;
	uint16_t	_expect;
	uint16_t	_payload_length;
	uint16_t	_payload_counter;

	// Receive buffer
	union {
		ubx_nav_posllh		posllh;
		ubx_nav_status		status;
		ubx_nav_solution	solution;
		ubx_nav_velned		velned;
		uint8_t	bytes[];
	} _buffer;

	// Buffer parse & GPS state update
	void		_parse_gps();
};

#endif
