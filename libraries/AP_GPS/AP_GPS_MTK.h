// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//
#ifndef AP_GPS_MTK_h
#define AP_GPS_MTK_h

#include <GPS.h>
#define MAXPAYLOAD 32

#define MTK_SET_BINARY	"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA	"$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_OUTPUT_1HZ	"$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_2HZ	"$PMTK220,500*2B\r\n"
#define MTK_OUTPUT_4HZ	"$PMTK220,250*29\r\n"
#define MTK_OTUPUT_5HZ	"$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ	"$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define SBAS_ON			"$PMTK313,1*2E\r\n"
#define SBAS_OFF		"$PMTK313,0*2F\r\n"

#define WAAS_ON			"$PSRF151,1*3F\r\n"
#define WAAS_OFF		"$PSRF151,0*3E\r\n"

class AP_GPS_MTK : public GPS {
public:
	AP_GPS_MTK(Stream *s);
	virtual void	init(void);
	virtual void	update(void);

private:
#pragma pack(1)
	struct diyd_mtk_msg {
		int32_t		latitude;
		int32_t		longitude;
		int32_t		altitude;
		int32_t		ground_speed;
		int32_t		ground_course;
		uint8_t		satellites;
		uint8_t		fix_type;
		uint32_t	utc_time;
	};
#pragma pack(pop)
	enum diyd_mtk_fix_type {
		FIX_NONE = 1,
		FIX_2D = 2,
		FIX_3D = 3
	};

	enum diyd_mtk_protocol_bytes {
		PREAMBLE1 = 0xb5,
		PREAMBLE2 = 0x62,
		MESSAGE_CLASS = 1,
		MESSAGE_ID = 5
	};

	// Packet checksum accumulators
	uint8_t 	_ck_a;
	uint8_t 	_ck_b;

	// State machine state
	uint8_t 	_step;
	uint8_t		_payload_length;
	uint8_t		_payload_counter;

	// Receive buffer
	union {
		diyd_mtk_msg	msg;
		uint8_t			bytes[];
	} _buffer;

	// Buffer parse & GPS state update
	void		_parse_gps();
};

#endif	// AP_GPS_MTK_H
