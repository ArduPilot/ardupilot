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
// Common definitions for MediaTek GPS modules.

#ifndef __AP_GPS_MTK_COMMON_H__
#define __AP_GPS_MTK_COMMON_H__

#define MTK_SET_BINARY	"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA	"$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_OUTPUT_1HZ	"$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_2HZ	"$PMTK220,500*2B\r\n"
#define MTK_OUTPUT_4HZ	"$PMTK220,250*29\r\n"
#define MTK_OUTPUT_5HZ	"$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ	"$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define MTK_NAVTHRES_OFF "$PMTK397,0*23\r\n"  // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s

#define SBAS_ON	        "$PMTK313,1*2E\r\n"
#define SBAS_OFF	"$PMTK313,0*2F\r\n"

#define WAAS_ON         "$PMTK301,2*2E\r\n"
#define WAAS_OFF        "$PMTK301,0*2C\r\n"

#endif // __AP_GPS_MTK_COMMON_H__
