// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

///
/// @file 		AP_Common.h
/// @brief		Common definitions and utility routines for the ArduPilot
///				libraries.
///

#ifndef _AP_COMMON_H
#define _AP_COMMON_H

#include <stdint.h>

#include "include/menu.h"		/// simple menu subsystem

////////////////////////////////////////////////////////////////////////////////
/// @name	Types
///
/// Data structures and types used throughout the libraries and applications.
///
//@{

struct Location {
	uint8_t		id;					///< command id
	uint8_t		p1;					///< param 1
	int32_t		alt;				///< param 2 - Altitude in centimeters (meters * 100)
	int32_t		lat;				///< param 3 - Lattitude * 10**7
	int32_t		lng;				///< param 4 - Longitude * 10**7
};

//@}

////////////////////////////////////////////////////////////////////////////////
/// @name	Conversions
///
/// Conversion macros and factors.
///
//@{

/// XXX this should probably be replaced with radians()/degrees(), but their
/// inclusion in wiring.h makes doing that here difficult.
#define ToDeg(x) (x*57.2957795131)	// *180/pi

//@}


#endif // _AP_COMMON_H
