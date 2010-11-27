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
/// @name	Warning control
//@{
//
// Turn on/off warnings of interest.
//
// These warnings are normally suppressed by the Arduino IDE,
// but with some minor hacks it's possible to have warnings
// emitted.  This helps greatly when diagnosing subtle issues.
//
#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wlogical-op"
#pragma GCC diagnostic ignored "-Wredundant-decls"

// Make some dire warnings into errors
//
// Some warnings indicate questionable code; rather than let
// these slide, we force them to become errors so that the
// developer has to find a safer alternative.
//
#pragma GCC diagnostic error "-Wfloat-equal"

// The following is strictly for type-checking arguments to printf_P calls
// in conjunction with a suitably modified Arduino IDE; never define for
// production as it generates bad code.
//
#if PRINTF_FORMAT_WARNING_DEBUG
# undef PSTR
# define PSTR(_x)		_x		// help the compiler with printf_P
# define float double			// silence spurious format warnings for %f
#else
// This is a workaround for GCC bug c++/34734.
//
// The C++ compiler normally emits many spurious warnings for the use
// of PSTR (even though it generates correct code).  This workaround
// has an equivalent effect but avoids the warnings, which otherwise
// make finding real issues difficult.
//
# undef PROGMEM 
# define PROGMEM __attribute__(( section(".progmem.data") )) 
# undef PSTR 
# define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 
#endif

//@}

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
