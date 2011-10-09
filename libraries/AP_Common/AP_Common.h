// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

// Get the common arduino functions
#include "wiring.h"
// ... and remove some of their stupid macros
#undef round
#undef abs

// prog_char_t is used as a wrapper type for prog_char, which is
// a character stored in flash. By using this wrapper type we can
// auto-detect at compile time if a call to a string function is using
// a flash-stored string or not
typedef struct {
	char c;
} prog_char_t;

#include <stdint.h>
#include "include/menu.h"		/// simple menu subsystem
#include "c++.h" // c++ additions
//#include "AP_Vector.h"
//#include "AP_Loop.h"
#include "AP_Var.h"



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
//#pragma GCC diagnostic error "-Wfloat-equal"

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
#ifdef DESKTOP_BUILD
# undef PROGMEM
# define PROGMEM __attribute__(())
#else
# undef PROGMEM
# define PROGMEM __attribute__(( section(".progmem.data") ))
#endif

# undef PSTR
# define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); \
                (prog_char_t *)&__c[0];}))
#endif

// a varient of PSTR() for progmem strings passed to %S in printf()
// this gets the gcc __format__ checking right
#define FPSTR(s) (wchar_t *)(s)


static inline int strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
	return strcasecmp_P(str1, (const prog_char *)pstr);
}

static inline int strcmp_P(const char *str1, const prog_char_t *pstr)
{
	return strcmp_P(str1, (const prog_char *)pstr);
}

static inline size_t strlcat_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
	return strlcat_P(buffer, (const prog_char *)pstr, buffer_size);
}

//@}


///
/// @name Macros
/// @{

/// Define a constant string in program memory.  This is a little more obvious
/// and less error-prone than typing the declaration out by hand.  It's required
/// when passing PROGMEM strings to static object constructors because the PSTR
/// hack can't be used at global scope.
///
#define PROGMEM_STRING(_v, _s)	static const char _v[] PROGMEM = _s

#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi
// @}


////////////////////////////////////////////////////////////////////////////////
/// @name	Types
///
/// Data structures and types used throughout the libraries and applications. 0 = default
/// bit 0: Altitude is stored 			    0: Absolute,	1: Relative
/// bit 1: Chnage Alt between WP 		    0: Gradually,	1: ASAP
/// bit 2:
/// bit 3: Req.to hit WP.alt to continue    0: No,          1: Yes
/// bit 4: Relative to Home					0: No, 			1: Yes
/// bit 5:
/// bit 6:
/// bit 7: Move to next Command 		    0: YES, 		1: Loiter until commanded

//@{

struct Location {
	uint8_t		id;					///< command id
	uint8_t		options;			///< options bitmask (1<<0 = relative altitude)
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
#define ToRad(x) (x*0.01745329252)	// *pi/180

//@}


#endif // _AP_COMMON_H
