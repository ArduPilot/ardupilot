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
#include "c++.h" // c++ additions
#include "AP_Vector.h"
#include "AP_Loop.h"

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

///
/// @name Macros
/// @{
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi
// @}


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

/// The AP variable interface. This allows different types
/// of variables to be passed to blocks for floating point
/// math, memory management, etc. 
class AP_VarI
{
public:

	/// Set the variable value as a float
	virtual void setF(const float & val) = 0;

	/// Get the variable as a float
	virtual const float & getF() = 0;

	/// Save a variable to eeprom
	virtual void save() = 0;

	/// Load a variable from eeprom
	virtual void load() = 0;

	/// Get the name. This is useful for ground stations.
	virtual const char * getName() = 0;

	/// If sync is true the a load will always occure before a get and a save will always
	/// occure before a set.
	virtual const bool & getSync() = 0;

	/// Set the sync property
	virtual void setSync(bool sync) = 0;
};

/// The variable template class. This class
/// implements get/set/save/load etc for the 
/// abstract template type.
template  <class type>
class AP_Var : public AP_VarI
{
public:
	/// The default constrcutor
	AP_Var(const type & data, const char * name = "", const bool & sync=false) : 
		_data(data), _name(name), _sync(sync)
	{
	}

	/// Set the variable value
	virtual void set(const type & val) {
		_data = val; 
		if (_sync) save();
	}

	/// Get the variable value.
	virtual const type & get() {
		if (_sync) load();
		return _data;
	}

	/// Set the variable value as a float
	virtual void setF(const float & val) {
		set(val);
	}

	/// Get the variable as a float
	virtual const float & getF() {
		return get();
	}

	/// Save a variable to eeprom
	virtual void save()
	{
	}

	/// Load a variable from eeprom
	virtual void load()
	{
	}

	/// Get the name. This is useful for ground stations.
	virtual const char * getName() { return _name; }

	/// If sync is true the a load will always occure before a get and a save will always
	/// occure before a set.
	virtual const bool & getSync() { return _sync; }
	virtual void setSync(bool sync) { _sync = sync; }

protected:
	type _data; /// The data that is stored on the heap */
	const char * _name; /// The variable name, useful for gcs and terminal output
	bool _sync; /// Whether or not to call save/load on get/set
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
