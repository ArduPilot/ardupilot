// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//
/// The AP variable interface. This allows different types
/// of variables to be passed to blocks for floating point
/// math, memory management, etc.

#include "AP_Var.h"

const AP_Float	AP_GainUnity(1.0);
const AP_Float	AP_GainNegativeUnity(-1.0);
const AP_Float	AP_Zero(0);

// Destructor
//
// Removes named variables from the list.
//
AP_Var::~AP_Var(void)
{
	AP_Var	*p;

	// only do this for variables that have names
	if (_name) {
		// if we are at the head of the list - for variables
		// recently constructed this is usually the case
		if (_variables == this) {
			// remove us from the head of the list
			_variables = _link;
		} else {
			// traverse the list looking for the entry that points to us
			p = _variables;
			while (p) {
				// is it pointing at us?
				if (p->_link == this) {
					// make it point at what we point at, and done
					p->_link = _link;
					break;
				}
				// try the next one
				p = p->_link;
			}
		}
	}
}


//
// Lookup interface for variables.
//
AP_Var	*AP_Var::_variables = NULL;
AP_Var	*AP_Var::_lookupHint = NULL;
int		AP_Var::_lookupHintIndex = 0;

AP_Var *
AP_Var::lookup(int index)
{
	AP_Var	*p;
	int			i;

	// establish initial search state
	if (_lookupHintIndex &&				// we have a cached hint
		(index >= _lookupHintIndex)) {	// the desired index is at or after the hint

		p = _lookupHint;				// start at the hint point
		i = index - _lookupHintIndex;	// count only the distance from the hint to the index
	} else {

		p = _variables;					// start at the beginning of the list
		i = index;						// count to the index
	}

	// search
	while (index-- && p)				// count until we hit the index or the end of the list
		p = p->_link;

	// update the cache on hit
	if (p) {
		_lookupHintIndex = i;
		_lookupHint = p;
	}

	return(p);
}

// Save all variables that have an identity.
//
void
AP_Var::save_all(void)
{
	AP_Var	*p = _variables;

	while (p) {
		p->save();
		p = p->_link;
	}
}

// Load all variables that have an identity.
//
void
AP_Var::load_all(void)
{
	AP_Var	*p = _variables;

	while (p) {
		p->load();
		p = p->_link;
	}
}
