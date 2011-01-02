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

//
// Lookup interface for variables.
//
AP_VarBase	*AP_VarBase::_variables = NULL;
AP_VarBase	*AP_VarBase::_lookupHint = NULL;
int			AP_VarBase::_lookupHintIndex = 0;

AP_VarBase *
AP_VarBase::lookup(int index)
{
	AP_VarBase	*p;
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
AP_VarBase::save_all(void)
{
	AP_VarBase	*p = _variables;

	while (p) {
		p->save();
		p = p->_link;
	}
}

// Load all variables that have an identity.
//
void
AP_VarBase::load_all(void)
{
	AP_VarBase	*p = _variables;

	while (p) {
		p->load();
		p = p->_link;
	}
}
