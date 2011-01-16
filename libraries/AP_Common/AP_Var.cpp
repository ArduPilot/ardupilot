// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

// Global constants exported
//
AP_Float AP_Float_unity(1.0);
AP_Float AP_Float_negative_unity(-1.0);
AP_Float AP_Float_zero(0);

// Local state for the lookup interface
//
AP_Var *AP_Var::_variables = NULL;
AP_Var *AP_Var::_lookup_hint = NULL;
int AP_Var::_lookup_hint_index = 0;


// Constructor
//
AP_Var::AP_Var(Address address, const prog_char *name, const AP_Var_scope *scope, Flags flags) :
    _address(address), _name(name), _scope(scope), _flags(flags)
{
    // link the variable into the list of known variables
    _link = _variables;
    _variables = this;

    // reset the lookup cache
    _lookup_hint_index = 0;
}

// Destructor
//
// Removes named variables from the list.
//
AP_Var::~AP_Var(void)
{
    // if we are at the head of the list - for variables
    // recently constructed this is usually the case
    if (_variables == this) {
        // remove us from the head of the list
        _variables = _link;

    } else {
        // traverse the list looking for the entry that points to us
        AP_Var *p = _variables;

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

    // reset the lookup cache
    _lookup_hint_index = 0;
}

void AP_Var::set_default(void)
{
    // Default implementation of ::set_default does nothing
}

// Copy the variable name to the provided buffer.
//
void AP_Var::copy_name(char *buffer, size_t buffer_size) const
{
    buffer[0] = '\0';
    if (_scope) {
        _scope->copy_name(buffer, buffer_size);
    }
    strlcat_P(buffer, _name, buffer_size);
}

// Compute any offsets that should be applied to a variable in this scope.
//
// Note that this depends on the default _address value for scopes being zero.
//
AP_Var::Address AP_Var_scope::get_address(void) const
{
    AP_Var::Address addr = _address;

    if (_parent) {
        addr += _parent->get_address();
    }

    return addr;
}

// Compute the address in EEPROM where the variable is stored
//
AP_Var::Address AP_Var::_get_address(void) const
{
    Address addr = _address;

    // if we have an address at all
    if ((addr != k_no_address) && _scope) {
        addr += _scope->get_address();
    }

    return addr;
}

// Save the variable to EEPROM, if supported
//
void AP_Var::save(void) const
{
    Address addr = _get_address();

    if (addr != k_no_address) {
        uint8_t vbuf[k_max_size];
        size_t size;

        // serialize the variable into the buffer and work out how big it is
        size = serialize(vbuf, sizeof(vbuf));

        // if it fit in the buffer, save it to EEPROM
        if (size <= sizeof(vbuf)) {
            eeprom_write_block(vbuf, (void *)addr, size);
        }
    }
}

// Load the variable from EEPROM, if supported
//
void AP_Var::load(void)
{
    Address addr = _get_address();

    if (addr != k_no_address) {
        uint8_t vbuf[k_max_size];
        size_t size;

        // ask the unserializer how big the variable is
        size = unserialize(NULL, 0);

        // read the buffer from EEPROM
        if (size <= sizeof(vbuf)) {
            eeprom_read_block(vbuf, (void *)addr, size);
            unserialize(vbuf, size);
        }
    }
}

//
// Lookup interface for variables.
//

AP_Var *
AP_Var::lookup(int index)
{
    AP_Var *p;
    int i;

    // establish initial search state
    if (_lookup_hint_index && // we have a cached hint
        (index >= _lookup_hint_index)) { // the desired index is at or after the hint

        p = _lookup_hint; // start at the hint point
        i = index - _lookup_hint_index; // count only the distance from the hint to the index
    } else {

        p = _variables; // start at the beginning of the list
        i = index; // count to the index
    }

    // search
    while (i-- && p) { // count until we hit the index or the end of the list
        p = p->_link;
    }

    // update the cache on hit
    if (p) {
        _lookup_hint_index = index;
        _lookup_hint = p;
    }

    return (p);
}

// Save all variables that have an identity.
//
void AP_Var::save_all(void)
{
    AP_Var *p = _variables;

    while (p) {
        if (!p->has_flags(k_no_auto_load)) {
            p->save();
        }
        p = p->_link;
    }
}

// Load all variables that have an identity.
//
void AP_Var::load_all(void)
{
    AP_Var *p = _variables;

    while (p) {
        if (!p->has_flags(k_no_auto_load)) {
            p->load();
        }
        p = p->_link;
    }
}
