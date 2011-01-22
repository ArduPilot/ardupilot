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
uint16_t AP_Var::_tail_sentinel;
bool AP_Var::_EEPROM_scanned;

AP_Var_group::AP_Var_group()
{
}

AP_Var_group::~AP_Var_group()
{
}

// Constructor
//
AP_Var::AP_Var(Key key, const prog_char *name, AP_Var_group *group, Flags flags) :
        _group(group),
        _key(key | k_not_located),
        _name(name),
        _flags(flags)
{
    AP_Var  *vp;

    // Insert the variable or group into the list of known variables.
    //
    // Variables belonging to a group are inserted into the list following the group,
    // which may not itself yet be in the global list.  Thus groups must be
    // statically constructed (which guarantees _link will be zero due to the BSS
    // being cleared).
    //

    if (group) {

        // Sort the variable into the list of variables following the group itself.
        vp = group;

        for (;;) {
            // if there are no more entries, we insert at the end
            if (vp->_link == NULL)
                break;

            // if the next entry in the list is a group, insert before it
            if (meta_type_equivalent(this, vp->_link))
                break;

            // if the next entry has a higher index, insert before it
            if ((vp->_link->_key & k_key_mask) > key)
                break;
            vp = vp->_link;
        }

        // insert into the group's list
        _link = vp->_link;
        vp->_link = this;

    } else {
        // Insert directly at the head of the list.  Take into account the possibility
        // that what is being inserted is a group that already has variables sorted after it.
        //
        vp = this;
        if (meta_cast<AP_Var_group>(this) != NULL) {
            // we are inserting a group, scan to the end of any list of pre-attached variables
            while (vp->_link != NULL)
                vp = vp->_link;
        }

        // insert at the head of the global list
        vp->_link = _variables;
        _variables = this;
    }

    // reset the lookup cache
    _lookup_hint_index = 0;
}

// Destructor
//
// Removes named variables from the list.
//
AP_Var::~AP_Var(void)
{
    AP_Var  **vp;

    // Groups can only be destroyed when they have no members.
    //
    // If this is a group with one or more members, _link is not NULL
    // and _link->group is this.
    //
    if ((_link != NULL) && (_link->_group == this))
        return;

    // Walk the list and remove this when we find it.
    //
    vp = &_variables;
    while (*vp != NULL) {
        // pointer pointing at this?
        if (*vp == this) {
            *vp = _link;
            break;
        }

        // address of next entry's link pointer
        vp = &((*vp)->_link);
    }

    // reset the lookup cache
    _lookup_hint_index = 0;
}

// Copy the variable's whole name to the supplied buffer.
//
// If the variable is a group member, prepend the group name.
//
void AP_Var::copy_name(char *buffer, size_t buffer_size) const
{
    buffer[0] = '\0';
    if (_group)
        _group->copy_name(buffer, buffer_size);
    strlcat_P(buffer, _name, buffer_size);
}

// Save the variable to EEPROM, if supported
//
bool AP_Var::save(void)
{
    uint8_t vbuf[k_max_size];
    size_t size;

    // if the variable is a group member, save the group
    if (_group) {
        return _group->save();
    }

    // locate the variable in EEPROM, allocating space as required
    if (!_EEPROM_locate(true)) {
        return false;
    }

    // serialize the variable into the buffer and work out how big it is
    size = serialize(vbuf, sizeof(vbuf));

    // if it fit in the buffer, save it to EEPROM
    if (size <= sizeof(vbuf)) {
        eeprom_write_block(vbuf, (void *)_key, size);
    }
    return true;
}

// Load the variable from EEPROM, if supported
//
bool AP_Var::load(void)
{
    uint8_t vbuf[k_max_size];
    size_t size;

    // if the variable is a group member, load the group
    if (_group) {
        return _group->load();
    }

    // locate the variable in EEPROM, but do not allocate space
    if (!_EEPROM_locate(false)) {
        return false;
    }

    // ask the unserializer how big the variable is
    //
    // XXX should check size in EEPROM var header too...
    //
    size = unserialize(NULL, 0);

    // Read the buffer from EEPROM, now that _EEPROM_locate
    // has converted _key into an EEPROM address.
    //
    if (size <= sizeof(vbuf)) {
        eeprom_read_block(vbuf, (void *)_key, size);
        unserialize(vbuf, size);
    }
    return true;
}

//
// Lookup interface for variables.
//

AP_Var *
AP_Var::lookup_by_index(int index)
{
    AP_Var *p;
    int i;

    // establish initial search state
    //
    if (_lookup_hint_index &&               // we have a cached hint (cannot use a hint index of zero)
        (index >= _lookup_hint_index)) {    // the desired index is at or after the hint

        p = _lookup_hint;                   // start at the hint point
        i = index - _lookup_hint_index;     // count only the distance from the hint to the index
    } else {

        p = _variables;                     // start at the beginning of the list
        i = index;                          // count to the index
    }

    // search
    while (p && i--) { // count until we hit the index or the end of the list
        p = p->_link;
    }

    // update the cache on hit
    if (p) {
        _lookup_hint_index = index;
        _lookup_hint = p;
    }
    return p;
}

// Save all variables that don't opt out.
//
//
bool AP_Var::save_all(void)
{
    bool result = true;
    AP_Var *p = _variables;

    while (p) {
        if (!p->has_flags(k_no_auto_load) &&    // not opted out
            !(p->_group)) {                     // not saved with a group

            if (!p->save()) {
                result = false;
            }
        }
        p = p->_link;
    }
    return result;
}

// Load all variables that don't opt out.
//
bool AP_Var::load_all(void)
{
    bool result;
    AP_Var *p = _variables;

    while (p) {
        if (!p->has_flags(k_no_auto_load) &&    // not opted out
            !(p->_group)) {                     // not loaded with a group

            if (!p->load()) {
                result = false;
            }
        }
        p = p->_link;
    }
    return result;
}

// Scan the list of variables for a matching key.
//
AP_Var *
AP_Var::_lookup_by_key(Key key)
{
    AP_Var      *p;
    Key         nl_key;
    Var_header  var_header;

    nl_key = key | k_not_located;       // key to expect in memory

    // scan the list of variables
    p = _variables;
    while (p) {

        // if the variable is a group member, it cannot be found by key search
        if (p->_group) {
            continue;
        }

        // has this variable been located?
        if (p->_key & k_not_located) {
            // does the variable have the non-located form of the key?
            if (p->_key == nl_key)
                return p;                   // found it
        } else {
            // read the header from EEPROM and compare it with the search key
            eeprom_read_block(&var_header, (void *)p->_key, sizeof(var_header));
            if (var_header.key == key)
                return p;
        }

        // try the next variable
        p = p->_link;
    }
    return NULL;
}

// Scan the EEPROM and assign addresses to all the variables that
// are known and found therein.
//
bool AP_Var::_EEPROM_scan(void)
{
    struct EEPROM_header    ee_header;
    struct Var_header       var_header;
    AP_Var                  *vp;

    // assume that the EEPROM is empty
    _tail_sentinel = 0;

    // read the header and validate
    eeprom_read_block(0, &ee_header, sizeof(ee_header));
    if ((ee_header.magic != k_EEPROM_magic) ||
        (ee_header.revision != k_EEPROM_revision))
        return false;

    // scan the EEPROM
    //
    // Avoid trying to read a header when there isn't enough space left.
    //
    _tail_sentinel = sizeof(ee_header);
    while (_tail_sentinel < (k_EEPROM_size - sizeof(var_header) - 1)) {

        // read a variable header
        eeprom_read_block(&var_header, (void *)_tail_sentinel, sizeof(var_header));

        // if the header is for the sentinel, scanning is complete
        if (var_header.key == k_tail_sentinel)
            break;

        // if the variable plus the sentinel would extend past the end of EEPROM, we are done
        if (k_EEPROM_size <= (
                _tail_sentinel +        // current position
                sizeof(ee_header) +     // header for this variable
                var_header.size + 1 +   // data for this variable
                sizeof(ee_header)))     // header for sentinel
            break;

        // look for a variable with this key
        vp = _lookup_by_key(var_header.key);
        if (vp) {
            // adjust the variable's key to point to this entry
            vp->_key = _tail_sentinel;
        }

        // move to the next variable header
        _tail_sentinel += sizeof(var_header) + var_header.size + 1;
    }
    _EEPROM_scanned = true;
    return true;
}

// Locate a variable in EEPROM, allocating space if required.
//
bool AP_Var::_EEPROM_locate(bool allocate)
{
    Var_header  var_header;
    size_t      size;

    // Has the variable already been located?
    if (!(_key & k_not_located)) {
        return true;                // it has
    }

    // Does it have a no-key key?
    if (_key == k_no_key) {
        return false;               // it does, and thus it has no location
    }

    // If the EEPROM has not been scanned, try that now
    if (!_EEPROM_scanned) {
        _EEPROM_scan();
    }

    // If not located and not permitted to allocate, we have failed
    if ((_key & k_not_located) && !allocate) {
        return false;
    }

    // Ask the serializer for the size of the thing we are allocating, and fail
    // if it is too large or if it has no size
    size = serialize(NULL, 0);
    if ((0 == size) || (size > k_max_size))
        return false;

    // Make sure there will be space in the EEPROM for the variable, its
    // header and the new tail sentinel
    if ((_tail_sentinel + size + sizeof(Var_header) * 2) > k_EEPROM_size)
        return false;

    // If there is no data in the EEPROM, write the header and move the
    // sentinel
    if (0 == _tail_sentinel) {
        EEPROM_header   ee_header;

        ee_header.magic = k_EEPROM_magic;
        ee_header.revision = k_EEPROM_revision;
        ee_header.spare = 0;

        eeprom_write_block(0, &ee_header, sizeof(ee_header));

        _tail_sentinel = sizeof(ee_header);
    }

    // Write a new sentinel first
    var_header.key = k_tail_sentinel;
    var_header.size = 0;
    eeprom_write_block(&var_header, (void *)(_tail_sentinel + sizeof(Var_header) + size), sizeof(var_header));

    // Write the header for the block we have just located
    var_header.key = _key & k_key_mask;
    var_header.size = size - 1;
    eeprom_write_block(&var_header, (void *)_tail_sentinel, sizeof(Var_header));

    // Save the located address for the variable
    _key = _tail_sentinel + sizeof(Var_header);

    // Update to the new tail sentinel
    _tail_sentinel += sizeof(Var_header) + size;

    // We have successfully allocated space and thus located the variable
    return true;
}

