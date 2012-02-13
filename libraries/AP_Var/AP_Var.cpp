// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file   AP_Var.cpp
/// @brief  The AP variable store.

#define USE_AP_VAR

#include <AP_Common.h>
#include <AP_Var.h>
#include <math.h>
#include <string.h>

//#define ENABLE_FASTSERIAL_DEBUG

#ifdef ENABLE_FASTSERIAL_DEBUG
# include <FastSerial.h>
# define serialDebug(fmt, args...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
#else
# define serialDebug(fmt, args...)
#endif

// Global constants exported for general use.
//
AP_Float AP_Float_unity         ( 1.0, AP_Var::k_key_none, NULL, AP_Var::k_flag_unlisted);
AP_Float AP_Float_negative_unity(-1.0, AP_Var::k_key_none, NULL, AP_Var::k_flag_unlisted);
AP_Float AP_Float_zero          ( 0.0, AP_Var::k_key_none, NULL, AP_Var::k_flag_unlisted);

// Static member variables for AP_Var.
//
AP_Var      *AP_Var::_variables;
AP_Var      *AP_Var::_grouped_variables;
uint16_t    AP_Var::_tail_sentinel;
uint16_t    AP_Var::_bytes_in_use;

// Constructor for standalone variables
//
AP_Var::AP_Var(Key p_key, const prog_char_t *name, Flags flags) :
    _group(NULL),
    _key(p_key | k_key_not_located),
    _name(name),
    _flags(flags)
{
    // Insert the variable or group into the list of known variables, unless
    // it wants to be unlisted.
    //
    if (!has_flags(k_flag_unlisted)) {
        _link = _variables;
        _variables = this;
    }
}

// Constructor for variables in a group
//
AP_Var::AP_Var(AP_Var_group *pGroup, Key index, const prog_char_t *name, Flags flags) :
    _group(pGroup),
    _key(index),
    _name(name),
    _flags(flags)
{
    AP_Var  **vp;

    // Sort the variable into the list of group-member variables.
    //
    // This list is kept sorted so that groups can traverse forwards along
    // it in order to enumerate their members in key order.
    //
    // We use a pointer-to-pointer insertion technique here; vp points
    // to the pointer to the node that we are considering inserting in front of.
    //
    vp = &_grouped_variables;
    size_t loopCount = 0;
    while (*vp != NULL) {

        if (loopCount++>k_num_max) return;

        if ((*vp)->_key >= _key) {
            break;
        }
        vp = &((*vp)->_link);
    }
    _link = *vp;
    *vp = this;
}

// Destructor
//
AP_Var::~AP_Var(void)
{
    AP_Var  **vp;

    // Determine which list the variable may be in.
    // If the variable is a group member and the group has already
    // been destroyed, it may not be in any list.
    //
    if (_group) {
        vp = &_grouped_variables;
    } else {
        vp = &_variables;
    }

    // Scan the list and remove this if we find it

    {
        size_t loopCount = 0;
        while (*vp) {

            if (loopCount++>k_num_max) return;

            if (*vp == this) {
                *vp = _link;
                break;
            }
            vp = &((*vp)->_link);
        }
    }

    // If we are destroying a group, remove all its variables from the list
    //
    if (has_flags(k_flag_is_group)) {

        // Scan the list and remove any variable that has this as its group
        vp = &_grouped_variables;
        size_t loopCount = 0;

        while (*vp) {

            if (loopCount++>k_num_max) return;

            // Does the variable claim us as its group?
            if ((*vp)->_group == this) {
                *vp = (*vp)->_link;
                continue;
            }
            vp = &((*vp)->_link);
        }
    }
}

// Copy the variable's whole name to the supplied buffer.
//
// If the variable is a group member, prepend the group name.
//
void AP_Var::copy_name(char *buffer, size_t buffer_size) const
{
    buffer[0] = '\0';
    if (_name) {
        if (_group)
            _group->copy_name(buffer, buffer_size);
        strlcat_P(buffer, _name, buffer_size);
    }
}

// Find a variable by name.
//
AP_Var *
AP_Var::find(const char *name)
{
    AP_Var  *vp;

    size_t loopCount = 0;

    for (vp = first(); vp; vp = vp->next()) {

        if (loopCount++>k_num_max) return NULL;

        char    name_buffer[32];

        // copy the variable's name into our scratch buffer
        vp->copy_name(name_buffer, sizeof(name_buffer));

        // compare with the user-supplied name
        if (!strcmp(name, name_buffer)) {
            return vp;
        }
    }
    return NULL;
}

// Find a variable by key.
//
AP_Var *
AP_Var::find(Key key)
{
    AP_Var  *vp;
    size_t loopCount = 0;
    for (vp = first(); vp; vp = vp->next()) {
        if (loopCount++>k_num_max) return NULL;
        if (key == vp->key()) {
            return vp;
        }
    }
    return NULL;
}


// Save the variable to EEPROM, if supported
//
bool AP_Var::save(void)
{
    uint8_t vbuf[k_size_max];
    size_t size;

    // if the variable is a group member, save the group
    if (_group) {
        return _group->save();
    }

    serialDebug("save: %S", _name ? _name : PSTR("??"));

    // locate the variable in EEPROM, allocating space as required
    if (!_EEPROM_locate(true)) {
        serialDebug("locate failed");
        return false;
    }

    // serialize the variable into the buffer and work out how big it is
    size = serialize(vbuf, sizeof(vbuf));
    if (0 == size) {
        // variable cannot be serialised into the buffer
        serialDebug("cannot save (too big or not supported)");
        return false;
    }

    // if it fit in the buffer, save it to EEPROM
    if (size <= sizeof(vbuf)) {
        serialDebug("saving %u to %u", size, _key);
        // XXX this should use eeprom_update_block if/when Arduino moves to
        // avr-libc >= 1.7
        uint8_t *ep = (uint8_t *)_key;
        for (size_t i = 0; i < size; i++, ep++) {
            uint8_t newv;
            // if value needs to change, change it
            if (eeprom_read_byte(ep) != vbuf[i])
                eeprom_write_byte(ep, vbuf[i]);

            // now read it back
            newv = eeprom_read_byte(ep);
            if (newv != vbuf[i]) {
                serialDebug("readback failed at offset %p: got %u, expected %u",
                            ep, newv, vbuf[i]);
                return false;
            }
        }
        return true;
    }
    return false;
}

// Load the variable from EEPROM, if supported
//
bool AP_Var::load(void)
{
    uint8_t vbuf[k_size_max];
    size_t size;

    // if the variable is a group member, load the group
    if (_group) {
        return _group->load();
    }

    serialDebug("load: %S", _name ? _name : PSTR("??"));

    // locate the variable in EEPROM, but do not allocate space
    if (!_EEPROM_locate(false)) {
        serialDebug("locate failed");
        return false;
    }

    // ask the serializer how big the variable is
    //
    // XXX should check size in EEPROM var header too...
    //
    size = serialize(NULL, 0);
    if (0 == size) {
        serialDebug("cannot load (too big or not supported)");
        return false;
    }

    // Read the buffer from EEPROM, now that _EEPROM_locate
    // has converted _key into an EEPROM address.
    //
    if (size <= sizeof(vbuf)) {
        serialDebug("loading %u from %u", size, _key);
        eeprom_read_block(vbuf, (void *)_key, size);
        return unserialize(vbuf, size);
    }
    return false;
}

// Save all variables that don't opt out.
//
//
bool AP_Var::save_all(void)
{
    bool result = true;
    AP_Var *vp = _variables;

    size_t loopCount = 0;

    while (vp) {

        if (loopCount++>k_num_max) return false;

        if (!vp->has_flags(k_flag_no_auto_load) &&  // not opted out of autosave
                (vp->_key != k_key_none)) {              // has a key

            if (!vp->save()) {
                result = false;
            }
        }
        vp = vp->_link;
    }
    return result;
}

// Load all variables that don't opt out.
//
bool AP_Var::load_all(void)
{
    bool result = true;
    AP_Var *vp = _variables;

    size_t loopCount = 0;

    while (vp) {

        if (loopCount++>k_num_max) return false;

        if (!vp->has_flags(k_flag_no_auto_load) &&  // not opted out of autoload
                (vp->_key != k_key_none)) {             // has a key

            if (!vp->load()) {
                result = false;
            }
        }
        vp = vp->_link;
    }
    return result;
}

// Erase all variables in EEPROM.
//
// We first walk the variable set and recover their key values
// from EEPROM, so that we have a chance of saving them later.
//
void
AP_Var::erase_all()
{
    AP_Var  *vp;
    uint16_t i;

    serialDebug("erase EEPROM");

    // Scan the list of variables/groups, fetching their key values and
    // reverting them to their not-located state.
    //
    vp = _variables;

    size_t loopCount = 0;

    while (vp) {

        if (loopCount++>k_num_max) return;

        vp->_key = vp->key() | k_key_not_located;
        vp = vp->_link;
    }

    // wipe the whole EEPROM, including waypoints, as we call this
    // on firmware revison changes, which may include a change to the
    // waypoint format
    for (i = 0; i < k_EEPROM_size; i++) {
        eeprom_write_byte((uint8_t *)i, 0xff);
    }

    // revert to ignorance about the state of the EEPROM
    _tail_sentinel = 0;
}

// Return the key for a variable.
//
AP_Var::Key
AP_Var::key(void)
{
    Var_header  var_header;

    if (_group) {                   // group members don't have keys
        return k_key_none;
    }
    if (_key & k_key_not_located) {    // if not located, key is in memory
        return _key & k_key_mask;
    }

    // Read key from EEPROM, note that _key points to the space
    // allocated for storage; the header is immediately before.
    //
    eeprom_read_block(&var_header, (void *)(_key - sizeof(var_header)), sizeof(var_header));
    return var_header.key;
}

// Default implementation of cast_to_float, which always fails.
//
float
AP_Var::cast_to_float(void) const
{
    return NAN;
}

// Return the next variable in the global list.
//
AP_Var *
AP_Var::next(void)
{
    // If there is a variable after this one, return it.
    //
    if (_link)
        return _link;

    // If we are at the end of the _variables list, _group will be NULL; in that
    // case, move to the _grouped_variables list.
    //
    if (!_group) {
        return _grouped_variables;
    }

    // We must be at the end of the _grouped_variables list, nothing remains.
    //
    return NULL;
}


// Return the first variable that is a member of the group.
//
AP_Var *
AP_Var::first_member(AP_Var_group *group)
{
    AP_Var  **vp;

    vp = &_grouped_variables;

    serialDebug("seeking %p", group);

    size_t loopCount = 0;

    while (*vp) {

        if (loopCount++>k_num_max) return NULL;

        serialDebug("consider %p with %p", *vp, (*vp)->_group);
        if ((*vp)->_group == group) {
            return *vp;
        }
        vp = &((*vp)->_link);
    }
    return NULL;
}

// Return the next variable that is a member of the same group.
AP_Var *
AP_Var::next_member()
{
    AP_Var  *vp;

    vp = _link;
    size_t loopCount = 0;

    while (vp) {

        if (loopCount++>k_num_max) return NULL;

        if (vp->_group == _group) {
            return vp;
        }
        vp = vp->_link;
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
    uint16_t                eeprom_address;

    // Assume that the EEPROM contents are invalid
    _tail_sentinel = 0;

    // read the header and validate
    eeprom_address = 0;
    eeprom_read_block(&ee_header, (void *)eeprom_address, sizeof(ee_header));
    if ((ee_header.magic != k_EEPROM_magic) ||
            (ee_header.revision != k_EEPROM_revision)) {

        serialDebug("no header, magic 0x%x revision %u", ee_header.magic, ee_header.revision);
        return false;
    }

    // scan the EEPROM
    //
    // Avoid trying to read a header when there isn't enough space left.
    //
    eeprom_address = sizeof(ee_header);

    size_t loopCount = 0;

    while (eeprom_address < (k_EEPROM_size - sizeof(var_header) - 1)) {

        if (loopCount++>k_num_max) return NULL;

        // Read a variable header
        //
        serialDebug("reading header from %u", eeprom_address);
        eeprom_read_block(&var_header, (void *)eeprom_address, sizeof(var_header));

        // If the header is for the sentinel, scanning is complete
        //
        if (var_header.key == k_key_sentinel) {
            serialDebug("found tail sentinel");
            break;
        }

        // Sanity-check the variable header and abort if it looks bad
        //
        if (k_EEPROM_size <= (
                    eeprom_address +        // current position
                    sizeof(var_header) +    // header for this variable
                    var_header.size + 1 +   // data for this variable
                    sizeof(var_header))) {  // header for sentinel

            serialDebug("header overruns EEPROM");
            return false;
        }

        // look for a variable with this key
        vp = _variables;
        size_t loopCount2 = 0;
        while(vp) {
            if (loopCount2++>k_num_max) return false;
            if (vp->key() == var_header.key) {
                // adjust the variable's key to point to this entry
                vp->_key = eeprom_address + sizeof(var_header);
                serialDebug("update %p with key %u -> %u", vp, var_header.key, vp->_key);
                break;
            }
            vp = vp->_link;
        }
        if (!vp) {
            serialDebug("key %u not claimed (already scanned or unknown)", var_header.key);
        }

        // move to the next variable header
        eeprom_address += sizeof(var_header) + var_header.size + 1;
    }

    // Mark any variables that weren't assigned addresses as not-allocated,
    // so that we don't waste time looking for them again later.
    //
    // Note that this isn't done when the header is not found on an empty EEPROM.
    // The first variable written on an empty EEPROM falls out as soon as the
    // header is not found.  The second will scan and find one variable, then
    // mark all the rest as not allocated.
    //
    vp = _variables;
    size_t loopCount3 = 0;
    while(vp) {
        if (loopCount3++>k_num_max) return false;
        if (vp->_key & k_key_not_located) {
            vp->_key |= k_key_not_allocated;
            serialDebug("key %u not allocated", vp->key());
        }
        vp = vp->_link;
    }

    // Scanning is complete
    serialDebug("scan done");
    _tail_sentinel = eeprom_address;
    return true;
}

// Locate a variable in EEPROM, allocating space if required.
//
bool AP_Var::_EEPROM_locate(bool allocate)
{
    Var_header  var_header;
    Key         new_location;
    size_t      size;

    // Is it a group member, or does it have a no-location key?
    //
    if (_group || (_key == k_key_none)) {
        serialDebug("not addressable");
        return false;               // it is/does, and thus it has no location
    }

    // Has the variable already been located?
    //
    if (!(_key & k_key_not_located)) {
        return true;                // it has
    }

    // We don't know where this variable belongs.  If the variable isn't
    // marked as already having been looked for and not found in EEPROM,
    // try scanning to see if we can locate it.
    //
    if (!(_key & k_key_not_allocated)) {
        serialDebug("need scan");
        _EEPROM_scan();

        // Has the variable now been located?
        //
        if (!(_key & k_key_not_located)) {
            return true;                // it has
        }
    }

    // If not located and not permitted to allocate, we have failed.
    //
    if (!allocate) {
        return false;
    }
    serialDebug("needs allocation");

    // Ask the serializer for the size of the thing we are allocating, and fail
    // if it is too large or if it has no size, as we will not be able to allocate
    // space for it.
    //
    size = serialize(NULL, 0);
    if ((size == 0) || (size > k_size_max)) {
        serialDebug("size %u out of bounds", size);
        return false;
    }

    // Make sure there will be space in the EEPROM for the variable, its
    // header and the new tail sentinel.
    //
    if ((_tail_sentinel + size + sizeof(Var_header) * 2) > k_EEPROM_size) {
        serialDebug("no space in EEPROM");
        return false;
    }

    // If there is no data in the EEPROM, write the header and move the
    // sentinel.
    //
    if (0 == _tail_sentinel) {
        uint8_t pad_size;

        serialDebug("writing header");
        EEPROM_header   ee_header;

        ee_header.magic = k_EEPROM_magic;
        ee_header.revision = k_EEPROM_revision;
        ee_header.spare = 0;

        eeprom_write_block(&ee_header, (void *)0, sizeof(ee_header));

        _tail_sentinel = sizeof(ee_header);

        // Write a variable-sized pad header with a reserved key value
        // to help wear-level the EEPROM a bit.
        pad_size = (((uint8_t)micros()) % k_size_max) + 1;     // should be fairly random
        var_header.key = k_key_pad;
        var_header.size = pad_size - 1;
        var_header.spare = 0;

        eeprom_write_block(&var_header, (void *)_tail_sentinel, sizeof(var_header));
        _tail_sentinel += sizeof(var_header) + pad_size;
    }

    // Save the location we are going to insert at, and compute the new
    // tail sentinel location.
    //
    new_location = _tail_sentinel;
    _tail_sentinel += sizeof(var_header) + size;
    serialDebug("allocated %u/%u for key %u new sentinel %u", new_location, size, key(), _tail_sentinel);

    // Write the new sentinel first.  If we are interrupted during this operation
    // the old sentinel will still correctly terminate the EEPROM image.
    //
    var_header.key = k_key_sentinel;
    var_header.size = 0;
    var_header.spare = 0;
    eeprom_write_block(&var_header, (void *)_tail_sentinel, sizeof(var_header));

    // Write the header for the block we have just located, claiming the EEPROM space.
    //
    var_header.key = key();
    var_header.size = size - 1;
    eeprom_write_block(&var_header, (void *)new_location, sizeof(var_header));

    // We have successfully allocated space and thus located the variable.
    // Update _key to point to the space allocated for it.
    //
    _key = new_location + sizeof(var_header);
    return true;
}

size_t
AP_Var_group::serialize(void *buf, size_t buf_size) const
{
    // We have to cast away the const in order to call _serialize_unserialize,
    // as it cannot be const due to changing this when called to unserialize.
    //
    // XXX it's questionable how much advantage we get from having ::serialize
    //     const in the first place...
    //
    return const_cast<AP_Var_group *>(this)->_serialize_unserialize(buf, buf_size, true);
}

size_t
AP_Var_group::unserialize(void *buf, size_t buf_size)
{
    return _serialize_unserialize(buf, buf_size, false);
}

size_t
AP_Var_group::_serialize_unserialize(void *buf, size_t buf_size, bool do_serialize)
{
    AP_Var  *vp;
    size_t  size, total_size;

    // Traverse the list of group members, serializing each in order
    //
    vp = first_member(this);
    serialDebug("starting with %p", vp);
    total_size = 0;

    size_t loopCount = 0;

    while (vp) {

        if (loopCount++>k_num_max) return false;

        // (un)serialise the group member
        if (do_serialize) {
            size = vp->serialize(buf, buf_size);
            serialDebug("serialize %p -> %u", vp, size);
        } else {
            size = vp->unserialize(buf, buf_size);
            serialDebug("unserialize %p -> %u", vp, size);
        }

        // Unserialize will return zero if the buffer is too small
        // Serialize will only return zero if the variable cannot be serialised
        // Either case is fatal for any operation we might be trying.
        //
        if (0 == size) {
            serialDebug("group (un)serialize failed, buffer too small or not supported");
            return 0;
        }


        // Account for the space that this variable consumes in the buffer
        //
        // We always count the total size, and we always advance the buffer pointer
        // if there was room for the variable.  This does mean that in the case where
        // the buffer was too small for a variable in the middle of the group, that
        // a smaller variable after it in the group may still be serialised into
        // the buffer.  Since that's a rare case it's not worth optimising for - in
        // either case this function will return a size greater than the buffer size
        // and the calling function will have to treat it as an error.
        //
        total_size += size;
        serialDebug("used %u", total_size);
        if (size <= buf_size) {
            // there was space for this one, account for it
            buf_size -= size;
            buf = (void *)((uint8_t *)buf + size);
        }

        vp = vp->next_member();
    }
    return total_size;
}

// Static pseudo-constant type IDs for known AP_VarT subclasses.
//
AP_Meta_class::Type_id  AP_Var::k_typeid_float;     ///< meta_type_id() value for AP_Float
AP_Meta_class::Type_id  AP_Var::k_typeid_float16;   ///< meta_type_id() value for AP_Float16
AP_Meta_class::Type_id  AP_Var::k_typeid_int32;     ///< meta_type_id() value for AP_Int32
AP_Meta_class::Type_id  AP_Var::k_typeid_int16;     ///< meta_type_id() value for AP_Int16
AP_Meta_class::Type_id  AP_Var::k_typeid_int8;      ///< meta_type_id() value for AP_Int8
AP_Meta_class::Type_id  AP_Var::k_typeid_group;     ///< meta_type_id() value for AP_Var_group

/// A special class used to initialise the k_typeid_* values that AP_Var exports.
///
class AP_Var_typesetup
{
public:
    /// Constructor
    ///
    /// This constructor should be run just once by creating a static instance
    /// of the class.  It will initialise the k_typeid_* values for the well-known
    /// AP_VarT subclasses.
    ///
    /// When a new subclass is created, a new k_typeid_* constant should also be
    /// created and the list below should likewise be expanded.
    ///
    AP_Var_typesetup(void);
};

/// Initialise AP_Var's k_typeid_* values
AP_Var_typesetup::AP_Var_typesetup(void)
{
    AP_Var::k_typeid_float      = AP_Meta_class::meta_type_id<AP_Float>();
    AP_Var::k_typeid_float16    = AP_Meta_class::meta_type_id<AP_Float16>();
    AP_Var::k_typeid_int32      = AP_Meta_class::meta_type_id<AP_Int32>();
    AP_Var::k_typeid_int16      = AP_Meta_class::meta_type_id<AP_Int16>();
    AP_Var::k_typeid_int8       = AP_Meta_class::meta_type_id<AP_Int8>();
    AP_Var::k_typeid_group      = AP_Meta_class::meta_type_id<AP_Var_group>();
}

/// Cause the AP_Var_typesetup constructor to be run.
///
static AP_Var_typesetup _typesetup __attribute__((used));
