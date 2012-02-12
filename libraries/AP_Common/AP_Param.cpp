// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

// total up and check overflow
// check size of group var_info

/// @file   AP_Param.cpp
/// @brief  The AP variable store.


#include <FastSerial.h>
#include <AP_Common.h>

#include <math.h>
#include <string.h>

// #define ENABLE_FASTSERIAL_DEBUG

#ifdef ENABLE_FASTSERIAL_DEBUG
# define serialDebug(fmt, args...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
#else
# define serialDebug(fmt, args...)
#endif

// some useful progmem macros
#define PGM_UINT8(addr) pgm_read_byte((const prog_char *)addr)
#define PGM_UINT16(addr) pgm_read_word((const uint16_t *)addr)
#define PGM_POINTER(addr) pgm_read_pointer((const void *)addr)

// the 'GROUP_ID' of a element of a group is the 8 bit identifier used
// to distinguish between this element of the group and other elements
// of the same group. It is calculated using a bit shift per level of
// nesting, so the first level of nesting gets 4 bits, and the next
// level gets the next 4 bits. This limits groups to having at most 16
// elements.
#define GROUP_ID(grpinfo, base, i, shift) ((base)+(((uint16_t)PGM_UINT8(&grpinfo[i].idx))<<(shift)))


// Static member variables for AP_Param.
//

// max EEPROM write size. This is usually less than the physical
// size as only part of the EEPROM is reserved for parameters
uint16_t AP_Param::_eeprom_size;

// number of rows in the _var_info[] table
uint8_t AP_Param::_num_vars;

// storage and naming information about all types that can be saved
const AP_Param::Info *AP_Param::_var_info;

// write to EEPROM, checking each byte to avoid writing
// bytes that are already correct
void AP_Param::eeprom_write_check(const void *ptr, uint16_t ofs, uint8_t size)
{
    const uint8_t *b = (const uint8_t *)ptr;
    while (size--) {
        uint8_t v = eeprom_read_byte((const uint8_t *)ofs);
        if (v != *b) {
            eeprom_write_byte((uint8_t *)ofs, *b);
        }
        b++;
        ofs++;
    }
}

// write a sentinal value at the given offset
void AP_Param::write_sentinal(uint16_t ofs)
{
    struct Param_header phdr;
    phdr.type = _sentinal_type;
    phdr.key  = _sentinal_key;
    phdr.group_element = _sentinal_group;
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
}

// erase all EEPROM variables by re-writing the header and adding
// a sentinal
void AP_Param::erase_all(void)
{
    struct EEPROM_header hdr;

    serialDebug("erase_all");

    // write the header
    hdr.magic[0] = k_EEPROM_magic0;
    hdr.magic[1] = k_EEPROM_magic1;
    hdr.revision = k_EEPROM_revision;
    hdr.spare    = 0;
    eeprom_write_check(&hdr, 0, sizeof(hdr));

    // add a sentinal directly after the header
    write_sentinal(sizeof(struct EEPROM_header));
}

// validate a group info table
bool AP_Param::check_group_info(const struct AP_Param::GroupInfo *group_info,
                                uint16_t *total_size,
                                uint8_t group_shift)
{
    uint8_t type;
    int8_t max_idx = -1;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            // a nested group
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            if (group_shift + _group_level_shift >= _group_bits) {
                // double nesting of groups is not allowed
                return false;
            }
            if (ginfo == NULL ||
                !check_group_info(ginfo, total_size, group_shift + _group_level_shift)) {
                return false;
            }
            continue;
        }
        uint8_t idx = PGM_UINT8(&group_info[i].idx);
        if (idx >= (1<<_group_level_shift)) {
            // passed limit on table size
            return false;
        }
        if ((int8_t)idx <= max_idx) {
            // the indexes must be in increasing order
            return false;
        }
        max_idx = (int8_t)idx;
        uint8_t size = type_size((enum ap_var_type)type);
        if (size == 0) {
            // not a valid type
            return false;
        }
        (*total_size) += size + sizeof(struct Param_header);
    }
    return true;
}

// validate the _var_info[] table
bool AP_Param::check_var_info(void)
{
    uint16_t total_size = sizeof(struct EEPROM_header);

    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            if (i == 0) {
                // first element can't be a group, for first() call
                return false;
            }
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            if (group_info == NULL ||
                !check_group_info(group_info, &total_size, 0)) {
                return false;
            }
        } else {
            uint8_t size = type_size((enum ap_var_type)type);
            if (size == 0) {
                // not a valid type - the top level list can't contain
                // AP_PARAM_NONE
                return false;
            }
            total_size += size + sizeof(struct Param_header);
        }
    }
    if (total_size > _eeprom_size) {
        serialDebug("total_size %u exceeds _eeprom_size %u",
                    total_size, _eeprom_size);
        return false;
    }
    return true;
}


// setup the _var_info[] table
bool AP_Param::setup(const AP_Param::Info *info, uint8_t num_vars, uint16_t eeprom_size)
{
    struct EEPROM_header hdr;

    _eeprom_size = eeprom_size;
    _var_info = info;
    _num_vars = num_vars;

    if (!check_var_info()) {
        return false;
    }

    serialDebug("setup %u vars", (unsigned)num_vars);

    // check the header
    eeprom_read_block(&hdr, 0, sizeof(hdr));
    if (hdr.magic[0] != k_EEPROM_magic0 ||
        hdr.magic[1] != k_EEPROM_magic1 ||
        hdr.revision != k_EEPROM_revision) {
        // header doesn't match. We can't recover any variables. Wipe
        // the header and setup the sentinal directly after the header
        serialDebug("bad header in setup - erasing");
        erase_all();
    }

    return true;
}

// find the info structure given a header and a group_info table
// return the Info structure and a pointer to the variables storage
const struct AP_Param::Info *AP_Param::find_by_header_group(struct Param_header phdr, void **ptr,
                                                            uint8_t vindex,
                                                            const struct GroupInfo *group_info,
                                                            uint8_t group_base,
                                                            uint8_t group_shift)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            // a nested group
            if (group_shift + _group_level_shift >= _group_bits) {
                // too deeply nested - this should have been caught by
                // setup() !
                return NULL;
            }
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            const struct AP_Param::Info *ret = find_by_header_group(phdr, ptr, vindex, ginfo,
                                                                    GROUP_ID(group_info, group_base, i, group_shift),
                                                                    group_shift + _group_level_shift);
            if (ret != NULL) {
                return ret;
            }
            continue;
        }
        if (GROUP_ID(group_info, group_base, i, group_shift) == phdr.group_element) {
            // found a group element
            *ptr = (void*)(PGM_POINTER(&_var_info[vindex].ptr) + PGM_UINT16(&group_info[i].offset));
            return &_var_info[vindex];
        }
    }
    return NULL;
}

// find the info structure given a header
// return the Info structure and a pointer to the variables storage
const struct AP_Param::Info *AP_Param::find_by_header(struct Param_header phdr, void **ptr)
{
    // loop over all named variables
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        uint8_t key = PGM_UINT8(&_var_info[i].key);
        if (key != phdr.key) {
            // not the right key
            continue;
        }
        if (type != AP_PARAM_GROUP) {
            // if its not a group then we are done
            *ptr = (void*)PGM_POINTER(&_var_info[i].ptr);
            return &_var_info[i];
        }

        const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
        return find_by_header_group(phdr, ptr, i, group_info, 0, 0);
    }
    return NULL;
}

// find the info structure for a variable in a group
const struct AP_Param::Info *AP_Param::find_var_info_group(const struct GroupInfo *group_info,
                                                           uint8_t vindex,
                                                           uint8_t group_base,
                                                           uint8_t group_shift,
                                                           uint8_t *group_element,
                                                           const struct GroupInfo **group_ret)
{
    uintptr_t base = PGM_POINTER(&_var_info[vindex].ptr);
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            // a nested group
            if (group_shift + _group_level_shift >= _group_bits) {
                // too deeply nested - this should have been caught by
                // setup() !
                return NULL;
            }
            const struct AP_Param::Info *info;
            info = find_var_info_group(ginfo, vindex,
                                       GROUP_ID(group_info, group_base, i, group_shift),
                                       group_shift + _group_level_shift,
                                       group_element,
                                       group_ret);
            if (info != NULL) {
                return info;
            }
        } else if ((uintptr_t)this == base + PGM_POINTER(&group_info[i].offset)) {
            *group_element = GROUP_ID(group_info, group_base, i, group_shift);
            *group_ret = &group_info[i];
            return &_var_info[vindex];
        }
    }
    return NULL;
}

// find the info structure for a variable
const struct AP_Param::Info *AP_Param::find_var_info(uint8_t *group_element,
                                                     const struct GroupInfo **group_ret)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        uintptr_t base = PGM_POINTER(&_var_info[i].ptr);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            const struct AP_Param::Info *info;
            info = find_var_info_group(group_info, i, 0, 0, group_element, group_ret);
            if (info != NULL) {
                return info;
            }
        } else if (base == (uintptr_t)this) {
            *group_element = 0;
            *group_ret = NULL;
            return &_var_info[i];
        }
    }
    return NULL;
}

// return the storage size for a AP_PARAM_* type
const uint8_t AP_Param::type_size(enum ap_var_type type)
{
    switch (type) {
    case AP_PARAM_NONE:
    case AP_PARAM_GROUP:
        return 0;
    case AP_PARAM_INT8:
        return 1;
    case AP_PARAM_INT16:
        return 2;
    case AP_PARAM_INT32:
        return 4;
    case AP_PARAM_FLOAT:
        return 4;
    case AP_PARAM_VECTOR3F:
        return 3*4;
    case AP_PARAM_VECTOR6F:
        return 6*4;
    case AP_PARAM_MATRIX3F:
        return 3*3*4;
    }
    serialDebug("unknown type %u\n", type);
    return 0;
}

// scan the EEPROM looking for a given variable by header content
// return true if found, along with the offset in the EEPROM where
// the variable is stored
// if not found return the offset of the sentinal, or
bool AP_Param::scan(const AP_Param::Param_header *target, uint16_t *pofs)
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AP_Param::EEPROM_header);
    while (ofs < _eeprom_size) {
        eeprom_read_block(&phdr, (void *)ofs, sizeof(phdr));
        if (phdr.type == target->type &&
            phdr.key == target->key &&
            phdr.group_element == target->group_element) {
            // found it
            *pofs = ofs;
            return true;
        }
        // note that this is an ||, not an &&, as this makes us more
        // robust to power off while adding a variable to EEPROM
        if (phdr.type == _sentinal_type ||
            phdr.key == _sentinal_key ||
            phdr.group_element == _sentinal_group) {
            // we've reached the sentinal
            *pofs = ofs;
            return false;
        }
        ofs += type_size((enum ap_var_type)phdr.type) + sizeof(phdr);
    }
    *pofs = ~0;
    serialDebug("scan past end of eeprom");
    return false;
}

// Copy the variable's whole name to the supplied buffer.
//
// If the variable is a group member, prepend the group name.
//
void AP_Param::copy_name(char *buffer, size_t buffer_size)
{
    uint8_t group_element;
    const struct GroupInfo *ginfo;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo);
    if (info == NULL) {
        *buffer = 0;
        serialDebug("no info found");
        return;
    }
    strncpy_P(buffer, info->name, buffer_size);
    if (ginfo != NULL) {
        uint8_t len = strnlen(buffer, buffer_size);
        if (len < buffer_size) {
            strncpy_P(&buffer[len], ginfo->name, buffer_size-len);
        }
    }
}

// Find a variable by name in a group
AP_Param *
AP_Param::find_group(const char *name, uint8_t vindex, const struct GroupInfo *group_info, enum ap_var_type *ptype)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            AP_Param *ap = find_group(name, vindex, ginfo, ptype);
            if (ap != NULL) {
                return ap;
            }
        } else if (strcasecmp_P(name, group_info[i].name) == 0) {
            uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)(p + PGM_POINTER(&group_info[i].offset));
        }
    }
    return NULL;
}


// Find a variable by name.
//
AP_Param *
AP_Param::find(const char *name, enum ap_var_type *ptype)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            uint8_t len = strnlen_P(_var_info[i].name, AP_MAX_NAME_SIZE);
            if (strncmp_P(name, _var_info[i].name, len) != 0) {
                continue;
            }
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            return find_group(name + len, i, group_info, ptype);
        } else if (strcasecmp_P(name, _var_info[i].name) == 0) {
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)PGM_POINTER(&_var_info[i].ptr);
        }
    }
    return NULL;
}

// Save the variable to EEPROM, if supported
//
bool AP_Param::save(void)
{
    uint8_t group_element = 0;
    const struct GroupInfo *ginfo;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo);

    if (info == NULL) {
        // we don't have any info on how to store it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to store the variable
    if (ginfo != NULL) {
        phdr.type = PGM_UINT8(&ginfo->type);
    } else {
        phdr.type = PGM_UINT8(&info->type);
    }
    phdr.key  = PGM_UINT8(&info->key);
    phdr.group_element = group_element;

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (scan(&phdr, &ofs)) {
        // found an existing copy of the variable
        eeprom_write_check(this, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
        return true;
    }
    if (ofs == (uint16_t)~0) {
        return false;
    }

    // write a new sentinal, then the data, then the header
    write_sentinal(ofs + sizeof(phdr) + type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(this, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
    return true;
}

// Load the variable from EEPROM, if supported
//
bool AP_Param::load(void)
{
    uint8_t group_element = 0;
    const struct GroupInfo *ginfo;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo);
    if (info == NULL) {
        // we don't have any info on how to load it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to match the variable
    if (ginfo != NULL) {
        phdr.type = PGM_UINT8(&ginfo->type);
    } else {
        phdr.type = PGM_UINT8(&info->type);
    }
    phdr.key  = PGM_UINT8(&info->key);
    phdr.group_element = group_element;

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (!scan(&phdr, &ofs)) {
        return false;
    }

    // found it
    eeprom_read_block(this, (void*)(ofs+sizeof(phdr)), type_size((enum ap_var_type)phdr.type));
    return true;
}

// Load all variables from EEPROM
//
bool AP_Param::load_all(void)
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AP_Param::EEPROM_header);
    while (ofs < _eeprom_size) {
        eeprom_read_block(&phdr, (void *)ofs, sizeof(phdr));
        // note that this is an || not an && for robustness
        // against power off while adding a variable
        if (phdr.type == _sentinal_type ||
            phdr.key == _sentinal_key ||
            phdr.group_element == _sentinal_group) {
            // we've reached the sentinal
            return true;
        }

        const struct AP_Param::Info *info;
        void *ptr;

        info = find_by_header(phdr, &ptr);
        if (info != NULL) {
            eeprom_read_block(ptr, (void*)(ofs+sizeof(phdr)), type_size((enum ap_var_type)phdr.type));
        }

        ofs += type_size((enum ap_var_type)phdr.type) + sizeof(phdr);
    }

    // we didn't find the sentinal
    serialDebug("no sentinal in load_all");
    return false;
}


// return the first variable in _var_info
AP_Param *AP_Param::first(uint16_t *token, enum ap_var_type *ptype)
{
    *token = 0;
    if (_num_vars == 0) {
        return NULL;
    }
    if (ptype != NULL) {
        *ptype = (enum ap_var_type)PGM_UINT8(&_var_info[0].type);
    }
    return (AP_Param *)(PGM_POINTER(&_var_info[0].ptr));
}

/// Returns the next variable in a group, recursing into groups
/// as needed
AP_Param *AP_Param::next_group(uint8_t vindex, const struct GroupInfo *group_info,
                               bool *found_current,
                               uint8_t group_base,
                               uint8_t group_shift,
                               uint16_t *token,
                               enum ap_var_type *ptype)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            // a nested group
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            AP_Param *ap;
            ap = next_group(vindex, ginfo, found_current, GROUP_ID(group_info, group_base, i, group_shift),
                            group_shift + _group_level_shift, token, ptype);
            if (ap != NULL) {
                return ap;
            }
        } else {
            if (*found_current) {
                // got a new one
                (*token) = ((uint16_t)GROUP_ID(group_info, group_base, i, group_shift)<<8) | vindex;
                if (ptype != NULL) {
                    *ptype = (enum ap_var_type)type;
                }
                return (AP_Param*)(PGM_POINTER(&_var_info[vindex].ptr) + PGM_UINT16(&group_info[i].offset));
            }
            if (GROUP_ID(group_info, group_base, i, group_shift) == (uint8_t)((*token)>>8)) {
                *found_current = true;
            }
        }
    }
    return NULL;
}

/// Returns the next variable in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next(uint16_t *token, enum ap_var_type *ptype)
{
    uint8_t i = (*token)&0xFF;
    bool found_current = false;
    if (i >= _num_vars) {
        // illegal token
        return NULL;
    }
    uint8_t type = PGM_UINT8(&_var_info[i].type);
    if (type != AP_PARAM_GROUP) {
        i++;
        found_current = true;
    }
    for (; i<_num_vars; i++) {
        type = PGM_UINT8(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            AP_Param *ap = next_group(i, group_info, &found_current, 0, 0, token, ptype);
            if (ap != NULL) {
                return ap;
            }
        } else {
            // found the next one
            (*token) = i;
            if (ptype != NULL) {
                *ptype = (enum ap_var_type)type;
            }
            return (AP_Param *)(PGM_POINTER(&_var_info[i].ptr));
        }
    }
    return NULL;
}

/// Returns the next scalar in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next_scalar(uint16_t *token, enum ap_var_type *ptype)
{
    AP_Param *ap;
    enum ap_var_type type;
    while ((ap = next(token, &type)) != NULL && type > AP_PARAM_FLOAT) ;
    if (ap != NULL && ptype != NULL) {
        *ptype = type;
    }
    return ap;
}


/// cast a variable to a float given its type
float AP_Param::cast_to_float(enum ap_var_type type)
{
    switch (type) {
    case AP_PARAM_INT8:
        return ((AP_Int8 *)this)->cast_to_float();
    case AP_PARAM_INT16:
        return ((AP_Int16 *)this)->cast_to_float();
    case AP_PARAM_INT32:
        return ((AP_Int32 *)this)->cast_to_float();
    case AP_PARAM_FLOAT:
        return ((AP_Float *)this)->cast_to_float();
    default:
        return NAN;
    }
}


// print the value of all variables
void AP_Param::show_all(void)
{
    uint16_t token;
    AP_Param *ap;
    enum ap_var_type type;

    for (ap=AP_Param::first(&token, &type);
         ap;
         ap=AP_Param::next_scalar(&token, &type)) {
        char s[AP_MAX_NAME_SIZE+1];
        ap->copy_name(s, sizeof(s));
        s[AP_MAX_NAME_SIZE] = 0;

        switch (type) {
        case AP_PARAM_INT8:
            Serial.printf_P(PSTR("%s: %d\n"), s, (int)((AP_Int8 *)ap)->get());
            break;
        case AP_PARAM_INT16:
            Serial.printf_P(PSTR("%s: %d\n"), s, (int)((AP_Int16 *)ap)->get());
            break;
        case AP_PARAM_INT32:
            Serial.printf_P(PSTR("%s: %ld\n"), s, (long)((AP_Int32 *)ap)->get());
            break;
        case AP_PARAM_FLOAT:
            Serial.printf_P(PSTR("%s: %f\n"), s, ((AP_Float *)ap)->get());
            break;
        default:
            break;
        }
    }
}
