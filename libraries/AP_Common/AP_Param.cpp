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
#include <AP_Math.h>

#include <math.h>
#include <string.h>

// #define ENABLE_FASTSERIAL_DEBUG

#ifdef ENABLE_FASTSERIAL_DEBUG
 # define serialDebug(fmt, args ...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); delay(0); } while(0)
#else
 # define serialDebug(fmt, args ...)
#endif

// some useful progmem macros
#define PGM_UINT8(addr) pgm_read_byte((const prog_char *)addr)
#define PGM_UINT16(addr) pgm_read_word((const uint16_t *)addr)
#define PGM_FLOAT(addr) pgm_read_float((const float *)addr)
#define PGM_POINTER(addr) pgm_read_pointer((const void *)addr)

// the 'GROUP_ID' of a element of a group is the 8 bit identifier used
// to distinguish between this element of the group and other elements
// of the same group. It is calculated using a bit shift per level of
// nesting, so the first level of nesting gets 4 bits, and the next
// level gets the next 4 bits. This limits groups to having at most 16
// elements.
#define GROUP_ID(grpinfo, base, i, shift) ((base)+(((uint16_t)PGM_UINT8(&grpinfo[i].idx))<<(shift)))

// Note about AP_Vector3f handling.
// The code has special cases for AP_Vector3f to allow it to be viewed
// as both a single 3 element vector and as a set of 3 AP_Float
// variables. This is done to make it possible for MAVLink to see
// vectors as parameters, which allows users to save their compass
// offsets in MAVLink parameter files. The code involves quite a few
// special cases which could be generalised to any vector/matrix type
// if we end up needing this behaviour for other than AP_Vector3f


// static member variables for AP_Param.
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
        uint8_t v = eeprom_read_byte((const uint8_t *)(uintptr_t)ofs);
        if (v != *b) {
            eeprom_write_byte((uint8_t *)(uintptr_t)ofs, *b);
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
bool AP_Param::check_group_info(const struct AP_Param::GroupInfo *  group_info,
                                uint16_t *                          total_size,
                                uint8_t                             group_shift)
{
    uint8_t type;
    int8_t max_idx = -1;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
#ifdef AP_NESTED_GROUPS_ENABLED
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
#endif // AP_NESTED_GROUPS_ENABLED
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

// check for duplicate key values
bool AP_Param::duplicate_key(uint8_t vindex, uint8_t key)
{
    for (uint8_t i=vindex+1; i<_num_vars; i++) {
        uint8_t key2 = PGM_UINT8(&_var_info[i].key);
        if (key2 == key) {
            // no duplicate keys allowed
            return true;
        }
    }
    return false;
}

// validate the _var_info[] table
bool AP_Param::check_var_info(void)
{
    uint16_t total_size = sizeof(struct EEPROM_header);

    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        uint8_t key = PGM_UINT8(&_var_info[i].key);
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
        if (duplicate_key(i, key)) {
            return false;
        }
    }

    // we no longer check if total_size is larger than _eeprom_size,
    // as we allow for more variables than could fit, relying on not
    // saving default values

    return true;
}


// setup the _var_info[] table
bool AP_Param::setup(const struct AP_Param::Info *info, uint16_t eeprom_size)
{
    struct EEPROM_header hdr;
    uint8_t i;

    _eeprom_size = eeprom_size;
    _var_info = info;

    for (i=0; PGM_UINT8(&info[i].type) != AP_PARAM_NONE; i++) ;
    _num_vars = i;

    if (!check_var_info()) {
        return false;
    }

    serialDebug("setup %u vars", (unsigned)_num_vars);

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

// check if AP_Param has been initialised
bool AP_Param::initialised(void)
{
    return _var_info != NULL;
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
#ifdef AP_NESTED_GROUPS_ENABLED
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
#endif // AP_NESTED_GROUPS_ENABLED
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
const struct AP_Param::Info *AP_Param::find_var_info_group(const struct GroupInfo * group_info,
                                                           uint8_t                  vindex,
                                                           uint8_t                  group_base,
                                                           uint8_t                  group_shift,
                                                           uint8_t *                group_element,
                                                           const struct GroupInfo **group_ret,
                                                           uint8_t *                idx)
{
    uintptr_t base = PGM_POINTER(&_var_info[vindex].ptr);
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        uintptr_t ofs = PGM_POINTER(&group_info[i].offset);
#ifdef AP_NESTED_GROUPS_ENABLED
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
                                       group_ret,
                                       idx);
            if (info != NULL) {
                return info;
            }
        } else // Forgive the poor formatting - if continues below.
#endif // AP_NESTED_GROUPS_ENABLED
        if ((uintptr_t) this == base + ofs) {
            *group_element = GROUP_ID(group_info, group_base, i, group_shift);
            *group_ret = &group_info[i];
            *idx = 0;
            return &_var_info[vindex];
        } else if (type == AP_PARAM_VECTOR3F &&
                   (base+ofs+sizeof(float) == (uintptr_t) this ||
                    base+ofs+2*sizeof(float) == (uintptr_t) this)) {
            // we are inside a Vector3f. We need to work out which
            // element of the vector the current object refers to.
            *idx = (((uintptr_t) this) - (base+ofs))/sizeof(float);
            *group_element = GROUP_ID(group_info, group_base, i, group_shift);
            *group_ret = &group_info[i];
            return &_var_info[vindex];
        }
    }
    return NULL;
}

// find the info structure for a variable
const struct AP_Param::Info *AP_Param::find_var_info(uint8_t *                  group_element,
                                                     const struct GroupInfo **  group_ret,
                                                     uint8_t *                  idx)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        uintptr_t base = PGM_POINTER(&_var_info[i].ptr);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            const struct AP_Param::Info *info;
            info = find_var_info_group(group_info, i, 0, 0, group_element, group_ret, idx);
            if (info != NULL) {
                return info;
            }
        } else if (base == (uintptr_t) this) {
            *group_element = 0;
            *group_ret = NULL;
            *idx = 0;
            return &_var_info[i];
        } else if (type == AP_PARAM_VECTOR3F &&
                   (base+sizeof(float) == (uintptr_t) this ||
                    base+2*sizeof(float) == (uintptr_t) this)) {
            // we are inside a Vector3f. Work out which element we are
            // referring to.
            *idx = (((uintptr_t) this) - base)/sizeof(float);
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
        eeprom_read_block(&phdr, (void *)(uintptr_t)ofs, sizeof(phdr));
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

// add a X,Y,Z suffix to the name of a Vector3f element
void AP_Param::add_vector3f_suffix(char *buffer, size_t buffer_size, uint8_t idx)
{
    uint8_t len = strnlen(buffer, buffer_size);
    if ((size_t)(len+3) >= buffer_size) {
        // the suffix doesn't fit
        return;
    }
    buffer[len] = '_';
    if (idx == 0) {
        buffer[len+1] = 'X';
    } else if (idx == 1) {
        buffer[len+1] = 'Y';
    } else if (idx == 2) {
        buffer[len+1] = 'Z';
    }
    buffer[len+2] = 0;
}

// Copy the variable's whole name to the supplied buffer.
//
// If the variable is a group member, prepend the group name.
//
void AP_Param::copy_name(char *buffer, size_t buffer_size, bool force_scalar)
{
    uint8_t group_element;
    const struct GroupInfo *ginfo;
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo, &idx);
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
        if ((force_scalar || idx != 0) && AP_PARAM_VECTOR3F == PGM_UINT8(&ginfo->type)) {
            // the caller wants a specific element in a Vector3f
            add_vector3f_suffix(buffer, buffer_size, idx);
        }
    } else if ((force_scalar || idx != 0) && AP_PARAM_VECTOR3F == PGM_UINT8(&info->type)) {
        add_vector3f_suffix(buffer, buffer_size, idx);
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
#ifdef AP_NESTED_GROUPS_ENABLED
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            AP_Param *ap = find_group(name, vindex, ginfo, ptype);
            if (ap != NULL) {
                return ap;
            }
        } else
#endif // AP_NESTED_GROUPS_ENABLED
        if (strcasecmp_P(name, group_info[i].name) == 0) {
            uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)(p + PGM_POINTER(&group_info[i].offset));
        } else if (type == AP_PARAM_VECTOR3F) {
            // special case for finding Vector3f elements
            uint8_t suffix_len = strlen_P(group_info[i].name);
            if (strncmp_P(name, group_info[i].name, suffix_len) == 0 &&
                name[suffix_len] == '_' &&
                name[suffix_len+1] != 0 &&
                name[suffix_len+2] == 0) {
                uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
                AP_Float *v = (AP_Float *)(p + PGM_POINTER(&group_info[i].offset));
                *ptype = AP_PARAM_FLOAT;
                switch (name[suffix_len+1]) {
                case 'X':
                    return (AP_Float *)&v[0];
                case 'Y':
                    return (AP_Float *)&v[1];
                case 'Z':
                    return (AP_Float *)&v[2];
                }
            }
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
            AP_Param *ap = find_group(name + len, i, group_info, ptype);
            if (ap != NULL) {
                return ap;
            }
            // we continue looking as we want to allow top level
            // parameter to have the same prefix name as group
            // parameters, for example CAM_P_G
        } else if (strcasecmp_P(name, _var_info[i].name) == 0) {
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)PGM_POINTER(&_var_info[i].ptr);
        }
    }
    return NULL;
}

// Find a variable by index. Note that this is quite slow.
//
AP_Param *
AP_Param::find_by_index(uint16_t idx, enum ap_var_type *ptype)
{
    ParamToken token;
    AP_Param *ap;
    uint16_t count=0;
    for (ap=AP_Param::first(&token, ptype);
         ap && count < idx;
         ap=AP_Param::next_scalar(&token, ptype)) {
        count++;
    }
    return ap;    
}

// Save the variable to EEPROM, if supported
//
bool AP_Param::save(void)
{
    uint8_t group_element = 0;
    const struct GroupInfo *ginfo;
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo, &idx);
    const AP_Param *ap;

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

    ap = this;
    if (phdr.type != AP_PARAM_VECTOR3F && idx != 0) {
        // only vector3f can have non-zero idx for now
        return false;
    }
    if (idx != 0) {
        ap = (const AP_Param *)((uintptr_t)ap) - (idx*sizeof(float));
    }

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (scan(&phdr, &ofs)) {
        // found an existing copy of the variable
        eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
        return true;
    }
    if (ofs == (uint16_t) ~0) {
        return false;
    }

    // if the value is the default value then don't save
    if (phdr.type <= AP_PARAM_FLOAT) {
        float v1 = cast_to_float((enum ap_var_type)phdr.type);
        float v2;
        if (ginfo != NULL) {
            v2 = PGM_FLOAT(&ginfo->def_value);
        } else {
            v2 = PGM_FLOAT(&info->def_value);
        }
        if (v1 == v2) {
            return true;
        }
        if (phdr.type != AP_PARAM_INT32 &&
            (fabs(v1-v2) < 0.0001*fabs(v1))) {
            // for other than 32 bit integers, we accept values within
            // 0.01 percent of the current value as being the same
            return true;
        }
    }

    if (ofs+type_size((enum ap_var_type)phdr.type)+2*sizeof(phdr) >= _eeprom_size) {
        // we are out of room for saving variables
        return false;
    }

    // write a new sentinal, then the data, then the header
    write_sentinal(ofs + sizeof(phdr) + type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
    return true;
}

// Load the variable from EEPROM, if supported
//
bool AP_Param::load(void)
{
    uint8_t group_element = 0;
    const struct GroupInfo *ginfo;
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo, &idx);
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
        // if the value isn't stored in EEPROM then set the default value
        if (ginfo != NULL) {
            uintptr_t base = PGM_POINTER(&info->ptr);
            set_value((enum ap_var_type)phdr.type, (void*)(base + PGM_UINT16(&ginfo->offset)),
                      PGM_FLOAT(&ginfo->def_value));
        } else {
            set_value((enum ap_var_type)phdr.type, (void*)PGM_POINTER(&info->ptr), PGM_FLOAT(&info->def_value));
        }
        return false;
    }

    if (phdr.type != AP_PARAM_VECTOR3F && idx != 0) {
        // only vector3f can have non-zero idx for now
        return false;
    }

    AP_Param *ap;
    ap = this;
    if (idx != 0) {
        ap = (AP_Param *)((uintptr_t)ap) - (idx*sizeof(float));
    }

    // found it
    eeprom_read_block(ap, (void*)(ofs+sizeof(phdr)), type_size((enum ap_var_type)phdr.type));
    return true;
}

// set a AP_Param variable to a specified value
void AP_Param::set_value(enum ap_var_type type, void *ptr, float def_value)
{
    switch (type) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)ptr)->set(def_value);
        break;
    case AP_PARAM_INT16:
        ((AP_Int16 *)ptr)->set(def_value);
        break;
    case AP_PARAM_INT32:
        ((AP_Int32 *)ptr)->set(def_value);
        break;
    case AP_PARAM_FLOAT:
        ((AP_Float *)ptr)->set(def_value);
        break;
    default:
        break;
    }
}

// load default values for scalars in a group
void AP_Param::load_defaults_group(const struct GroupInfo *group_info, uintptr_t base)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            load_defaults_group(ginfo, base);
        } else if (type <= AP_PARAM_FLOAT) {
            void *ptr = (void *)(base + PGM_UINT16(&group_info[i].offset));
            set_value((enum ap_var_type)type, ptr, PGM_FLOAT(&group_info[i].def_value));
        }
    }
}


// load default values for all scalars
void AP_Param::load_defaults(void)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            uintptr_t base = PGM_POINTER(&_var_info[i].ptr);
            load_defaults_group(group_info, base);
        } else if (type <= AP_PARAM_FLOAT) {
            void *ptr = (void*)PGM_POINTER(&_var_info[i].ptr);
            set_value((enum ap_var_type)type, ptr, PGM_FLOAT(&_var_info[i].def_value));
        }
    }
}


// Load all variables from EEPROM
//
bool AP_Param::load_all(void)
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AP_Param::EEPROM_header);

    while (ofs < _eeprom_size) {
        eeprom_read_block(&phdr, (void *)(uintptr_t)ofs, sizeof(phdr));
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
AP_Param *AP_Param::first(ParamToken *token, enum ap_var_type *ptype)
{
    token->key = 0;
    token->group_element = 0;
    token->idx = 0;
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
                               ParamToken *token,
                               enum ap_var_type *ptype)
{
    enum ap_var_type type;
    for (uint8_t i=0;
         (type=(enum ap_var_type)PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
#ifdef AP_NESTED_GROUPS_ENABLED
        if (type == AP_PARAM_GROUP) {
            // a nested group
            const struct GroupInfo *ginfo = (const struct GroupInfo *)PGM_POINTER(&group_info[i].group_info);
            AP_Param *ap;
            ap = next_group(vindex, ginfo, found_current, GROUP_ID(group_info, group_base, i, group_shift),
                            group_shift + _group_level_shift, token, ptype);
            if (ap != NULL) {
                return ap;
            }
        } else
#endif // AP_NESTED_GROUPS_ENABLED
        {
            if (*found_current) {
                // got a new one
                token->key = vindex;
                token->group_element = GROUP_ID(group_info, group_base, i, group_shift);
                token->idx = 0;
                if (ptype != NULL) {
                    *ptype = type;
                }
                return (AP_Param*)(PGM_POINTER(&_var_info[vindex].ptr) + PGM_UINT16(&group_info[i].offset));
            }
            if (GROUP_ID(group_info, group_base, i, group_shift) == token->group_element) {
                *found_current = true;
                if (type == AP_PARAM_VECTOR3F && token->idx < 3) {
                    // return the next element of the vector as a
                    // float
                    token->idx++;
                    if (ptype != NULL) {
                        *ptype = AP_PARAM_FLOAT;
                    }
                    uintptr_t ofs = (uintptr_t)PGM_POINTER(&_var_info[vindex].ptr) + PGM_UINT16(&group_info[i].offset);
                    ofs += sizeof(float)*(token->idx-1);
                    return (AP_Param *)ofs;
                }
            }
        }
    }
    return NULL;
}

/// Returns the next variable in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next(ParamToken *token, enum ap_var_type *ptype)
{
    uint8_t i = token->key;
    bool found_current = false;
    if (i >= _num_vars) {
        // illegal token
        return NULL;
    }
    enum ap_var_type type = (enum ap_var_type)PGM_UINT8(&_var_info[i].type);

    // allow Vector3f to be seen as 3 variables. First as a vector,
    // then as 3 separate floats
    if (type == AP_PARAM_VECTOR3F && token->idx < 3) {
        token->idx++;
        if (ptype != NULL) {
            *ptype = AP_PARAM_FLOAT;
        }
        return (AP_Param *)(((token->idx-1)*sizeof(float))+(uintptr_t)PGM_POINTER(&_var_info[i].ptr));
    }

    if (type != AP_PARAM_GROUP) {
        i++;
        found_current = true;
    }
    for (; i<_num_vars; i++) {
        type = (enum ap_var_type)PGM_UINT8(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            AP_Param *ap = next_group(i, group_info, &found_current, 0, 0, token, ptype);
            if (ap != NULL) {
                return ap;
            }
        } else {
            // found the next one
            token->key = i;
            token->group_element = 0;
            token->idx = 0;
            if (ptype != NULL) {
                *ptype = type;
            }
            return (AP_Param *)(PGM_POINTER(&_var_info[i].ptr));
        }
    }
    return NULL;
}

/// Returns the next scalar in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next_scalar(ParamToken *token, enum ap_var_type *ptype)
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
    ParamToken token;
    AP_Param *ap;
    enum ap_var_type type;

    for (ap=AP_Param::first(&token, &type);
         ap;
         ap=AP_Param::next_scalar(&token, &type)) {
        char s[AP_MAX_NAME_SIZE+1];
        ap->copy_name(s, sizeof(s), true);
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
