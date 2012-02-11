// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file   AP_Param.cpp
/// @brief  The AP variable store.


#include <AP_Common.h>

#include <math.h>
#include <string.h>

// #define ENABLE_FASTSERIAL_DEBUG

#ifdef ENABLE_FASTSERIAL_DEBUG
# include <FastSerial.h>
# define serialDebug(fmt, args...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
#else
# define serialDebug(fmt, args...)
#endif

// Static member variables for AP_Param.
//

// number of rows in the _var_info[] table
uint16_t AP_Param::_num_vars;

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
    phdr.type = AP_PARAM_NONE;
    phdr.key  = 0;
    phdr.group_element = 0;
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
}

// erase all EEPROM variables by re-writing the header and adding
// a sentinal
void AP_Param::erase_all(void)
{
    struct EEPROM_header hdr;

    serialDebug("erase_all");

    // write the header
    hdr.magic = k_EEPROM_magic;
    hdr.revision = k_EEPROM_revision;
    hdr.spare = 0;
    eeprom_write_check(&hdr, 0, sizeof(hdr));

    // add a sentinal directly after the header
    write_sentinal(sizeof(struct EEPROM_header));
}

// setup the _var_info[] table
bool AP_Param::setup(const AP_Param::Info *info, uint16_t num_vars)
{
    struct EEPROM_header hdr;

    _var_info = info;
    _num_vars = num_vars;

    serialDebug("setup %u vars", (unsigned)num_vars);

    // check the header
    eeprom_read_block(&hdr, 0, sizeof(hdr));
    if (hdr.magic != k_EEPROM_magic ||
        hdr.revision != k_EEPROM_revision) {
        // header doesn't match. We can't recover any variables. Wipe
        // the header and setup the sentinal directly after the header
        serialDebug("bad header in setup - erasing");
        erase_all();
    }

    return true;
}

// find the info structure given a header
// return the Info structure and a pointer to the variables storage
const struct AP_Param::Info *AP_Param::find_by_header(struct Param_header phdr, void **ptr)
{
    // loop over all named variables
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = pgm_read_byte(&_var_info[i].type);
        uint16_t key = pgm_read_word(&_var_info[i].key);
        if (key != phdr.key) {
            // not the right key
            continue;
        }
        if (type != AP_PARAM_GROUP) {
            // if its not a group then we are done
            *ptr = (void*)pgm_read_pointer(&_var_info[i].ptr);
            return &_var_info[i];
        }

        // for groups we need to check each group element
        const struct GroupInfo *group_info = (const struct GroupInfo *)pgm_read_pointer(&_var_info[i].group_info);
        for (uint8_t j=0;
             pgm_read_byte(&group_info[j].type) != AP_PARAM_NONE;
             j++) {
            if (j == phdr.group_element) {
                // found a group element
                *ptr = (void*)(pgm_read_pointer(&_var_info[i].ptr) + pgm_read_word(&group_info[j].offset));
                return &_var_info[i];
            }
        }
    }
    serialDebug("failed to find type=%u key=%u\n",
                (unsigned)phdr.type,
                (unsigned)phdr.key);
    return NULL;
}

// find the info structure for a variable
const struct AP_Param::Info *AP_Param::find_var_info(uint8_t *group_element)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = pgm_read_byte(&_var_info[i].type);
        uintptr_t base = pgm_read_pointer(&_var_info[i].ptr);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)pgm_read_pointer(&_var_info[i].group_info);
            for (uint8_t j=0;
                 (type=pgm_read_byte(&group_info[j].type)) != AP_PARAM_NONE ;
                 j++) {
                if ((uintptr_t)this == base + pgm_read_pointer(&group_info[j].offset)) {
                    if (group_element != NULL) {
                        *group_element = j;
                    }
                    return &_var_info[i];
                }
            }
        } else if (base == (uintptr_t)this) {
            *group_element = 0;
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
    while (ofs < k_EEPROM_size) {
        eeprom_read_block(&phdr, (const void *)ofs, sizeof(phdr));
        if (phdr.type == target->type &&
            phdr.key == target->key &&
            phdr.group_element == target->group_element) {
            // found it
            *pofs = ofs;
            return true;
        }
        if (phdr.type == AP_PARAM_NONE &&
            phdr.key == 0) {
            // we've reached the sentinal
            *pofs = ofs;
            serialDebug("failed to scan type=%u key=%u\n",
                        (unsigned)target->type,
                        (unsigned)target->key);
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
    const struct AP_Param::Info *info = find_var_info(&group_element);
    if (info == NULL) {
        *buffer = 0;
        serialDebug("no info found");
        return;
    }
    strncpy_P(buffer, info->name, buffer_size);
    if (pgm_read_byte(&info->type) == AP_PARAM_GROUP) {
        uint8_t len = strnlen(buffer, buffer_size);
        if (len < buffer_size) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)pgm_read_pointer(&info->group_info);
            strncpy_P(&buffer[len], group_info->name, buffer_size-len);
        }
    }
}

// Find a variable by name.
//
AP_Param *
AP_Param::find(const char *name)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = pgm_read_byte(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            uint8_t len = strnlen_P(_var_info[i].name, AP_MAX_NAME_SIZE);
            if (strncmp_P(name, _var_info[i].name, len) != 0) {
                continue;
            }
            const struct GroupInfo *group_info = (const struct GroupInfo *)pgm_read_pointer(&_var_info[i].group_info);
            for (uint8_t j=0;
                 (type=pgm_read_byte(&group_info[j].type)) != AP_PARAM_NONE ;
                 j++) {
                if (strcasecmp_P(name+len, group_info[j].name) == 0) {
                    uintptr_t p = pgm_read_pointer(&_var_info[i].ptr);
                    return (AP_Param *)(p + pgm_read_pointer(&group_info[j].offset));
                }
            }
        } else if (strcasecmp_P(name, _var_info[i].name) == 0) {
            return (AP_Param *)pgm_read_pointer(&_var_info[i].ptr);
        }
    }
    return NULL;
}

// Save the variable to EEPROM, if supported
//
bool AP_Param::save(void)
{
    uint8_t group_element;
    const struct AP_Param::Info *info = find_var_info(&group_element);

    if (info == NULL) {
        // we don't have any info on how to store it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to store the variable
    phdr.type = pgm_read_byte(&info->type);
    phdr.key  = pgm_read_word(&info->key);
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
    uint8_t group_element;
    const struct AP_Param::Info *info = find_var_info(&group_element);
    if (info == NULL) {
        // we don't have any info on how to load it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to match the variable
    phdr.type = pgm_read_byte(&info->type);
    phdr.key  = pgm_read_word(&info->key);
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
    while (ofs < k_EEPROM_size) {
        eeprom_read_block(&phdr, (const void *)ofs, sizeof(phdr));
        if (phdr.type == AP_PARAM_NONE &&
            phdr.key == 0) {
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
