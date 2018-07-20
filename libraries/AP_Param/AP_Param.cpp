/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//

// total up and check overflow
// check size of group var_info

/// @file   AP_Param.cpp
/// @brief  The AP variable store.
#include "AP_Param.h"

#include <cmath>
#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <StorageManager/StorageManager.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
 # define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

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

// number of rows in the _var_info[] table
uint16_t AP_Param::_num_vars;

// cached parameter count
uint16_t AP_Param::_parameter_count;

// storage and naming information about all types that can be saved
const AP_Param::Info *AP_Param::_var_info;

struct AP_Param::param_override *AP_Param::param_overrides = nullptr;
uint16_t AP_Param::num_param_overrides = 0;

/*
  this holds default parameters in the normal NAME=value form for a
  parameter file. It can be manipulated by apj_tool.py to change the
  defaults on a binary without recompiling
 */
const AP_Param::param_defaults_struct AP_Param::param_defaults_data = {
    "PARMDEF",
    { 0x55, 0x37, 0xf4, 0xa0, 0x38, 0x5d, 0x48, 0x5b },
    AP_PARAM_MAX_EMBEDDED_PARAM,
    0
};

// storage object
StorageAccess AP_Param::_storage(StorageManager::StorageParam);

// flags indicating frame type
uint16_t AP_Param::_frame_type_flags;

// write to EEPROM
void AP_Param::eeprom_write_check(const void *ptr, uint16_t ofs, uint8_t size)
{
    _storage.write_block(ofs, ptr, size);
}

bool AP_Param::_hide_disabled_groups = true;

// write a sentinal value at the given offset
void AP_Param::write_sentinal(uint16_t ofs)
{
    struct Param_header phdr;
    phdr.type = _sentinal_type;
    set_key(phdr, _sentinal_key);
    phdr.group_element = _sentinal_group;
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
}

// erase all EEPROM variables by re-writing the header and adding
// a sentinal
void AP_Param::erase_all(void)
{
    struct EEPROM_header hdr;

    // write the header
    hdr.magic[0] = k_EEPROM_magic0;
    hdr.magic[1] = k_EEPROM_magic1;
    hdr.revision = k_EEPROM_revision;
    hdr.spare    = 0;
    eeprom_write_check(&hdr, 0, sizeof(hdr));

    // add a sentinal directly after the header
    write_sentinal(sizeof(struct EEPROM_header));
}

/* the 'group_id' of a element of a group is the 18 bit identifier
   used to distinguish between this element of the group and other
   elements of the same group. It is calculated using a bit shift per
   level of nesting, so the first level of nesting gets 6 bits the 2nd
   level gets the next 6 bits, and the 3rd level gets the last 6
   bits. This limits groups to having at most 64 elements.
*/
uint32_t AP_Param::group_id(const struct GroupInfo *grpinfo, uint32_t base, uint8_t i, uint8_t shift)
{
    if (grpinfo[i].idx == 0 && shift != 0 && !(grpinfo[i].flags & AP_PARAM_NO_SHIFT)) {
        /*
          this is a special case for a bug in the original design. An
          idx of 0 shifted by n bits is still zero, which makes it
          indistinguishable from a different parameter. This can lead
          to parameter loops. We use index 63 for that case.
        */
        return base + (63U<<shift);
    }
    return base + (grpinfo[i].idx<<shift);
}


/*
  check if a frame type should be included. A frame is included if
  either there are no frame type flags on a parameter or if at least
  one of the flags has been set by set_frame_type_flags()
 */
bool AP_Param::check_frame_type(uint16_t flags)
{
    uint16_t frame_flags = flags >> AP_PARAM_FRAME_TYPE_SHIFT;
    if (frame_flags == 0) {
        return true;
    }
    return (frame_flags & _frame_type_flags) != 0;
}

// validate a group info table
bool AP_Param::check_group_info(const struct AP_Param::GroupInfo *  group_info,
                                uint16_t *                          total_size,
                                uint8_t                             group_shift,
                                uint8_t                             prefix_length)
{
    uint8_t type;
    uint64_t used_mask = 0;
    for (uint8_t i=0;
         (type=group_info[i].type) != AP_PARAM_NONE;
         i++) {
        uint8_t idx = group_info[i].idx;
        if (idx >= (1<<_group_level_shift)) {
            Debug("idx too large (%u) in %s", idx, group_info[i].name);
            return false;
        }
        if (group_shift != 0 && idx == 0) {
            // great idx 0 as 63 for duplicates. See group_id()
            idx = 63;
        }
        if (used_mask & (1ULL<<idx)) {
            Debug("Duplicate group idx %u for %s", idx, group_info[i].name);
            return false;
        }
        used_mask |= (1ULL<<idx);
        if (type == AP_PARAM_GROUP) {
            // a nested group
            if (group_shift + _group_level_shift >= _group_bits) {
                Debug("double group nesting in %s", group_info[i].name);
                return false;
            }
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo == nullptr) {
                continue;
            }
            if (!check_group_info(ginfo, total_size, group_shift + _group_level_shift, prefix_length + strlen(group_info[i].name))) {
                return false;
            }
            continue;
        }
        uint8_t size = type_size((enum ap_var_type)type);
        if (size == 0) {
            Debug("invalid type in %s", group_info[i].name);
            return false;
        }
        if (prefix_length + strlen(group_info[i].name) > 16) {
            Debug("suffix is too long in %s", group_info[i].name);
            return false;
        }
        (*total_size) += size + sizeof(struct Param_header);
    }
    return true;
}

// check for duplicate key values
bool AP_Param::duplicate_key(uint16_t vindex, uint16_t key)
{
    for (uint16_t i=vindex+1; i<_num_vars; i++) {
        uint16_t key2 = _var_info[i].key;
        if (key2 == key) {
            // no duplicate keys allowed
            return true;
        }
    }
    return false;
}

/*
  get group_info pointer for a group
 */
const struct AP_Param::GroupInfo *AP_Param::get_group_info(const struct GroupInfo &ginfo)
{
    if (ginfo.flags & AP_PARAM_FLAG_INFO_POINTER) {
        return *ginfo.group_info_ptr;
    }
    return ginfo.group_info;
}

/*
  get group_info pointer for a group
 */
const struct AP_Param::GroupInfo *AP_Param::get_group_info(const struct Info &info)
{
    if (info.flags & AP_PARAM_FLAG_INFO_POINTER) {
        return *info.group_info_ptr;
    }
    return info.group_info;
}

// validate the _var_info[] table
bool AP_Param::check_var_info(void)
{
    uint16_t total_size = sizeof(struct EEPROM_header);

    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        uint16_t key = _var_info[i].key;
        if (type == AP_PARAM_GROUP) {
            if (i == 0) {
                // first element can't be a group, for first() call
                return false;
            }
            const struct GroupInfo *group_info = get_group_info(_var_info[i]);
            if (group_info == nullptr) {
                continue;
            }
            if (!check_group_info(group_info, &total_size, 0, strlen(_var_info[i].name))) {
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
        if (type != AP_PARAM_GROUP && (_var_info[i].flags & AP_PARAM_FLAG_POINTER)) {
            // only groups can be pointers
            return false;
        }
    }

    // we no longer check if total_size is larger than _eeprom_size,
    // as we allow for more variables than could fit, relying on not
    // saving default values

    return true;
}


// setup the _var_info[] table
bool AP_Param::setup(void)
{
    struct EEPROM_header hdr;

    // check the header
    _storage.read_block(&hdr, 0, sizeof(hdr));
    if (hdr.magic[0] != k_EEPROM_magic0 ||
        hdr.magic[1] != k_EEPROM_magic1 ||
        hdr.revision != k_EEPROM_revision) {
        // header doesn't match. We can't recover any variables. Wipe
        // the header and setup the sentinal directly after the header
        Debug("bad header in setup - erasing");
        erase_all();
    }

    return true;
}

// check if AP_Param has been initialised
bool AP_Param::initialised(void)
{
    return _var_info != nullptr;
}

/*
  adjust offset of a group element for nested groups and group pointers

  The new_offset variable is relative to the vindex base. This makes
  dealing with pointer groups tricky
 */
bool AP_Param::adjust_group_offset(uint16_t vindex, const struct GroupInfo &group_info, ptrdiff_t &new_offset)
{
    if (group_info.flags & AP_PARAM_FLAG_NESTED_OFFSET) {
        new_offset += group_info.offset;
        return true;
    }
    if (group_info.flags & AP_PARAM_FLAG_POINTER) {
        // group_info.offset refers to a pointer
        ptrdiff_t base;
        if (!get_base(_var_info[vindex], base)) {
            // the object is not allocated yet
            return false;
        }
        void **p = (void **)(base + new_offset + group_info.offset);
        if (*p == nullptr) {
            // the object is not allocated yet
            return false;
        }
        // calculate offset that is needed to take base object and adjust for this object
        new_offset = ((ptrdiff_t)*p) - base;
    }
    return true;
}

/*
  get the base pointer for a variable, accounting for AP_PARAM_FLAG_POINTER
 */
bool AP_Param::get_base(const struct Info &info, ptrdiff_t &base)
{
    if (info.flags & AP_PARAM_FLAG_POINTER) {
        base = *(ptrdiff_t *)info.ptr;
        return (base != (ptrdiff_t)0);
    }
    base = (ptrdiff_t)info.ptr;
    return true;
}


// find the info structure given a header and a group_info table
// return the Info structure and a pointer to the variables storage
const struct AP_Param::Info *AP_Param::find_by_header_group(struct Param_header phdr, void **ptr,
                                                            uint16_t vindex,
                                                            const struct GroupInfo *group_info,
                                                            uint32_t group_base,
                                                            uint8_t group_shift,
                                                            ptrdiff_t group_offset)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=group_info[i].type) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            // a nested group
            if (group_shift + _group_level_shift >= _group_bits) {
                // too deeply nested - this should have been caught by
                // setup() !
                return nullptr;
            }
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo == nullptr) {
                continue;
            }
            ptrdiff_t new_offset = group_offset;

            if (!adjust_group_offset(vindex, group_info[i], new_offset)) {
                continue;
            }

            const struct AP_Param::Info *ret = find_by_header_group(phdr, ptr, vindex, ginfo,
                                                                    group_id(group_info, group_base, i, group_shift),
                                                                    group_shift + _group_level_shift, new_offset);
            if (ret != nullptr) {
                return ret;
            }
            continue;
        }
        if (group_id(group_info, group_base, i, group_shift) == phdr.group_element && type == phdr.type) {
            // found a group element
            ptrdiff_t base;
            if (!get_base(_var_info[vindex], base)) {
                continue;
            }
            *ptr = (void*)(base + group_info[i].offset + group_offset);
            return &_var_info[vindex];
        }
    }
    return nullptr;
}

// find the info structure given a header
// return the Info structure and a pointer to the variables storage
const struct AP_Param::Info *AP_Param::find_by_header(struct Param_header phdr, void **ptr)
{
    // loop over all named variables
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        uint16_t key = _var_info[i].key;
        if (key != get_key(phdr)) {
            // not the right key
            continue;
        }
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = get_group_info(_var_info[i]);
            if (group_info == nullptr) {
                continue;
            }
            return find_by_header_group(phdr, ptr, i, group_info, 0, 0, 0);
        }
        if (type == phdr.type) {
            // found it
            ptrdiff_t base;
            if (!get_base(_var_info[i], base)) {
                return nullptr;
            }
            *ptr = (void*)base;
            return &_var_info[i];
        }
    }
    return nullptr;
}

// find the info structure for a variable in a group
const struct AP_Param::Info *AP_Param::find_var_info_group(const struct GroupInfo * group_info,
                                                           uint16_t                 vindex,
                                                           uint32_t                 group_base,
                                                           uint8_t                  group_shift,
                                                           ptrdiff_t                group_offset,
                                                           uint32_t *               group_element,
                                                           const struct GroupInfo * &group_ret,
                                                           struct GroupNesting      &group_nesting,
                                                           uint8_t *                idx) const
{
    ptrdiff_t base;
    if (!get_base(_var_info[vindex], base)) {
        return nullptr;
    }
    uint8_t type;
    for (uint8_t i=0;
         (type=group_info[i].type) != AP_PARAM_NONE;
         i++) {
        ptrdiff_t ofs = group_info[i].offset + group_offset;
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo == nullptr) {
                continue;
            }
            // a nested group
            if (group_shift + _group_level_shift >= _group_bits) {
                // too deeply nested - this should have been caught by
                // setup() !
                return nullptr;
            }
            const struct AP_Param::Info *info;
            ptrdiff_t new_offset = group_offset;
            if (!adjust_group_offset(vindex, group_info[i], new_offset)) {
                continue;
            }
            if (group_nesting.level >= group_nesting.numlevels) {
                return nullptr;
            }
            group_nesting.group_ret[group_nesting.level++] = &group_info[i];
            info = find_var_info_group(ginfo, vindex,
                                       group_id(group_info, group_base, i, group_shift),
                                       group_shift + _group_level_shift,
                                       new_offset,
                                       group_element,
                                       group_ret,
                                       group_nesting,
                                       idx);
            if (info != nullptr) {
                return info;
            }
            group_nesting.level--;
        } else if ((ptrdiff_t) this == base + ofs) {
            *group_element = group_id(group_info, group_base, i, group_shift);
            group_ret = &group_info[i];
            *idx = 0;
            return &_var_info[vindex];
        } else if (type == AP_PARAM_VECTOR3F &&
                   (base+ofs+(ptrdiff_t)sizeof(float) == (ptrdiff_t) this ||
                    base+ofs+2*(ptrdiff_t)sizeof(float) == (ptrdiff_t) this)) {
            // we are inside a Vector3f. We need to work out which
            // element of the vector the current object refers to.
            *idx = (((ptrdiff_t) this) - (base+ofs))/sizeof(float);
            *group_element = group_id(group_info, group_base, i, group_shift);
            group_ret = &group_info[i];
            return &_var_info[vindex];
        }
    }
    return nullptr;
}

// find the info structure for a variable
const struct AP_Param::Info *AP_Param::find_var_info(uint32_t *                 group_element,
                                                     const struct GroupInfo *   &group_ret,
                                                     struct GroupNesting        &group_nesting,
                                                     uint8_t *                  idx) const
{
    group_ret = nullptr;
    
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        ptrdiff_t base;
        if (!get_base(_var_info[i], base)) {
            continue;
        }
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = get_group_info(_var_info[i]);
            if (group_info == nullptr) {
                continue;
            }
            const struct AP_Param::Info *info;
            info = find_var_info_group(group_info, i, 0, 0, 0, group_element, group_ret, group_nesting, idx);
            if (info != nullptr) {
                return info;
            }
        } else if (base == (ptrdiff_t) this) {
            *group_element = 0;
            *idx = 0;
            return &_var_info[i];
        } else if (type == AP_PARAM_VECTOR3F &&
                   (base+(ptrdiff_t)sizeof(float) == (ptrdiff_t) this ||
                    base+2*(ptrdiff_t)sizeof(float) == (ptrdiff_t) this)) {
            // we are inside a Vector3f. Work out which element we are
            // referring to.
            *idx = (((ptrdiff_t) this) - base)/sizeof(float);
            *group_element = 0;
            return &_var_info[i];
        }
    }
    return nullptr;
}


// find the info structure for a variable
const struct AP_Param::Info *AP_Param::find_var_info_token(const ParamToken &token,
                                                           uint32_t *                 group_element,
                                                           const struct GroupInfo *   &group_ret,
                                                           struct GroupNesting        &group_nesting,
                                                           uint8_t *                  idx) const
{
    uint16_t i = token.key;
    uint8_t type = _var_info[i].type;
    ptrdiff_t base;
    if (!get_base(_var_info[i], base)) {
        return nullptr;
    }
    group_ret = nullptr;
    
    if (type == AP_PARAM_GROUP) {
        const struct GroupInfo *group_info = get_group_info(_var_info[i]);
        if (group_info == nullptr) {
            return nullptr;
        }
        const struct AP_Param::Info *info;
        info = find_var_info_group(group_info, i, 0, 0, 0, group_element, group_ret, group_nesting, idx);
        if (info != nullptr) {
            return info;
        }
    } else if (base == (ptrdiff_t) this) {
        *group_element = 0;
        *idx = 0;
        return &_var_info[i];
    } else if (type == AP_PARAM_VECTOR3F &&
               (base+(ptrdiff_t)sizeof(float) == (ptrdiff_t) this ||
                base+2*(ptrdiff_t)sizeof(float) == (ptrdiff_t) this)) {
        // we are inside a Vector3f. Work out which element we are
        // referring to.
        *idx = (((ptrdiff_t) this) - base)/sizeof(float);
        *group_element = 0;
        return &_var_info[i];
    }
    return nullptr;
}

// return the storage size for a AP_PARAM_* type
uint8_t AP_Param::type_size(enum ap_var_type type)
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
    }
    Debug("unknown type %u\n", type);
    return 0;
}

/*
  extract 9 bit key from Param_header
 */
uint16_t AP_Param::get_key(const Param_header &phdr)
{
    return ((uint16_t)phdr.key_high)<<8 | phdr.key_low;
}

/*
  set 9 bit key in Param_header
 */
void AP_Param::set_key(Param_header &phdr, uint16_t key)
{
    phdr.key_low  = key & 0xFF;
    phdr.key_high = key >> 8;
}

/*
  return true if a header is the end of eeprom sentinal
 */
bool AP_Param::is_sentinal(const Param_header &phdr)
{
    // note that this is an ||, not an &&, as this makes us more
    // robust to power off while adding a variable to EEPROM
    if (phdr.type == _sentinal_type ||
        get_key(phdr) == _sentinal_key ||
        phdr.group_element == _sentinal_group) {
        return true;
    }
    // also check for 0xFFFFFFFF and 0x00000000, which are the fill
    // values for storage. These can appear if power off occurs while
    // writing data
    uint32_t v = *(uint32_t *)&phdr;
    if (v == 0 || v == 0xFFFFFFFF) {
        return true;
    }
    return false;
}

// scan the EEPROM looking for a given variable by header content
// return true if found, along with the offset in the EEPROM where
// the variable is stored
// if not found return the offset of the sentinal
// if the sentinal isn't found either, the offset is set to 0xFFFF
bool AP_Param::scan(const AP_Param::Param_header *target, uint16_t *pofs)
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AP_Param::EEPROM_header);
    while (ofs < _storage.size()) {
        _storage.read_block(&phdr, ofs, sizeof(phdr));
        if (phdr.type == target->type &&
            get_key(phdr) == get_key(*target) &&
            phdr.group_element == target->group_element) {
            // found it
            *pofs = ofs;
            return true;
        }
        if (is_sentinal(phdr)) {
            // we've reached the sentinal
            *pofs = ofs;
            return false;
        }
        ofs += type_size((enum ap_var_type)phdr.type) + sizeof(phdr);
    }
    *pofs = 0xffff;
    Debug("scan past end of eeprom");
    return false;
}

/**
 * add a _X, _Y, _Z suffix to the name of a Vector3f element
 * @param buffer
 * @param buffer_size
 * @param idx Suffix: 0 --> _X; 1 --> _Y; 2 --> _Z; (other --> undefined)
 */
void AP_Param::add_vector3f_suffix(char *buffer, size_t buffer_size, uint8_t idx) const
{
    const size_t len = strnlen(buffer, buffer_size);
    if (len + 2 <= buffer_size) {
        buffer[len] = '_';
        buffer[len + 1] = static_cast<char>('X' + idx);
        if (len + 3 <= buffer_size) {
            buffer[len + 2] = 0;
        }
    }
}

// Copy the variable's whole name to the supplied buffer.
//
// If the variable is a group member, prepend the group name.
//
void AP_Param::copy_name_token(const ParamToken &token, char *buffer, size_t buffer_size, bool force_scalar) const
{
    uint32_t group_element;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info_token(token, &group_element, ginfo, group_nesting, &idx);
    if (info == nullptr) {
        *buffer = 0;
        Debug("no info found");
        return;
    }
    copy_name_info(info, ginfo, group_nesting, idx, buffer, buffer_size, force_scalar);
}

void AP_Param::copy_name_info(const struct AP_Param::Info *info,
                              const struct GroupInfo *ginfo,
                              const struct GroupNesting &group_nesting,
                              uint8_t idx, char *buffer, size_t buffer_size, bool force_scalar) const
{
    strncpy(buffer, info->name, buffer_size);
    for (uint8_t i=0; i<group_nesting.level; i++) {
        uint8_t len = strnlen(buffer, buffer_size);
        if (len < buffer_size) {
            strncpy(&buffer[len], group_nesting.group_ret[i]->name, buffer_size-len);
        }
    }
    if (ginfo != nullptr) {
        uint8_t len = strnlen(buffer, buffer_size);
        if (len < buffer_size) {
            strncpy(&buffer[len], ginfo->name, buffer_size-len);
        }
        if ((force_scalar || idx != 0) && AP_PARAM_VECTOR3F == ginfo->type) {
            // the caller wants a specific element in a Vector3f
            add_vector3f_suffix(buffer, buffer_size, idx);
        }
    } else if ((force_scalar || idx != 0) && AP_PARAM_VECTOR3F == info->type) {
        add_vector3f_suffix(buffer, buffer_size, idx);
    }
}

// Find a variable by name in a group
AP_Param *
AP_Param::find_group(const char *name, uint16_t vindex, ptrdiff_t group_offset,
                     const struct GroupInfo *group_info, enum ap_var_type *ptype)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=group_info[i].type) != AP_PARAM_NONE;
         i++) {
        if (type == AP_PARAM_GROUP) {
            if (strncasecmp(name, group_info[i].name, strlen(group_info[i].name)) != 0) {
                continue;
            }
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo == nullptr) {
                continue;
            }
            ptrdiff_t new_offset = group_offset;

            if (!adjust_group_offset(vindex, group_info[i], new_offset)) {
                continue;
            }

            AP_Param *ap = find_group(name+strlen(group_info[i].name), vindex, new_offset, ginfo, ptype);
            if (ap != nullptr) {
                return ap;
            }
        } else if (strcasecmp(name, group_info[i].name) == 0) {
            ptrdiff_t base;
            if (!get_base(_var_info[vindex], base)) {
                continue;
            }
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)(base + group_info[i].offset + group_offset);
        } else if (type == AP_PARAM_VECTOR3F) {
            // special case for finding Vector3f elements
            uint8_t suffix_len = strnlen(group_info[i].name, AP_MAX_NAME_SIZE);
            if (strncmp(name, group_info[i].name, suffix_len) == 0 &&
                name[suffix_len] == '_' &&
                (name[suffix_len+1] == 'X' ||
                 name[suffix_len+1] == 'Y' ||
                 name[suffix_len+1] == 'Z')) {
                ptrdiff_t base;
                if (!get_base(_var_info[vindex], base)) {
                    continue;
                }
                AP_Float *v = (AP_Float *)(base + group_info[i].offset + group_offset);
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
    return nullptr;
}


// Find a variable by name.
//
AP_Param *
AP_Param::find(const char *name, enum ap_var_type *ptype)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        if (type == AP_PARAM_GROUP) {
            uint8_t len = strnlen(_var_info[i].name, AP_MAX_NAME_SIZE);
            if (strncmp(name, _var_info[i].name, len) != 0) {
                continue;
            }
            const struct GroupInfo *group_info = get_group_info(_var_info[i]);
            if (group_info == nullptr) {
                continue;
            }
            AP_Param *ap = find_group(name + len, i, 0, group_info, ptype);
            if (ap != nullptr) {
                return ap;
            }
            // we continue looking as we want to allow top level
            // parameter to have the same prefix name as group
            // parameters, for example CAM_P_G
        } else if (strcasecmp(name, _var_info[i].name) == 0) {
            *ptype = (enum ap_var_type)type;
            ptrdiff_t base;
            if (!get_base(_var_info[i], base)) {
                return nullptr;
            }
            return (AP_Param *)base;
        }
    }
    return nullptr;
}

// Find a variable by index. Note that this is quite slow.
//
AP_Param *
AP_Param::find_by_index(uint16_t idx, enum ap_var_type *ptype, ParamToken *token)
{
    AP_Param *ap;
    uint16_t count=0;
    for (ap=AP_Param::first(token, ptype);
         ap && count < idx;
         ap=AP_Param::next_scalar(token, ptype)) {
        count++;
    }
    return ap;    
}


/*
  Find a variable by pointer, returning key. This is used for loading pointer variables
*/
bool AP_Param::find_key_by_pointer_group(const void *ptr, uint16_t vindex,
                                         const struct GroupInfo *group_info,
                                         ptrdiff_t offset, uint16_t &key)
{
    for (uint8_t i=0; group_info[i].type != AP_PARAM_NONE; i++) {
        if (group_info[i].type != AP_PARAM_GROUP) {
            continue;
        }
        ptrdiff_t base;
        if (!get_base(_var_info[vindex], base)) {
            continue;
        }
        if (group_info[i].flags & AP_PARAM_FLAG_POINTER) {
            if (ptr == *(void **)(base+group_info[i].offset+offset)) {
                key = _var_info[vindex].key;
                return true;
            }
        } else if (ptr == (void *)(base+group_info[i].offset+offset)) {
            key = _var_info[vindex].key;
            return true;
        }
        ptrdiff_t new_offset = offset;
        if (!adjust_group_offset(vindex, group_info[i], new_offset)) {
            continue;
        }
        const struct GroupInfo *ginfo = get_group_info(group_info[i]);
        if (ginfo == nullptr) {
            continue;
        }
        if (find_key_by_pointer_group(ptr, vindex, ginfo, new_offset, key)) {
            return true;
        }
    }
    return false;
}


/*
  Find a variable by pointer, returning key. This is used for loading pointer variables
*/
bool AP_Param::find_key_by_pointer(const void *ptr, uint16_t &key)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        if (_var_info[i].type != AP_PARAM_GROUP) {
            continue;
        }
        if ((_var_info[i].flags & AP_PARAM_FLAG_POINTER) &&
            ptr == *(void **)_var_info[i].ptr) {
            key = _var_info[i].key;
            return true;
        }
        ptrdiff_t offset = 0;
        const struct GroupInfo *ginfo = get_group_info(_var_info[i]);
        if (ginfo == nullptr) {
            continue;
        }
        if (find_key_by_pointer_group(ptr, i, ginfo, offset, key)) {
            return true;
        }
    }
    return false;
}


// Find a object by name.
//
AP_Param *
AP_Param::find_object(const char *name)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        if (strcasecmp(name, _var_info[i].name) == 0) {
            ptrdiff_t base;
            if (!get_base(_var_info[i], base)) {
                return nullptr;
            }
            return (AP_Param *)base;
        }
    }
    return nullptr;
}

// notify GCS of current value of parameter
void AP_Param::notify() const {
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;

    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);
    if (info == nullptr) {
        // this is probably very bad
        return;
    }

    char name[AP_MAX_NAME_SIZE+1];
    copy_name_info(info, ginfo, group_nesting, idx, name, sizeof(name), true);

    uint32_t param_header_type;
    if (ginfo != nullptr) {
        param_header_type = ginfo->type;
    } else {
        param_header_type = info->type;
    }

    send_parameter(name, (enum ap_var_type)param_header_type, idx);
}


// Save the variable to EEPROM, if supported
//
bool AP_Param::save(bool force_save)
{
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);
    const AP_Param *ap;

    if (info == nullptr) {
        // we don't have any info on how to store it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to store the variable
    if (ginfo != nullptr) {
        phdr.type = ginfo->type;
    } else {
        phdr.type = info->type;
    }
    set_key(phdr, info->key);
    phdr.group_element = group_element;

    ap = this;
    if (phdr.type != AP_PARAM_VECTOR3F && idx != 0) {
        // only vector3f can have non-zero idx for now
        return false;
    }
    if (idx != 0) {
        ap = (const AP_Param *)((ptrdiff_t)ap) - (idx*sizeof(float));
    }

    if (phdr.type == AP_PARAM_INT8 && ginfo != nullptr && (ginfo->flags & AP_PARAM_FLAG_ENABLE)) {
        // clear cached parameter count
        _parameter_count = 0;
    }
    
    char name[AP_MAX_NAME_SIZE+1];
    copy_name_info(info, ginfo, group_nesting, idx, name, sizeof(name), true);

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (scan(&phdr, &ofs)) {
        // found an existing copy of the variable
        eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
        send_parameter(name, (enum ap_var_type)phdr.type, idx);
        return true;
    }
    if (ofs == (uint16_t) ~0) {
        return false;
    }

    // if the value is the default value then don't save
    if (phdr.type <= AP_PARAM_FLOAT) {
        float v1 = cast_to_float((enum ap_var_type)phdr.type);
        float v2;
        if (ginfo != nullptr) {
            v2 = get_default_value(this, &ginfo->def_value);
        } else {
            v2 = get_default_value(this, &info->def_value);
        }
        if (is_equal(v1,v2) && !force_save) {
            gcs().send_parameter_value(name, (enum ap_var_type)info->type, v2);
            return true;
        }
        if (!force_save &&
            (phdr.type != AP_PARAM_INT32 &&
             (fabsf(v1-v2) < 0.0001f*fabsf(v1)))) {
            // for other than 32 bit integers, we accept values within
            // 0.01 percent of the current value as being the same
            gcs().send_parameter_value(name, (enum ap_var_type)info->type, v2);
            return true;
        }
    }

    if (ofs+type_size((enum ap_var_type)phdr.type)+2*sizeof(phdr) >= _storage.size()) {
        // we are out of room for saving variables
        hal.console->printf("EEPROM full\n");
        return false;
    }

    // write a new sentinal, then the data, then the header
    write_sentinal(ofs + sizeof(phdr) + type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(&phdr, ofs, sizeof(phdr));

    send_parameter(name, (enum ap_var_type)phdr.type, idx);
    return true;
}

// Load the variable from EEPROM, if supported
//
bool AP_Param::load(void)
{
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);
    if (info == nullptr) {
        // we don't have any info on how to load it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to match the variable
    if (ginfo != nullptr) {
        phdr.type = ginfo->type;
    } else {
        phdr.type = info->type;
    }
    set_key(phdr, info->key);
    phdr.group_element = group_element;

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (!scan(&phdr, &ofs)) {
        // if the value isn't stored in EEPROM then set the default value
        ptrdiff_t base;
        if (!get_base(*info, base)) {
            return false;
        }

        if (ginfo != nullptr) {
            // add in nested group offset
            ptrdiff_t group_offset = 0;
            for (uint8_t i=0; i<group_nesting.level; i++) {
                group_offset += group_nesting.group_ret[i]->offset;
            }
            set_value((enum ap_var_type)phdr.type, (void*)(base + ginfo->offset + group_offset),
                      get_default_value(this, &ginfo->def_value));
        } else {
            set_value((enum ap_var_type)phdr.type, (void*)base, 
                      get_default_value(this, &info->def_value));
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
        ap = (AP_Param *)((ptrdiff_t)ap) - (idx*sizeof(float));
    }

    // found it
    _storage.read_block(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
    return true;
}

bool AP_Param::configured_in_storage(void) const
{
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);
    if (info == nullptr) {
        // we don't have any info on how to load it
        return false;
    }

    struct Param_header phdr;

    // create the header we will use to match the variable
    if (ginfo != nullptr) {
        phdr.type = ginfo->type;
    } else {
        phdr.type = info->type;
    }
    set_key(phdr, info->key);
    phdr.group_element = group_element;

    // scan EEPROM to find the right location
    uint16_t ofs;

    // only vector3f can have non-zero idx for now
    return scan(&phdr, &ofs) && (phdr.type == AP_PARAM_VECTOR3F || idx == 0);
}

bool AP_Param::configured_in_defaults_file(void) const
{
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);
    if (info == nullptr) {
        // we don't have any info on how to load it
        return false;
    }

    for (uint16_t i=0; i<num_param_overrides; i++) {
        if (this == param_overrides[i].object_ptr) {
            return true;
        }
    }

    return false;
}

// set a AP_Param variable to a specified value
void AP_Param::set_value(enum ap_var_type type, void *ptr, float value)
{
    switch (type) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)ptr)->set(value);
        break;
    case AP_PARAM_INT16:
        ((AP_Int16 *)ptr)->set(value);
        break;
    case AP_PARAM_INT32:
        ((AP_Int32 *)ptr)->set(value);
        break;
    case AP_PARAM_FLOAT:
        ((AP_Float *)ptr)->set(value);
        break;
    default:
        break;
    }
}

// load default values for scalars in a group. This does not recurse
// into other objects. This is a static function that should be called
// in the objects constructor
void AP_Param::setup_object_defaults(const void *object_pointer, const struct GroupInfo *group_info)
{
    ptrdiff_t base = (ptrdiff_t)object_pointer;
    uint8_t type;
    for (uint8_t i=0;
         (type=group_info[i].type) != AP_PARAM_NONE;
         i++) {
        if (type <= AP_PARAM_FLOAT) {
            void *ptr = (void *)(base + group_info[i].offset);
            set_value((enum ap_var_type)type, ptr,
                      get_default_value((const AP_Param *)ptr, &group_info[i].def_value));
        }
    }
}

// set a value directly in an object. This should only be used by
// example code, not by mainline vehicle code
bool AP_Param::set_object_value(const void *object_pointer,
                                const struct GroupInfo *group_info,
                                const char *name, float value)
{
    ptrdiff_t base = (ptrdiff_t)object_pointer;
    uint8_t type;
    bool found = false;
    for (uint8_t i=0;
         (type=group_info[i].type) != AP_PARAM_NONE;
         i++) {
        if (strcmp(name, group_info[i].name) == 0 && type <= AP_PARAM_FLOAT) {
            void *ptr = (void *)(base + group_info[i].offset);
            set_value((enum ap_var_type)type, ptr, value);
            // return true here ?
            found = true;
        }
    }
    return found;
}


// load default values for all scalars in a sketch. This does not
// recurse into sub-objects
void AP_Param::setup_sketch_defaults(void)
{
    setup();
    for (uint16_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        if (type <= AP_PARAM_FLOAT) {
            ptrdiff_t base;
            if (get_base(_var_info[i], base)) {
                set_value((enum ap_var_type)type, (void*)base,
                          get_default_value((const AP_Param *)base, &_var_info[i].def_value));
            }
        }
    }
}


// Load all variables from EEPROM
//
bool AP_Param::load_all()
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AP_Param::EEPROM_header);

    reload_defaults_file(false);

    while (ofs < _storage.size()) {
        _storage.read_block(&phdr, ofs, sizeof(phdr));
        // note that this is an || not an && for robustness
        // against power off while adding a variable
        if (is_sentinal(phdr)) {
            // we've reached the sentinal
            return true;
        }

        const struct AP_Param::Info *info;
        void *ptr;

        info = find_by_header(phdr, &ptr);
        if (info != nullptr) {
            _storage.read_block(ptr, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
        }

        ofs += type_size((enum ap_var_type)phdr.type) + sizeof(phdr);
    }

    // we didn't find the sentinal
    Debug("no sentinal in load_all");
    return false;
}

/*
 * reload from hal.util defaults file or embedded param region
 * @last_pass: if this is the last pass on defaults - unknown parameters are
 *             ignored but if this is set a warning will be emitted
 */
void AP_Param::reload_defaults_file(bool last_pass)
{
    if (param_defaults_data.length != 0) {
        load_embedded_param_defaults(last_pass);
        return;
    }

#if HAL_OS_POSIX_IO == 1
    /*
      if the HAL specifies a defaults parameter file then override
      defaults using that file
     */
    const char *default_file = hal.util->get_custom_defaults_file();
    if (default_file) {
        if (load_defaults_file(default_file, last_pass)) {
            printf("Loaded defaults from %s\n", default_file);
        } else {
            printf("Failed to load defaults from %s\n", default_file);
        }
    }
#endif
}


/* 
   Load all variables from EEPROM for a particular object. This is
   required for dynamically loaded objects
 */
void AP_Param::load_object_from_eeprom(const void *object_pointer, const struct GroupInfo *group_info)
{
    struct Param_header phdr;
    uint16_t key;

    // reset cached param counter as we may be loading a dynamic var_info
    _parameter_count = 0;
    
    if (!find_key_by_pointer(object_pointer, key)) {
        hal.console->printf("ERROR: Unable to find param pointer\n");
        return;
    }
    
    for (uint8_t i=0; group_info[i].type != AP_PARAM_NONE; i++) {
        if (group_info[i].type == AP_PARAM_GROUP) {
            ptrdiff_t new_offset = 0;
            if (!adjust_group_offset(key, group_info[i], new_offset)) {
                continue;
            }
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo != nullptr) {
                load_object_from_eeprom((void *)(((ptrdiff_t)object_pointer)+new_offset), ginfo);
            }
        }
        uint16_t ofs = sizeof(AP_Param::EEPROM_header);
        while (ofs < _storage.size()) {
            _storage.read_block(&phdr, ofs, sizeof(phdr));
            // note that this is an || not an && for robustness
            // against power off while adding a variable
            if (is_sentinal(phdr)) {
                // we've reached the sentinal
                break;
            }
            if (get_key(phdr) == key) {
                const struct AP_Param::Info *info;
                void *ptr;
                
                info = find_by_header(phdr, &ptr);
                if (info != nullptr) {
                    if ((ptrdiff_t)ptr == ((ptrdiff_t)object_pointer)+group_info[i].offset) {
                        _storage.read_block(ptr, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
                        break;
                    }
                }
            }
            ofs += type_size((enum ap_var_type)phdr.type) + sizeof(phdr);
        }
    }
}


// return the first variable in _var_info
AP_Param *AP_Param::first(ParamToken *token, enum ap_var_type *ptype)
{
    token->key = 0;
    token->group_element = 0;
    token->idx = 0;
    if (_num_vars == 0) {
        return nullptr;
    }
    if (ptype != nullptr) {
        *ptype = (enum ap_var_type)_var_info[0].type;
    }
    ptrdiff_t base;
    if (!get_base(_var_info[0], base)) {
        // should be impossible, first var needs to be non-pointer
        return nullptr;
    }
    return (AP_Param *)base;
}

/// Returns the next variable in a group, recursing into groups
/// as needed
AP_Param *AP_Param::next_group(uint16_t vindex, const struct GroupInfo *group_info,
                               bool *found_current,
                               uint32_t group_base,
                               uint8_t group_shift,
                               ptrdiff_t group_offset,
                               ParamToken *token,
                               enum ap_var_type *ptype)
{
    enum ap_var_type type;
    for (uint8_t i=0;
         (type=(enum ap_var_type)group_info[i].type) != AP_PARAM_NONE;
         i++) {
        if (!check_frame_type(group_info[i].flags)) {
            continue;
        }
        if (type == AP_PARAM_GROUP) {
            // a nested group
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo == nullptr) {
                continue;
            }
            AP_Param *ap;
            ptrdiff_t new_offset = group_offset;

            if (!adjust_group_offset(vindex, group_info[i], new_offset)) {
                continue;
            }

            ap = next_group(vindex, ginfo, found_current, group_id(group_info, group_base, i, group_shift),
                            group_shift + _group_level_shift, new_offset, token, ptype);
            if (ap != nullptr) {
                return ap;
            }
        } else {
            if (*found_current) {
                // got a new one
                token->key = vindex;
                token->group_element = group_id(group_info, group_base, i, group_shift);
                token->idx = 0;
                if (ptype != nullptr) {
                    *ptype = type;
                }
                ptrdiff_t base;
                if (!get_base(_var_info[vindex], base)) {
                    continue;
                }
                return (AP_Param*)(base + group_info[i].offset + group_offset);
            }
            if (group_id(group_info, group_base, i, group_shift) == token->group_element) {
                *found_current = true;
                if (type == AP_PARAM_VECTOR3F && token->idx < 3) {
                    // return the next element of the vector as a
                    // float
                    token->idx++;
                    if (ptype != nullptr) {
                        *ptype = AP_PARAM_FLOAT;
                    }
                    ptrdiff_t base;
                    if (!get_base(_var_info[vindex], base)) {
                        continue;
                    }
                    ptrdiff_t ofs = base + group_info[i].offset + group_offset;
                    ofs += sizeof(float)*(token->idx - 1u);
                    return (AP_Param *)ofs;
                }
            }
        }
    }
    return nullptr;
}

/// Returns the next variable in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next(ParamToken *token, enum ap_var_type *ptype)
{
    uint16_t i = token->key;
    bool found_current = false;
    if (i >= _num_vars) {
        // illegal token
        return nullptr;
    }
    enum ap_var_type type = (enum ap_var_type)_var_info[i].type;

    // allow Vector3f to be seen as 3 variables. First as a vector,
    // then as 3 separate floats
    if (type == AP_PARAM_VECTOR3F && token->idx < 3) {
        token->idx++;
        if (ptype != nullptr) {
            *ptype = AP_PARAM_FLOAT;
        }
        return (AP_Param *)(((token->idx - 1u)*sizeof(float))+(ptrdiff_t)_var_info[i].ptr);
    }

    if (type != AP_PARAM_GROUP) {
        i++;
        found_current = true;
    }
    for (; i<_num_vars; i++) {
        if (!check_frame_type(_var_info[i].flags)) {
            continue;
        }
        type = (enum ap_var_type)_var_info[i].type;
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = get_group_info(_var_info[i]);
            if (group_info == nullptr) {
                continue;
            }
            AP_Param *ap = next_group(i, group_info, &found_current, 0, 0, 0, token, ptype);
            if (ap != nullptr) {
                return ap;
            }
        } else {
            // found the next one
            token->key = i;
            token->group_element = 0;
            token->idx = 0;
            if (ptype != nullptr) {
                *ptype = type;
            }
            return (AP_Param *)(_var_info[i].ptr);
        }
    }
    return nullptr;
}

/// Returns the next scalar in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next_scalar(ParamToken *token, enum ap_var_type *ptype)
{
    AP_Param *ap;
    enum ap_var_type type;
    while ((ap = next(token, &type)) != nullptr && type > AP_PARAM_FLOAT) ;

    if (ap != nullptr && type == AP_PARAM_INT8) {
        /* 
           check if this is an enable variable. To do that we need to
           find the info structures for the variable
         */
        uint32_t group_element;
        const struct GroupInfo *ginfo;
        struct GroupNesting group_nesting {};
        uint8_t idx;
        const struct AP_Param::Info *info = ap->find_var_info_token(*token, &group_element,
                                                                    ginfo, group_nesting, &idx);
        if (info && ginfo &&
            (ginfo->flags & AP_PARAM_FLAG_ENABLE) &&
            !(ginfo->flags & AP_PARAM_FLAG_IGNORE_ENABLE) &&
            ((AP_Int8 *)ap)->get() == 0 &&
            _hide_disabled_groups) {
            /*
              this is a disabled parameter tree, include this
              parameter but not others below it. We need to keep
              looking until we go past the parameters in this object
            */
            ParamToken token2 = *token;
            enum ap_var_type type2;
            AP_Param *ap2;
            while ((ap2 = next(&token2, &type2)) != nullptr) {
                if (token2.key != token->key) {
                    break;
                }
                if (group_nesting.level != 0 && (token->group_element & 0x3F) != (token2.group_element & 0x3F)) {
                    break;
                }
                // update the returned token so the next() call goes from this point
                *token = token2;
            }
            
        }
    }

    if (ap != nullptr && ptype != nullptr) {
        *ptype = type;
    }
    return ap;
}


/// cast a variable to a float given its type
float AP_Param::cast_to_float(enum ap_var_type type) const
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

/*
  find an old parameter and return it.
 */
bool AP_Param::find_old_parameter(const struct ConversionInfo *info, AP_Param *value)
{
    // find the old value in EEPROM.
    uint16_t pofs;
    AP_Param::Param_header header;
    header.type = info->type;
    set_key(header, info->old_key);
    header.group_element = info->old_group_element;
    if (!scan(&header, &pofs)) {
        // the old parameter isn't saved in the EEPROM.
        return false;
    }

    // load the old value from EEPROM
    _storage.read_block(value, pofs+sizeof(header), type_size((enum ap_var_type)header.type));
    return true;
}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
// convert one old vehicle parameter to new object parameter
void AP_Param::convert_old_parameter(const struct ConversionInfo *info, float scaler, uint8_t flags)
{
    uint8_t old_value[type_size(info->type)];
    AP_Param *ap = (AP_Param *)&old_value[0];

    if (!find_old_parameter(info, ap)) {
        // the old parameter isn't saved in the EEPROM. It was
        // probably still set to the default value, which isn't stored
        // no need to convert
        return;
    }

    // find the new variable in the variable structures
    enum ap_var_type ptype;
    AP_Param *ap2;
    ap2 = find(&info->new_name[0], &ptype);
    if (ap2 == nullptr) {
        hal.console->printf("Unknown conversion '%s'\n", info->new_name);
        return;
    }

    // see if we can load it from EEPROM
    if (!(flags & CONVERT_FLAG_FORCE) && ap2->configured_in_storage()) {
        // the new parameter already has a value set by the user, or
        // has already been converted
        return;
    }

    // see if they are the same type and no scaling applied
    if (ptype == info->type && is_equal(scaler, 1.0f) && flags == 0) {
        // copy the value over only if the new parameter does not already
        // have the old value (via a default).
        if (memcmp(ap2, ap, sizeof(old_value)) != 0) {
            memcpy(ap2, ap, sizeof(old_value));
            // and save
            ap2->save();
        }
    } else if (ptype <= AP_PARAM_FLOAT && info->type <= AP_PARAM_FLOAT) {
        // perform scalar->scalar conversion
        float v = ap->cast_to_float(info->type);
        if (flags & CONVERT_FLAG_REVERSE) {
            // convert a _REV parameter to a _REVERSED parameter
            v = is_equal(v, -1.0f)?1:0;
        }
        if (!is_equal(v,ap2->cast_to_float(ptype))) {
            // the value needs to change
            set_value(ptype, ap2, v * scaler);
            ap2->save();
        }
    } else {
        // can't do vector<->scalar conversion, or different vector types
        hal.console->printf("Bad conversion type '%s'\n", info->new_name);
    }
}
#pragma GCC diagnostic pop


// convert old vehicle parameters to new object parametersv
void AP_Param::convert_old_parameters(const struct ConversionInfo *conversion_table, uint8_t table_size, uint8_t flags)
{
    for (uint8_t i=0; i<table_size; i++) {
        convert_old_parameter(&conversion_table[i], 1.0f, flags);
    }
}

/*
  move old class variables for a class that was sub-classed to one that isn't
  For example, used when the AP_MotorsTri class changed from having its own parameter table
  plus a subgroup for AP_MotorsMulticopter to just inheriting the AP_MotorsMulticopter var_info

  This does not handle nesting beyond the single level shift
*/
void AP_Param::convert_parent_class(uint8_t param_key, void *object_pointer,
                                    const struct AP_Param::GroupInfo *group_info)
{
    for (uint8_t i=0; group_info[i].type != AP_PARAM_NONE; i++) {
        struct ConversionInfo info;
        info.old_key = param_key;
        info.type = (ap_var_type)group_info[i].type;
        info.new_name = nullptr;
        info.old_group_element = uint16_t(group_info[i].idx)<<6;
        uint8_t old_value[type_size(info.type)];
        AP_Param *ap = (AP_Param *)&old_value[0];
        
        if (!AP_Param::find_old_parameter(&info, ap)) {
            // the parameter wasn't set in the old eeprom
            continue;
        }

        uint8_t *new_value = group_info[i].offset + (uint8_t *)object_pointer;
        memcpy(new_value, old_value, sizeof(old_value));
    }
}


/*
  set a parameter to a float value
 */
void AP_Param::set_float(float value, enum ap_var_type var_type)
{
    if (isnan(value) || isinf(value)) {
        return;
    }

    // add a small amount before casting parameter values
    // from float to integer to avoid truncating to the
    // next lower integer value.
    float rounding_addition = 0.01f;
        
    // handle variables with standard type IDs
    if (var_type == AP_PARAM_FLOAT) {
        ((AP_Float *)this)->set(value);
    } else if (var_type == AP_PARAM_INT32) {
        if (value < 0) rounding_addition = -rounding_addition;
        float v = value+rounding_addition;
        v = constrain_float(v, -2147483648.0, 2147483647.0);
        ((AP_Int32 *)this)->set(v);
    } else if (var_type == AP_PARAM_INT16) {
        if (value < 0) rounding_addition = -rounding_addition;
        float v = value+rounding_addition;
        v = constrain_float(v, -32768, 32767);
        ((AP_Int16 *)this)->set(v);
    } else if (var_type == AP_PARAM_INT8) {
        if (value < 0) rounding_addition = -rounding_addition;
        float v = value+rounding_addition;
        v = constrain_float(v, -128, 127);
        ((AP_Int8 *)this)->set(v);
    }
}


/*
  parse a parameter file line
 */
bool AP_Param::parse_param_line(char *line, char **vname, float &value)
{
    if (line[0] == '#') {
        return false;
    }
    char *saveptr = nullptr;
    char *pname = strtok_r(line, ", =\t", &saveptr);
    if (pname == nullptr) {
        return false;
    }
    if (strlen(pname) > AP_MAX_NAME_SIZE) {
        return false;
    }
    const char *value_s = strtok_r(nullptr, ", =\t", &saveptr);
    if (value_s == nullptr) {
        return false;
    }
    value = atof(value_s);
    *vname = pname;
    return true;
}


#if HAL_OS_POSIX_IO == 1
#include <stdio.h>

// increments num_defaults for each default found in filename
bool AP_Param::count_defaults_in_file(const char *filename, uint16_t &num_defaults)
{
    FILE *f = fopen(filename, "r");
    if (f == nullptr) {
        return false;
    }
    char line[100];

    /*
      work out how many parameter default structures to allocate
     */
    while (fgets(line, sizeof(line)-1, f)) {
        char *pname;
        float value;
        if (!parse_param_line(line, &pname, value)) {
            continue;
        }
        enum ap_var_type var_type;
        if (!find(pname, &var_type)) {
            continue;
        }
        num_defaults++;
    }

    fclose(f);

    return true;
}

bool AP_Param::read_param_defaults_file(const char *filename, bool last_pass)
{
    FILE *f = fopen(filename, "r");
    if (f == nullptr) {
        AP_HAL::panic("AP_Param: Failed to re-open defaults file");
        return false;
    }

    uint16_t idx = 0;
    char line[100];
    while (fgets(line, sizeof(line)-1, f)) {
        char *pname;
        float value;
        if (!parse_param_line(line, &pname, value)) {
            continue;
        }
        enum ap_var_type var_type;
        AP_Param *vp = find(pname, &var_type);
        if (!vp) {
            if (last_pass) {
                ::printf("Ignored unknown param %s in defaults file %s\n",
                         pname, filename);
                hal.console->printf(
                         "Ignored unknown param %s in defaults file %s\n",
                         pname, filename);
            }
            continue;
        }
        param_overrides[idx].object_ptr = vp;
        param_overrides[idx].value = value;
        idx++;
        if (!vp->configured_in_storage()) {
            vp->set_float(value, var_type);
        }
    }
    fclose(f);
    return true;
}

/*
  load a default set of parameters from a file
 */
bool AP_Param::load_defaults_file(const char *filename, bool last_pass)
{
    if (filename == nullptr) {
        return false;
    }

    char *mutable_filename = strdup(filename);
    if (mutable_filename == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate mutable string");
    }

    uint16_t num_defaults = 0;
    char *saveptr = nullptr;
    for (char *pname = strtok_r(mutable_filename, ",", &saveptr);
         pname != nullptr;
         pname = strtok_r(nullptr, ",", &saveptr)) {
        if (!count_defaults_in_file(pname, num_defaults)) {
            free(mutable_filename);
            return false;
        }
    }
    free(mutable_filename);

    if (param_overrides != nullptr) {
        free(param_overrides);
    }
    num_param_overrides = 0;

    param_overrides = new param_override[num_defaults];
    if (param_overrides == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate overrides");
        return false;
    }

    saveptr = nullptr;
    mutable_filename = strdup(filename);
    if (mutable_filename == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate mutable string");
    }
    for (char *pname = strtok_r(mutable_filename, ",", &saveptr);
         pname != nullptr;
         pname = strtok_r(nullptr, ",", &saveptr)) {
        if (!read_param_defaults_file(pname, last_pass)) {
            free(mutable_filename);
            return false;
        }
    }
    free(mutable_filename);

    num_param_overrides = num_defaults;

    return true;
}

#endif // HAL_OS_POSIX_IO

/*
  count the number of embedded parameter defaults
 */
bool AP_Param::count_embedded_param_defaults(uint16_t &count)
{
    const volatile char *ptr = param_defaults_data.data;
    uint16_t length = param_defaults_data.length;
    count = 0;
    
    while (length) {
        char line[100];
        char *pname;
        float value;
        uint16_t i;
        uint16_t n = MIN(length, sizeof(line)-1);
        for (i=0;i<n;i++) {
            if (ptr[i] == '\n') {
                break;
            }
        }
        if (i == n) {
            // no newline
            break;
        }
        
        memcpy(line, (void *)ptr, i);
        line[i] = 0;

        length -= i+1;
        ptr += i+1;
        
        if (line[0] == '#' || line[0] == 0) {
            continue;
        }

        if (!parse_param_line(line, &pname, value)) {
            continue;
        }

        enum ap_var_type var_type;
        if (!find(pname, &var_type)) {
            continue;
        }

        count++;
    }
    return true;
}

/*
 * load a default set of parameters from a embedded parameter region
 * @last_pass: if this is the last pass on defaults - unknown parameters are
 *             ignored but if this is set a warning will be emitted
 */
void AP_Param::load_embedded_param_defaults(bool last_pass)
{
    if (param_overrides != nullptr) {
        free(param_overrides);
        param_overrides = nullptr;
    }
    
    num_param_overrides = 0;
    uint16_t num_defaults = 0;
    if (!count_embedded_param_defaults(num_defaults)) {
        return;
    }

    param_overrides = new param_override[num_defaults];
    if (param_overrides == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate overrides");
        return;
    }
    
    const volatile char *ptr = param_defaults_data.data;
    uint16_t length = param_defaults_data.length;
    uint16_t idx = 0;
    
    while (length) {
        char line[100];
        char *pname;
        float value;
        uint16_t i;
        uint16_t n = MIN(length, sizeof(line)-1);
        for (i=0;i<n;i++) {
            if (ptr[i] == '\n') {
                break;
            }
        }
        if (i == n) {
            // no newline
            break;
        }
        memcpy(line, (void*)ptr, i);
        line[i] = 0;
        
        length -= i+1;
        ptr += i+1;

        if (line[0] == '#' || line[0] == 0) {
            continue;
        }
        
        if (!parse_param_line(line, &pname, value)) {
            continue;
        }
        enum ap_var_type var_type;
        AP_Param *vp = find(pname, &var_type);
        if (!vp) {
            if (last_pass) {
                ::printf("Ignored unknown param %s from embedded region (offset=%u)\n",
                         pname, unsigned(ptr - param_defaults_data.data));
                hal.console->printf(
                         "Ignored unknown param %s from embedded region (offset=%u)\n",
                         pname, unsigned(ptr - param_defaults_data.data));
            }
            continue;
        }
        param_overrides[idx].object_ptr = vp;
        param_overrides[idx].value = value;
        idx++;
        if (!vp->configured_in_storage()) {
            vp->set_float(value, var_type);
        }
    }
    num_param_overrides = num_defaults;
}

/* 
   find a default value given a pointer to a default value in flash
 */
float AP_Param::get_default_value(const AP_Param *vp, const float *def_value_ptr)
{
    for (uint16_t i=0; i<num_param_overrides; i++) {
        if (vp == param_overrides[i].object_ptr) {
            return param_overrides[i].value;
        }
    }
    return *def_value_ptr;
}


void AP_Param::send_parameter(const char *name, enum ap_var_type var_type, uint8_t idx) const
{
    if (idx != 0 && var_type == AP_PARAM_VECTOR3F) {
        var_type = AP_PARAM_FLOAT;
    }
    if (var_type > AP_PARAM_VECTOR3F) {
        // invalid
        return;
    }
    if (var_type != AP_PARAM_VECTOR3F) {
        // nice and simple for scalar types
        gcs().send_parameter_value(name, var_type, cast_to_float(var_type));
        return;
    }

    // for vectors we need to send 3 messages. Note that we also come here for the case
    // of a set of the first element of a AP_Vector3f. This happens as the ap->save() call can't
    // distinguish between a vector and scalar save. It means that setting first element of a vector
    // via MAVLink results in sending all 3 elements to the GCS
    const Vector3f &v = ((AP_Vector3f *)this)->get();
    char name2[AP_MAX_NAME_SIZE+1];
    strncpy(name2, name, AP_MAX_NAME_SIZE);
    name2[AP_MAX_NAME_SIZE] = 0;
    char &name_axis = name2[strlen(name)-1];
    
    name_axis = 'X';
    gcs().send_parameter_value(name2, AP_PARAM_FLOAT, v.x);
    name_axis = 'Y';
    gcs().send_parameter_value(name2, AP_PARAM_FLOAT, v.y);
    name_axis = 'Z';
    gcs().send_parameter_value(name2, AP_PARAM_FLOAT, v.z);
}

/*
  return count of all scalar parameters.
  Note that this function may be called from the IO thread, so needs
  to be thread safe
 */
uint16_t AP_Param::count_parameters(void)
{
    // if we haven't cached the parameter count yet...
    uint16_t ret = _parameter_count;
    if (0 == ret) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        for (vp = AP_Param::first(&token, nullptr);
             vp != nullptr;
             vp = AP_Param::next_scalar(&token, nullptr)) {
            ret++;
        }
        _parameter_count = ret;
    }
    return ret;
}

/*
  set a default value by name
 */
bool AP_Param::set_default_by_name(const char *name, float value)
{
    enum ap_var_type vtype;
    AP_Param *vp = find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_default(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_default(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_default(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_default(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

/*
  set a value by name
 */
bool AP_Param::set_by_name(const char *name, float value)
{
    enum ap_var_type vtype;
    AP_Param *vp = find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

/*
  set and save a value by name
 */
bool AP_Param::set_and_save_by_name(const char *name, float value)
{
    enum ap_var_type vtype;
    AP_Param *vp = find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_and_save(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

#if AP_PARAM_KEY_DUMP
/*
  do not remove this show_all() code, it is essential for debugging
  and creating conversion tables
 */

// print the value of all variables
void AP_Param::show(const AP_Param *ap, const char *s,
                    enum ap_var_type type, AP_HAL::BetterStream *port)
{
    switch (type) {
    case AP_PARAM_INT8:
        port->printf("%s: %d\n", s, (int)((AP_Int8 *)ap)->get());
        break;
    case AP_PARAM_INT16:
        port->printf("%s: %d\n", s, (int)((AP_Int16 *)ap)->get());
        break;
    case AP_PARAM_INT32:
        port->printf("%s: %ld\n", s, (long)((AP_Int32 *)ap)->get());
        break;
    case AP_PARAM_FLOAT:
        port->printf("%s: %f\n", s, (double)((AP_Float *)ap)->get());
        break;
    default:
        break;
    }
}

// print the value of all variables
void AP_Param::show(const AP_Param *ap, const ParamToken &token,
                    enum ap_var_type type, AP_HAL::BetterStream *port)
{
    char s[AP_MAX_NAME_SIZE+1];
    ap->copy_name_token(token, s, sizeof(s), true);
    s[AP_MAX_NAME_SIZE] = 0;
    show(ap, s, type, port);
}

// print the value of all variables
void AP_Param::show_all(AP_HAL::BetterStream *port, bool showKeyValues)
{
    ParamToken token;
    AP_Param *ap;
    enum ap_var_type type;

    for (ap=AP_Param::first(&token, &type);
         ap;
         ap=AP_Param::next_scalar(&token, &type)) {
        if (showKeyValues) {
            port->printf("Key %i: Index %i: GroupElement %i  :  ", token.key, token.idx, token.group_element);
        }
        show(ap, token, type, port);
    }
}
#endif // AP_PARAM_KEY_DUMP

