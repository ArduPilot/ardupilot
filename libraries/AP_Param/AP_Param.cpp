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
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <stdio.h>
#include <AP_ROMFS/AP_ROMFS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #include <SITL/SITL.h>
#endif

#include "AP_Param_config.h"

extern const AP_HAL::HAL &hal;

uint16_t AP_Param::sentinel_offset;

// singleton instance
AP_Param *AP_Param::_singleton;

#ifndef AP_PARAM_STORAGE_BAK_ENABLED
// we only have a storage region for backup storage if we have at
// least 32768 bytes or storage. We also don't enable when using flash
// storage as this can lead to loss of storage when updating to a
// larger storage size
#define AP_PARAM_STORAGE_BAK_ENABLED (HAL_STORAGE_SIZE>=32768) && !defined(STORAGE_FLASH_PAGE)
#endif


#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
# define FATAL(fmt, args ...) AP_HAL::panic(fmt, ## args);
 # define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define FATAL(fmt, args ...) AP_HAL::panic("Bad parameter table");
 # define Debug(fmt, args ...)
#endif

#if HAL_GCS_ENABLED
#define GCS_SEND_PARAM(name, type, v) gcs().send_parameter_value(name, type, v)
#else
#define GCS_SEND_PARAM(name, type, v)
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

#if AP_PARAM_DYNAMIC_ENABLED
uint16_t AP_Param::_num_vars_base;
AP_Param::Info *AP_Param::_var_info_dynamic;
static const char *_empty_string = "";
uint8_t AP_Param::_dynamic_table_sizes[AP_PARAM_MAX_DYNAMIC];
#endif

// cached parameter count
uint16_t AP_Param::_parameter_count;
uint16_t AP_Param::_count_marker;
uint16_t AP_Param::_count_marker_done;
HAL_Semaphore AP_Param::_count_sem;

// storage and naming information about all types that can be saved
const AP_Param::Info *AP_Param::_var_info;

struct AP_Param::param_override *AP_Param::param_overrides;
uint16_t AP_Param::param_overrides_len;
uint16_t AP_Param::num_param_overrides;
uint16_t AP_Param::num_read_only;

// goes true if we run out of param space
bool AP_Param::eeprom_full;

ObjectBuffer_TS<AP_Param::param_save> AP_Param::save_queue{30};
bool AP_Param::registered_save_handler;

bool AP_Param::done_all_default_params;

AP_Param::defaults_list *AP_Param::default_list;

// we need a dummy object for the parameter save callback
static AP_Param save_dummy;

#if AP_PARAM_MAX_EMBEDDED_PARAM > 0
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
#endif

// storage object
StorageAccess AP_Param::_storage(StorageManager::StorageParam);

#if AP_PARAM_STORAGE_BAK_ENABLED
// backup storage object
StorageAccess AP_Param::_storage_bak(StorageManager::StorageParamBak);
#endif

// flags indicating frame type
uint16_t AP_Param::_frame_type_flags;

// write to EEPROM
void AP_Param::eeprom_write_check(const void *ptr, uint16_t ofs, uint8_t size)
{
    _storage.write_block(ofs, ptr, size);
#if AP_PARAM_STORAGE_BAK_ENABLED
    _storage_bak.write_block(ofs, ptr, size);
#endif
}

bool AP_Param::_hide_disabled_groups = true;

// write a sentinel value at the given offset
void AP_Param::write_sentinel(uint16_t ofs)
{
    struct Param_header phdr;
    phdr.type = _sentinel_type;
    set_key(phdr, _sentinel_key);
    phdr.group_element = _sentinel_group;
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
    sentinel_offset = ofs;
}

// erase all EEPROM variables by re-writing the header and adding
// a sentinel
void AP_Param::erase_all(void)
{
    struct EEPROM_header hdr;

    // write the header
    hdr.magic[0] = k_EEPROM_magic0;
    hdr.magic[1] = k_EEPROM_magic1;
    hdr.revision = k_EEPROM_revision;
    hdr.spare    = 0;
    eeprom_write_check(&hdr, 0, sizeof(hdr));

    // add a sentinel directly after the header
    write_sentinel(sizeof(struct EEPROM_header));
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
    if (grpinfo[i].idx == 0 && shift != 0 && !(grpinfo[i].flags & AP_PARAM_FLAG_NO_SHIFT)) {
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
    if (flags & AP_PARAM_FLAG_HIDDEN) {
        // hidden on all frames
        return false;
    }
    uint16_t frame_flags = flags >> AP_PARAM_FRAME_TYPE_SHIFT;
    if (frame_flags == 0) {
        return true;
    }
    return (frame_flags & _frame_type_flags) != 0;
}

// validate a group info table
void AP_Param::check_group_info(const struct AP_Param::GroupInfo *  group_info,
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
            FATAL("idx too large (%u) in %s", idx, group_info[i].name);
        }
        if (group_shift != 0 && idx == 0) {
            // treat idx 0 as 63 for duplicates. See group_id()
            idx = 63;
        }
        if (used_mask & (1ULL<<idx)) {
            FATAL("Duplicate group idx %u for %s", idx, group_info[i].name);
        }
        used_mask |= (1ULL<<idx);
        if (type == AP_PARAM_GROUP) {
            // a nested group
            if (group_shift + _group_level_shift >= _group_bits) {
                FATAL("double group nesting in %s", group_info[i].name);
            }
            const struct GroupInfo *ginfo = get_group_info(group_info[i]);
            if (ginfo == nullptr) {
                continue;
            }
            check_group_info(ginfo, total_size, group_shift + _group_level_shift, prefix_length + strlen(group_info[i].name));
            continue;
        }
        uint8_t size = type_size((enum ap_var_type)type);
        if (size == 0) {
            FATAL("invalid type in %s", group_info[i].name);
        }
        uint8_t param_name_length = prefix_length + strlen(group_info[i].name);
        if (type == AP_PARAM_VECTOR3F) {
            // need room for _X/_Y/_Z
            param_name_length += 2;
        }
        if (param_name_length > 16) {
            FATAL("suffix is too long in %s (%u > 16)", group_info[i].name, param_name_length);
        }
        (*total_size) += size + sizeof(struct Param_header);
    }
}

// check for duplicate key values
bool AP_Param::duplicate_key(uint16_t vindex, uint16_t key)
{
    for (uint16_t i=vindex+1; i<_num_vars; i++) {
        uint16_t key2 = var_info(i).key;
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
void AP_Param::check_var_info(void)
{
    uint16_t total_size = sizeof(struct EEPROM_header);

    for (uint16_t i=0; i<_num_vars; i++) {
        const auto &info = var_info(i);
        uint8_t type = info.type;
        uint16_t key = info.key;
        if (type == AP_PARAM_GROUP) {
            if (i == 0) {
                FATAL("first element can't be a group, for first() call");
            }
            const struct GroupInfo *group_info = get_group_info(info);
            if (group_info == nullptr) {
                continue;
            }
            check_group_info(group_info, &total_size, 0, strlen(info.name));
        } else {
            uint8_t size = type_size((enum ap_var_type)type);
            if (size == 0) {
                // not a valid type - the top level list can't contain
                // AP_PARAM_NONE
                FATAL("AP_PARAM_NONE at top level");
            }
            total_size += size + sizeof(struct Param_header);
        }
        if (duplicate_key(i, key)) {
            FATAL("duplicate key");
        }
        if (type != AP_PARAM_GROUP && (info.flags & AP_PARAM_FLAG_POINTER)) {
            FATAL("only groups can be pointers");
        }
    }

    // we no longer check if total_size is larger than _eeprom_size,
    // as we allow for more variables than could fit, relying on not
    // saving default values
}


// setup the _var_info[] table
bool AP_Param::setup(void)
{
    struct EEPROM_header hdr {};

    // check the header
    _storage.read_block(&hdr, 0, sizeof(hdr));

#if AP_PARAM_STORAGE_BAK_ENABLED
    struct EEPROM_header hdr2 {};
    _storage_bak.read_block(&hdr2, 0, sizeof(hdr2));
#endif

    if (hdr.magic[0] != k_EEPROM_magic0 ||
        hdr.magic[1] != k_EEPROM_magic1 ||
        hdr.revision != k_EEPROM_revision) {
#if AP_PARAM_STORAGE_BAK_ENABLED
        if (hdr2.magic[0] == k_EEPROM_magic0 &&
            hdr2.magic[1] == k_EEPROM_magic1 &&
            hdr2.revision == k_EEPROM_revision &&
            _storage.copy_area(_storage_bak)) {
            // restored from backup
            INTERNAL_ERROR(AP_InternalError::error_t::params_restored);
            return true;
        }
#endif // AP_PARAM_STORAGE_BAK_ENABLED
        // header doesn't match. We can't recover any variables. Wipe
        // the header and setup the sentinel directly after the header
        Debug("bad header in setup - erasing");
        erase_all();
    }

#if AP_PARAM_STORAGE_BAK_ENABLED
    // ensure that backup is in sync with primary
    _storage_bak.copy_area(_storage);
#endif

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
        if (!get_base(var_info(vindex), base)) {
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

namespace AP {

AP_Param *param()
{
    return AP_Param::get_singleton();
}

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
            if (!get_base(var_info(vindex), base)) {
                continue;
            }
            *ptr = (void*)(base + group_info[i].offset + group_offset);
            return &var_info(vindex);
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
        const auto &info = var_info(i);
        uint8_t type = info.type;
        uint16_t key = info.key;
        if (key != get_key(phdr)) {
            // not the right key
            continue;
        }
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = get_group_info(info);
            if (group_info == nullptr) {
                continue;
            }
            return find_by_header_group(phdr, ptr, i, group_info, 0, 0, 0);
        }
        if (type == phdr.type) {
            // found it
            ptrdiff_t base;
            if (!get_base(info, base)) {
                return nullptr;
            }
            *ptr = (void*)base;
            return &info;
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
    if (!get_base(var_info(vindex), base)) {
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
            return &var_info(vindex);
        } else if (type == AP_PARAM_VECTOR3F &&
                   (base+ofs+(ptrdiff_t)sizeof(float) == (ptrdiff_t) this ||
                    base+ofs+2*(ptrdiff_t)sizeof(float) == (ptrdiff_t) this)) {
            // we are inside a Vector3f. We need to work out which
            // element of the vector the current object refers to.
            *idx = (((ptrdiff_t) this) - (base+ofs))/sizeof(float);
            *group_element = group_id(group_info, group_base, i, group_shift);
            group_ret = &group_info[i];
            return &var_info(vindex);
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
        const auto &info = var_info(i);
        uint8_t type = info.type;
        ptrdiff_t base;
        if (!get_base(info, base)) {
            continue;
        }
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = get_group_info(info);
            if (group_info == nullptr) {
                continue;
            }
            const struct AP_Param::Info *info2;
            info2 = find_var_info_group(group_info, i, 0, 0, 0, group_element, group_ret, group_nesting, idx);
            if (info2 != nullptr) {
                return info2;
            }
        } else if (base == (ptrdiff_t) this) {
            *group_element = 0;
            *idx = 0;
            return &info;
        } else if (type == AP_PARAM_VECTOR3F &&
                   (base+(ptrdiff_t)sizeof(float) == (ptrdiff_t) this ||
                    base+2*(ptrdiff_t)sizeof(float) == (ptrdiff_t) this)) {
            // we are inside a Vector3f. Work out which element we are
            // referring to.
            *idx = (((ptrdiff_t) this) - base)/sizeof(float);
            *group_element = 0;
            return &info;
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
    const auto &info = var_info(i);
    uint8_t type = info.type;
    ptrdiff_t base;
    if (!get_base(info, base)) {
        return nullptr;
    }
    group_ret = nullptr;
    
    if (type == AP_PARAM_GROUP) {
        const struct GroupInfo *group_info = get_group_info(info);
        if (group_info == nullptr) {
            return nullptr;
        }
        const struct AP_Param::Info *info2;
        info2 = find_var_info_group(group_info, i, 0, 0, 0, group_element, group_ret, group_nesting, idx);
        if (info2 != nullptr) {
            return info2;
        }
    } else if (base == (ptrdiff_t) this) {
        *group_element = 0;
        *idx = 0;
        return &info;
    } else if (type == AP_PARAM_VECTOR3F &&
               (base+(ptrdiff_t)sizeof(float) == (ptrdiff_t) this ||
                base+2*(ptrdiff_t)sizeof(float) == (ptrdiff_t) this)) {
        // we are inside a Vector3f. Work out which element we are
        // referring to.
        *idx = (((ptrdiff_t) this) - base)/sizeof(float);
        *group_element = 0;
        return &info;
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
    Debug("unknown type %d\n", type);
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
  return true if a header is the end of eeprom sentinel
 */
bool AP_Param::is_sentinel(const Param_header &phdr)
{
    // note that this is an ||, not an && on the key and group, as
    // this makes us more robust to power off while adding a variable
    // to EEPROM
    if (phdr.type == _sentinel_type ||
        get_key(phdr) == _sentinel_key) {
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
// if not found return the offset of the sentinel
// if the sentinel isn't found either, the offset is set to 0xFFFF
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
        if (is_sentinel(phdr)) {
            // we've reached the sentinel
            *pofs = ofs;
            sentinel_offset = ofs;
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
            if (!get_base(var_info(vindex), base)) {
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
                if (!get_base(var_info(vindex), base)) {
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
AP_Param::find(const char *name, enum ap_var_type *ptype, uint16_t *flags)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        const auto &info = var_info(i);
        uint8_t type = info.type;
        if (type == AP_PARAM_GROUP) {
            uint8_t len = strnlen(info.name, AP_MAX_NAME_SIZE);
            if (strncmp(name, info.name, len) != 0) {
                continue;
            }
            const struct GroupInfo *group_info = get_group_info(info);
            if (group_info == nullptr) {
                continue;
            }
            AP_Param *ap = find_group(name + len, i, 0, group_info, ptype);
            if (ap != nullptr) {
                if (flags != nullptr) {
                    uint32_t group_element = 0;
                    const struct GroupInfo *ginfo;
                    struct GroupNesting group_nesting {};
                    uint8_t idx;
                    ap->find_var_info(&group_element, ginfo, group_nesting, &idx);
                    if (ginfo != nullptr) {
                        *flags = ginfo->flags;
                    } else {
                        *flags = 0;
                    }
                }
                return ap;
            }
            // we continue looking as we want to allow top level
            // parameter to have the same prefix name as group
            // parameters, for example CAM_P_G
        } else if (strcasecmp(name, info.name) == 0) {
            *ptype = (enum ap_var_type)type;
            ptrdiff_t base;
            if (!get_base(info, base)) {
                return nullptr;
            }
            if (flags != nullptr) {
                *flags = 0;
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

// by-name equivalent of find_by_index()
AP_Param* AP_Param::find_by_name(const char* name, enum ap_var_type *ptype, ParamToken *token)
{
    AP_Param *ap;
    for (ap = AP_Param::first(token, ptype);
         ap && *ptype != AP_PARAM_GROUP && *ptype != AP_PARAM_NONE;
         ap = AP_Param::next_scalar(token, ptype)) {
        const auto nlen = strlen(var_info(token->key).name);
        /*
          the name must either match the token name (if a non-group top level param)
          or match up to the length (if a group).
          This check avoids us traversing down into most groups, saving a lot of calls to copy_name_token()
         */
        int32_t ret = strncasecmp(name, var_info(token->key).name, nlen);
        if (ret == 0) {
            char buf[AP_MAX_NAME_SIZE];
            ap->copy_name_token(*token, buf, AP_MAX_NAME_SIZE);
            if (strncasecmp(name, buf, AP_MAX_NAME_SIZE) == 0) {
                break;
            }
        }
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
        if (!get_base(var_info(vindex), base)) {
            continue;
        }
        if (group_info[i].flags & AP_PARAM_FLAG_POINTER) {
            if (ptr == *(void **)(base+group_info[i].offset+offset)) {
                key = var_info(vindex).key;
                return true;
            }
        } else if (ptr == (void *)(base+group_info[i].offset+offset)) {
            key = var_info(vindex).key;
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
        const auto &info = var_info(i);
        if (info.type != AP_PARAM_GROUP) {
            continue;
        }
        if ((info.flags & AP_PARAM_FLAG_POINTER) &&
            ptr == *(void **)info.ptr) {
            key = info.key;
            return true;
        }
        ptrdiff_t offset = 0;
        const struct GroupInfo *ginfo = get_group_info(info);
        if (ginfo == nullptr) {
            continue;
        }
        if (find_key_by_pointer_group(ptr, i, ginfo, offset, key)) {
            return true;
        }
    }
    return false;
}

/*
  Find key to top level group parameters by pointer
*/
bool AP_Param::find_top_level_key_by_pointer(const void *ptr, uint16_t &key)
{
    for (uint16_t i=0; i<_num_vars; i++) {
        const auto &info = var_info(i);
        if (info.type != AP_PARAM_GROUP) {
            continue;
        }
        if (ptr == (void **)info.ptr) {
            key = info.key;
            return true;
        }
    }
    return false;
}

/*
  fetch a parameter value based on the index within a group. This
  is used to find the old value of a parameter that has been
  removed from an object.
*/
bool AP_Param::get_param_by_index(void *obj_ptr, uint32_t idx, ap_var_type old_ptype, void *pvalue)
{
    uint16_t key;
    if (!find_top_level_key_by_pointer(obj_ptr, key)) {
        return false;
    }
    const ConversionInfo type_info = {key, idx, old_ptype, nullptr };
    return AP_Param::find_old_parameter(&type_info, (AP_Param *)pvalue);
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


/*
  Save the variable to HAL storage, synchronous version
*/
void AP_Param::save_sync(bool force_save, bool send_to_gcs)
{
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);
    const AP_Param *ap;

    if (info == nullptr) {
        // we don't have any info on how to store it
        return;
    }

    struct Param_header phdr;

    // create the header we will use to store the variable
    if (ginfo != nullptr) {
        phdr.type = ginfo->type;
        if (ginfo->flags & AP_PARAM_FLAG_HIDDEN) {
            send_to_gcs = false;
        }
    } else {
        phdr.type = info->type;
        if (info->flags & AP_PARAM_FLAG_HIDDEN) {
            send_to_gcs = false;
        }
    }
    set_key(phdr, info->key);
    phdr.group_element = group_element;

    ap = this;
    if (phdr.type != AP_PARAM_VECTOR3F && idx != 0) {
        // only vector3f can have non-zero idx for now
        return;
    }
    if (idx != 0) {
        ap = (const AP_Param *)((ptrdiff_t)ap) - (idx*sizeof(float));
    }

    if (phdr.type == AP_PARAM_INT8 && ginfo != nullptr && (ginfo->flags & AP_PARAM_FLAG_ENABLE)) {
        // clear cached parameter count
        invalidate_count();
    }
    
    char name[AP_MAX_NAME_SIZE+1];
    copy_name_info(info, ginfo, group_nesting, idx, name, sizeof(name), true);

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (scan(&phdr, &ofs)) {
        // found an existing copy of the variable
        eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
        if (send_to_gcs) {
            send_parameter(name, (enum ap_var_type)phdr.type, idx);
        }
        return;
    }
    if (ofs == (uint16_t) ~0) {
        eeprom_full = true;
        DEV_PRINTF("EEPROM full\n");
        return;
    }

    // if the value is the default value then don't save
    if (phdr.type <= AP_PARAM_FLOAT) {
        float v1 = cast_to_float((enum ap_var_type)phdr.type);
        float v2;
        if (ginfo != nullptr) {
            v2 = get_default_value(this, *ginfo);
        } else {
            v2 = get_default_value(this, *info);
        }
        if (is_equal(v1,v2) && !force_save) {
            if (send_to_gcs) {
                GCS_SEND_PARAM(name, (enum ap_var_type)info->type, v2);
            }
            return;
        }
        if (!force_save &&
            (phdr.type != AP_PARAM_INT32 &&
             (fabsf(v1-v2) < 0.0001f*fabsf(v1)))) {
            // for other than 32 bit integers, we accept values within
            // 0.01 percent of the current value as being the same
            if (send_to_gcs) {
                GCS_SEND_PARAM(name, (enum ap_var_type)info->type, v2);
            }
            return;
        }
    }

    if (ofs+type_size((enum ap_var_type)phdr.type)+2*sizeof(phdr) >= _storage.size()) {
        // we are out of room for saving variables
        eeprom_full = true;
        DEV_PRINTF("EEPROM full\n");
        return;
    }

    // write a new sentinel, then the data, then the header
    write_sentinel(ofs + sizeof(phdr) + type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum ap_var_type)phdr.type));
    eeprom_write_check(&phdr, ofs, sizeof(phdr));

    if (send_to_gcs) {
        send_parameter(name, (enum ap_var_type)phdr.type, idx);
    }
}

/*
  put variable into queue to be saved
*/
void AP_Param::save(bool force_save)
{
    struct param_save p, p2;
    p.param = this;
    p.force_save = force_save;
    if (save_queue.peek(p2) &&
        p2.param == this &&
        p2.force_save == force_save) {
        // this one is already at the head of the list to be
        // saved. This check is cheap and catches the case where we
        // are flooding the save queue with one parameter (eg. mission
        // creation, changing MIS_TOTAL)
        return;
    }
    while (!save_queue.push(p)) {
        // if we can't save to the queue
        if (hal.util->get_soft_armed() && hal.scheduler->in_main_thread()) {
            // if we are armed in main thread then don't sleep, instead we lose the
            // parameter save
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }
        // when we are disarmed then loop waiting for a slot to become
        // available. This guarantees completion for large parameter
        // set loads
        hal.scheduler->expect_delay_ms(1);
        hal.scheduler->delay_microseconds(500);
        hal.scheduler->expect_delay_ms(0);
    }
}

/*
  background function for saving parameters. This runs on the IO thread
 */
void AP_Param::save_io_handler(void)
{
    struct param_save p;
    while (save_queue.pop(p)) {
        p.param->save_sync(p.force_save, true);
    }
    if (hal.scheduler->is_system_initialized()) {
        // pay the cost of parameter counting in the IO thread
        count_parameters();
    }
}

/*
  wait for all parameters to save
*/
void AP_Param::flush(void)
{
    uint16_t counter = 200; // 2 seconds max
    while (counter-- && save_queue.available()) {
        hal.scheduler->expect_delay_ms(10);
        hal.scheduler->delay(10);
        hal.scheduler->expect_delay_ms(0);
    }
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
                      get_default_value(this, *ginfo));
        } else {
            set_value((enum ap_var_type)phdr.type, (void*)base, 
                      get_default_value(this, *info));
        }
        return false;
    }

    if (phdr.type != AP_PARAM_VECTOR3F && idx != 0) {
        // only vector3f can have non-zero idx for now
        return false;
    }

    AP_Param *ap = this;
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

bool AP_Param::configured_in_defaults_file(bool &read_only) const
{
    if (num_param_overrides == 0) {
        return false;
    }
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
            read_only = param_overrides[i].read_only;
            return true;
        }
    }

    return false;
}

bool AP_Param::configured(void) const
{
    bool read_only;
    return configured_in_defaults_file(read_only) || configured_in_storage();
}

bool AP_Param::is_read_only(void) const
{
    if (num_read_only == 0) {
        return false;
    }
    bool read_only;
    if (configured_in_defaults_file(read_only)) {
        return read_only;
    }
    return false;
}

// returns true if this parameter should be settable via the
// MAVLink interface:
bool AP_Param::allow_set_via_mavlink(uint16_t flags) const
{
    if (is_read_only()) {
        return false;
    }

    if (flags & AP_PARAM_FLAG_INTERNAL_USE_ONLY) {
        // the user can set BRD_OPTIONS to enable set of internal
        // parameters, for developer testing or unusual use cases
        if (!AP_BoardConfig::allow_set_internal_parameters()) {
            return false;
        }
    }

#if HAL_GCS_ENABLED
    // check the MAVLink library is OK with the concept:
    if (!gcs().get_allow_param_set()) {
        return false;
    }
#endif  // HAL_GCS_ENABLED

    return true;
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
                      get_default_value((const AP_Param *)ptr, group_info[i]));
        } else if (type == AP_PARAM_VECTOR3F) {
            // Single default for all components
            void *ptr = (void *)(base + group_info[i].offset);
            const float default_val = get_default_value((const AP_Param *)ptr, group_info[i]);
            ((AP_Vector3f *)ptr)->set(Vector3f{default_val, default_val, default_val});
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
        const auto &info = var_info(i);
        uint8_t type = info.type;
        if (type <= AP_PARAM_FLOAT) {
            ptrdiff_t base;
            if (get_base(info, base)) {
                set_value((enum ap_var_type)type, (void*)base,
                          get_default_value((const AP_Param *)base, info));
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

    if (!registered_save_handler) {
        registered_save_handler = true;
        hal.scheduler->register_io_process(FUNCTOR_BIND((&save_dummy), &AP_Param::save_io_handler, void));
    }
    
    while (ofs < _storage.size()) {
        _storage.read_block(&phdr, ofs, sizeof(phdr));
        if (is_sentinel(phdr)) {
            // we've reached the sentinel
            sentinel_offset = ofs;
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

    // we didn't find the sentinel
    Debug("no sentinel in load_all");
    return false;
}

/*
 * reload from hal.util defaults file or embedded param region
 * @last_pass: if this is the last pass on defaults - unknown parameters are
 *             ignored but if this is set a warning will be emitted
 */
void AP_Param::reload_defaults_file(bool last_pass)
{
#if AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED

    /*
      if the HAL specifies a defaults parameter file then override
      defaults using that file
     */
    const char *default_file = hal.util->get_custom_defaults_file();
    if (default_file) {
#if AP_FILESYSTEM_FILE_READING_ENABLED
        load_defaults_file_from_filesystem(default_file, last_pass);
#elif defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)
        load_defaults_file_from_romfs(default_file, last_pass);
#endif
    }

#endif  // AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED

#if AP_PARAM_MAX_EMBEDDED_PARAM > 0
    if (param_defaults_data.length != 0) {
        load_embedded_param_defaults(last_pass);
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    hal.util->set_cmdline_parameters();
#endif
}

#if AP_FILESYSTEM_FILE_READING_ENABLED
void AP_Param::load_defaults_file_from_filesystem(const char *default_file, bool last_pass)
{
    if (load_defaults_file(default_file, last_pass)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        printf("Loaded defaults from %s\n", default_file);
#endif
    } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Failed to load defaults from %s", default_file);
#else
        printf("Failed to load defaults from %s\n", default_file);
#endif
    }
}
#endif  // AP_FILESYSTEM_FILE_READING_ENABLED

#if defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)
void AP_Param::load_defaults_file_from_romfs(const char *default_file, bool last_pass)
{
    const char *prefix = "@ROMFS/";
    if (strncmp(default_file, prefix, strlen(prefix)) != 0) {
        // does not start with ROMFS, do not attempt to retrieve from it
        return;
    }

    // filename without the prefix:
    const char *trimmed_filename = &default_file[strlen(prefix)];

    uint32_t string_length;
    const uint8_t *text = AP_ROMFS::find_decompress(trimmed_filename, string_length);
    if (text == nullptr) {
        return;
    }

    load_param_defaults((const char*)text, string_length, last_pass);

    AP_ROMFS::free(text);

}
#endif  // HAL_HAVE_AP_ROMFS_EMBEDDED_H

/* 
   Load all variables from EEPROM for a particular object. This is
   required for dynamically loaded objects
 */
void AP_Param::load_object_from_eeprom(const void *object_pointer, const struct GroupInfo *group_info)
{
    struct Param_header phdr;
    uint16_t key;

    if (!find_key_by_pointer(object_pointer, key)) {
        DEV_PRINTF("ERROR: Unable to find param pointer\n");
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
            if (is_sentinel(phdr)) {
                // we've reached the sentinel
                sentinel_offset = ofs;
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

    if (!done_all_default_params) {
        /*
          the new subtree may need defaults from defaults.parm
         */
        reload_defaults_file(false);
    }

    // reset cached param counter as we may be loading a dynamic var_info
    invalidate_count();
}


// return the first variable in _var_info
AP_Param *AP_Param::first(ParamToken *token, enum ap_var_type *ptype, float *default_val)
{
    token->key = 0;
    token->group_element = 0;
    token->idx = 0;
    if (_num_vars == 0) {
        return nullptr;
    }
    ptrdiff_t base;
    if (!get_base(var_info(0), base)) {
        // should be impossible, first var needs to be non-pointer
        return nullptr;
    }
    if (ptype != nullptr) {
        *ptype = (enum ap_var_type)var_info(0).type;
    }
#if AP_PARAM_DEFAULTS_ENABLED
    if (default_val != nullptr) {
        *default_val = get_default_value((AP_Param *)base, var_info(0));
    }
    check_default((AP_Param *)base, default_val);
#endif
    return (AP_Param *)base;
}

/// Returns the next variable in a group, recursing into groups
/// as needed
AP_Param *AP_Param::next_group(const uint16_t vindex, const struct GroupInfo *group_info,
                               bool *found_current,
                               const uint32_t group_base,
                               const uint8_t group_shift,
                               const ptrdiff_t group_offset,
                               ParamToken *token,
                               enum ap_var_type *ptype,
                               bool skip_disabled,
                               float *default_val)
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
                            group_shift + _group_level_shift, new_offset, token, ptype, skip_disabled, default_val);
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
                if (!get_base(var_info(vindex), base)) {
                    continue;
                }

                AP_Param *ret = (AP_Param*)(base + group_info[i].offset + group_offset);

                if (skip_disabled &&
                    _hide_disabled_groups &&
                    group_info[i].type == AP_PARAM_INT8 &&
                    (group_info[i].flags & AP_PARAM_FLAG_ENABLE) &&
                    ((AP_Int8 *)ret)->get() == 0) {
                    token->last_disabled = 1;
                }
#if AP_PARAM_DEFAULTS_ENABLED
                if (default_val != nullptr) {
                    *default_val = get_default_value(ret, group_info[i]);
                }
#endif
                return ret;
            }
            if (group_id(group_info, group_base, i, group_shift) == token->group_element) {
                *found_current = true;
                if (token->last_disabled) {
                    token->last_disabled = 0;
                    return nullptr;
                }
                if (type == AP_PARAM_VECTOR3F && token->idx < 3) {
                    // return the next element of the vector as a
                    // float
                    token->idx++;
                    if (ptype != nullptr) {
                        *ptype = AP_PARAM_FLOAT;
                    }
                    ptrdiff_t base;
                    if (!get_base(var_info(vindex), base)) {
                        continue;
                    }
                    ptrdiff_t ofs = base + group_info[i].offset + group_offset;
                    ofs += sizeof(float)*(token->idx - 1u);
#if AP_PARAM_DEFAULTS_ENABLED
                    if (default_val != nullptr) {
                        *default_val = get_default_value((AP_Param *)ofs, group_info[i]);
                    }
#endif
                    return (AP_Param *)ofs;
                }
            }
        }
    }
    return nullptr;
}

/// Returns the next variable in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next(ParamToken *token, enum ap_var_type *ptype, bool skip_disabled, float *default_val)
{
    uint16_t i = token->key;
    bool found_current = false;
    if (i >= _num_vars) {
        // illegal token
        return nullptr;
    }
    enum ap_var_type type = (enum ap_var_type)var_info(i).type;

    // allow Vector3f to be seen as 3 variables. First as a vector,
    // then as 3 separate floats
    if (type == AP_PARAM_VECTOR3F && token->idx < 3) {
        token->idx++;
        if (ptype != nullptr) {
            *ptype = AP_PARAM_FLOAT;
        }
        AP_Param *ret = (AP_Param *)(((token->idx - 1u)*sizeof(float))+(ptrdiff_t)var_info(i).ptr);
#if AP_PARAM_DEFAULTS_ENABLED
        if (default_val != nullptr) {
            *default_val = get_default_value(ret, var_info(i));
        }
#endif
        return ret;
    }

    if (type != AP_PARAM_GROUP) {
        i++;
        found_current = true;
    }
    for (; i<_num_vars; i++) {
        const auto &info = var_info(i);
        if (!check_frame_type(info.flags)) {
            continue;
        }
        type = (enum ap_var_type)info.type;
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = get_group_info(info);
            if (group_info == nullptr) {
                continue;
            }
            AP_Param *ap = next_group(i, group_info, &found_current, 0, 0, 0, token, ptype, skip_disabled, default_val);
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
#if AP_PARAM_DEFAULTS_ENABLED
            if (default_val != nullptr) {
                *default_val = get_default_value((AP_Param *)info.ptr, info);
            }
#endif
            return (AP_Param *)(info.ptr);
        }
    }
    return nullptr;
}

/// Returns the next scalar in _var_info, recursing into groups
/// as needed
AP_Param *AP_Param::next_scalar(ParamToken *token, enum ap_var_type *ptype, float *default_val)
{
    AP_Param *ap;
    enum ap_var_type type;
    while ((ap = next(token, &type, true, default_val)) != nullptr && type > AP_PARAM_FLOAT) ;

    if (ap != nullptr) {
        if (ptype != nullptr) {
            *ptype = type;
        }
    }
#if AP_PARAM_DEFAULTS_ENABLED
    check_default(ap, default_val);
#endif
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
    AP_Param *ap2 = find(&info->new_name[0], &ptype);
    if (ap2 == nullptr) {
        DEV_PRINTF("Unknown conversion '%s'\n", info->new_name);
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
        DEV_PRINTF("Bad conversion type '%s'\n", info->new_name);
    }
}
#pragma GCC diagnostic pop


// convert old vehicle parameters to new object parameters
void AP_Param::convert_old_parameters(const struct ConversionInfo *conversion_table, uint8_t table_size, uint8_t flags)
{
    convert_old_parameters_scaled(conversion_table, table_size, 1.0f, flags);
}

// convert old vehicle parameters to new object parameters with scaling - assumes all parameters will have the same scaling factor
void AP_Param::convert_old_parameters_scaled(const struct ConversionInfo *conversion_table, uint8_t table_size, float scaler, uint8_t flags)
{
    for (uint8_t i=0; i<table_size; i++) {
        convert_old_parameter(&conversion_table[i], scaler, flags);
    }
    // we need to flush here to prevent a later set_default_by_name()
    // causing a save to be done on a converted parameter
    flush();
}

// move all parameters from a class to a new location
// is_top_level: Is true if the class had its own top level key, param_key. It is false if the class was a subgroup
void AP_Param::convert_class(uint16_t param_key, void *object_pointer,
                                    const struct AP_Param::GroupInfo *group_info,
                                    uint16_t old_index, bool is_top_level, bool recurse_sub_groups)
{
    const uint8_t group_shift = is_top_level ? 0 : 6;

    for (uint8_t i=0; group_info[i].type != AP_PARAM_NONE; i++) {
        struct ConversionInfo info;
        info.old_key = param_key;
        info.type = (ap_var_type)group_info[i].type;
        info.new_name = nullptr;

        uint16_t idx = group_info[i].idx;
        if (group_shift != 0 && idx == 0) {
            // Note: Index 0 is treated as 63 for group bit shifting purposes. See group_id()
            idx = 63;
        }

        if (info.type == AP_PARAM_GROUP) {
            // Convert subgroups if enabled
            if (recurse_sub_groups) {
                // Only recurse once
                convert_class(param_key, (uint8_t *)object_pointer + group_info[i].offset, get_group_info(group_info[i]), idx, false, false);
            }
            continue;
        }

        info.old_group_element = (idx << group_shift) + old_index;

        uint8_t old_value[type_size(info.type)];
        AP_Param *ap = (AP_Param *)&old_value[0];
        
        if (!AP_Param::find_old_parameter(&info, ap)) {
            // the parameter wasn't set in the old eeprom
            continue;
        }

        AP_Param *ap2 = (AP_Param *)(group_info[i].offset + (uint8_t *)object_pointer);
        if (ap2->configured_in_storage()) {
            // user has already set a value, or previous conversion was done
            continue;
        }
        memcpy(ap2, ap, sizeof(old_value));
        // and save
        ap2->save();
    }

    // we need to flush here to prevent a later set_default_by_name()
    // causing a save to be done on a converted parameter
    flush();
}

// convert an object which was stored in a vehicle's G2 into a new
// object in AP_Vehicle.cpp:
void AP_Param::convert_g2_objects(const void *g2, const G2ObjectConversion g2_conversions[], uint8_t num_conversions)
{
    // Find G2's Top Level Key
    ConversionInfo info;
    if (!find_top_level_key_by_pointer(g2, info.old_key)) {
        return;
    }
    for (uint8_t i=0; i<num_conversions; i++) {
        const auto &c { g2_conversions[i] };
        convert_class(info.old_key, c.object_pointer, c.var_info, c.old_index, false);
    }
}

void AP_Param::convert_toplevel_objects(const TopLevelObjectConversion conversions[], uint8_t num_conversions)
{
    for (uint8_t i=0; i<num_conversions; i++) {
        const auto &c { conversions[i] };
        convert_class(c.old_index, c.object_pointer, c.var_info, 0, true);
    }
}

/*
 convert width of a parameter, allowing update to wider scalar values
 without changing the parameter indexes
*/
bool AP_Param::_convert_parameter_width(ap_var_type old_ptype, float scale_factor, bool bitmask)
{
    if (configured_in_storage()) {
        // already converted or set by the user
        return false;
    }

    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    struct GroupNesting group_nesting {};
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, ginfo, group_nesting, &idx);

    if (info == nullptr) {
        return false;
    }

    // remember the type
    ap_var_type new_ptype;
    if (ginfo != nullptr) {
        new_ptype = (ap_var_type)ginfo->type;
    } else {
        new_ptype = (ap_var_type)info->type;
    }
    
    // create the header we will use to scan for the variable
    struct Param_header phdr;
    phdr.type = old_ptype;
    set_key(phdr, info->key);
    phdr.group_element = group_element;

    // scan EEPROM to find the right location
    uint16_t pofs;
    if (!scan(&phdr, &pofs)) {
        // it isn't in storage
        return false;
    }

    // load the old value from EEPROM
    uint8_t old_value[type_size(old_ptype)];
    _storage.read_block(old_value, pofs+sizeof(phdr), sizeof(old_value));
    
    AP_Param *old_ap = (AP_Param *)&old_value[0];

    if (!bitmask) {
        // Numeric conversion
        // going via float is safe as the only time we would be converting
        // from AP_Int32 is when converting to float
        float old_float_value = old_ap->cast_to_float(old_ptype);
        set_value(new_ptype, this, old_float_value*scale_factor);

    } else {
        // Bitmask conversion, go via uint32
        // int8 -1 should convert to int16 255
        uint32_t mask;
        switch (old_ptype) {
        case AP_PARAM_INT8:
            mask = (uint8_t)(*(AP_Int8*)old_ap);
            break;
        case AP_PARAM_INT16:
            mask = (uint16_t)(*(AP_Int16*)old_ap);
            break;
        case AP_PARAM_INT32:
            mask = (uint32_t)(*(AP_Int32*)old_ap);
            break;
        default:
            return false;
        }

        switch (new_ptype) {
        case AP_PARAM_INT8:
            ((AP_Int8 *)this)->set(mask);
            break;
        case AP_PARAM_INT16:
            ((AP_Int16 *)this)->set(mask);
            break;
        case AP_PARAM_INT32:
            ((AP_Int32 *)this)->set(mask);
            break;
        default:
            return false;
        }
    }


    // force save as the new type
    save(true);

    return true;
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
        v = constrain_float(v, INT32_MIN, INT32_MAX);
        ((AP_Int32 *)this)->set(v);
    } else if (var_type == AP_PARAM_INT16) {
        if (value < 0) rounding_addition = -rounding_addition;
        float v = value+rounding_addition;
        v = constrain_float(v, INT16_MIN, INT16_MAX);
        ((AP_Int16 *)this)->set(v);
    } else if (var_type == AP_PARAM_INT8) {
        if (value < 0) rounding_addition = -rounding_addition;
        float v = value+rounding_addition;
        v = constrain_float(v, INT8_MIN, INT8_MAX);
        ((AP_Int8 *)this)->set(v);
    }
}


/*
  parse a parameter file line
 */
bool AP_Param::parse_param_line(char *line, char **vname, float &value, bool &read_only)
{
    if (line[0] == '#') {
        return false;
    }
    char *saveptr = nullptr;
    /*
      note that we need the \r\n as delimiters to prevent us getting
      strings with line termination in the results
     */
    char *pname = strtok_r(line, ", =\t\r\n", &saveptr);
    if (pname == nullptr) {
        return false;
    }
    if (strlen(pname) > AP_MAX_NAME_SIZE) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Workaround to prevent FORMAT_VERSION in param file resulting in invalid
    // EEPROM. For details, see: https://github.com/ArduPilot/ardupilot/issues/15579
    if (strcmp(pname, "FORMAT_VERSION") == 0) {
        ::printf("Warning: Ignoring FORMAT_VERSION in param file\n");
        return false;
    }
#endif

    const char *value_s = strtok_r(nullptr, ", =\t\r\n", &saveptr);
    if (value_s == nullptr) {
        return false;
    }
    value = strtof(value_s, NULL);
    *vname = pname;

    const char *flags_s = strtok_r(nullptr, ", =\t\r\n", &saveptr);
    if (flags_s && strcmp(flags_s, "@READONLY") == 0) {
        read_only = true;
    } else {
        read_only = false;
    }

    return true;
}


#if AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED || AP_PARAM_DYNAMIC_ENABLED

// increments num_defaults for each default found in filename
bool AP_Param::count_defaults_in_file(const char *filename, uint16_t &num_defaults)
{
    // try opening the file both in the posix filesystem and using AP::FS
    int file_apfs = AP::FS().open(filename, O_RDONLY, true);
    if (file_apfs == -1) {
        return false;
    }
    char line[100];

    /*
      work out how many parameter default structures to allocate
     */
    while (AP::FS().fgets(line, sizeof(line)-1, file_apfs)) {
        char *pname;
        float value;
        bool read_only;
        if (!parse_param_line(line, &pname, value, read_only)) {
            continue;
        }
        enum ap_var_type var_type;
        if (!find(pname, &var_type)) {
            continue;
        }
        num_defaults++;
    }
    AP::FS().close(file_apfs);

    return true;
}

bool AP_Param::read_param_defaults_file(const char *filename, bool last_pass, uint16_t &idx)
{
    // try opening the file both in the posix filesystem and using AP::FS
    int file_apfs = AP::FS().open(filename, O_RDONLY, true);
    if (file_apfs == -1) {
        AP_HAL::panic("AP_Param: Failed to re-open defaults file");
        return false;
    }

    bool done_all = true;
    char line[100];
    while (AP::FS().fgets(line, sizeof(line)-1, file_apfs)) {
        char *pname;
        float value;
        bool read_only;
        if (!parse_param_line(line, &pname, value, read_only)) {
            continue;
        }
        enum ap_var_type var_type;
        AP_Param *vp = find(pname, &var_type);
        if (!vp) {
            if (last_pass) {
#if ENABLE_DEBUG
                ::printf("Ignored unknown param %s in defaults file %s\n",
                         pname, filename);
                hal.console->printf(
                         "Ignored unknown param %s in defaults file %s\n",
                         pname, filename);
#endif
            }
            done_all = false;
            continue;
        }
        if (idx >= param_overrides_len) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        param_overrides[idx].object_ptr = vp;
        param_overrides[idx].value = value;
        param_overrides[idx].read_only = read_only;
        if (read_only) {
            num_read_only++;
        }
        idx++;
        if (!vp->configured_in_storage()) {
            vp->set_float(value, var_type);
        }
    }
    AP::FS().close(file_apfs);

    done_all_default_params = done_all;

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

    delete[] param_overrides;
    param_overrides_len = 0;
    num_param_overrides = 0;
    num_read_only = 0;

    param_overrides = NEW_NOTHROW param_override[num_defaults];
    if (param_overrides == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate overrides");
        return false;
    }
    param_overrides_len = num_defaults;

    if (num_defaults == 0) {
        return true;
    }

    saveptr = nullptr;
    mutable_filename = strdup(filename);
    if (mutable_filename == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate mutable string");
    }
    uint16_t idx = 0;
    for (char *pname = strtok_r(mutable_filename, ",", &saveptr);
         pname != nullptr;
         pname = strtok_r(nullptr, ",", &saveptr)) {
        if (!read_param_defaults_file(pname, last_pass, idx)) {
            free(mutable_filename);
            return false;
        }
    }
    free(mutable_filename);

    num_param_overrides = num_defaults;

    return true;
}
#endif // AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED

#if AP_PARAM_MAX_EMBEDDED_PARAM > 0 || defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)
/*
  count the number of parameter defaults present in supplied string
 */
bool AP_Param::count_param_defaults(const volatile char *ptr, int32_t length, uint16_t &count)
{
    count = 0;
    
    while (length>0) {
        char line[100];
        char *pname;
        float value;
        bool read_only;
        uint16_t i;
        uint16_t n = length;
        for (i=0;i<n;i++) {
            if (ptr[i] == '\n') {
                break;
            }
        }

        uint16_t linelen = MIN(i,sizeof(line)-1);
        memcpy(line, (void *)ptr, linelen);
        line[linelen] = 0;

        length -= i+1;
        ptr += i+1;
        
        if (line[0] == '#' || line[0] == 0) {
            continue;
        }

        if (!parse_param_line(line, &pname, value, read_only)) {
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
 *  load parameter defaults from supplied string
 */
void AP_Param::load_param_defaults(const volatile char *ptr, int32_t length, bool last_pass)
{
    delete[] param_overrides;
    param_overrides = nullptr;
    param_overrides_len = 0;
    num_param_overrides = 0;
    num_read_only = 0;

    uint16_t num_defaults = 0;
    if (!count_param_defaults(ptr, length, num_defaults)) {
        return;
    }

    param_overrides = NEW_NOTHROW param_override[num_defaults];
    if (param_overrides == nullptr) {
        AP_HAL::panic("AP_Param: Failed to allocate overrides");
        return;
    }

    param_overrides_len = num_defaults;

    uint16_t idx = 0;
    
    while (idx < num_defaults && length > 0) {
        char line[100];
        char *pname;
        float value;
        bool read_only;
        uint16_t i;
        uint16_t n = length;
        for (i=0;i<n;i++) {
            if (ptr[i] == '\n') {
                break;
            }
        }

        uint16_t linelen = MIN(i,sizeof(line)-1);
        memcpy(line, (void *)ptr, linelen);
        line[linelen] = 0;

        length -= i+1;
        ptr += i+1;

        if (line[0] == '#' || line[0] == 0) {
            continue;
        }
        
        if (!parse_param_line(line, &pname, value, read_only)) {
            continue;
        }
        enum ap_var_type var_type;
        AP_Param *vp = find(pname, &var_type);
        if (!vp) {
            if (last_pass) {
#if ENABLE_DEBUG && (AP_PARAM_MAX_EMBEDDED_PARAM > 0)
                ::printf("Ignored unknown param %s from embedded region (offset=%u)\n",
                         pname, unsigned(ptr - param_defaults_data.data));
                hal.console->printf(
                         "Ignored unknown param %s from embedded region (offset=%u)\n",
                         pname, unsigned(ptr - param_defaults_data.data));
#endif
            }
            continue;
        }
        param_overrides[idx].object_ptr = vp;
        param_overrides[idx].value = value;
        param_overrides[idx].read_only = read_only;
        if (read_only) {
            num_read_only++;
        }
        idx++;
        if (!vp->configured_in_storage()) {
            vp->set_float(value, var_type);
        }
    }
    num_param_overrides = num_defaults;
}
#endif // AP_PARAM_MAX_EMBEDDED_PARAM > 0 || defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)


#if AP_PARAM_MAX_EMBEDDED_PARAM > 0
/*
 * load a default set of parameters from a embedded parameter region
 * @last_pass: if this is the last pass on defaults - unknown parameters are
 *             ignored but if this is set a warning will be emitted
 */
void AP_Param::load_embedded_param_defaults(bool last_pass)
{
    load_param_defaults(param_defaults_data.data, param_defaults_data.length, last_pass);
}
#endif  // AP_PARAM_MAX_EMBEDDED_PARAM > 0


/* 
   find a default value given a pointer to a default value in flash
 */
float AP_Param::get_default_value(const AP_Param *vp, const struct GroupInfo &info)
{
    for (uint16_t i=0; i<num_param_overrides; i++) {
        if (vp == param_overrides[i].object_ptr) {
            return param_overrides[i].value;
        }
    }
    if ((info.flags & AP_PARAM_FLAG_DEFAULT_POINTER) != 0) {
        return *((float*)((ptrdiff_t)vp - info.def_value_offset));
    }
    return info.def_value;
}

float AP_Param::get_default_value(const AP_Param *vp, const struct Info &info)
{
    for (uint16_t i=0; i<num_param_overrides; i++) {
        if (vp == param_overrides[i].object_ptr) {
            return param_overrides[i].value;
        }
    }
    if ((info.flags & AP_PARAM_FLAG_DEFAULT_POINTER) != 0) {
        return *((float*)((ptrdiff_t)vp - info.def_value_offset));
    }
    return info.def_value;
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
        GCS_SEND_PARAM(name, var_type, cast_to_float(var_type));
        return;
    }

    // for vectors we need to send 3 messages. Note that we also come here for the case
    // of a set of the first element of a AP_Vector3f. This happens as the ap->save() call can't
    // distinguish between a vector and scalar save. It means that setting first element of a vector
    // via MAVLink results in sending all 3 elements to the GCS
#if HAL_GCS_ENABLED
    const Vector3f &v = ((AP_Vector3f *)this)->get();
    char name2[AP_MAX_NAME_SIZE+1];
    strncpy(name2, name, AP_MAX_NAME_SIZE);
    name2[AP_MAX_NAME_SIZE] = 0;
    char &name_axis = name2[strlen(name)-1];
    
    name_axis = 'X';
    GCS_SEND_PARAM(name2, AP_PARAM_FLOAT, v.x);
    name_axis = 'Y';
    GCS_SEND_PARAM(name2, AP_PARAM_FLOAT, v.y);
    name_axis = 'Z';
    GCS_SEND_PARAM(name2, AP_PARAM_FLOAT, v.z);
#endif // HAL_GCS_ENABLED
}

/*
  return count of all scalar parameters.
  Note that this function may be called from the IO thread, so needs
  to be thread safe
 */
uint16_t AP_Param::count_parameters(void)
{
    // if we haven't cached the parameter count yet...
    WITH_SEMAPHORE(_count_sem);
    if (_parameter_count != 0 &&
        _count_marker == _count_marker_done) {
        return _parameter_count;
    }
    /*
      cope with another thread invalidating the count while we are
      counting
     */
    uint8_t limit = 4;
    while ((_parameter_count == 0 ||
            _count_marker != _count_marker_done) &&
           limit--) {
        AP_Param  *vp;
        AP_Param::ParamToken token {};
        uint16_t count = 0;
        uint16_t marker = _count_marker;

        for (vp = AP_Param::first(&token, nullptr);
             vp != nullptr;
             vp = AP_Param::next_scalar(&token, nullptr)) {
            count++;
        }
        _parameter_count = count;
        _count_marker_done = marker;
    }
    return _parameter_count;
}

/*
  invalidate parameter count cache
 */
void AP_Param::invalidate_count(void)
{
    // we don't take the semaphore here as we don't want to block. The
    // not-equal test is strong enough to ensure we get the right
    // answer
    _count_marker++;
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
  set parameter defaults from a defaults_struct table
  sends GCS message and panics (in SITL only) if parameter is not found
 */
void AP_Param::set_defaults_from_table(const struct defaults_table_struct *table, uint8_t count)
{
    for (uint8_t i=0; i<count; i++) {
        if (!AP_Param::set_default_by_name(table[i].name, table[i].value)) {
            AP_BoardConfig::config_error("param deflt fail:%s", table[i].name);
        }
    }
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
  get a value by name
 */
bool AP_Param::get(const char *name, float &value)
{
    enum ap_var_type vtype;
    AP_Param *vp = find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        value = ((AP_Int8 *)vp)->get();
        break;
    case AP_PARAM_INT16:
        value = ((AP_Int16 *)vp)->get();
        break;

    case AP_PARAM_INT32:
        value = ((AP_Int32 *)vp)->get();
        break;

    case AP_PARAM_FLOAT:
        value = ((AP_Float *)vp)->get();
        break;

    default:
        // not a supported type
        return false;
    }
    return true;
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

/*
  set and save a value by name
 */
bool AP_Param::set_and_save_by_name_ifchanged(const char *name, float value)
{
    enum ap_var_type vtype;
    AP_Param *vp = find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_and_save_ifchanged(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_and_save_ifchanged(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_and_save_ifchanged(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_and_save_ifchanged(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

#if AP_PARAM_DEFAULTS_ENABLED
void AP_Param::check_default(AP_Param *ap, float *default_value)
{
    if (default_value == nullptr || ap == nullptr) {
        return;
    }
    if (default_list != nullptr) {
        for (defaults_list *item = default_list; item; item = item->next) {
            if (item->ap == ap) {
                *default_value = item->val;
                return;
            }
        }
    }
}

void AP_Param::add_default(AP_Param *ap, float v)
{
    // Embedded defaults trump runtime, don't allow override
    for (uint16_t i=0; i<num_param_overrides; i++) {
        if (ap == param_overrides[i].object_ptr) {
            return;
        }
    }

    if (default_list != nullptr) {
        // check is param is already in list
        for (defaults_list *item = default_list; item; item = item->next) {
            // update existing entry
            if (item->ap == ap) {
                item->val = v;
                return;
            }
        }
    }

    // add to list
    defaults_list *new_item = NEW_NOTHROW defaults_list;
    if (new_item == nullptr) {
        return;
    }
    new_item->ap = ap;
    new_item->val = v;
    new_item->next = default_list;
    default_list = new_item;
}
#endif // AP_PARAM_DEFAULTS_ENABLED


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
        ::printf("%s: %d\n", s, (int)((AP_Int8 *)ap)->get());
        break;
    case AP_PARAM_INT16:
        ::printf("%s: %d\n", s, (int)((AP_Int16 *)ap)->get());
        break;
    case AP_PARAM_INT32:
        ::printf("%s: %ld\n", s, (long)((AP_Int32 *)ap)->get());
        break;
    case AP_PARAM_FLOAT:
        ::printf("%s: %f\n", s, (double)((AP_Float *)ap)->get());
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
    float default_value = NaNf;  // from logger quiet_nanf

    for (ap=AP_Param::first(&token, &type, &default_value);
         ap;
         ap=AP_Param::next_scalar(&token, &type, &default_value)) {
        if (showKeyValues) {
            ::printf("Key %u: Index %u: GroupElement %u : Default %f  :", (unsigned)var_info(token.key).key, (unsigned)token.idx, (unsigned)token.group_element, default_value);
            default_value = NaNf;
        }
        show(ap, token, type, port);
        hal.scheduler->delay(1);
    }

#if AP_PARAM_DEFAULTS_ENABLED
    uint16_t list_len = 0;
    if (default_list != nullptr) {
        for (defaults_list *item = default_list; item; item = item->next) {
            list_len++;
        }
    }
    ::printf("Defaults list containts %i params (%li bytes)\n", list_len, list_len*sizeof(defaults_list));
#endif
}
#endif // AP_PARAM_KEY_DUMP


#if AP_PARAM_DYNAMIC_ENABLED
/*
  allow for dynamically added parameter tables from scripts

  The layout we create is as follows:
    - a top level Info with the given prefix, using one of the 10 possible slots in _var_info_dynamic
    - a dynamically allocated GroupInfo table, never freed, of size (num_params+2)
    - the GroupInfo table has an initial AP_Int32 hidden entry with a 32 bit CRC of the prefix
    - the last GroupInfo is a footer
*/
bool AP_Param::add_table(uint8_t _key, const char *prefix, uint8_t num_params)
{
    // check if the key already exists. We only check base params to allow
    // for scripting reload without a conflict
    uint16_t key = uint16_t(_key) + AP_PARAM_DYNAMIC_KEY_BASE;
    for (uint16_t i=0; i<_num_vars_base; i++) {
        if (var_info(i).key == key) {
            return false;
        }
    }
    if (num_params > 63) {
        return false;
    }

    // we use a crc of the prefix to ensure the table key isn't re-used
    const int32_t crc = int32_t(crc32_small(0, (const uint8_t *)prefix, strlen(prefix)));
    int32_t current_crc;
    if (load_int32(key, 0, current_crc) && current_crc != crc) {
        // crc mismatch, we have a conflict with an existing use of this key
        return false;
    }

    // create the dynamic table if needed. This is never freed
    if (_var_info_dynamic == nullptr) {
        _var_info_dynamic = (Info *)calloc(AP_PARAM_MAX_DYNAMIC, sizeof(struct Info));
        if (_var_info_dynamic == nullptr) {
            return false;
        }
        for (uint8_t i=0; i<AP_PARAM_MAX_DYNAMIC; i++) {
            auto &info = _var_info_dynamic[i];
            info.type = AP_PARAM_NONE;
            info.name = _empty_string;
            info.key = 0xFFFF;
            info.ptr = nullptr;
            info.group_info = nullptr;
            info.flags = 0;
        }
        // make tables available
        _num_vars += AP_PARAM_MAX_DYNAMIC;
    }

    // find existing key (allows for script reload)
    uint8_t i;
    for (i=0; i<AP_PARAM_MAX_DYNAMIC; i++) {
        auto &info = _var_info_dynamic[i];
        if (info.type != AP_PARAM_NONE && info.key == key) {
            if (_dynamic_table_sizes[i] != 0 &&
                num_params > _dynamic_table_sizes[i]) {
                // can't expand the table at runtime
                return false;
            }
            if (strcmp(prefix, info.name) != 0) {
                // prefix has changed, reject as two scripts running
                // with the same key
                return false;
            }
            break;
        }
    }

    if (i == AP_PARAM_MAX_DYNAMIC) {
        // find an unused slot
        for (i=0; i<AP_PARAM_MAX_DYNAMIC; i++) {
            auto &info = _var_info_dynamic[i];
            if (info.type == AP_PARAM_NONE ) {
                break;
            }
        }
    }

    if (i == AP_PARAM_MAX_DYNAMIC) {
        // no empty slots
        return false;
    }

    auto &info = _var_info_dynamic[i];

    // create memory for the array of floats if needed
    // first float is used for the crc
    if (info.ptr == nullptr) {
        info.ptr = calloc(num_params+1, sizeof(float));
        if (info.ptr == nullptr) {
            return false;
        }
    }

    // allocate the name
    if (info.name == _empty_string) {
        info.name = strdup(prefix);
        if (info.name == nullptr) {
            free(const_cast<void*>(info.ptr));
            info.ptr = nullptr;
            info.name = _empty_string;
            return false;
        }
    }

    // if it doesn't exist then create the table
    if (info.group_info == nullptr) {
        info.group_info = (GroupInfo *)calloc(num_params+2, sizeof(GroupInfo));
        if (info.group_info == nullptr) {
            free(const_cast<void*>(info.ptr));
            free(const_cast<char*>(info.name));
            info.ptr = nullptr;
            info.name = _empty_string;
            return false;
        }
    }
    // fill in footer for all entries
    for (uint8_t gi=1; gi<num_params+2; gi++) {
        auto &ginfo = const_cast<GroupInfo*>(info.group_info)[gi];
        ginfo.name = _empty_string;
        ginfo.idx = 0xff;
        ginfo.flags = AP_PARAM_FLAG_HIDDEN;
    }
    // hidden first parameter containing AP_Int32 crc
    auto &hinfo = const_cast<GroupInfo*>(info.group_info)[0];
    hinfo.flags = AP_PARAM_FLAG_HIDDEN;
    hinfo.name = _empty_string;
    hinfo.idx = 0;
    hinfo.offset = 0;
    hinfo.type = AP_PARAM_INT32;
    // fill in default value with the CRC. Relies on sizeof crc == sizeof float
    memcpy((uint8_t *)&hinfo.def_value, (const uint8_t *)&crc, sizeof(crc));

    // remember the table size
    if (_dynamic_table_sizes[i] == 0) {
        _dynamic_table_sizes[i] = num_params;
    }
    
    // make the group active
    info.key = key;
    info.type = AP_PARAM_GROUP;

    invalidate_count();

    // save the CRC
    AP_Int32 *crc_param = const_cast<AP_Int32 *>((AP_Int32 *)info.ptr);
    crc_param->set(crc);
    crc_param->save(true);

    return true;
}

/*
 Load an AP_Int32 variable from EEPROM using top level key and group element. Used to confirm
 a key in add_table()
*/
bool AP_Param::load_int32(uint16_t key, uint32_t group_element, int32_t &value)
{
    struct Param_header phdr;

    phdr.type = AP_PARAM_INT32;
    set_key(phdr, key);
    phdr.group_element = group_element;

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (!scan(&phdr, &ofs)) {
        return false;
    }

    // found it
    _storage.read_block(&value, ofs+sizeof(phdr), type_size(AP_PARAM_INT32));
    return true;
}

/*
  add a parameter to a dynamic table
 */
bool AP_Param::add_param(uint8_t _key, uint8_t param_num, const char *pname, float default_value)
{
    if (_var_info_dynamic == nullptr) {
        // No dynamic tables available
        return false;
    }

    // check for valid values
    if (param_num == 0 || param_num > 63 || strlen(pname) > AP_MAX_NAME_SIZE) {
        return false;
    }

    uint16_t key = uint16_t(_key) + AP_PARAM_DYNAMIC_KEY_BASE;
    // find the info
    uint8_t i;
    for (i=0; i<AP_PARAM_MAX_DYNAMIC; i++) {
        auto &info = _var_info_dynamic[i];
        if (info.key == key) {
            break;
        }
    }
    if (i == AP_PARAM_MAX_DYNAMIC) {
        // not found
        return false;
    }

    if (param_num > _dynamic_table_sizes[i]) {
        return false;
    }

    auto &info = _var_info_dynamic[i];
    if (info.ptr == nullptr) {
        return false;
    }

    // check CRC
    const auto &hinfo = const_cast<GroupInfo*>(info.group_info)[0];
    const int32_t crc = float_to_int32_le(hinfo.def_value);

    int32_t current_crc;
    if (load_int32(key, 0, current_crc) && current_crc != crc) {
        // crc mismatch, we have a conflict with an existing use of this key
        return false;
    }

    // Check length
    char fullName[AP_MAX_NAME_SIZE+1] = {};
    const int fullNameLength = hal.util->snprintf(fullName, sizeof(fullName), "%s%s", info.name, pname);
    if ((fullNameLength < 0) || (fullNameLength > AP_MAX_NAME_SIZE)) {
        // Param is too long with table prefix (or snprintf failed)
        return false;
    }

    // Get param object
    AP_Float *pvalues = const_cast<AP_Float *>((const AP_Float *)info.ptr);
    AP_Float &p = pvalues[param_num];

    // Check for conflicting name
    enum ap_var_type existingType;
    AP_Param* existingParam = find(fullName, &existingType);
    if ((existingParam != nullptr) && ((existingType != AP_PARAM_FLOAT) || ((AP_Float*)existingParam != &p))) {
        // There is a existing parameter with this name (which is not this parameter from a previous script run)
        return false;
    }

    // fill in idx of any gaps, leaving them hidden, this allows
    // scripts to remove parameters
    for (uint8_t j=1; j<param_num; j++) {
        auto &g = const_cast<GroupInfo*>(info.group_info)[j];
        if (g.idx == 0xff) {
            g.idx = j;
            g.flags = AP_PARAM_FLAG_HIDDEN;
            g.offset = j*sizeof(float);
            g.type = AP_PARAM_FLOAT;
        }
    }

    auto &ginfo = const_cast<GroupInfo*>(info.group_info)[param_num];

    if (ginfo.name == _empty_string) {
        // we don't allow name change while running
        ginfo.name = strdup(pname);
        if (ginfo.name == nullptr) {
            ginfo.name = _empty_string;
            return false;
        }
    }
    ginfo.offset = param_num*sizeof(float);
    ginfo.idx = param_num;
    float *def_value = const_cast<float *>(&ginfo.def_value);
    *def_value = default_value;
    ginfo.type = AP_PARAM_FLOAT;

    // load from storage if available, the param is hidden during this
    // load so the param is not visible to MAVLink until after it is
    // loaded
    p.set_default(default_value);
    p.load();

    // clear the hidden flag if set and invalidate the count
    // so we recount the parameters
    ginfo.flags = 0;
    invalidate_count();
    
    return true;
}
#endif
