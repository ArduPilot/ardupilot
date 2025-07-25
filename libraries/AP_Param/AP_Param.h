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
/// @file	AP_Param.h
/// @brief	A system for managing and storing variables that are of
///			general interest to the system.
#pragma once

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <StorageManager/StorageManager.h>
#include <AP_Scripting/AP_Scripting_config.h>

#include "AP_Param_config.h"

#include "float.h"

#define AP_MAX_NAME_SIZE 16

// optionally enable debug code for dumping keys
#ifndef AP_PARAM_KEY_DUMP
#define AP_PARAM_KEY_DUMP 0
#endif

#if defined(HAL_GCS_ENABLED)
    #define AP_PARAM_DEFAULTS_ENABLED HAL_GCS_ENABLED
#else
    #define AP_PARAM_DEFAULTS_ENABLED 1
#endif

/*
  maximum size of embedded parameter file
 */
#ifndef AP_PARAM_MAX_EMBEDDED_PARAM
  #if FORCE_APJ_DEFAULT_PARAMETERS
    #if HAL_PROGRAM_SIZE_LIMIT_KB <= 1024
      #define AP_PARAM_MAX_EMBEDDED_PARAM 1024
    #else
      #define AP_PARAM_MAX_EMBEDDED_PARAM 8192
    #endif
  #else
    #define AP_PARAM_MAX_EMBEDDED_PARAM 0
  #endif
#endif

// allow for dynamically added tables when scripting enabled
#ifndef AP_PARAM_DYNAMIC_ENABLED
#define AP_PARAM_DYNAMIC_ENABLED AP_SCRIPTING_ENABLED
#endif

// maximum number of dynamically created tables (from scripts)
#ifndef AP_PARAM_MAX_DYNAMIC
#define AP_PARAM_MAX_DYNAMIC 10
#endif
#define AP_PARAM_DYNAMIC_KEY_BASE 300

/*
  flags for variables in var_info and group tables
 */

// a nested offset is for subgroups that are not subclasses
#define AP_PARAM_FLAG_NESTED_OFFSET (1<<0)

// a pointer variable is for dynamically allocated objects
#define AP_PARAM_FLAG_POINTER       (1<<1)

// an enable variable allows a whole subtree of variables to be made
// invisible
#define AP_PARAM_FLAG_ENABLE        (1<<2)

// don't shift index 0 to index 63. Use this when you know there will be
// no conflict with the parent
#define AP_PARAM_FLAG_NO_SHIFT      (1<<3)

// the var_info is a pointer, allowing for dynamic definition of the var_info tree
#define AP_PARAM_FLAG_INFO_POINTER  (1<<4)

// this parameter is visible to GCS via mavlink but should never be
// set by anything other than the ArduPilot code responsible for its
// use.
#define AP_PARAM_FLAG_INTERNAL_USE_ONLY (1<<5)

// hide parameter from param download
#define AP_PARAM_FLAG_HIDDEN (1<<6)

// Default value is a "pointer" actually its a offest from the base value, but the idea is the same
#define AP_PARAM_FLAG_DEFAULT_POINTER (1<<7)

// keep all flags before the FRAME tags

// vehicle and frame type flags, used to hide parameters when not
// relevent to a vehicle type. Use AP_Param::set_frame_type_flags() to
// enable parameters flagged in this way. frame type flags are stored
// in flags field, shifted by AP_PARAM_FRAME_TYPE_SHIFT.
#define AP_PARAM_FRAME_TYPE_SHIFT   8

// supported frame types for parameters
#define AP_PARAM_FRAME_COPTER       (1<<0)
#define AP_PARAM_FRAME_ROVER        (1<<1)
#define AP_PARAM_FRAME_PLANE        (1<<2)
#define AP_PARAM_FRAME_SUB          (1<<3)
#define AP_PARAM_FRAME_TRICOPTER    (1<<4)
#define AP_PARAM_FRAME_HELI         (1<<5)
#define AP_PARAM_FRAME_BLIMP        (1<<6)

// a variant of offsetof() to work around C++ restrictions.
// this can only be used when the offset of a variable in a object
// is constant and known at compile time
#define AP_VAROFFSET(type, element) (((ptrdiff_t)(&((const type *)1)->element))-1)

// find the type of a variable given the class and element
#define AP_CLASSTYPE(clazz, element) ((uint8_t)(((const clazz *) 1)->element.vtype))

// declare a group var_info line
#define AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags) { name, AP_VAROFFSET(clazz, element), {def_value : def}, flags, idx, AP_CLASSTYPE(clazz, element)}

// declare a group var_info line with a frame type mask
#define AP_GROUPINFO_FRAME(name, idx, clazz, element, def, frame_flags) AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, (frame_flags)<<AP_PARAM_FRAME_TYPE_SHIFT )

// declare a group var_info line with both flags and frame type mask
#define AP_GROUPINFO_FLAGS_FRAME(name, idx, clazz, element, def, flags, frame_flags) AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags|((frame_flags)<<AP_PARAM_FRAME_TYPE_SHIFT) )

// declare a group var_info line with a default "pointer"
#define AP_GROUPINFO_FLAGS_DEFAULT_POINTER(name, idx, clazz, element, def) {  name, AP_VAROFFSET(clazz, element), {def_value_offset : AP_VAROFFSET(clazz, element) - AP_VAROFFSET(clazz, def)}, AP_PARAM_FLAG_DEFAULT_POINTER, idx, AP_CLASSTYPE(clazz, element) }

// declare a group var_info line
#define AP_GROUPINFO(name, idx, clazz, element, def) AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, 0)

// declare a nested group entry in a group var_info
#define AP_NESTEDGROUPINFO(clazz, idx) { "", 0, { group_info : clazz::var_info }, 0, idx, AP_PARAM_GROUP }

// declare a subgroup entry in a group var_info. This is for having another arbitrary object as a member of the parameter list of
// an object
#define AP_SUBGROUPINFO(element, name, idx, thisclazz, elclazz) { name, AP_VAROFFSET(thisclazz, element), { group_info : elclazz::var_info }, AP_PARAM_FLAG_NESTED_OFFSET, idx, AP_PARAM_GROUP }

// declare a second parameter table for the same object
#define AP_SUBGROUPEXTENSION(name, idx, clazz, vinfo) { name, 0, { group_info : clazz::vinfo }, AP_PARAM_FLAG_NESTED_OFFSET, idx, AP_PARAM_GROUP }

// declare a pointer subgroup entry in a group var_info
#define AP_SUBGROUPPTR(element, name, idx, thisclazz, elclazz) { name, AP_VAROFFSET(thisclazz, element), { group_info : elclazz::var_info }, AP_PARAM_FLAG_POINTER, idx, AP_PARAM_GROUP }

// declare a pointer subgroup entry in a group var_info with a pointer var_info
#define AP_SUBGROUPVARPTR(element, name, idx, thisclazz, var_info) { name, AP_VAROFFSET(thisclazz, element), { group_info_ptr : &var_info }, AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER, idx, AP_PARAM_GROUP }

#define AP_GROUPEND     { "", 0,       { group_info : nullptr }, 0, 0xFF, AP_PARAM_NONE }

// Vehicle defines for info struct
#define GSCALAR(v, name, def)                { name, &AP_PARAM_VEHICLE_NAME.g.v,                   {def_value : def},                   0,                                                  Parameters::k_param_ ## v,          AP_PARAM_VEHICLE_NAME.g.v.vtype }
#define GARRAY(v, index, name, def)          { name, &AP_PARAM_VEHICLE_NAME.g.v[index],            {def_value : def},                   0,                                                  Parameters::k_param_ ## v ## index, AP_PARAM_VEHICLE_NAME.g.v[index].vtype }
#define ASCALAR(v, name, def)                { name, (const void *)&AP_PARAM_VEHICLE_NAME.aparm.v, {def_value : def},                   0,                                                  Parameters::k_param_ ## v,          AP_PARAM_VEHICLE_NAME.aparm.v.vtype }
#define GGROUP(v, name, class)               { name, &AP_PARAM_VEHICLE_NAME.g.v,                   {group_info : class::var_info},      0,                                                  Parameters::k_param_ ## v,          AP_PARAM_GROUP }
#define GOBJECT(v, name, class)              { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info : class::var_info},      0,                                                  Parameters::k_param_ ## v,          AP_PARAM_GROUP }
#define GOBJECTPTR(v, name, class)           { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info : class::var_info},      AP_PARAM_FLAG_POINTER,                              Parameters::k_param_ ## v,          AP_PARAM_GROUP }
#define GOBJECTVARPTR(v, name, var_info_ptr) { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info_ptr : var_info_ptr},     AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER, Parameters::k_param_ ## v,          AP_PARAM_GROUP }
#define GOBJECTN(v, pname, name, class)      { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info : class::var_info},      0,                                                  Parameters::k_param_ ## pname,      AP_PARAM_GROUP }
#define PARAM_VEHICLE_INFO                   { "",   (const void *)&AP_PARAM_VEHICLE_NAME,         {group_info : AP_Vehicle::var_info}, 0,                                                  Parameters::k_param_vehicle,        AP_PARAM_GROUP }
#define AP_VAREND                            { "",   nullptr,                                      {group_info : nullptr },             0,                                                  0,                                  AP_PARAM_NONE }
#define AP_GROUP_ELEM_IDX(subgrp_idx, grp_idx) (grp_idx << 6 | subgrp_idx)

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_GROUP
};


/// Base class for variables.
///
/// Provides naming and lookup services for variables.
///
class AP_Param
{
public:
    // the Info and GroupInfo structures are passed by the main
    // program in setup() to give information on how variables are
    // named and their location in memory
    struct GroupInfo {
        const char *name;
        ptrdiff_t offset; // offset within the object
        union {
            const struct GroupInfo *group_info;
            const struct GroupInfo **group_info_ptr; // when AP_PARAM_FLAG_INFO_POINTER is set in flags
            const float def_value;
            ptrdiff_t def_value_offset; // Default value offset from param object, when AP_PARAM_FLAG_DEFAULT_POINTER is set in flags
        };
        uint16_t flags;
        uint8_t idx;  // identifier within the group
        uint8_t type; // AP_PARAM_*
    };
    struct Info {
        const char *name;
        const void *ptr;    // pointer to the variable in memory
        union {
            const struct GroupInfo *group_info;
            const struct GroupInfo **group_info_ptr; // when AP_PARAM_FLAG_INFO_POINTER is set in flags
            const float def_value;
            ptrdiff_t def_value_offset; // Default value offset from param object, when AP_PARAM_FLAG_DEFAULT_POINTER is set in flags
        };
        uint16_t flags;
        uint16_t key; // k_param_*
        uint8_t type; // AP_PARAM_*
    };
    struct ConversionInfo {
        uint16_t old_key; // k_param_*
        uint32_t old_group_element; // index in old object
        enum ap_var_type type; // AP_PARAM_*
        const char *new_name;
    };

    // param default table element
    struct defaults_table_struct {
        const char *name;   // parameter name
        float value;        // parameter value
    };

    // called once at startup to setup the _var_info[] table. This
    // will also check the EEPROM header and re-initialise it if the
    // wrong version is found
    static bool setup();

    // constructor with var_info
    AP_Param(const struct Info *info)
    {
        _var_info = info;
        uint16_t i;
        for (i=0; info[i].type != AP_PARAM_NONE; i++) ;
        _num_vars = i;
#if AP_PARAM_DYNAMIC_ENABLED
        _num_vars_base = _num_vars;
#endif
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_Param must be singleton");
        }
        _singleton = this;
    }

    // empty constructor
    AP_Param() {}

    // a token used for first()/next() state
    typedef struct {
        uint32_t key : 9;
        uint32_t idx : 4; // offset into array types
        uint32_t group_element : 18;
        uint32_t last_disabled : 1;
    } ParamToken;


    // nesting structure for recursive call states
    struct GroupNesting {
        static const uint8_t numlevels = 2;
        uint8_t level;
        const struct GroupInfo *group_ret[numlevels];
    };

    // return true if AP_Param has been initialised via setup()
    static bool initialised(void);

    // the 'group_id' of a element of a group is the 18 bit identifier
    // used to distinguish between this element of the group and other
    // elements of the same group. It is calculated using a bit shift per
    // level of nesting, so the first level of nesting gets 6 bits the 2nd
    // level gets the next 6 bits, and the 3rd level gets the last 6
    // bits. This limits groups to having at most 64 elements.
    static uint32_t group_id(const struct GroupInfo *grpinfo, uint32_t base, uint8_t i, uint8_t shift);

    /// Copy the variable's name, prefixed by any containing group name, to a
    /// buffer.
    ///
    /// If the variable has no name, the buffer will contain an empty string.
    ///
    /// Note that if the combination of names is larger than the buffer, the
    /// result in the buffer will be truncated.
    ///
    /// @param	token			token giving current variable
    /// @param	buffer			The destination buffer
    /// @param	bufferSize		Total size of the destination buffer.
    ///
    void copy_name_info(const struct AP_Param::Info *info,
                        const struct GroupInfo *ginfo,
                        const struct GroupNesting &group_nesting,
                        uint8_t idx, char *buffer, size_t bufferSize, bool force_scalar=false) const;

    /// Copy the variable's name, prefixed by any containing group name, to a
    /// buffer.
    ///
    /// Uses token to look up AP_Param::Info for the variable
    void copy_name_token(const ParamToken &token, char *buffer, size_t bufferSize, bool force_scalar=false) const;

    /// Find a variable by name.
    ///
    /// If the variable has no name, it cannot be found by this interface.
    ///
    /// @param  name            The full name of the variable to be found.
    /// @param  flags           If non-null will be filled with parameter flags
    /// @return                 A pointer to the variable, or nullptr if
    ///                         it does not exist.
    ///
    static AP_Param * find(const char *name, enum ap_var_type *ptype, uint16_t *flags = nullptr);

    /// set a default value by name
    ///
    /// @param  name            The full name of the variable to be found.
    /// @param  value           The default value
    /// @return                 true if the variable is found
    static bool set_default_by_name(const char *name, float value);

    /// set parameter defaults from a defaults_table_struct
    ///
    /// @param table            pointer to array of defaults_table_struct structures
    /// @param count            number of elements in table array
    static void set_defaults_from_table(const struct defaults_table_struct *table, uint8_t count);

    /// set a value by name
    ///
    /// @param  name            The full name of the variable to be found.
    /// @param  value           The new value
    /// @return                 true if the variable is found
    static bool set_by_name(const char *name, float value);

    /// gat a value by name, used by scripting
    ///
    /// @param  name            The full name of the variable to be found.
    /// @param  value           A reference to the variable
    /// @return                 true if the variable is found
    static bool get(const char *name, float &value);

    /// set and save a value by name
    ///
    /// @param  name            The full name of the variable to be found.
    /// @param  value           The new value
    /// @return                 true if the variable is found
    static bool set_and_save_by_name(const char *name, float value);
    static bool set_and_save_by_name_ifchanged(const char *name, float value);

    /// Find a variable by index.
    ///
    ///
    /// @param  idx             The index of the variable
    /// @return                 A pointer to the variable, or nullptr if
    ///                         it does not exist.
    ///
    static AP_Param * find_by_index(uint16_t idx, enum ap_var_type *ptype, ParamToken *token);

    // by-name equivalent of find_by_index()
    static AP_Param* find_by_name(const char* name, enum ap_var_type *ptype, ParamToken *token);

    /// Find a variable by pointer
    ///
    ///
    /// @param  p               Pointer to variable
    /// @return                 key for variable
    static bool find_key_by_pointer_group(const void *ptr, uint16_t vindex, const struct GroupInfo *group_info,
                                          ptrdiff_t offset, uint16_t &key);
    static bool find_key_by_pointer(const void *ptr, uint16_t &key);

    /// Find key of top level group variable by pointer
    ///
    ///
    /// @param  p               Pointer to variable
    /// @return                 key for variable
    static bool find_top_level_key_by_pointer(const void *ptr, uint16_t &key);


    /// Find a object in the top level var_info table
    ///
    /// If the variable has no name, it cannot be found by this interface.
    ///
    /// @param  name            The full name of the variable to be found.
    ///
    static AP_Param * find_object(const char *name);

    /// Notify GCS of current parameter value
    ///
    void notify() const;

    /// Save the current value of the variable to storage, synchronous API
    ///
    /// @param  force_save     If true then force save even if default
    ///
    /// @return                True if the variable was saved successfully.
    ///
    void save_sync(bool force_save, bool send_to_gcs);

    /// flush all pending parameter saves
    /// used on reboot
    static void flush(void);

    /// Save the current value of the variable to storage, async interface
    ///
    /// @param  force_save     If true then force save even if default
    ///
    void save(bool force_save=false);

    /// Load the variable from EEPROM.
    ///
    /// @return                True if the variable was loaded successfully.
    ///
    bool load(void);

    /// Load all variables from EEPROM
    ///
    /// This function performs a best-efforts attempt to load all
    /// of the variables from EEPROM.  If some fail to load, their
    /// values will remain as they are.
    ///
    /// @return                False if any variable failed to load
    ///
    static bool load_all();

    // return true if eeprom is full, used for arming check
    static bool get_eeprom_full(void) {
        return eeprom_full;
    }

    // returns storage space used:
    static uint16_t storage_used() { return sentinal_offset; }

    // returns storage space :
    static uint16_t storage_size() { return _storage.size(); }

    /// reoad the hal.util defaults file. Called after pointer parameters have been allocated
    ///
    static void reload_defaults_file(bool last_pass);

    static void load_object_from_eeprom(const void *object_pointer, const struct GroupInfo *group_info);

    // set a AP_Param variable to a specified value
    static void         set_value(enum ap_var_type type, void *ptr, float def_value);

    /*
      set a parameter to a float
    */
    void set_float(float value, enum ap_var_type var_type);

    // load default values for scalars in a group
    static void         setup_object_defaults(const void *object_pointer, const struct GroupInfo *group_info);

    // set a value directly in an object.
    // return true if the name was found and set, else false.
    // This should only be used by example code, not by mainline vehicle code
    static bool set_object_value(const void *object_pointer,
                                 const struct GroupInfo *group_info,
                                 const char *name, float value);

    // load default values for all scalars in the main sketch. This
    // does not recurse into the sub-objects    
    static void         setup_sketch_defaults(void);

    // find an old parameter and return it.
    static bool find_old_parameter(const struct ConversionInfo *info, AP_Param *value);

    // convert old vehicle parameters to new object parameters
    static void         convert_old_parameters(const struct ConversionInfo *conversion_table, uint8_t table_size, uint8_t flags=0);
    // convert old vehicle parameters to new object parameters with scaling - assumes we use the same scaling factor for all values in the table
    static void         convert_old_parameters_scaled(const ConversionInfo *conversion_table, uint8_t table_size, float scaler, uint8_t flags);

    // convert an object which was stored in a vehicle's G2 into a new
    // object in AP_Vehicle.cpp:
    struct G2ObjectConversion {
        void *object_pointer;
        const struct AP_Param::GroupInfo *var_info;
        uint16_t old_index;  // Old parameter index in g2
    };
    static void         convert_g2_objects(const void *g2, const G2ObjectConversion g2_conversions[], uint8_t num_conversions);

    // convert an object which was stored in a vehicle's top-level
    // Parameters object into a new object in AP_Vehicle.cpp:
    struct TopLevelObjectConversion {
        void *object_pointer;
        const struct AP_Param::GroupInfo *var_info;
        uint16_t old_index;  // Old parameter index in g
    };
    static void         convert_toplevel_objects(const TopLevelObjectConversion g2_conversions[], uint8_t num_conversions);

    /*
      convert width of a parameter, allowing update to wider scalar
      values without changing the parameter indexes. This will return
      true if the parameter was converted from an old parameter value
    */
    bool convert_parameter_width(ap_var_type old_ptype, float scale_factor=1.0) {
        return _convert_parameter_width(old_ptype, scale_factor, false);
    }
    bool convert_centi_parameter(ap_var_type old_ptype) {
        return convert_parameter_width(old_ptype, 0.01f);
    }
    // Converting bitmasks should be done bitwise rather than numerically
    bool convert_bitmask_parameter_width(ap_var_type old_ptype) {
        return _convert_parameter_width(old_ptype, 1.0, true);
    }

    // convert a single parameter with scaling
    enum {
        CONVERT_FLAG_REVERSE=1, // handle _REV -> _REVERSED conversion
        CONVERT_FLAG_FORCE=2    // store new value even if configured in eeprom already
    };
    static void         convert_old_parameter(const struct ConversionInfo *info, float scaler, uint8_t flags=0);

    // move all parameters from a class to a new location
    // is_top_level: Is true if the class had its own top level key, param_key. It is false if the class was a subgroup
    static void         convert_class(uint16_t param_key, void *object_pointer,
                                        const struct AP_Param::GroupInfo *group_info,
                                        uint16_t old_index, bool is_top_level, bool recurse_sub_groups = false);

    /*
      fetch a parameter value based on the index within a group. This
      is used to find the old value of a parameter that has been
      removed from an object.
    */
    static bool get_param_by_index(void *obj_ptr, uint8_t idx, ap_var_type old_ptype, void *pvalue);
    
    /// Erase all variables in EEPROM.
    ///
    static void         erase_all(void);

    /// Returns the first variable
    ///
    /// @return             The first variable in _var_info, or nullptr if
    ///                     there are none.
    ///
    static AP_Param *      first(ParamToken *token, enum ap_var_type *ptype, float *default_val = nullptr);

    /// Returns the next variable in _var_info, recursing into groups
    /// as needed
    static AP_Param *      next(ParamToken *token, enum ap_var_type *ptype) { return  next(token, ptype, false); }
    static AP_Param *      next(ParamToken *token, enum ap_var_type *ptype, bool skip_disabled, float *default_val = nullptr);

    /// Returns the next scalar variable in _var_info, recursing into groups
    /// as needed
    static AP_Param *       next_scalar(ParamToken *token, enum ap_var_type *ptype, float *default_val = nullptr);

    /// get the size of a type in bytes
    static uint8_t				type_size(enum ap_var_type type);

    /// cast a variable to a float given its type
    float                   cast_to_float(enum ap_var_type type) const;

    // check var table for consistency
    static void             check_var_info(void);

    // return true if the parameter is configured
    bool configured(void) const;

    // return true if the parameter is read-only
    bool is_read_only(void) const;

    // return the persistent top level key for the ParamToken key
    static uint16_t get_persistent_key(uint16_t key) { return var_info(key).key; }

    // returns true if this parameter should be settable via the
    // MAVLink interface:
    bool allow_set_via_mavlink(uint16_t flags) const;

    // count of parameters in tree
    static uint16_t count_parameters(void);

    // invalidate parameter count
    static void invalidate_count(void);

    static void set_hide_disabled_groups(bool value) { _hide_disabled_groups = value; }

    // set frame type flags. Used to unhide frame specific parameters
    static void set_frame_type_flags(uint16_t flags_to_set) {
        invalidate_count();
        _frame_type_flags |= flags_to_set;
    }

    // check if a given frame type should be included
    static bool check_frame_type(uint16_t flags);

#if AP_PARAM_KEY_DUMP
    /// print the value of all variables
    static void         show_all(AP_HAL::BetterStream *port, bool showKeyValues=false);

    /// print the value of one variable
    static void         show(const AP_Param *param, 
                             const char *name,
                             enum ap_var_type ptype, 
                             AP_HAL::BetterStream *port);

    /// print the value of one variable
    static void         show(const AP_Param *param, 
                             const ParamToken &token,
                             enum ap_var_type ptype, 
                             AP_HAL::BetterStream *port);
#endif // AP_PARAM_KEY_DUMP

    static AP_Param *get_singleton() { return _singleton; }

#if AP_PARAM_DYNAMIC_ENABLED
    // allow for dynamically added parameter tables from scripts
    static bool add_table(uint8_t key, const char *prefix, uint8_t num_params);
    static bool add_param(uint8_t key, uint8_t param_num, const char *pname, float default_value);
    static bool load_int32(uint16_t key, uint32_t group_element, int32_t &value);
#endif

    static bool load_defaults_file(const char *filename, bool last_pass);

protected:

    // store default value in linked list
    static void add_default(AP_Param *ap, float v);

private:
    static AP_Param *_singleton;

    /// EEPROM header
    ///
    /// This structure is placed at the head of the EEPROM to indicate
    /// that the ROM is formatted for AP_Param.
    ///
    struct EEPROM_header {
        uint8_t magic[2];
        uint8_t revision;
        uint8_t spare;
    };
    static_assert(sizeof(struct EEPROM_header) == 4, "Bad EEPROM_header size!");

    static uint16_t sentinal_offset;

/* This header is prepended to a variable stored in EEPROM.
 *  The meaning is as follows:
 *
 *  - key: the k_param enum value from Parameter.h in the sketch
 *
 *  - group_element: This is zero for top level parameters. For
 *                   parameters stored within an object this is divided
 *                   into 3 lots of 6 bits, allowing for three levels
 *                   of object to be stored in the eeprom
 *
 *  - type: the ap_var_type value for the variable
 */
    struct Param_header {
        // to get 9 bits for key we needed to split it into two parts to keep binary compatibility
        uint32_t key_low : 8;
        uint32_t type : 5;
        uint32_t key_high : 1;
        uint32_t group_element : 18;
    };
    static_assert(sizeof(struct Param_header) == 4, "Bad Param_header size!");

    // number of bits in each level of nesting of groups
    static const uint8_t        _group_level_shift = 6;
    static const uint8_t        _group_bits  = 18;

    static const uint16_t       _sentinal_key   = 0x1FF;
    static const uint8_t        _sentinal_type  = 0x1F;
    static const uint8_t        _sentinal_group = 0xFF;

    static uint16_t             _frame_type_flags;

    /*
      this is true if when scanning a defaults file we find all of the parameters
     */
    static bool done_all_default_params;

    /*
      structure for built-in defaults file that can be modified using apj_tool.py
     */
#if AP_PARAM_MAX_EMBEDDED_PARAM > 0
    struct PACKED param_defaults_struct {
        char magic_str[8];
        uint8_t param_magic[8];
        uint16_t max_length;
        volatile uint16_t length;
        volatile char data[AP_PARAM_MAX_EMBEDDED_PARAM];
    };
    static const param_defaults_struct param_defaults_data;
#endif


    static void                 check_group_info(const struct GroupInfo *group_info, uint16_t *total_size, 
                                                 uint8_t max_bits, uint8_t prefix_length);
    static bool                 duplicate_key(uint16_t vindex, uint16_t key);

    static bool adjust_group_offset(uint16_t vindex, const struct GroupInfo &group_info, ptrdiff_t &new_offset);
    static bool get_base(const struct Info &info, ptrdiff_t &base);

    /// get group_info pointer based on flags
    static const struct GroupInfo *get_group_info(const struct GroupInfo &ginfo);

    /// get group_info pointer based on flags
    static const struct GroupInfo *get_group_info(const struct Info &ginfo);

    const struct Info *         find_var_info_group(
                                    const struct GroupInfo *    group_info,
                                    uint16_t                    vindex,
                                    uint32_t                    group_base,
                                    uint8_t                     group_shift,
                                    ptrdiff_t                   group_offset,
                                    uint32_t *                  group_element,
                                    const struct GroupInfo *   &group_ret,
                                    struct GroupNesting        &group_nesting,
                                    uint8_t *                   idx) const;
    const struct Info *         find_var_info(
                                    uint32_t *                group_element,
                                    const struct GroupInfo *  &group_ret,
                                    struct GroupNesting       &group_nesting,
                                    uint8_t *                 idx) const;
    const struct Info *			find_var_info_token(const ParamToken &token,
                                                    uint32_t *                 group_element,
                                                    const struct GroupInfo *  &group_ret,
                                                    struct GroupNesting       &group_nesting,
                                                    uint8_t *                  idx) const;
    static const struct Info *  find_by_header_group(
                                    struct Param_header phdr, void **ptr,
                                    uint16_t vindex,
                                    const struct GroupInfo *group_info,
                                    uint32_t group_base,
                                    uint8_t group_shift,
                                    ptrdiff_t group_offset);
    static const struct Info *  find_by_header(
                                    struct Param_header phdr,
                                    void **ptr);
    void                        add_vector3f_suffix(
                                    char *buffer,
                                    size_t buffer_size,
                                    uint8_t idx) const;
    static AP_Param *           find_group(
                                    const char *name,
                                    uint16_t vindex,
                                    ptrdiff_t group_offset,
                                    const struct GroupInfo *group_info,
                                    enum ap_var_type *ptype);
    static void                 write_sentinal(uint16_t ofs);
    static uint16_t             get_key(const Param_header &phdr);
    static void                 set_key(Param_header &phdr, uint16_t key);
    static bool                 is_sentinal(const Param_header &phrd);
    static bool                 scan(
                                    const struct Param_header *phdr,
                                    uint16_t *pofs);
    static void                 eeprom_write_check(
                                    const void *ptr,
                                    uint16_t ofs,
                                    uint8_t size);
    static AP_Param *           next_group(
                                    const uint16_t vindex,
                                    const struct GroupInfo *group_info,
                                    bool *found_current,
                                    const uint32_t group_base,
                                    const uint8_t group_shift,
                                    const ptrdiff_t group_offset,
                                    ParamToken *token,
                                    enum ap_var_type *ptype,
                                    bool skip_disabled,
                                    float *default_val);

    // find a default value given a pointer to a default value in flash
    static float get_default_value(const AP_Param *object_ptr, const struct GroupInfo &info);
    static float get_default_value(const AP_Param *object_ptr, const struct Info &info);

    static bool parse_param_line(char *line, char **vname, float &value, bool &read_only);

    /*
      load a parameter defaults file. This happens as part of load_all()
     */
    static bool count_defaults_in_file(const char *filename, uint16_t &num_defaults);
    static bool count_param_defaults(const volatile char *ptr, int32_t length, uint16_t &count);
    static bool read_param_defaults_file(const char *filename, bool last_pass, uint16_t &idx);

    // load a defaults.parm using AP_FileSystem:
    static void load_defaults_file_from_filesystem(const char *filename, bool lastpass);
    // load an @ROMFS defaults.parm using ROMFS API:
    static void load_defaults_file_from_romfs(const char *filename, bool lastpass);

    // load defaults from supplied string:
    static void load_param_defaults(const volatile char *ptr, int32_t length, bool last_pass);

    /*
      load defaults from embedded parameters
     */
    static bool count_embedded_param_defaults(uint16_t &count);
    static void load_embedded_param_defaults(bool last_pass);

    // return true if the parameter is configured in the defaults file
    bool configured_in_defaults_file(bool &read_only) const;

    // return true if the parameter is configured in EEPROM/FRAM
    bool configured_in_storage(void) const;

    /*
      convert width of a parameter, allowing update to wider scalar
      values without changing the parameter indexes. This will return
      true if the parameter was converted from an old parameter value
    */
    bool _convert_parameter_width(ap_var_type old_ptype, float scale_factor, bool bitmask);

    // send a parameter to all GCS instances
    void send_parameter(const char *name, enum ap_var_type param_header_type, uint8_t idx) const;

    static StorageAccess        _storage;
    static StorageAccess        _storage_bak;
    static uint16_t             _num_vars;
    static uint16_t             _parameter_count;
    static uint16_t             _count_marker;
    static uint16_t             _count_marker_done;
    static HAL_Semaphore        _count_sem;
    static const struct Info *  _var_info;

#if AP_PARAM_DYNAMIC_ENABLED
    // allow for a dynamically allocated var table
    static uint16_t             _num_vars_base;
    static struct Info *        _var_info_dynamic;
    static const struct AP_Param::Info &var_info(uint16_t i) {
        return i<_num_vars_base? _var_info[i] : _var_info_dynamic[i-_num_vars_base];
    }
    static uint8_t _dynamic_table_sizes[AP_PARAM_MAX_DYNAMIC];
#else
    // simple static var table in flash
    static const struct Info &var_info(uint16_t i) {
        return _var_info[i];
    }
#endif

    /*
      list of overridden values from load_defaults_file()
    */
    struct param_override {
        const AP_Param *object_ptr;
        float value;
        bool read_only; // param is marked @READONLY
    };
    static struct param_override *param_overrides;
    static uint16_t num_param_overrides;
    static uint16_t param_overrides_len;
    static uint16_t num_read_only;

    // values filled into the EEPROM header
    static const uint8_t        k_EEPROM_magic0      = 0x50;
    static const uint8_t        k_EEPROM_magic1      = 0x41; ///< "AP"
    static const uint8_t        k_EEPROM_revision    = 6; ///< current format revision

    static bool _hide_disabled_groups;

    // support for background saving of parameters. We pack it to reduce memory for the
    // queue
    struct PACKED param_save {
        AP_Param *param;
        bool force_save;
    };
    static ObjectBuffer_TS<struct param_save> save_queue;
    static bool registered_save_handler;

    // background function for saving parameters
    void save_io_handler(void);

    // Store default values from add_default() calls in linked list
    struct defaults_list {
        AP_Param *ap;
        float val;
        defaults_list *next;
    };
    static defaults_list *default_list;
    static void check_default(AP_Param *ap, float *default_value);

    static bool eeprom_full;
};

namespace AP {
    AP_Param *param();
};

/// Template class for scalar variables.
///
/// Objects of this type have a value, though the infrastructure to actually
/// treat them as a value is delegated to a type-specialized subclass.
///
/// @tparam T			The scalar type of the variable
/// @tparam PT			The AP_PARAM_* type
///
template<typename T, ap_var_type PT>
class AP_ParamTBase : public AP_Param
{
public:
    static const ap_var_type        vtype = PT;

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    // set a parameter that is an ENABLE param
    void set_enable(const T &v);
    
    /// Sets if the parameter is unconfigured
    ///
    void set_default(const T &v);

    /// Sets parameter and default
    ///
    void set_and_default(const T &v);

    /// Value setter - set value, tell GCS
    ///
    void set_and_notify(const T &v);

    /// Combined set and save
    ///
    void set_and_save(const T &v);

    /// Combined set and save, but only does the save if the value if
    /// different from the current ram value, thus saving us a
    /// scan(). This should only be used where we have not set() the
    /// value separately, as otherwise the value in EEPROM won't be
    /// updated correctly.
    void set_and_save_ifchanged(const T &v);

    /// AP_ParamTBase types can implement AP_Param::cast_to_float
    ///
    float cast_to_float(void) const;

protected:
    T _value;
};

template<typename T, ap_var_type PT>
class AP_ParamT : public AP_ParamTBase<T, PT> // for int and smaller types
{
public:
    /// Conversion to T returns a reference to the value. A reference is
    /// necessary as some users expect to pass a reference around.
    ///
    /// This allows the class to be used in many situations where the value
    /// would be legal.
    ///
    /// Note that this can cause strange conversions: the value can be silently
    /// converted to a smaller type, causing unexpected truncation in an
    /// expression like `int16_t v = true ? int16_param : (int8_t)0`.
    ///
    /// C numeric conversion rules can be reinstated where needed by simply
    /// calling `.get()` on the value.
    ///
    operator const T &() const {
        return this->_value;
    }
};

template<>
class AP_ParamT<float, AP_PARAM_FLOAT> : public AP_ParamTBase<float, AP_PARAM_FLOAT>
{
public:
    /// Conversion to float returns a reference to the value. A reference is
    /// necessary as some users expect to pass a reference around.
    ///
    /// This allows the class to be used in many situations where the value
    /// would be legal.
    ///
    /// We must return a float and specifically make this function a template to
    /// forbid further conversions: paraphrasing [over.ics.user] clause 3,
    /// templated user-defined conversion functions require that a further
    /// conversion be an exact match. This prevents the value from silently
    /// converting to an int and causing unexpected truncation in an expression
    /// like `float v = true ? float_param : 0`.
    ///
    /// This does also prevent implicit conversion to double, but that is a
    /// relatively small price to pay. C numeric conversion rules can be
    /// reinstated where needed by simply calling `.get()` on the value, or by
    /// manually casting to `double` or `int`.
    ///
    template<bool X = true>
    operator const float &() const {
        return this->_value;
    }

    explicit operator int () const { // convenience function for int casts
        return (int)this->_value;
    }

    explicit operator double () const { // convenience function for double casts
        return (double)this->_value;
    }

#if defined(__clang__)
    // inexplicably, clang will not use the built-in operator implementations
    // for floats on two AP_ParamT<float>s, so provide them for it.

    float operator -() const { return -this->_value; } // unary minus

#define PARAM_SELF_OPER(R, OP) \
    R operator OP (const AP_ParamT<float, AP_PARAM_FLOAT>& other) const { return this->_value OP other._value; }

    PARAM_SELF_OPER(float, +);
    PARAM_SELF_OPER(float, -);
    PARAM_SELF_OPER(float, *);
    PARAM_SELF_OPER(float, /);
    PARAM_SELF_OPER(bool, >);
    PARAM_SELF_OPER(bool, <);
    PARAM_SELF_OPER(bool, <=);
    PARAM_SELF_OPER(bool, >=);
    // != and == are unsafe on floats

#undef PARAM_SELF_OPER

#endif
};

/// Template class for non-scalar variables, intended for non-C types.
///
/// Objects of this type have an object value, and can be treated in many ways
/// as though they were the value.
///
/// @tparam T			The scalar type of the variable
/// @tparam PT			AP_PARAM_* type
///
template<typename T, ap_var_type PT>
class AP_ParamV : public AP_Param
{
public:

    static const ap_var_type        vtype = PT;

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    /// Value setter - set value, tell GCS
    ///
    void set_and_notify(const T &v);

    /// Combined set and save
    ///
    void set_and_save(const T &v);

    /// Combined set and save, but only does the save if the value is
    /// different from the current ram value, thus saving us a
    /// scan(). This should only be used where we have not set() the
    /// value separately, as otherwise the value in EEPROM won't be
    /// updated correctly.
    void set_and_save_ifchanged(const T &v);


    /// Conversion to T returns a reference to the value. A reference is
    /// necessary as some users expect to pass a reference around.
    ///
    /// This allows the class to be used in many situations where the value
    /// would be legal. As T is a user-defined class and not a C type, we don't
    /// have to worry about weird numeric conversions.
    ///
    operator const T &() const {
        return _value;
    }

protected:
    T        _value;
};


/// Convenience macro for defining instances of the AP_ParamT template.
///
// declare a scalar type
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _pt is the enum ap_var_type type
#define AP_PARAMDEF(_t, _suffix, _pt)   typedef AP_ParamT<_t, _pt> AP_ ## _suffix;
AP_PARAMDEF(float, Float, AP_PARAM_FLOAT);    // defines AP_Float, requires specialization!
AP_PARAMDEF(int8_t, Int8, AP_PARAM_INT8);     // defines AP_Int8
AP_PARAMDEF(int16_t, Int16, AP_PARAM_INT16);  // defines AP_Int16
AP_PARAMDEF(int32_t, Int32, AP_PARAM_INT32);  // defines AP_Int32

// declare a non-scalar type
// this is used in AP_Math.h
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _pt is the enum ap_var_type type
#define AP_PARAMDEFV(_t, _suffix, _pt)   typedef AP_ParamV<_t, _pt> AP_ ## _suffix;

// see comment in the AP_ParamT float specialization
static_assert(not std::is_convertible<AP_Float, int>::value, "illegal conversion possible");

/*
  template class for enum types based on AP_Int8
 */
template<typename eclass>
class AP_Enum : public AP_Int8
{
public:
    operator const eclass () const {
        return (eclass)_value;
    }
    void set(eclass v) {
        AP_Int8::set(int8_t(v));
    }
    void set_and_save(eclass v) {
        AP_Int8::set_and_save(int8_t(v));
    }
};

template<typename eclass>
class AP_Enum16 : public AP_Int16
{
public:
    operator const eclass () const {
        return (eclass)_value;
    }
    void set(eclass v) {
        AP_Int16::set(int16_t(v));
    }
    void set_and_save(eclass v) {
        AP_Int16::set_and_save(int16_t(v));
    }
};
