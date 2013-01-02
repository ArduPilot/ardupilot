// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	AP_Param.h
/// @brief	A system for managing and storing variables that are of
///			general interest to the system.

#ifndef AP_PARAM_H
#define AP_PARAM_H
#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include <AP_Progmem.h>

#define AP_MAX_NAME_SIZE 16
#define AP_NESTED_GROUPS_ENABLED

// a variant of offsetof() to work around C++ restrictions.
// this can only be used when the offset of a variable in a object
// is constant and known at compile time
#define AP_VAROFFSET(type, element) (((uintptr_t)(&((const type *)1)->element))-1)

// find the type of a variable given the class and element
#define AP_CLASSTYPE(class, element) (((const class *) 1)->element.vtype)

// declare a group var_info line
#define AP_GROUPINFO(name, idx, class, element, def) { AP_CLASSTYPE(class, element), idx, name, AP_VAROFFSET(class, element), {def_value : def} }

// declare a nested group entry in a group var_info
#ifdef AP_NESTED_GROUPS_ENABLED
 #define AP_NESTEDGROUPINFO(class, idx) { AP_PARAM_GROUP, idx, "", 0, { group_info : class::var_info } }
#endif

#define AP_GROUPEND     { AP_PARAM_NONE, 0xFF, "", 0, { group_info : NULL } }
#define AP_VAREND       { AP_PARAM_NONE, "", 0, NULL, { group_info : NULL } }

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_VECTOR6F,
    AP_PARAM_MATRIX3F,
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
        uint8_t type; // AP_PARAM_*
        uint8_t idx;  // identifier within the group
        const char name[AP_MAX_NAME_SIZE+1];
        uintptr_t offset; // offset within the object
        union {
            const struct GroupInfo *group_info;
            const float def_value;
        };
    };
    struct Info {
        uint8_t type; // AP_PARAM_*
        const char name[AP_MAX_NAME_SIZE+1];
        uint8_t key; // k_param_*
        void *ptr;    // pointer to the variable in memory
        union {
            const struct GroupInfo *group_info;
            const float def_value;
        };
    };

    // called once at startup to setup the _var_info[] table. This
    // will also check the EEPROM header and re-initialise it if the
    // wrong version is found
    static bool setup();

    // constructor with var_info
    AP_Param(const struct Info *info, uint16_t eeprom_size) {
        _eeprom_size = eeprom_size;
        _var_info = info;

        uint16_t i;
        for (i=0; pgm_read_byte(&info[i].type) != AP_PARAM_NONE; i++) ;
        _num_vars = i;
        
        check_var_info();
    }

    // empty constructor
    AP_Param() {}

    // a token used for first()/next() state
    typedef struct {
        uint32_t key : 8;
        uint32_t idx : 6; // offset into array types
        uint32_t group_element : 18;
    } ParamToken;

    // return true if AP_Param has been initialised via setup()
    static bool initialised(void);

    /// Copy the variable's name, prefixed by any containing group name, to a
    /// buffer.
    ///
    /// If the variable has no name, the buffer will contain an empty string.
    ///
    /// Note that if the combination of names is larger than the buffer, the
    /// result in the buffer will be truncated.
    ///
    /// @param	buffer			The destination buffer
    /// @param	bufferSize		Total size of the destination buffer.
    ///
    void copy_name(char *buffer, size_t bufferSize, bool force_scalar=false);

    /// Find a variable by name.
    ///
    /// If the variable has no name, it cannot be found by this interface.
    ///
    /// @param  name            The full name of the variable to be found.
    /// @return                 A pointer to the variable, or NULL if
    ///                         it does not exist.
    ///
    static AP_Param * find(const char *name, enum ap_var_type *ptype);

    /// Find a variable by index.
    ///
    ///
    /// @param  idx             The index of the variable
    /// @return                 A pointer to the variable, or NULL if
    ///                         it does not exist.
    ///
    static AP_Param * find_by_index(uint16_t idx, enum ap_var_type *ptype);

    /// Find a object in the top level var_info table
    ///
    /// If the variable has no name, it cannot be found by this interface.
    ///
    /// @param  name            The full name of the variable to be found.
    ///
    static AP_Param * find_object(const char *name);

    /// Save the current value of the variable to EEPROM.
    ///
    /// @return                True if the variable was saved successfully.
    ///
    bool save(void);

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
    static bool load_all(void);

    // set a AP_Param variable to a specified value
    static void         set_value(enum ap_var_type type, void *ptr, float def_value);

    // load default values for scalars in a group
    static void         setup_object_defaults(const void *object_pointer, const struct GroupInfo *group_info);

    // load default values for all scalars in the main sketch. This
    // does not recurse into the sub-objects    
    static void         setup_sketch_defaults(void);

    /// Erase all variables in EEPROM.
    ///
    static void         erase_all(void);

    /// print the value of all variables
    static void         show_all(void);

    /// Returns the first variable
    ///
    /// @return             The first variable in _var_info, or NULL if
    ///                     there are none.
    ///
    static AP_Param *      first(ParamToken *token, enum ap_var_type *ptype);

    /// Returns the next variable in _var_info, recursing into groups
    /// as needed
    static AP_Param *      next(ParamToken *token, enum ap_var_type *ptype);

    /// Returns the next scalar variable in _var_info, recursing into groups
    /// as needed
    static AP_Param *       next_scalar(ParamToken *token, enum ap_var_type *ptype);

    /// cast a variable to a float given its type
    float                   cast_to_float(enum ap_var_type type);

private:
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
        uint32_t key : 8;
        uint32_t type : 6;
        uint32_t group_element : 18;
    };

    // number of bits in each level of nesting of groups
    static const uint8_t        _group_level_shift = 6;
    static const uint8_t        _group_bits  = 18;

    static const uint8_t        _sentinal_key   = 0xFF;
    static const uint8_t        _sentinal_type  = 0x3F;
    static const uint8_t        _sentinal_group = 0xFF;

    static bool                 check_group_info(const struct GroupInfo *group_info, uint16_t *total_size, uint8_t max_bits);
    static bool                 duplicate_key(uint8_t vindex, uint8_t key);
    static bool                 check_var_info(void);
    const struct Info *         find_var_info_group(
                                    const struct GroupInfo *    group_info,
                                    uint8_t                     vindex,
                                    uint8_t                     group_base,
                                    uint8_t                     group_shift,
                                    uint32_t *                  group_element,
                                    const struct GroupInfo **   group_ret,
                                    uint8_t *                   idx);
    const struct Info *         find_var_info(
                                    uint32_t *                group_element,
                                    const struct GroupInfo ** group_ret,
                                    uint8_t *                 idx);
    static const struct Info *  find_by_header_group(
                                    struct Param_header phdr, void **ptr,
                                    uint8_t vindex,
                                    const struct GroupInfo *group_info,
                                    uint8_t group_base,
                                    uint8_t group_shift);
    static const struct Info *  find_by_header(
                                    struct Param_header phdr,
                                    void **ptr);
    void                        add_vector3f_suffix(
                                    char *buffer,
                                    size_t buffer_size,
                                    uint8_t idx);
    static AP_Param *           find_group(
                                    const char *name,
                                    uint8_t vindex,
                                    const struct GroupInfo *group_info,
                                    enum ap_var_type *ptype);
    static void                 write_sentinal(uint16_t ofs);
    bool                        scan(
                                    const struct Param_header *phdr,
                                    uint16_t *pofs);
    static uint8_t				type_size(enum ap_var_type type);
    static void                 eeprom_write_check(
                                    const void *ptr,
                                    uint16_t ofs,
                                    uint8_t size);
    static AP_Param *           next_group(
                                    uint8_t vindex, 
                                    const struct GroupInfo *group_info,
                                    bool *found_current,
                                    uint8_t group_base,
                                    uint8_t group_shift,
                                    ParamToken *token,
                                    enum ap_var_type *ptype);

    static uint16_t             _eeprom_size;
    static uint8_t              _num_vars;
    static const struct Info *  _var_info;

    // values filled into the EEPROM header
    static const uint8_t        k_EEPROM_magic0      = 0x50;
    static const uint8_t        k_EEPROM_magic1      = 0x41; ///< "AP"
    static const uint8_t        k_EEPROM_revision    = 6; ///< current format revision
};

/// Template class for scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
/// @tparam PT			The AP_PARAM_* type
///
template<typename T, ap_var_type PT>
class AP_ParamT : public AP_Param
{
public:
    static const ap_var_type        vtype = PT;

    /// Value getter
    ///
    T get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(T v) {
        _value = v;
    }

    /// Combined set and save
    ///
    bool set_and_save(T v) {
        set(v);
        return save();
    }

    /// Combined set and save, but only does the save if the value if
    /// different from the current ram value, thus saving us a
    /// scan(). This should only be used where we have not set() the
    /// value separately, as otherwise the value in EEPROM won't be
    /// updated correctly.
    bool set_and_save_ifchanged(T v) {
        if (v == _value) {
            return true;
        }
        set(v);
        return save();
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator T &() {
        return _value;
    }

    /// Copy assignment from self does nothing.
    ///
    AP_ParamT<T,PT>& operator= (AP_ParamT<T,PT>& v) {
        return v;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_ParamT<T,PT>& operator= (T v) {
        _value = v;
        return *this;
    }

    /// AP_ParamT types can implement AP_Param::cast_to_float
    ///
    float cast_to_float(void) {
        return (float)_value;
    }

protected:
    T _value;
};


/// Template class for non-scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
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
    T        get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void        set(T v) {
        _value = v;
    }

    /// Combined set and save
    ///
    bool        set_and_save(T v) {
        set(v);
        return save();
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator T &() {
        return _value;
    }

    /// Copy assignment from self does nothing.
    ///
    AP_ParamV<T,PT>& operator        =(AP_ParamV<T,PT>& v) {
        return v;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_ParamV<T,PT>& operator        =(T v) {
        _value = v;
        return *this;
    }

protected:
    T        _value;
};


/// Template class for array variables.
///
/// Objects created using this template behave like arrays of the type T,
/// but are stored like single variables.
///
/// @tparam T           The scalar type of the variable
/// @tparam N           number of elements
/// @tparam PT          the AP_PARAM_* type
///
template<typename T, uint8_t N, ap_var_type PT>
class AP_ParamA : public AP_Param
{
public:

    static const ap_var_type vtype = PT;

    /// Array operator accesses members.
    ///
    /// @note It would be nice to range-check i here, but then what would we return?
    ///
    T & operator[](uint8_t i) {
        return _value[i];
    }

    T & operator[](int8_t i) {
        return _value[(uint8_t)i];
    }

    /// Value getter
    ///
    /// @note   Returns zero for index values out of range.
    ///
    T get(uint8_t i) const {
        if (i < N) {
            return _value[i];
        } else {
            return (T)0;
        }
    }

    /// Value setter
    ///
    /// @note   Attempts to set an index out of range are discarded.
    ///
    void  set(uint8_t i, T v) {
        if (i < N) {
            _value[i] = v;
        }
    }

    /// Copy assignment from self does nothing.
    ///
    AP_ParamA<T,N,PT>& operator= (AP_ParamA<T,N,PT>& v) {
        return v;
    }

protected:
    T _value[N];
};



/// Convenience macro for defining instances of the AP_ParamT template.
///
// declare a scalar type
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _pt is the enum ap_var_type type
#define AP_PARAMDEF(_t, _suffix, _pt)   typedef AP_ParamT<_t, _pt> AP_ ## _suffix;
AP_PARAMDEF(float, Float, AP_PARAM_FLOAT);    // defines AP_Float
AP_PARAMDEF(int8_t, Int8, AP_PARAM_INT8);     // defines AP_Int8
AP_PARAMDEF(int16_t, Int16, AP_PARAM_INT16);  // defines AP_Int16
AP_PARAMDEF(int32_t, Int32, AP_PARAM_INT32);  // defines AP_Int32

// declare an array type
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _size is the size of the array
// _pt is the enum ap_var_type type
#define AP_PARAMDEFA(_t, _suffix, _size, _pt)   typedef AP_ParamA<_t, _size, _pt> AP_ ## _suffix;
AP_PARAMDEFA(float, Vector6f, 6, AP_PARAM_VECTOR6F);

// declare a non-scalar type
// this is used in AP_Math.h
// _t is the base type
// _suffix is the suffix on the AP_* type name
// _pt is the enum ap_var_type type
#define AP_PARAMDEFV(_t, _suffix, _pt)   typedef AP_ParamV<_t, _pt> AP_ ## _suffix;

#endif // AP_PARAM_H
