// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	AP_Var.h
/// @brief	A system for managing and storing variables that are of
///			general interest to the system.
///
///

/// The AP variable interface. This allows different types
/// of variables to be passed to blocks for floating point
/// math, memory management, etc.

#ifndef AP_VAR_H
#define AP_VAR_H
#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "AP_MetaClass.h"

class AP_Var_group;

/// Base class for variables.
///
/// Provides naming and lookup services for variables.
///
class AP_Var : public AP_Meta_class
{
public:
    /// EEPROM header
    ///
    /// This structure is placed at the head of the EEPROM to indicate
    /// that the ROM is formatted for AP_Var.
    ///
    /// The EEPROM may thus be cheaply erased by overwriting just one
    /// byte of the header magic.
    ///
    struct EEPROM_header {
        uint16_t    magic;
        uint8_t     revision;
        uint8_t     spare;
    };

    static const uint16_t   k_EEPROM_magic      = 0x5041;   ///< "AP"
    static const uint16_t   k_EEPROM_revision   = 1;        ///< current format revision

    /// Storage key for variables that are saved in EEPROM.
    ///
    /// The key is used to associate a known variable in memory
    /// with storage for the variable in the EEPROM.  When the
    /// variable's storage is located in EEPROM, the key value is
    /// replaced with an EEPROM address.
    ///
    /// When the variable is saved to EEPROM, a header is attached
    /// which contains the key along with other details.
    ///
    typedef uint16_t Key;

    /// The header prepended to a variable stored in EEPROM
    ///
    struct Var_header {
        /// The size of the variable, minus one.
        /// This allows a variable to be anything from one to 32 bytes long.
        ///
        uint16_t    size:5;

        /// The key assigned to the variable.
        ///
        uint16_t    key:8;

        /// Spare bits, currently unused
        ///
        /// @todo   One could be a parity bit?
        ///
        uint16_t    spare:3;
    };

    /// An key that indicates that a variable is not to be saved to EEPROM.
    ///
    /// As the value has all bits set to 1, it's not a legal EEPROM address for any
    /// EEPROM smaller than 64K (and it's too big to fit the Var_header::key field).
    ///
    /// This value is normally the default.
    ///
    static const Key k_no_key = 0xffff;

    /// A key that has this bit set is still a key; if it has been cleared then
    /// the key's value is actually an address in EEPROM.
    ///
    static const Key k_not_located = (Key)1 << 15;

    /// Key assigned to the terminal entry in EEPROM.
    ///
    static const Key k_tail_sentinel = 0xff;

    /// A bitmask that removes any control bits from a key giving just the
    /// value.
    ///
    static const Key k_key_mask = ~(k_not_located);

    /// The largest variable that will be saved to EEPROM.
    /// This affects the amount of stack space that is required by the ::save, ::load,
    /// ::save_all and ::load_all functions.  It should match the maximum size that can
    /// be encoded in the Var_header::size field.
    ///
    static const size_t k_max_size = 32;

    /// Optional flags affecting the behavior and usage of the variable.
    ///
    enum Flags {
        k_no_flags     = 0,

        /// The variable will not be loaded by ::load_all or saved by ::save_all, but it
        /// has a key and can be loaded/saved manually.
        k_no_auto_load = (1 << 0),

        /// This flag is advisory; it indicates that the variable's value should not be
        /// imported from outside, e.g. from a GCS.
        k_no_import    = (1 << 1)
    };

    AP_Var() {}

    /// Constructor
    ///
    /// @param  key             The storage key to be associated with this variable.
    /// @param	name			An optional name by which the variable may be known.
    ///							This name may be looked up via the ::lookup function.
    /// @param	group			An optional group to which the variable may belong.
    ///							The scope's name will be prepended to the variable name
    ///							by ::copy_name.
    /// @param  flags           Optional flags which control how the variable behaves.
    ///
    AP_Var(Key key, const prog_char *name, AP_Var_group *group, Flags flags);

    /// Destructor
    ///
    /// Note that the linked-list removal can be inefficient when variables
    /// are destroyed in an order other than the reverse of the order in which
    /// they are created.  This is not a major issue for variables created
    /// and destroyed automatically at block boundaries, and the creation and
    /// destruction of variables by hand is generally discouraged.
    ///
    ~AP_Var(void);

    /// Copy the variable's name, prefixed by any parent class names, to a buffer.
    ///
    /// If the variable has no name, the buffer will contain an empty string.
    ///
    /// Note that if the combination of names is larger than the buffer, the
    /// result in the buffer will be truncated.
    ///
    /// @param	buffer			The destination buffer
    /// @param	bufferSize		Total size of the destination buffer.
    ///
    void copy_name(char *buffer, size_t bufferSize) const;

    /// Return a pointer to the n'th known variable.
    ///
    /// This function is used to iterate the set of variables, and is optimised
    /// for the case where the index argument is increased sequentially from one
    /// call to the next.
    ///
    /// Note that variable index numbers are not constant; they will vary
    /// from build to build.
    ///
    /// @param	index			Enumerator for the variable to be returned
    /// @return                Pointer to the index'th variable, or NULL if
    ///                         the set of variables has been exhausted.
    ///
    static AP_Var *lookup_by_index(int index);

    /// Save the current value of the variable to EEPROM.
    ///
    /// This interface works for any subclass that implements
    /// AP_Meta_class::serialize.
    ///
    /// Note that invoking this method on a variable that is a group
    /// member will cause the entire group to be saved.
    ///
    /// @return                True if the variable was saved successfully.
    ///
    bool save(void);

    /// Load the variable from EEPROM.
    ///
    /// This interface works for any subclass that implements
    /// AP_Meta_class::unserialize.
    ///
    /// If the variable has not previously been saved to EEPROM, this
    /// routine will return failure.
    ///
    /// Note that invoking this method on a variable that is a group
    /// member will cause the entire group to be loaded.
    ///
    /// @return                True if the variable was loaded successfully.
    ///
    bool load(void);

    /// Save all variables to EEPROM
    ///
    /// This routine performs a best-efforts attempt to save all
    /// of the variables to EEPROM.  If some fail to save, the others
    /// that succeed will still all be saved.
    ///
    /// @return                False if any variable failed to save.
    ///
    static bool save_all(void);

    /// Load all variables from EEPROM
    ///
    /// This function performs a best-efforts attempt to load all
    /// of the variables from EEPROM.  If some fail to load, their
    /// values will remain as they are.
    ///
    /// @return                False if any variable failed to load.  Note
    ///                         That this may be caused by a variable not having
    ///                         previously been saved.
    ///
    static bool load_all(void);

    /// Test for flags that may be set.
    ///
    /// @param	flagval			Flag or flags to be tested
    /// @return				True if all of the bits in flagval are set in the flags.
    ///
    bool has_flags(Flags flagval) const {
        return (_flags & flagval) == flagval;
    }

private:
    AP_Var_group        *_group;            ///< Group that the variable may be a member of
    Key                 _key;               ///< storage key
    const prog_char     *_name;             ///< name known to external agents (GCS, etc.)
    AP_Var              *_link;             ///< linked list pointer to next variable
    uint8_t             _flags;             ///< flag bits

    // static state used by ::lookup
    static AP_Var       *_variables;        ///< linked list of all variables
    static AP_Var       *_lookup_hint;      ///< pointer to the last variable that was looked up by ::lookup
    static int          _lookup_hint_index; ///< index of the last variable that was looked up by ::lookup


    // EEPROM space allocation and scanning
    static uint16_t     _tail_sentinel;     ///< EEPROM address of the tail sentinel
    static bool         _EEPROM_scanned;    ///< true once the EEPROM has been scanned and addresses assigned

    static const uint16_t k_EEPROM_size = 4096;    ///< XXX avr-libc doesn't consistently export this


    /// Return a pointer to the variable with a given key.
    ///
    /// This function is used to search the set of variables for
    /// one with a given key.
    ///
    /// Note that this search will fail (XXX should it?) if the variable
    /// assigned this key has been located in EEPROM.
    ///
    /// @param  key             The key to search for.
    /// @return                Pointer to the variable assigned the key,
    ///                         or NULL if no variable owns up to it.
    ///
    static AP_Var *_lookup_by_key(Key key);

    /// Scan the EEPROM and assign addresses to any known variables
    /// that have entries there.
    ///
    /// @return         True if the EEPROM was scanned successfully.
    ///
    static bool         _EEPROM_scan(void);

    /// Locate this variable in the EEPROM.
    ///
    /// @param  allocate        If true, and the variable does not already have
    ///                         space allocated in EEPROM, allocate it.
    /// @return                 True if the _key field is a valid EEPROM address with
    ///                         space reserved for the variable to be saved.  False
    ///                         if the variable does not have a key, or space could not
    ///                         be allocated and the variable does not already exist in
    ///                         EEPROM.
    ///
    bool                _EEPROM_locate(bool allocate);

};

/// Variable groups.
///
///	Grouped variables are treated as a single variable when loaded from
/// or saved to EEPROM.  The size limits for variables apply to the entire
/// group; thus a group cannot be larger than the largest legal variable.
///
/// When AP_Var is asked for the name of a variable that is a member
/// of a group, it will prepend the name of the group; this helps save
/// memory.
///
/// Variables belonging to a group are always sorted into the global
/// variable list after the group.
///
class AP_Var_group : public AP_Var
{
public:
    AP_Var_group();
    virtual ~AP_Var_group();

    /// Constructor
    ///
    /// @param  key             Storage key for the group.
    /// @param	name			An optional name prefix for members of the group.
    ///
    AP_Var_group(Key key, const prog_char *name = NULL) :
        AP_Var(key, name, NULL, k_no_flags)
    {
    }

    /// Serialize the group.
    ///
    /// Iteratively serializes the entire group into the supplied buffer.
    ///
    /// @param  buf             Buffer into which serialized data should be placed.
    /// @param  bufSize         The size of the buffer provided.
    /// @return                 The size of the serialized data, even if that data would
    ///                         have overflowed the buffer.
    ///
    virtual size_t serialize(void *buf, size_t bufSize) const;

    /// Unserialize the group.
    ///
    /// Iteratively unserializes the group from the supplied buffer.
    ///
    /// @param  buf             Buffer containing serialized data.
    /// @param  bufSize         The size of the buffer.
    /// @return                 The number of bytes from the buffer that would be consumed
    ///                         unserializing the data.  If the value is less than or equal
    ///                         to bufSize, unserialization was successful.
    ///
    virtual size_t unserialize(void *buf, size_t bufSize);

};

/// Template class for scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
///
template<typename T>
class AP_VarT : public AP_Var
{
public:
    /// Constructor for non-grouped variable.
    ///
    /// Initialises a stand-alone variable with optional initial value, storage key, name and flags.
    ///
    /// @param  default_value   Value the variable should have at startup.
    /// @param  key             Storage key for the variable.  If not set, or set to AP_Var::k_no_key
    ///                         the variable cannot be loaded from or saved to EEPROM.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behaviour of the variable.
    ///
    AP_VarT<T> (T initial_value = 0,
                Key key = k_no_key,
                const prog_char *name = NULL,
                Flags flags = k_no_flags) :
        AP_Var(key, name, NULL, flags),
        _value(initial_value)
    {
    }

    /// Constructor for grouped variable.
    ///
    /// Initialises a variable that is a member of a group with optional initial value, name and flags.
    ///
    /// @param  group           The group that this variable belongs to.
    /// @param  index           The variable's index within the group.  Index values must be unique
    ///                         within the group, as they ensure that the group's layout in EEPROM
    ///                         is consistent.
    /// @param  initial_value   The value the variable takes at startup.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behaviour of the variable.
    ///
    AP_VarT<T> (AP_Var_group *group,
                Key index,
                T initial_value = 0,
                const prog_char *name = NULL,
                Flags flags = k_no_flags) :
        AP_Var(index, name, group, flags),
        _value(initial_value)
    {
    }

    // serialize _value into the buffer, but only if it is big enough.
    //
    virtual size_t serialize(void *buf, size_t size) const {
        if (size >= sizeof(T)) {
            *(T *)buf = _value;
        }
        return sizeof(T);
    }

    // Unserialize from the buffer, but only if it is big enough.
    //
    virtual size_t unserialize(void *buf, size_t size) {
        if (size >= sizeof(T)) {
            _value = *(T *)buf;
        }
        return sizeof(T);
    }

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

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator T &() {
        return _value;
    }

    /// Copy assignment from self does nothing.
    ///
    AP_VarT<T>& operator=(AP_VarT<T>& v) {
        return v;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_VarT<T>& operator=(T v) {
        _value = v;
        return *this;
    }

protected:
    T _value;
};

/// Convenience macro for defining instances of the AP_Var template
///
#define AP_VARDEF(_t, _n)   typedef AP_VarT<_t> AP_##_n;
AP_VARDEF(float, Float);    // defines AP_Float
AP_VARDEF(int8_t, Int8);    // defines AP_Int8
AP_VARDEF(int16_t, Int16);  // defines AP_Int16
AP_VARDEF(int32_t, Int32);  // defines AP_Int32

/// Rely on built in casting for other variable types
/// to minimize template creation and save memory
#define AP_Uint8 AP_Int8
#define AP_Uint16 AP_Int16
#define AP_Uint32 AP_Int32
#define AP_Bool AP_Int8

/// Many float values can be saved as 16-bit fixed-point values, reducing EEPROM
/// consumption.  AP_Float16 subclasses AP_Float and overloads serialize/unserialize
/// to achieve this.
///
/// Note that any protocol transporting serialized data should be aware that the
/// encoding used is effectively Q5.10 (one sign bit, 5 integer bits, 10 fractional bits),
/// giving an effective range of approximately +/-31.999
///
class AP_Float16 : public AP_Float
{
public:
    /// Constructors mimic AP_Float::AP_Float()
    ///
    AP_Float16(float initial_value = 0,
               Key key = k_no_key,
               const prog_char *name = NULL,
               Flags flags = k_no_flags) :
        AP_Float(initial_value, key, name, flags)
    {
    }

    AP_Float16(AP_Var_group *group,
               Key index,
               float initial_value = 0,
               const prog_char *name = NULL,
               Flags flags = k_no_flags) :
        AP_Float(group, index, initial_value, name, flags)
    {
    }


    // Serialize _value as Q5.10.
    //
    virtual size_t serialize(void *buf, size_t size) const {
        uint16_t *sval = (uint16_t *)buf;

        if (size >= sizeof(*sval)) {
            *sval = _value / 1024.0;    // scale by power of 2, may be more efficient
        }
        return sizeof(*sval);
    }

    // Unserialize _value from Q5.10.
    //
    virtual size_t unserialize(void *buf, size_t size) {
        uint16_t *sval = (uint16_t *)buf;

        if (size >= sizeof(*sval)) {
            _value = *sval * 1024.0;    // scale by power of 2, may be more efficient
        }
        return sizeof(*sval);
    }

    // copy operators must be redefined in subclasses to get correct behaviour
    AP_Float16 &operator=(AP_Float16 &v) {
        return v;
    }
    AP_Float16 &operator=(float v) {
        _value = v;
        return *this;
    }

};

/// Some convenient constant AP_Vars.
///
/// @todo	Work out why these can't be const and fix if possible.
///
/// @todo   Work out how to get these into a namespace and name them properly.
///
extern AP_Float AP_Float_unity;
extern AP_Float AP_Float_negative_unity;
extern AP_Float AP_Float_zero;

#endif // AP_VAR_H
