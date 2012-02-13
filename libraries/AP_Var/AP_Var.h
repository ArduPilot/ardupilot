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
    static const uint16_t   k_EEPROM_revision   = 2;        ///< current format revision

    /// Storage key for variables that are saved in EEPROM.
    ///
    /// The key is used to associate a known variable in memory
    /// with storage for the variable in the EEPROM.
    ///
    /// At creation time, the _key value has the k_key_not_located bit
    /// set, and its value is an ordinal uniquely identifying the
    /// variable.  Once the address of the variable in EEPROM is known,
    /// either as a result of scanning or due to allocation of new
    /// space in the EEPROM, the k_key_not_located bit will be cleared
    /// and the _key value gives the offset into the EEPROM where
    /// the variable's data can be found.
    ///
    typedef uint16_t Key;

    /// This header is prepended to a variable stored in EEPROM.
    ///
    /// Note that the size value is the first element in the header.
    /// The sentinel entry marking the end of the EEPROM is always written
    /// before updating the header of a new variable at the end of EEPROM.
    /// This provides protection against corruption that might be caused
    /// if the update process is interrupted.
    ///
    struct Var_header {
        /// The size of the variable, minus one.
        /// This allows a variable or group to be anything from one to 64 bytes long.
        ///
        uint8_t    size:6;

        /// Spare bits, currently unused
        ///
        /// @todo   One could be a parity bit?
        ///
        uint8_t    spare:2;

        /// The key assigned to the variable.
        ///
        uint8_t    key;

    };

    /// A key value that indicates that a variable is not to be saved to EEPROM.
    ///
    /// As the value has all bits set to 1, it's not a legal EEPROM address for any
    /// EEPROM smaller than 64K (and it's too big to fit the Var_header::key field).
    ///
    /// This value is normally the default.
    ///
    static const Key k_key_none = 0xffff;

    /// A key that has this bit set is still a key; if it has been cleared then
    /// the key's value is actually an address in EEPROM.
    ///
    static const Key k_key_not_located = (Key)1 << 15;

    /// A key that has this bit set was not found during a scan of the EEPROM.
    /// If this bit is set in the key, it's not useful to scan the EEPROM again
    /// in an attempt to find it.
    ///
    static const Key k_key_not_allocated = (Key)1 << 14;

    /// Key assigned to the terminal entry in EEPROM.
    ///
    static const Key k_key_sentinel = 0xff;

    /// Key assigned to the wear-balancing pad entry in EEPROM.
    ///
    static const Key k_key_pad = 0xfe;

    /// A bitmask that removes any control bits from a key giving just the
    /// value.
    ///
    static const Key k_key_mask = (Key)(~(k_key_not_located | k_key_not_allocated));

    /// The largest variable that will be saved to EEPROM.
    /// This affects the amount of stack space that is required by the ::save, ::load,
    /// ::save_all and ::load_all functions.  It should match the maximum size that can
    /// be encoded in the Var_header::size field.
    ///
    static const size_t k_size_max = 64;

    /// The maximum number of keys
    ///
    static const size_t k_num_max = 255;

    /// Optional flags affecting the behavior and usage of the variable.
    ///
    typedef uint8_t Flags;
    static const Flags k_flags_none         = 0;

    /// The variable will not be loaded by ::load_all or saved by ::save_all, but it
    /// has a key and can be loaded/saved manually.
    static const Flags k_flag_no_auto_load  = (1 << 0);

    /// This flag is advisory; it indicates that the variable's value should not be
    /// imported from outside, e.g. from a GCS.
    static const Flags k_flag_no_import     = (1 << 1);

    /// This flag indicates that the variable is really a group; it is normally
    /// set automatically by the AP_Var_group constructor.
    ///
    static const Flags k_flag_is_group      = (1 << 2);

    /// This flag indicates that the variable wants to opt out of being listed.
    ///
    static const Flags k_flag_unlisted      = (1 << 3);

    /// @todo   Flag that indicates variable is non-volatile (auto-saved when set).

    static AP_Meta_class::Type_id  k_typeid_float;     ///< meta_type_id() value for AP_Float
    static AP_Meta_class::Type_id  k_typeid_float16;   ///< meta_type_id() value for AP_Float16
    static AP_Meta_class::Type_id  k_typeid_int32;     ///< meta_type_id() value for AP_Int32
    static AP_Meta_class::Type_id  k_typeid_int16;     ///< meta_type_id() value for AP_Int16
    static AP_Meta_class::Type_id  k_typeid_int8;      ///< meta_type_id() value for AP_Int8
    static AP_Meta_class::Type_id  k_typeid_group;     ///< meta_type_id() value for AP_Var_group

    /// Constructor for a freestanding variable
    ///
    /// @param  key             The storage key to be associated with this variable.
    /// @param	name			An optional name by which the variable may be known.
    /// @param  flags           Optional flags which control how the variable behaves.
    ///
    AP_Var(Key key = k_key_none, const prog_char_t *name = NULL, Flags flags = k_flags_none);

    /// Constructor for variable belonging to a group
    ///
    /// @param  group           The group the variable belongs to.
    /// @param  index           The position of the variable in the group.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags which control how the variable behaves.
    ///
    AP_Var(AP_Var_group *group, Key index, const prog_char_t *name, Flags flags = k_flags_none);

    /// Destructor
    ///
    /// For freestanding variables, this will remove the variable from the global list
    /// of variables.  The list is organised FIFO, so locally-constructed freestanding
    /// variables are typically cheap to destroy as they tend to be at or very close to
    /// the head of the list.
    ///
    /// Destroying a variable that is a group member may be less efficient as the list
    /// of variables that are group members is sorted by key, requiring a traversal
    /// of the list up to the index before it can be removed.
    ///
    /// Destroying a group removes all variables that are members of the group from
    /// the list, requiring a complete traversal of the list of group-member variables.
    /// If the group is destroyed before its members, they will also traverse the entire
    /// list as they attempt to remove themselves.
    ///
    /// The moral of the story: be careful when creating groups with local scope.
    ///
    ~AP_Var(void);

    /// Copy the variable's name, prefixed by any containing group name, to a buffer.
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

    /// Find a variable by name.
    ///
    /// If the variable has no name, it cannot be found by this interface.
    ///
    /// @param  name            The full name of the variable to be found.
    /// @return                 A pointer to the variable, or NULL if
    ///                         it does not exist.
    ///
    static AP_Var *find(const char *name);

    /// Find a variable by key.
    ///
    /// @param  key             The key being looked up.
    /// @return                 A pointer to the variable, or NULL if
    ///                         it does not exist.
    ///
    static AP_Var *find(Key key);

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

    /// Erase all variables in EEPROM.
    ///
    /// This can be used prior to save_all to ensure that only known variables
    /// will be present in the EEPROM.
    ///
    /// It can also be used immediately prior to reset, followed by a save_all,
    /// to restore all saved variables to their initial value.
    ///
    static void erase_all(void);

    /// Test for flags that may be set.
    ///
    /// @param	flagval			Flag or flags to be tested
    /// @return				True if all of the bits in flagval are set in the flags.
    ///
    bool has_flags(Flags flagval) const {
        return (_flags & flagval) == flagval;
    }

    /// Returns the group that a variable belongs to
    ///
    /// @return             The parent group, or NULL if the variable is not grouped.
    ///
    AP_Var_group *group(void) {
        return _group;
    }

    /// Returns the first variable in the global list.
    ///
    /// @return             The first variable in the global list, or NULL if
    ///                     there are none.
    ///
    static AP_Var *first(void) {
        return _variables;
    }

    /// Returns the next variable in the global list.
    ///
    /// All standalone variables are returned first, then all grouped variables.
    /// Note that groups themselves are considered standalone variables.
    ///
    /// A caller not wishing to iterate grouped variables should test the return
    /// value from this function with ::group, and if non-null, ignore it.
    ///
    /// XXX how to ignore groups?
    ///
    /// @return             The next variable, either the next group member in order or
    ///                     the next variable in an arbitrary order, or NULL if this is
    ///                     the last variable in the list.
    ///
    AP_Var *next(void);

    /// Returns the first variable that is a member of a specific group.
    ///
    /// @param  group       The group whose member(s) are sought.
    /// @return             The first variable in the group, or NULL if there are none.
    ///
    static AP_Var *first_member(AP_Var_group *group);

    /// Returns the next variable that is a member of the same group.
    ///
    /// This will not behave correctly if called on a variable that is not a group member.
    ///
    /// @param  group       The group whose member(s) are sought.
    /// @return             The next variable in the group, or NULL if there are none.
    ///
    AP_Var *next_member();

    /// Returns the storage key for a variable.
    ///
    /// Note that group members do not have storage keys - the key is held by the group
    /// and instead the key indicates the variable's ordering within the group.
    ///
    /// @return             The variable's key, or k_key_none if it does not have a key.
    ///
    Key key(void);

    /// Casts the value to float, if possible.
    ///
    /// This is a convenience method for code that would rather not try to
    /// meta-cast to the various subtypes.  It cannot guarantee that a useful value
    /// will be returned; the caller must check the returned value against NAN
    /// before using it.
    ///
    /// @return             The value cast to float, or NAN if it cannot be cast.
    ///
    virtual float cast_to_float() const;

    /// Report the amount of memory being used by AP_Var subclasses.
    ///
    /// @return             The sum of sizeof(*this) for all constructed AP_Var subclass instances.
    ///
    static uint16_t     get_memory_use() {
        return _bytes_in_use;
    }

protected:
    // Memory statistics
    static uint16_t     _bytes_in_use;

private:
    AP_Var_group        *_group;            ///< Group that the variable may be a member of
    AP_Var              *_link;             ///< linked list pointer to next variable
    Key                 _key;               ///< Storage key; see the discussion of Key above.
    const prog_char_t   *_name;             ///< name known to external agents (GCS, etc.)
    uint8_t             _flags;             ///< flag bits

    // static state used by ::lookup
    static AP_Var       *_variables;        ///< linked list of all freestanding variables
    static AP_Var       *_grouped_variables; ///< linked list of all grouped variables

    // EEPROM space allocation and scanning
    static uint16_t     _tail_sentinel;     ///< EEPROM address of the tail sentinel

    static const uint16_t k_EEPROM_size = 4096;    ///< XXX avr-libc doesn't consistently export this

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
    /// Constructor
    ///
    /// @param  key             Storage key for the group.
    /// @param	name			An optional name prefix for members of the group.
    ///
    AP_Var_group(Key with_key = k_key_none, const prog_char_t *name = NULL, Flags flags = k_flags_none) :
        AP_Var(with_key, name, flags | k_flag_is_group)
    {
        _bytes_in_use += sizeof(*this);
    }

    /// Serialize the group.
    ///
    /// Iteratively serializes the entire group into the supplied buffer.
    ///
    /// @param  buf             Buffer into which serialized data should be placed.
    /// @param  buf_size        The size of the buffer provided.
    /// @return                 The size of the serialized data, even if that data would
    ///                         have overflowed the buffer.  If the value is less than or
    ///                         equal to buf_size, serialization was successful.
    ///
    virtual size_t serialize(void *buf, size_t buf_size) const;

    /// Unserialize the group.
    ///
    /// Iteratively unserializes the group from the supplied buffer.
    ///
    /// @param  buf             Buffer containing serialized data.
    /// @param  buf_size        The size of the buffer.
    /// @return                 The number of bytes from the buffer that would be consumed
    ///                         unserializing the data.  If the value is less than or equal
    ///                         to buf_size, unserialization was successful.
    ///
    virtual size_t unserialize(void *buf, size_t buf_size);

private:
    /// Common implementation of the group member traversal and accounting used
    /// by serialize/unserialize.
    ///
    /// @param  buf             Buffer containing serialized data.
    /// @param  buf_size        The size of the buffer.
    /// @param  do_serialize    True if the operation should serialize, false if it should
    ///                         unserialize.
    /// @return                 The number of bytes from the buffer that would be consumed
    ///                         operating on the data.  If the value is less than or equal
    ///                         to buf_size, the operation was successful.
    ///
    size_t                      _serialize_unserialize(void *buf, size_t buf_size, bool do_serialize);
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
    /// @param  key             Storage key for the variable.  If not set, or set to AP_Var::k_key_none
    ///                         the variable cannot be loaded from or saved to EEPROM.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behaviour of the variable.
    ///
    AP_VarT<T> (const T initial_value = 0,
                Key with_key = k_key_none,
                const prog_char_t *name = NULL,
                Flags flags = k_flags_none) :
        AP_Var(with_key, name, flags),
        _value(initial_value)
    {
        _bytes_in_use += sizeof(*this);
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
    AP_VarT<T> (AP_Var_group *with_group,  // XXX maybe make this a ref?
                Key vindex,
                T initial_value,
                const prog_char_t *name = NULL,
                Flags flags = k_flags_none) :
        AP_Var(with_group, vindex, name, flags),
        _value(initial_value)
    {
        _bytes_in_use += sizeof(*this);
    }

    // serialize _value into the buffer, but only if it is big enough.
    //
    virtual size_t serialize(void *buf, size_t size) const {
        if (size >= sizeof(_value)) {
            *(T *)buf = _value;
        }
        return sizeof(_value);
    }

    // Unserialize from the buffer, but only if it is big enough.
    //
    virtual size_t unserialize(void *buf, size_t size) {
        if (size >= sizeof(_value)) {
            _value = *(T *)buf;
        }
        return sizeof(_value);
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

    /// Combined set and save
    ///
    bool set_and_save(T v) {
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
    AP_VarT<T>& operator=(AP_VarT<T>& v) {
        return v;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_VarT<T>& operator=(T v) {
        _value = v;
        return *this;
    }

    /// AP_VarT types can implement AP_Var::cast_to_float
    ///
    virtual float cast_to_float() const;

protected:
    T _value;
};

// Implement AP_Var::cast_to_float.
//
template<typename T>
float
AP_VarT<T>::cast_to_float() const
{
    return (float)_value;
}

/// Template class for structure variables.
///
/// Objects created using this variable contain an instance of the type T
///
/// Note that the size of the structure cannot be greater than AP_Var::k_size_max
/// if it is ever to be loaded or saved to EEPROM.
/// Unfortunately there is no way to assert this at compile time; a larger
/// array will compile correctly and work as expected, except that ::save and
/// ::load will always fail.
///
/// @note   The initial value of the fields of the structure are set by
///         the type's constructor at the time that the template is constructed.
///
template<typename T>
class AP_VarS : public AP_Var
{
public:
    /// Constructor for a non-grouped structure variable.
    ///
    /// Initializes a stand-alone structure variable with optional storage key, name and flags.
    ///
    /// @param  key             Storage key for the variable.  If not set, or set to AP_Var::k_key_none
    ///                         the variable cannot be loaded from or saved to EEPROM.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behavior of the variable.
    ///
    AP_VarS<T> (Key with_key = k_key_none,
                const prog_char_t *name = NULL,
                Flags flags = k_flags_none) :
        AP_Var(with_key, name, flags)
    {
        _bytes_in_use += sizeof(*this);
    }

    /// Constructor for a grouped structure variable.
    ///
    /// Initializes a variable that is a member of a group with optional initial value, name and flags.
    ///
    /// @param  group           The group that this variable belongs to.
    /// @param  index           The variable's index within the group.  Index values must be unique
    ///                         within the group, as they ensure that the group's layout in EEPROM
    ///                         is consistent.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behavior of the variable.
    ///
    AP_VarS<T> (AP_Var_group *with_group,  // XXX maybe make this a ref?
                Key index,
                const prog_char_t *name = NULL,
                Flags flags = k_flags_none) :
        AP_Var(with_group, index, name, flags)
    {
        _bytes_in_use += sizeof(*this);
    }

    // serialize _value into the buffer, but only if it is big enough.
    //
    virtual size_t serialize(void *buf, size_t size) const {
        if (size >= sizeof(_value)) {
            memcpy(buf, &_value, sizeof(_value));
        }
        return sizeof(_value);
    }

    // Unserialize from the buffer, but only if it is big enough.
    //
    virtual size_t unserialize(void *buf, size_t size) {
        if (size >= sizeof(_value)) {
            memcpy(&_value, buf, sizeof(_value));
        }
        return sizeof(_value);
    }

    /// Value getter
    ///
    T& get() {
        return _value;
    }

    /// Value getter
    ///
    const T& get() const {
        return _value;
    }

    /// Value setter
    ///
    void set(T v) {
        _value = v;
    }

    /// Combined set and save
    ///
    void set_and_save(T v) {
        set(v);
        save();
    }

    // Note no attempt to do anything fancy with the assignment or cast operators.
    // Implementing something that works reliably and produces readable code is not
    // straightforward, so it's been deferred for now.

protected:
    T   _value;
};

/// Template class for array variables.
///
/// Objects created using this template behave like arrays of the type T,
/// but are stored like single variables.
///
/// Note that the size of the array cannot be greater than AP_Var::k_size_max
/// if it is ever to be loaded or saved to EEPROM.
/// Unfortunately there is no way to assert this at compile time; a larger
/// array will compile correctly and work as expected, except that ::save and
/// ::load will always fail.
///
/// For example:
///
///  AP_VarA<float,4>   float_array(some_key, PSTR("some_name"));
///
/// will create float_array as an array of four floats, stored using the
/// key some_key and visible with the name 'some_name'.
///
/// @note   Like regular arrays, the initial value of the members of the
///         array is zero if the object is declared with global scope, and
///         undefined if declared in a block (function, etc.).
///
/// @tparam T           The scalar type of the variable
///
template<typename T, uint8_t N>
class AP_VarA : public AP_Var
{
public:
    /// Constructor for non-grouped array.
    ///
    /// Initializes a stand-alone array with optional storage key, name and flags.
    ///
    /// @param  key             Storage key for the variable.  If not set, or set to AP_Var::k_key_none
    ///                         the variable cannot be loaded from or saved to EEPROM.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behavior of the variable.
    ///
    AP_VarA<T,N> (Key with_key = k_key_none,
                  const prog_char_t *name = NULL,
                  Flags flags = k_flags_none) :
        AP_Var(with_key, name, flags)
    {
        _bytes_in_use += sizeof(*this);
    }

    /// Constructor for a grouped array.
    ///
    /// Initializes a variable that is a member of a group with optional initial value, name and flags.
    ///
    /// @param  group           The group that this variable belongs to.
    /// @param  index           The variable's index within the group.  Index values must be unique
    ///                         within the group, as they ensure that the group's layout in EEPROM
    ///                         is consistent.
    /// @param  name            An optional name by which the variable may be known.
    /// @param  flags           Optional flags that may affect the behavior of the variable.
    ///
    AP_VarA<T,N> (AP_Var_group *with_group,  // XXX maybe make this a ref?
                  Key index,
                  const prog_char_t *name = NULL,
                  Flags flags = k_flags_none) :
        AP_Var(with_group, index, name, flags)
    {
        _bytes_in_use += sizeof(*this);
    }

    // serialize _value into the buffer, but only if it is big enough.
    //
    virtual size_t serialize(void *buf, size_t size) const {
        if (size >= sizeof(_value)) {
            memcpy(buf, &_value[0], sizeof(_value));
        }
        return sizeof(_value);
    }

    // Unserialize from the buffer, but only if it is big enough.
    //
    virtual size_t unserialize(void *buf, size_t size) {
        if (size >= sizeof(_value)) {
            memcpy(&_value[0], buf, sizeof(_value));
        }
        return sizeof(_value);
    }

    /// Array operator accesses members.
    ///
    /// @note It would be nice to range-check i here, but then what would we return?
    ///
    T &operator [](uint8_t i) {
        return _value[i];
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
    void set(uint8_t i, T v) {
        if (i < N) {
            _value[i] = v;
        }
    }

    /// Copy assignment from self does nothing.
    ///
    AP_VarA<T,N>& operator=(AP_VarA<T,N>& v) {
        return v;
    }

protected:
    T _value[N];
};

/// Convenience macro for defining instances of the AP_VarT template.
///
#define AP_VARDEF(_t, _n)   typedef AP_VarT<_t> AP_##_n;
AP_VARDEF(float, Float);    // defines AP_Float
AP_VARDEF(int8_t, Int8);    // defines AP_Int8
AP_VARDEF(int16_t, Int16);  // defines AP_Int16
AP_VARDEF(int32_t, Int32);  // defines AP_Int32

/// Rely on built in casting for other variable types
/// to minimize template creation and save memory
#define AP_Uint8    AP_Int8
#define AP_Uint16   AP_Int16
#define AP_Uint32   AP_Int32
#define AP_Bool     AP_Int8

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
               Key with_key = k_key_none,
               const prog_char_t *name = NULL,
               Flags flags = k_flags_none) :
        AP_Float(initial_value, with_key, name, flags)
    {
        _bytes_in_use += sizeof(*this);
    }

    AP_Float16(AP_Var_group *with_group,
               Key index,
               float initial_value = 0,
               const prog_char_t *name = NULL,
               Flags flags = k_flags_none) :
        AP_Float(with_group, index, initial_value, name, flags)
    {
        _bytes_in_use += sizeof(*this);
    }


    // Serialize _value as Q5.10.
    //
    virtual size_t serialize(void *buf, size_t size) const {
        uint16_t *sval = (uint16_t *)buf;

        if (size >= sizeof(*sval)) {
            *sval = _value * 1024.0;    // scale by power of 2, may be more efficient
        }
        return sizeof(*sval);
    }

    // Unserialize _value from Q5.10.
    //
    virtual size_t unserialize(void *buf, size_t size) {
        uint16_t *sval = (uint16_t *)buf;

        if (size >= sizeof(*sval)) {
            _value = (float)*sval / 1024.0;    // scale by power of 2, may be more efficient
            return sizeof(*sval);
        }
        return 0;
    }

    // copy operators must be redefined in subclasses to get correct behavior
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


/// Print the value of an AP_Var
///
/// This function knows about the types listed in the AP_Var::k_typeid_* constants,
/// and it will print their value using Serial, which is assumed to be a BetterStream
/// serial port (e.g. FastSerial).
///
/// @param  vp      The variable to print.
///
extern void     AP_Var_print(AP_Var *vp);

#ifndef __AP_COMMON_MENU_H
#include "include/menu.h"
#endif

/// Menu function for setting an AP_Var.
///
/// This function can be directly called from a Menu.  It expects two args, the
/// first is the full name of a variable, the second is the value to set the variable
/// to.
///
/// This function knows about the types listed in the AP_Var::k_typeid_* constants.
///
/// @param  argc        The number of arguments; must be 3.
/// @param  argv        Arguments.  argv[1] must contain a variable name, argv[2].f and .i
///                     are consulted when setting the value.
/// @return             Zero if the variable was set successfully, -1 if it could not be set.
///
extern int8_t   AP_Var_menu_set(uint8_t argc, const Menu::arg *argv);

/// Menu function for displaying AP_Vars.
///
/// This function can be directly called from a Menu.  It expects zero or one argument.
/// If no arguments are supplied, all known variables are printed.  If an argument is
/// supplied, it is expected to be the name of a variable and that variable will be
/// printed.
///
/// @param  argc        The number of arguments, must be 1 or 2.
/// @param  argv        Argument array; argv[1] may be set to the name of a variable to show.
///
extern int8_t   AP_Var_menu_show(uint8_t argc, const Menu::arg *argv);


#endif // AP_VAR_H
