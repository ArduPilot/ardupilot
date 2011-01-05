// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//
/// The AP variable interface. This allows different types
/// of variables to be passed to blocks for floating point
/// math, memory management, etc.

#ifndef AP_Var_H
#define AP_Var_H
#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "AP_MetaClass.h"

class AP_VarScope;

/// Base class for variables.
///
/// Provides naming and lookup services for variables.
///
class AP_Var : public AP_MetaClass
{
public:
	/// Storage address for variables that can be saved to EEPROM
	///
	/// If the variable is contained within a scope, then the address
	/// is relative to the scope.
	///
	/// @todo	This might be used as a token for mass serialisation,
	///			but for now it's just the address of the variable's backing
	///			store in EEPROM.
	///
	typedef uint16_t		AP_VarAddress;

	/// An address value that indicates that a variable is not to be saved to EEPROM.
	/// This value is normally the default.
	///
	static const AP_VarAddress	AP_VarNoAddress = !(AP_VarAddress)0;

	/// The largest variable that will be saved to EEPROM.
	/// This affects the amount of stack space that is required by the ::save, ::load,
	/// ::save_all and ::load_all functions.
	///
	static const size_t	AP_VarMaxSize = 16;

	/// Constructor
	///
	/// @param	name			An optional name by which the variable may be known.
	///							This name may be looked up via the ::lookup function.
	/// @param	scope			An optional scope that the variable may be a contained within.
	///							The scope's name will be prepended to the variable name
	///							by ::copy_name.
	///
	AP_Var(AP_VarAddress		address = AP_VarNoAddress,
	       const prog_char		*name = NULL,
	       const AP_VarScope	*scope = NULL);

	/// Destructor
	///
	/// Note that the linked-list removal can be inefficient when variables
	/// are destroyed in an order other than the reverse of the order in which
	/// they are created.  This is not a major issue for variables created
	/// and destroyed automatically at block boundaries, and the creation and
	/// destruction of variables by hand is generally discouraged.
	///
	~AP_Var(void);

	/// Set the variable to its default value
	///
	virtual void	set_default(void);

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
	void			copy_name(char *buffer, size_t bufferSize) const;

	/// Return a pointer to the n'th known variable.
	///
	/// This function is used to iterate the set of variables that are considered
	/// interesting; i.e. those that may be saved to EEPROM, or that have a name.
	///
	/// Note that variable index numbers are not constant, they depend on the
	/// the static construction order.
	///
	/// @param	index			enumerator for the variable to be returned
	///
	static AP_Var	*lookup(int index);

	/// Save the current value of the variable to EEPROM.
	///
	/// This interface works for any subclass that implements
	/// serialize.
	///
	void			save(void) const;

	/// Load the variable from EEPROM.
	///
	/// This interface works for any subclass that implements
	/// unserialize.
	///
	void			load(void);

	/// Save all variables to EEPROM
	///
	static void		save_all(void);

	/// Load all variables from EEPROM
	///
	static void		load_all(void);

private:
	const AP_VarAddress		_address;
	const prog_char			*_name;
	const AP_VarScope		* const _scope;
	AP_Var					*_link;

	/// Do the arithmetic required to compute the variable's address in EEPROM
	///
	/// @returns		The address at which the variable is stored in EEPROM,
	///					or AP_VarNoAddress if it is not saved.
	///
	AP_VarAddress			_get_address(void) const;

	// static state used by ::lookup
	static AP_Var			*_variables;
	static AP_Var			*_lookupHint;		/// pointer to the last variable that was looked up by ::lookup
	static int				_lookupHintIndex;	/// index of the last variable that was looked up by ::lookup
};

/// Nestable scopes for variable names.
///
///	This provides a mechanism for scoping variable names and their
/// EEPROM addresses.
///
/// When AP_Var is asked for the name of a variable, it will
/// prepend the names of all enclosing scopes.  This provides a way
/// of grouping variables and saving memory when many share a large
/// common prefix.
///
/// When AP_var computes the address of a variable, it will take
/// into account the address offsets of each of the variable's
/// enclosing scopes.
///
class AP_VarScope
{
public:
	/// Constructor
	///
	/// @param	name			The name of the scope.
	/// @param	address			An EEPROM address offset to be added to the address assigned to
	///							any variables within the scope.
	///	@param	parent			Optional parent scope to nest within.
	///
	AP_VarScope(const prog_char *name,
	            AP_Var::AP_VarAddress address = 0,
	            AP_VarScope *parent = NULL) :
					_name(name),
					_parent(parent),
					_address(address)
	{
	}

	/// Copy the scope name, prefixed by any parent scope names, to a buffer.
	///
	/// Note that if the combination of names is larger than the buffer, the
	/// result in the buffer will be truncated.
	///
	/// @param	buffer			The destination buffer
	/// @param	bufferSize		Total size of the destination buffer.
	///
	void			copy_name(char *buffer, size_t bufferSize) const
	{
		if (_parent)
			_parent->copy_name(buffer, bufferSize);
		strlcat_P(buffer, _name, bufferSize);
	}

	/// Compute the address offset that this and any parent scope might apply
	/// to variables inside the scope.
	///
	/// This provides a way for variables to be grouped into collections whose
	/// EEPROM addresses can be more easily managed.
	///
	AP_Var::AP_VarAddress	get_address(void) const;

private:
	const prog_char			*_name;		/// pointer to the scope name in program memory
	AP_VarScope				*_parent;	/// pointer to a parent scope, if one exists
	AP_Var::AP_VarAddress	_address;	/// container base address, offsets contents
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
	/// Constructor
	///
	/// @note		Constructors for AP_Var are specialised so that they can
	///				pass the correct typeCode argument to the AP_Var ctor.
	///
	/// @param	initialValue	Value the variable should have at startup.
	/// @param	identity		A unique token used when saving the variable to EEPROM.
	///							Note that this token must be unique, and should only be
	///							changed for a variable if the EEPROM version is updated
	///							to prevent the accidental unserialisation of old data
	///							into the wrong variable.
	/// @param	name			An optional name by which the variable may be known.
	/// @param	varClass		An optional class that the variable may be a member of.
	///
	AP_VarT<T>(T defaultValue			= 0,
	          AP_VarAddress address		= AP_VarNoAddress,
	          const prog_char *name		= NULL,
	          AP_VarScope *scope		= NULL) :
				  AP_Var(address, name, scope),
				  _value(defaultValue),
				  _defaultValue(defaultValue)
	{
	}

	// serialize _value into the buffer, but only if it is big enough.
	//
	virtual size_t			serialize(void *buf, size_t size) const {
		if (size >= sizeof(T))
			*(T *)buf = _value;
		return sizeof(T);
	}

	// Unserialize from the buffer, but only if it is big enough.
	//
	virtual size_t			unserialize(void *buf, size_t size) {
		if (size >= sizeof(T))
			_value = *(T*)buf;
		return sizeof(T);
	}

	/// Restore the variable to its default value
	virtual void			set_default(void) { _value = _defaultValue; }

	/// Value getter
	///
	T						get(void) const { return _value; }

	/// Value setter
	///
	void					set(T v) { _value = v; }

	/// Conversion to T returns a reference to the value.
	///
	/// This allows the class to be used in many situations where the value would be legal.
	///
	operator T&()			{ return _value; }

	/// Copy assignment from self does nothing.
	///
	AP_VarT<T>& operator=(AP_VarT<T>& v)	{ return v; }

	/// Copy assignment from T
	AP_VarT<T>& operator=(T v)				{ _value = v; return *this; }

protected:
	T				_value;
	T				_defaultValue;
};


/// Convenience macro for defining instances of the AP_Var template
///
#define AP_VARDEF(_t, _n)						\
	typedef AP_VarT<_t>			AP_##_n;

AP_VARDEF(float,	Float);		// defines AP_Float, AP_NamedFloat and AP_SavedFloat
AP_VARDEF(int8_t,	Int8);		// defines AP_UInt8, AP_NamedUInt8 and AP_SavedUInt8
AP_VARDEF(int16_t,	Int16);		// defines AP_UInt16, AP_NamedUInt16 and AP_SavedUInt16
AP_VARDEF(int32_t,  Int32);		// defines AP_UInt32, AP_NamedUInt32 and AP_SavedUInt32

/// Many float values can be saved as 16-bit fixed-point values, reducing EEPROM
/// consumption.  AP_Float16 subclasses AP_Float and overloads serialize/unserialize
/// to achieve this.
///
/// Note that any protocol transporting serialized data should be aware that the
/// encoding used is effectively Q5.10 (one sign bit, 5 integer bits, 10 fractional bits).
///
class AP_Float16 : public AP_Float
{
public:
	/// Constructor mimics AP_Float::AP_Float()
	///
	AP_Float16(float defaultValue			= 0,
	           AP_VarAddress address		= AP_VarNoAddress,
	           const prog_char *name		= NULL,
	           AP_VarScope *scope		= NULL) :
	        	   AP_Float(defaultValue, address, name, scope)
	{
	}

	// Serialize _value as Q5.10.
	//
	virtual size_t			serialize(void *buf, size_t size) const {
		uint16_t	*sval = (uint16_t *)buf;

		if (size >= sizeof(*sval))
			*sval = _value / 1024.0;	// scale by power of 2, may be more efficient
		return sizeof(*sval);
	}

	// Unserialize _value from Q5.10.
	//
	virtual size_t			unserialize(void *buf, size_t size) {
		uint16_t	*sval = (uint16_t *)buf;

		if (size >= sizeof(*sval))
			_value = *sval * 1024.0;	// scale by power of 2, may be more efficient
		return sizeof(*sval);
	}

	// copy operators must be redefined in subclasses to get correct behaviour
	AP_Float16& operator=(AP_Float16 &v)	{ return v; }
	AP_Float16& operator=(float v)			{ _value = v; return *this; }

};

/// Some convenient constant AP_Vars.
///
/// @todo	Work out why these can't be const and fix if possible.
///
extern AP_Float		AP_FloatUnity;
extern AP_Float		AP_FloatNegativeUnity;
extern AP_Float		AP_FloatZero;

#endif // AP_Var_H
