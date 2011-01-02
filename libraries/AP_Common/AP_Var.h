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

#include <inttypes.h>
#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <AP_MetaClass.h>

/// Nestable scopes for variable names.
///
///	This provides a mechanism for scoping variable names, and
///	may later be extended for other purposes.
///
/// When AP_VarBase is asked for the name of a variable, it will
/// prepend the names of all enclosing scopes.  This provides a way
/// of grouping variables and saving memory when many share a large
/// common prefix.
///
class AP_VarScope
{
public:
	/// Constructor
	///
	/// @param	scopeName		The name of the scope.
	///
	AP_VarScope(const prog_char *name,
	            AP_VarScope *parent = NULL) :
					_name(name),
					_parent(parent)
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
	void						copy_name(char *buffer, size_t bufferSize) const
	{
		if (_parent)
			_parent->copy_name(buffer, bufferSize);
		strlcat_P(buffer, _name, bufferSize);
	}

private:
	const prog_char	*_name;		/// pointer to the scope name in program memory
	AP_VarScope		*_parent;	/// pointer to a parent scope, if one exists
};

/// Base class for variables.
///
/// Provides naming and lookup services for variables.
///
class AP_VarBase : public AP_MetaClass
{
public:
	/// A unique identity for variables that can be saved to EEPROM
	///
	/// @todo	This might be used as a token for mass serialisation,
	///			but for now it's just the address of the variable's backing
	///			store in EEPROM.
	///
	typedef uint16_t		AP_VarIdentity;
	static const AP_VarIdentity	AP_VarUnsaved = 0;

	/// The largest variable that will be saved to EEPROM
	///
	static const size_t	AP_VarMaxSize = 16;

	/// Constructor
	///
	/// @param	name			An optional name by which the variable may be known.
	/// @param	scope			An optional scope that the variable may be a contained within.
	///
	AP_VarBase(AP_VarIdentity identity = AP_VarUnsaved,
	           const prog_char *name = NULL,
	           AP_VarScope *scope = NULL) :
	        	   _identity(identity),
	        	   _name(name),
	        	   _scope(scope)
	{
		if (name) {
			_link = _variables;
			_variables = this;
		}
	}

	/// Copy the variable name, prefixed by any parent class names, to a buffer.
	///
	/// Note that if the combination of names is larger than the buffer, the
	/// result in the buffer will be truncated.
	///
	/// @param	buffer			The destination buffer
	/// @param	bufferSize		Total size of the destination buffer.
	///
	void						copy_name(char *buffer, size_t bufferSize) const {
		if (_scope)
			_scope->copy_name(buffer, bufferSize);
		strlcat_P(buffer, _name, bufferSize);
	}

	/// Return a pointer to the n'th known variable
	///
	/// This interface is designed for the use of relatively low-rate clients;
	/// GCS interfaces in particular.  It may implement an acceleration scheme
	/// for optimising the lookup of parameters.
	///
	/// Note that variable index numbers are not constant, they depend on the
	/// the static construction order.
	///
	/// @param	index			enumerator for the variable to be returned
	///
	static AP_VarBase			*lookup(int index);

	/// Save the current value of the variable to EEPROM.
	///
	/// This interface works for any subclass that implements
	/// serialize.
	///
	void			save(void)	{
		if (_identity != AP_VarUnsaved) {
			uint8_t	vbuf[AP_VarMaxSize];
			size_t	size;

			// serialise the variable into the buffer and work out how big it is
			size = serialize(vbuf, sizeof(vbuf));

			// if it fit in the buffer, save it to EEPROM
			if (size <= sizeof(vbuf))
				eeprom_write_block(vbuf, (void *)_identity, size);
		}
	}

	/// Load the variable from EEPROM.
	///
	/// This interface works for any subclass that implements
	/// unserialize.
	///
	void			load(void) {
		if (_identity != AP_VarUnsaved) {
			uint8_t	vbuf[AP_VarMaxSize];
			size_t	size;

			// ask the unserialiser how big the variable is
			size = unserialize(NULL, 0);

			// read the buffer from EEPROM
			if (size <= sizeof(vbuf)) {
				eeprom_read_block(vbuf, (void *)_identity, size);
				unserialize(vbuf, size);
			}
		}
	}

	/// Save all variables to EEPROM
	///
	static void		save_all(void);

	/// Load all variables from EEPROM
	///
	static void		load_all(void);

private:
	const AP_VarIdentity	_identity;
	const prog_char			*_name;
	AP_VarScope				*_scope;
	AP_VarBase				*_link;

	// static state used by ::lookup
	static AP_VarBase	*_variables;
	static AP_VarBase	*_lookupHint;		/// pointer to the last variable that was looked up by ::lookup
	static int			_lookupHintIndex;	/// index of the last variable that was looked up by ::lookup
};


/// Template class for scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
///
template<typename T>
class AP_Var : public AP_VarBase
{
public:
	/// Constructor
	///
	/// @note		Constructors for AP_Var are specialised so that they can
	///				pass the correct typeCode argument to the AP_VarBase ctor.
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
	AP_Var<T>(T initialValue			= 0,
	          AP_VarIdentity identity	= AP_VarUnsaved,
	          const prog_char *name		= NULL,
	          AP_VarScope *scope		= NULL) :
				  AP_VarBase(identity, name, scope),
				  _value(initialValue)
	{
	}

	// Serialise _value into the buffer, but only if it is big enough.
	///
	virtual size_t			serialise(void *buf, size_t size) {
		if (size >= sizeof(T))
			*(T *)buf = _value;
		return sizeof(T);
	}

	// Unserialise from the buffer, but only if it is big enough.
	//
	virtual size_t			unserialise(void *buf, size_t size) {
		if (size >= sizeof(T))
			_value = *(T*)buf;
		return sizeof(T);
	}

	/// Value getter
	///
	T						get(void) { return _value; }

	/// Value setter
	///
	void					set(T v) { _value = v; }

	/// Conversion to T returns a reference to the value, may lead to simpler
	/// code than the getter/setter in some cases.
	///
	operator T&()						{ return _value; }

	/// Conversion to a pointer to T returns a pointer to the value,
	/// permits passing the value by reference.
	///
	operator T*() const					{ return &_value; }

	// Assignment from any value that T is compatible with.
	T operator = (const T v) const		{ return (_value = v); }

	// Comparison with any value that T is compatible with (should be avoided for T == float)
	bool operator == (const T &v) const	{ return _value == v; }
	bool operator != (const T &v) const	{ return _value != v; }

	// Negation
	T operator - (void) const			{ return -_value; }

	// Addition
	T operator +  (const T &v) const	{ return  _value +  v;  }
	T operator += (const T &v)			{ return (_value += v); }

	// Subtraction
	T operator -  (const T &v) const	{ return  _value -  v;  }
	T operator -= (const T &v)			{ return (_value -= v); }

	// Multiplication
	T operator *  (const T &v) const	{ return  _value *  v;  }
	T operator *= (const T &v)			{ return (_value *= v); }

	// Division
	T operator /  (const T &v) const	{ return  _value /  v;  }
	T operator /= (const T &v)			{ return (_value /= v); }

	// Modulus
	T operator %  (const T &v) const	{ return  _value % v;	}
	T operator %= (const T &v)			{ return (_value %= v); }

	// Bitwise operations
	T operator &   (const T &v) const	{ return  _value &   v;	 }
	T operator &=  (const T &v)			{ return (_value &=  v); }
	T operator |   (const T &v) const	{ return  _value |   v;	 }
	T operator |=  (const T &v)			{ return (_value |=  v); }
	T operator ^   (const T &v) const	{ return  _value ^   v;	 }
	T operator ^=  (const T &v)			{ return (_value ^=  v); }
	T operator <<  (const T &v) const	{ return  _value <<  v;	 }
	T operator <<= (const T &v)			{ return (_value <<= v); }
	T operator >>  (const T &v) const	{ return  _value >>  v;	 }
	T operator >>= (const T &v)			{ return (_value >>= v); }

private:
	T				_value;
};


/// Convenience macro for defining instances of the AP_Var template
///
#define AP_VARDEF(_t, _n)						\
	typedef AP_Var<_t>			AP_##_n;

AP_VARDEF(float,	Float);		// defines AP_Float, AP_NamedFloat and AP_SavedFloat
AP_VARDEF(int8_t,	Int8);		// defines AP_UInt8, AP_NamedUInt8 and AP_SavedUInt8
AP_VARDEF(int16_t,	Int16);		// defines AP_UInt16, AP_NamedUInt16 and AP_SavedUInt16
AP_VARDEF(int32_t,  Int32);		// defines AP_UInt32, AP_NamedUInt32 and AP_SavedUInt32

/// Some convenient constant AP_Vars.
extern const AP_Float		AP_FloatUnity;
extern const AP_Float		AP_FloatNegativeUnity;
extern const AP_Float		AP_FloatZero;

#endif // AP_Var_H
