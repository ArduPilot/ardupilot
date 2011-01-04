// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	AP_MetaClass.h
///			Abstract meta-class from which other AP classes may inherit.
///			Provides type introspection and some basic protocols that can
///			be implemented by subclasses.

#ifndef AP_METACLASS_H
#define AP_METACLASS_H

#include <stddef.h>			// for size_t
#include <inttypes.h>

#include <avr/io.h>			// for RAMEND

/// Basic meta-class from which other AP_* classes can derive.
///
/// Functions that form the public API to the metaclass are prefixed meta_.
///
class AP_MetaClass
{
public:
	/// Default constructor does nothing.
	AP_MetaClass(void);

	/// Default destructor is virtual, to ensure that all destructors
	/// are called for derived classes.
	virtual ~AP_MetaClass();

	/// Type code, unique to all instances of a given subclass.
	typedef uint16_t	AP_TypeID;

	/// Obtain a value unique to all instances of a specific subclass.
	///
	/// This is similar to the basic functionality of the C++ typeid
	/// keyword, but does not depend on std::type_info or any compiler-
	/// generated RTTI.
	///
	/// As the value is derived from the vtable address, it cannot be
	/// introspected outside the current state of the system.
	///
	/// @param	p				A pointer to an instance of a subclass of AP_MetaClass.
	/// @return					A type-unique value.
	///
	AP_TypeID					meta_type_id(void) const {
		return *(AP_TypeID *)this;
	}

	/// External handle for an instance of an AP_MetaClass subclass, contains
	/// enough information to construct and validate a pointer to the instance
	/// when passed back from an untrusted source.
	///
	typedef uint32_t	AP_MetaHandle;

	/// Return a value that can be used as an external pointer to an instance
	/// of a subclass.
	///
	/// The value can be passed to an untrusted agent, and validated on its return.
	///
	/// The value contains the 16-bit type ID of the actual class and
	/// a pointer to the class instance.
	///
	/// @return					An opaque handle
	///
	AP_MetaHandle				meta_get_handle(void) const	{
		return ((AP_MetaHandle)meta_type_id() << 16) | (uint16_t)this;
	}

	/// Validates an AP_MetaClass handle.
	///
	/// The value of the handle is not required to be valid; in particular the
	/// pointer encoded in the handle is validated before being dereferenced.
	///
	/// The handle is considered good if the pointer is valid and the object
	/// it points to has a type ID that matches the ID in the handle.
	///
	/// @param	handle			A possible AP_MetaClass handle
	/// @return					The instance pointer if the handle is good,
	///							or NULL if it is bad.
	///
	static AP_MetaClass			*meta_validate_handle(AP_MetaHandle handle) {
		AP_MetaClass	*candidate = (AP_MetaClass *)(handle & 0xffff);	// extract object pointer
		uint16_t		id = handle >> 16;								// and claimed type

		// Sanity-check the pointer to ensure it lies within the device RAM, so that
		// a bad handle won't cause ::meta_type_id to read outside of SRAM.
		// Assume that RAM (or addressable storage of some sort, at least) starts at zero.
		//
		// Note that this implies that we cannot deal with objects in ROM or EEPROM,
		// but the constructor wouldn't be able to populate a vtable pointer there anyway...
		//
		if ((uint16_t)candidate >= (RAMEND - 2))	// -2 to account for the type_id
			return NULL;

		// Compare the typeid of the object that candidate points to with the typeid
		// from the handle.  Note that it's safe to call meta_type_id() off the untrusted
		// candidate pointer because meta_type_id is non-virtual (and will in fact be
		// inlined here).
		//
		if (candidate->meta_type_id() == id)
			return candidate;

		return NULL;
	}

	/// Tests whether two objects are of precisely the same class.
	///
	/// Note that for p2 inheriting from p1, this will return false.
	/// Even with RTTI not disabled, there does not seem to be enough information
	/// to determine whether one class inherits from another.
	///
	/// @param	p1				The first object to be compared.
	/// @param	p2				The second object to be compared.
	/// @return					True if the two objects are of the same class, false
	///							if they are not.
	///
	static bool					meta_type_equivalent(AP_MetaClass *p1, AP_MetaClass *p2) {
		return p1->meta_type_id() == p2->meta_type_id();
	}

	/// Cast an object to an expected class type.
	///
	/// This should be used with caution, as _typename's default constructor and
	/// destructor will be run, possibly introducing undesired side-effects.
	///
	/// @todo	Consider whether we should make it difficult to have a default constructor
	///			with appreciable side-effects.
	///
	/// @todo	Check whether we need to reinterpret_cast to get the right return type.
	///
	/// @param	_p				An AP_MetaClass subclass whose type is to be tested.
	/// @param	_typename		The name of a type with which _p is to be compared.
	/// @return					True if _p is of type _typename, false otherwise.
	///
	template<typename T>
	static T* meta_cast(AP_MetaClass *p) {
		T	tmp;
		if (meta_type_equivalent(p, &tmp))
			return (T *)p;
		return NULL;
	}

	/// Serialise the class.
	///
	/// Serialisation provides a mechanism for exporting the state of the class to an
	/// external consumer, either for external introspection or for subsequent restoration.
	///
	/// Classes that wrap variables should define the format of their serialised data
	/// so that external consumers can reliably interpret it.
	///
	/// @param	buf				Buffer into which serialised data should be placed.
	/// @param	bufSize			The size of the buffer provided.
	/// @return					The size of the serialised data, even if that data would
	///							have overflowed the buffer.  If the return value is zero,
	///							the class does not support serialisation.
	///
	virtual size_t				serialize(void *buf, size_t bufSize) const;

	/// Unserialise the class.
	///
	/// Unserialising a class from a buffer into which the class previously serialised
	/// itself restores the instance to an identical state, where "identical" may be
	/// defined in context.
	///
	/// Classes that wrap variables should define the format of their serialised data so
	/// that external providers can reliably encode it.
	///
	/// @param	buf				Buffer containing serialised data.
	/// @param	bufSize			The size of the buffer.
	/// @return					The number of bytes from the buffer that would be consumed
	///							unserialising the data.  If the value is less than or equal
	///							to bufSize, unserialisation was successful.  If the return
	///							value is zero the class does not support unserialisation or
	///							the data in the buffer is invalid.
	///
	virtual size_t				unserialize(void *buf, size_t bufSize);
};

#endif // AP_METACLASS_H
