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
class AP_VarI
{
public:

	/// Set the variable value as a float
	virtual void setF(const float & val) = 0;

	/// Get the variable as a float
	virtual const float & getF() = 0;

	/// Save a variable to eeprom
	virtual void save() = 0;

	/// Load a variable from eeprom
	virtual void load() = 0;

	/// Get the name. This is useful for ground stations.
	virtual const char * getName() = 0;

	/// If sync is true the a load will always occure before a get and a save will always
	/// occure before a set.
	virtual const bool & getSync() = 0;

	/// Set the sync property
	virtual void setSync(bool sync) = 0;
};

/// The variable template class. This class
/// implements get/set/save/load etc for the
/// abstract template type.
template  <class type>
class AP_Var : public AP_VarI
{
public:
	/// The default constrcutor
	AP_Var(const type & data, const char * name = "", const bool & sync=false) :
		_data(data), _name(name), _sync(sync)
	{
	}

	/// Set the variable value
	virtual void set(const type & val) {
		_data = val;
		if (_sync) save();
	}

	/// Get the variable value.
	virtual const type & get() {
		if (_sync) load();
		return _data;
	}

	/// Set the variable value as a float
	virtual void setF(const float & val) {
		set(val);
	}

	/// Get the variable as a float
	virtual const float & getF() {
		return get();
	}

	/// Save a variable to eeprom
	virtual void save()
	{
	}

	/// Load a variable from eeprom
	virtual void load()
	{
	}

	/// Get the name. This is useful for ground stations.
	virtual const char * getName() { return _name; }

	/// If sync is true the a load will always occure before a get and a save will always
	/// occure before a set.
	virtual const bool & getSync() { return _sync; }
	virtual void setSync(bool sync) { _sync = sync; }

protected:
	type _data; /// The data that is stored on the heap */
	const char * _name; /// The variable name, useful for gcs and terminal output
	bool _sync; /// Whether or not to call save/load on get/set
};
