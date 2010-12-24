#ifndef AP_EEProm_H
#define AP_EEProm_H

/*
 * AP_EEProm.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Vector.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

/**
 * The interface for data entries in the eeprom registry
 */
class AP_EEPromEntry
{
public:
	/**
	 * Default constructor
	 */
	AP_EEPromEntry(const char * name = "") : _id(0), _address(0), _name(name), _sync(false)
	{
	}

	/**
	 * Note: The set/get entry functions are virtual and therefore require more machine code, so use set/get in 
	 * AP_EEPromVar unless you are accessing via the registry.
	 */
	
	/**
	 * Pure virtual function for setting the data value as a float. The function must handle the cast to 
	 * the stored variable types.
	 */
	virtual void setEntry(float val) = 0;

	/**
	 * Pure virtual function for getting data as a float. The function must handle the cast from the
	 * stored variable types.
	 */
	virtual float getEntry() = 0;

	/**
	 * Get the id of the variable.
	 */
	uint16_t getId() { return _id; }

	/**
	 * Get the address of the variable.
	 */
	uint16_t getAddress() { return _address; }

	/**
	 * Get the name. This is useful for ground stations.
	 */
	const char * getName() { return _name; }

	/**
	 * If sync is true the a load will always occure before a get and a save will always
	 * occure before a set.
	 */
	bool setSync(bool sync) { _sync = sync; }

protected:

	uint16_t _id; /** Variable identifier */ 
	uint16_t _address;  /** EEProm address of variable */
	const char * _name; /** The name of the variable, a string */
	bool _sync; /** true: save/load before get/set, false: don't call save/load automatically*/
};

/**
 *  The main EEProm Registry class.
 */
class AP_EEPromRegistry : public Vector<AP_EEPromEntry *>
{
public:

	/**
	 * Default constructor
	 */
	AP_EEPromRegistry(uint16_t maxSize) :
		_newAddress(0), _newId(0), _maxSize(maxSize)
	{
	}

	/**
	 * Add an entry to the registry
	 */
	void add(AP_EEPromEntry * entry, uint16_t & id, uint16_t & address, size_t size);

private:
	uint16_t _newAddress; /** the address for the next new variable*/
	uint16_t _newId; /** the id of the next new variable*/
	uint16_t _maxSize; /** the maximum size of the eeprom memory*/
};

/**
 * Global eepromRegistry declaration.
 */
extern AP_EEPromRegistry eepromRegistry;

/**
 * The EEProm Variable template class. This class
 * implements get/set/save/load etc for the 
 * abstract template type.
 */
template  <class type>
class AP_EEPromVar : public AP_EEPromEntry
{
public:
	/**
	 * The default constrcutor
	 */
	AP_EEPromVar(const char * name = "") : AP_EEPromEntry(name)
	{
		eepromRegistry.add(this,_id,_address,sizeof(_data));
	}

	/**
	 * Non-virtual set used for efficient variable setting without
	 * using eepromRegistry
	 */
	void set(type val) {
		_data = val; 
		if (_sync) save();
	}

	/**
	 * Non-virtual get used for efficient getting of a variable
	 * without using eepromRegistry
	 */
	type get() {
		if (_sync) load();
		return _data;
	}

	/**
	 * Save a variable to eeprom
	 */
	void save()
	{
		eeprom_write_block((const void*)&_data,(void*)(_address),sizeof(_data));
	}

	/**
	 * Load a variable from eeprom
	 */
	void load()
	{
		eeprom_read_block((void*)&_data,(const void*)(_address),sizeof(_data));
	}

	/**
	 * Virtual call that allows an type of variable to be set by a float.
	 * This requires casting and will require some thought if initializing a 
	 * boolean etc. Initializing a boolean to false, pass (0.0), true, pass (1.0) etc..
	 */
	virtual void setEntry(float val)
	{
		_data = (type)val;
		if (_sync) save();
	}

	/**
	 * Virtual call that retrieves any variable types as a float.
	 */
	virtual float getEntry()
	{
		if (_sync) load();
		return (float)_data;
	}

private:
	type _data; /** The data that is stored on the heap */
};

#endif
