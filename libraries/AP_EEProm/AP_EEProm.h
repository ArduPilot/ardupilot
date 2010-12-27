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

#include <AP_Common.h>
#include <AP_Vector.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

/// The interface for data entries in the eeprom registry
class AP_EEPromEntry
{
public:
	/// Pure virtual function for setting the data value 
	/// as a float. The function must handle the cast to 
	/// the stored variable types.
	virtual void setEntry(float val) = 0;

	/// Pure virtual function for getting data as a float. 
	/// The function must handle the cast from the
	/// stored variable types.
	virtual float getEntry() = 0;

	/// Pure virtual function for getting entry name.
	virtual const char * getEntryName() = 0;

	/// Get the id of the variable.
	virtual uint16_t getEntryId() = 0;

	/// Get the address of the variable.
	virtual uint16_t getEntryAddress() = 0;
};

///The main EEProm Registry class.
class AP_EEPromRegistry : public Vector<AP_EEPromEntry *>
{
public:

	/// Default constructor
	AP_EEPromRegistry(uint16_t maxSize) :
		_newAddress(0), _newId(0), _maxSize(maxSize)
	{
	}

	/// Add an entry to the registry
	void add(AP_EEPromEntry * entry, uint16_t & id, uint16_t & address, size_t size);

private:
	uint16_t _newAddress; /// the address for the next new variable
	uint16_t _newId; /// the id of the next new variable
	uint16_t _maxSize; /// the maximum size of the eeprom memory
};

/// Global eepromRegistry declaration.
extern AP_EEPromRegistry eepromRegistry;

/// The EEProm Variable template class.
/// This class implements get/set/save/load etc for the 
/// abstract template type.
template  <class type>
class AP_EEPromVar : public AP_EEPromEntry, public AP_Var<type>
{
public:
	/// The default constrcutor
	AP_EEPromVar(type data = 0, const char * name = "", bool sync=false) : 
		AP_Var<type>(data,name,sync)
	{
		eepromRegistry.add(this,_id,_address,sizeof(type));
	}

	virtual void setEntry(float val) { this->setAsFloat(val); }
	virtual float getEntry() { return this->getAsFloat(); }
	virtual const char * getEntryName() { return this->getName(); }

	/// Get the id of the variable.
	virtual uint16_t getEntryId() { return _id; }

	/// Get the address of the variable.
	virtual uint16_t getEntryAddress() { return _address; }

private:
	uint16_t _id; /// Variable identifier
	uint16_t _address;  /// EEProm address of variable
};

#endif
