#ifndef AP_EEProm_H
#define AP_EEProm_H

#include <AP_Vector.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

class AP_EEPromEntry
{
public:
	AP_EEPromEntry(const char * name = "") : _id(0), _address(0), _name(name), _sync(false)
	{
	}
	virtual void setEntry(float val) = 0;
	virtual float getEntry() = 0;
	uint16_t getId() { return _id; }
	uint16_t getAddress() { return _address; }
	const char * getName() { return _name; }
	bool setSync(bool sync) { _sync = sync; }
protected:
	uint16_t _id;
	uint16_t _address;
	const char * _name;
	bool _sync;
};

class AP_EEPromRegistry : public AP_Vector<AP_EEPromEntry *>
{
public:
	/**
	 * Default constructor
	 */

	AP_EEPromRegistry(uint16_t maxSize) :
		_lastAddress(0), _lastId(0), _maxSize(maxSize)
	{
	}

	/**
	 * Add an entry to the registry
	 */
	void add(AP_EEPromEntry * entry, uint16_t & id, uint16_t & address, size_t size)
	{
		if (_lastAddress + size > _maxSize) return;
		address = _lastAddress;
		_lastAddress += size;
		id = _lastId++;
		push_back(entry);
	}

private:
	uint16_t _lastAddress;
	uint16_t _lastId;
	uint16_t _maxSize;
};

AP_EEPromRegistry eepromRegistry(1024);

template  <class type>
class AP_EEPromVar : public AP_EEPromEntry
{
public:
	AP_EEPromVar(const char * name = "") : AP_EEPromEntry(name)
	{
		eepromRegistry.add(this,_id,_address,sizeof(_data));
	}
	void set(type val) {
		if (_sync) save();
		_data = val; 
	}
	type get() {
		if (_sync) load();
		return _data;
	}
	void save()
	{
		eeprom_write_block((const void*)&_data,(void*)(_address),sizeof(_data));
	}
	void load()
	{
		eeprom_read_block((void*)&_data,(const void*)(_address),sizeof(_data));
	}
	virtual void setEntry(float val)
	{
		_data = (type)val;
		if (_sync) save();
	}
	virtual float getEntry()
	{
		if (_sync) load();
		return (float)_data;
	}
private:
	type _data;
};

#endif
