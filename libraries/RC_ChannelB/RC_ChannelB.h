// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_ChannelB.h
/// @brief	RC_ChannelB manager, with EEPROM-backed storage of constants.

#ifndef RC_ChannelB_h
#define RC_ChannelB_h

#include <stdint.h>
#include <AP_EEProm.h>

/// @class	RC_ChannelB
/// @brief	Object managing one RC channel
class RC_ChannelB{
 
public:	

	enum storage_t
	{
		STORE_EXTERNAL,
		STORE_EEPROM
	};

	/// Constructor
	///
	RC_ChannelB() :
		_scale(0),
		_pwmMin(1000),
		_pwmMax(2000),
		_pwmNeutral(1500),
		_pwmDeadZone(100),
		_pwm(0),
		_pwmRadio(0),
		_filter(true),
		_reverse(false),
		_address(0),
		_storageType(STORE_EXTERNAL)
	{
	}

	/// Constructor
	///
	RC_ChannelB() :
		_scale(0),
		_pwmMin(1000),
		_pwmNeutral(1500),
		_pwmMax(2000),
		_pwmDeadZone(100),
		_pwm(0),
		_pwmRadio(0),
		_filter(true),
		_reverse(false),
		_address(address),
		_storageType(STORE_EEPROM)
	{
	}

	/// Constructor
	///
	RC_ChannelB(uint16_t address) :
		_scale(0),
		_pwmMin(1000),
		_pwmNeutral(1500),
		_pwmMax(2000),
		_pwmDeadZone(100),
		_pwm(0),
		_pwmRadio(0),
		_filter(true),
		_reverse(false),
		_address(address),
		_storageType(STORE_EEPROM)
	{
	}

	// set configuration
	void setScale(float scale) { _scale = scale; }
	void setReverse(bool reverse) { _reverse = reverse; }
	void setFilter(bool filter) { _filter = filter; }
	void setPwmMin(uint16_t pwmMin) { _pwmMin = pwmMin; }
	void setPwmMax(uint16_t pwmMax) { _pwmMax = pwmMax; }
	void setPwmNeutral(uint16_t pwmNeutral) { _pwmNeutral = pwmNeutral; }
	void setPwmNeutral() { _pwmNeutral = getPwm(); }
	void setPwmDeadZone(uint16_t pwmDeadZone) { _pwmDeadZone = pwmDeadZone; }

	// save/load
	void saveEEProm();
	void loadEEProm();

	// get configuration
	float getScale() { return _scale; }
	bool getReverse() { return _reverse; }
	bool getFilter() { return _filter; }
	uint16_t getPwmMin() { return _pwmMin; }
	uint16_t getPwmMax() { return _pwmMax; }
	uint16_t getPwmNeutral() { return _pwmNeutral; }
	uint16_t getPwmDeadZone() { return _pwmDeadZone; }

	// set servo state
	void readRadio(uint16_t pwmRadio);
	void setPwm(uint16_t pwm);
	void setPosition(float position);
	void mixRadio(uint16_t infStart);

	// get servo state
	uint16_t getPwm() { return _pwm; }
	uint16_t getPwmRadio() { return _pwmRadio; }
	float getPosition() { return _pwmToPosition(_pwm); }
	float getNormalized() { return getPosition()/_scale; }
	
	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (_pwmMin - 50); }

private:

	float _scale;
	AP_EEProm<uint16_t> _pwmMin;
	uint16_t _pwmNeutral;
	uint16_t _pwmMax;
	uint16_t _pwmDeadZone;
	uint16_t _pwm; // this is the internal state, positino is just created when needed
	uint16_t _pwmRadio; // radio pwm input
	
	// configuration
	bool _filter;
	int8_t _reverse;
	storage_t _storageType;
	uint16_t _address;

	// private methods
	uint16_t _positionToPwm(float position);
	float _pwmToPosition(uint16_t pwm);
};

#endif	




