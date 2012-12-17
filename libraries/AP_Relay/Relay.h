#ifndef RELAY_H_
#define RELAY_H_

/// @class	Relay
/// @brief	Abstract base class for Relays on all APM hardwares

class Relay
{
  public:
  
	// activate the relay
	virtual void on() = 0;

	// de-activate the relay
	virtual void off() = 0;

	// toggle the relay status
	virtual void toggle() = 0;

	// set the relay status (on/off)
	virtual void set(bool status) = 0;

	// get the relay status (on/off)
	virtual bool get() = 0;  
};

#endif /* RELAY_H_ */