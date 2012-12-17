#ifndef AP_RELAY_APM1_H_
#define AP_RELAY_APM1_H_

#include "Relay.h"

/// @class	AP_Relay_APM1
/// @brief	SubClass from Relay to manage the APM1 onboard relay


class AP_Relay_APM1: public Relay{
  public:
  
	// activate the relay
	void on();

	// de-activate the relay
	void off();

	// toggle the relay status
	void toggle();

	// set the relay status (on/off)
	void set(bool status);

	// get the relay status (on/off)
	bool get();  
};

#endif /* AP_RELAY_APM1_H_ */
