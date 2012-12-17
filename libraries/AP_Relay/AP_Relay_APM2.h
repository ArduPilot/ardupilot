#ifndef AP_RELAY_APM2_H_
#define AP_RELAY_APM2_H_

#include "Relay.h"

/// @class	AP_Relay_APM2
/// @brief	SubClass from Relay to manage the APM2 A9 pin as external relay port


class AP_Relay_APM2: public Relay{
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

#endif /* AP_RELAY_APM2_H_ */
