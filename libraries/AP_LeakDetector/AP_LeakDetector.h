#ifndef AP_LEAKDETECTOR_H
#define AP_LEAKDETECTOR_H

#include <AP_Param/AP_Param.h>

#define LEAKDETECTOR_MAX_INSTANCES 3

#define LEAKDETECTOR_COOLDOWN_MS 3000 // status will return true for this long after last time leak was detected

class AP_LeakDetector_Backend;

class AP_LeakDetector
{

	friend class AP_LeakDetector_Backend;

public:
	AP_LeakDetector(); // constructor

	enum LeakDetector_Type {
		LEAKDETECTOR_TYPE_NONE = 0,
		LEAKDETECTOR_TYPE_DIGITAL = 1,
		LEAKDETECTOR_TYPE_ANALOG = 2,
		LEAKDETECTOR_TYPE_MAVLINK = 3 // unused, status can be set after handling mavlink packets with set_detect
	};

	struct LeakDetector_State {
		uint8_t instance;
		bool status;
	};

	bool get_status(void) const { return _status; } // return current status
	void set_detect(void); // set status externally, ie via mavlink message from subsystems

	void init(void); // initialize all drivers
	bool update(void); // update all instances, should be called frequently by vehicle code

	AP_Int8 _type[LEAKDETECTOR_MAX_INSTANCES]; // Analog, Digital, Mavlink
	AP_Int8 _pin[LEAKDETECTOR_MAX_INSTANCES]; // pin that detector is connected to
	AP_Int8 _default_reading[LEAKDETECTOR_MAX_INSTANCES]; // default reading when leak detector is dry

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_LeakDetector_Backend *drivers[LEAKDETECTOR_MAX_INSTANCES];
	LeakDetector_State state[LEAKDETECTOR_MAX_INSTANCES];

	bool _status; // current status, true if leak detected, false if all sensors dry
	uint32_t _last_detect_ms;

};

#endif
