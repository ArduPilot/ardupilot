#ifndef AP_WATERDETECTOR_H
#define AP_WATERDETECTOR_H

#include <AP_Param/AP_Param.h>
//#include <AP_Common/AP_Common.h>

#define WATERDETECTOR_MAX_INSTANCES 3

#define WATERDETECTOR_COOLDOWN_MS 3000 // status will return true for this long after last time water was detected

class AP_WaterDetector_Backend;

class AP_WaterDetector
{

	friend class AP_WaterDetector_Backend;

public:
	AP_WaterDetector(); // constructor

	enum WaterDetector_Type {
		WATERDETECTOR_TYPE_NONE = 0,
		WATERDETECTOR_TYPE_DIGITAL = 1,
		WATERDETECTOR_TYPE_ANALOG = 2,
		WATERDETECTOR_TYPE_MAVLINK = 3 // unused, status can be set after handling mavlink packets with set_detect
	};

	struct WaterDetector_State {
		uint8_t instance;
		bool status;
	};

	bool get_status(void) const { return _status; } // return current status
	void set_detect(void); // set status externally, ie via mavlink message from subsystems

	void init(void); // initialize all drivers
	bool update(void); // update all instances, should be called frequently by vehicle code

	AP_Int8 _type[WATERDETECTOR_MAX_INSTANCES]; // Analog, Digital, Mavlink
	AP_Int8 _pin[WATERDETECTOR_MAX_INSTANCES]; // pin that detector is connected to
	AP_Int8 _default_reading[WATERDETECTOR_MAX_INSTANCES]; // default reading when water detector is dry

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_WaterDetector_Backend *drivers[WATERDETECTOR_MAX_INSTANCES];
	WaterDetector_State state[WATERDETECTOR_MAX_INSTANCES];

	bool _status; // current status, true if water detected, false if all sensors dry
	uint32_t _last_detect_ms;

};

#endif
