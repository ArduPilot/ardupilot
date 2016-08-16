#ifndef AP_WATERDETECTOR_H
#define AP_WATERDETECTOR_H

#include <AP_Param/AP_Param.h>

class AP_WaterDetector
{

public:
	AP_WaterDetector(); // constructor

	bool get_status(void) const { return _status; } // return current status
	void set_status(bool status) { _status = status; } // set status externally, ie via mavlink message from subsystems
	void read(void);

	static const struct AP_Param::GroupInfo var_info[];

protected:
	bool _status; // current status, true if water detected, false if dry
	AP_Int8 _pin; // pin that detector is connected to
	AP_Int8 _default_reading; // default reading when water detector is dry

};

#endif
