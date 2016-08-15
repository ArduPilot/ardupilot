#ifndef AP_WATERDETECTOR_H
#define AP_WATERDETECTOR_H
#include <AP_Param/AP_Param.h>

class AP_WaterDetector
{

public:
	AP_WaterDetector() : _status(false) { }; // constructor

	bool water_detected(void) const { return _status; } // return current status
	virtual void read(void) = 0;

protected:
	bool _status; // current status, true if water detected, false if dry

};

#endif
