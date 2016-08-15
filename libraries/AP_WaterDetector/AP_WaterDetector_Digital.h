#ifndef AP_WATERDETECTOR_DIGITAL_H
#define AP_WATERDETECTOR_DIGITAL_H

#include "AP_WaterDetector.h"

class AP_WaterDetector_Digital : public AP_WaterDetector
{

public:
	AP_WaterDetector_Digital();
	void read(void);

private:
	int8_t _pin; // GPIO pin that detector is connected to
	bool _default; // default off (not wet) state

};

#endif
