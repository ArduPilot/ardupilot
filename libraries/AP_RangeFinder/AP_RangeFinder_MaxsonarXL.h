#ifndef AP_RangeFinder_MaxsonarXL_H
#define AP_RangeFinder_MaxsonarXL_H

#include "RangeFinder.h"

// XL-EZ0 (aka XL)
#define AP_RANGEFINDER_MAXSONARXL 0
#define AP_RANGEFINDER_MAXSONARXL_SCALER 1.0
#define AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE 20
#define AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE 765

// LV-EZ0 (aka LV)
#define AP_RANGEFINDER_MAXSONARLV 1
#define AP_RANGEFINDER_MAXSONARLV_SCALER (2.54/2.0)
#define AP_RANGEFINDER_MAXSONARLV_MIN_DISTANCE 15
#define AP_RANGEFINDER_MAXSONARLV_MAX_DISTANCE 645

// XL-EZL0 (aka XLL)
#define AP_RANGEFINDER_MAXSONARXLL 2
#define AP_RANGEFINDER_MAXSONARXLL_SCALER 2.0
#define AP_RANGEFINDER_MAXSONARXLL_MIN_DISTANCE 20
#define AP_RANGEFINDER_MAXSONARXLL_MAX_DISTANCE 1068

class AP_RangeFinder_MaxsonarXL : public RangeFinder
{
  public:
	AP_RangeFinder_MaxsonarXL(AP_AnalogSource *source, ModeFilter *filter);
	int convert_raw_to_distance(int _raw_value) { return _raw_value * _scaler; }   // read value from analog port and return distance in cm
	float calculate_scaler(int sonar_type, float adc_refence_voltage); 
  private:
    float _scaler;  // used to account for different sonar types
};
#endif
