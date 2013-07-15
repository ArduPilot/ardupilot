#ifndef __AP_RangeFinder_MaxsonarXL_H__
#define __AP_RangeFinder_MaxsonarXL_H__

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

// HRLV-MaxSonar-EZ0 (aka HRLV)
#define AP_RANGEFINDER_MAXSONARHRLV 3
#define AP_RANGEFINDER_MAXSONARHRLV_SCALER 0.512
#define AP_RANGEFINDER_MAXSONARHRLV_MIN_DISTANCE 30
#define AP_RANGEFINDER_MAXSONARHRLV_MAX_DISTANCE 500

class AP_RangeFinder_MaxsonarXL : public RangeFinder
{
public:
    AP_RangeFinder_MaxsonarXL(AP_HAL::AnalogSource *source, FilterInt16 *filter);
    int             convert_raw_to_distance(int _raw_value) {
        return _raw_value * _scaler;
    }                                                                                              // read value from analog port and return distance in cm
    float           calculate_scaler(int sonar_type, float adc_refence_voltage);

private:
    float           _scaler; // used to account for different sonar types
};
#endif
