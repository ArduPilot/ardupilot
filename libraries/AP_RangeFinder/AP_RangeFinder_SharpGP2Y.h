#ifndef __AP_RangeFinder_SharpGP2Y_H__
#define __AP_RangeFinder_SharpGP2Y_H__

#include "RangeFinder.h"

#define AP_RANGEFINDER_SHARPEGP2Y 0
//#define AP_RANGEFINDER_SHARPEGP2Y_SCALER 1.0
#define AP_RANGEFINDER_SHARPEGP2Y_MIN_DISTANCE 20
#define AP_RANGEFINDER_SHARPEGP2Y_MAX_DISTANCE 150

class AP_RangeFinder_SharpGP2Y : public RangeFinder
{
public:
	//AP_RangeFinder_SharpGP2Y::AP_RangeFinder_SharpGP2Y(AP_HAL::AnalogSource *source, FilterInt16 *filter) : RangeFinder(source, filter)

	AP_RangeFinder_SharpGP2Y(AP_HAL::AnalogSource *source, FilterInt16 *filter);
    
	int  convert_raw_to_distance(float _raw_value) 
	{
        if( _raw_value == 0 ) 
		{
            return max_distance;
        } else 
		{
			//y = 16.453x4 - 121.92x3 + 335.93x2 - 430.23x + 259.34

			float term1 = 16.453 * pow(_raw_value, 4);
			float term2 = 121.92 * pow(_raw_value, 3) * -1;
			float term3 = 335.93 * pow(_raw_value, 2);
			float term4 = 430.23 * _raw_value * -1;
			float term5 = 259.34;

			return term1 + term2 + term3 + term4 + term5;
			


			//return (   (75.692 * pow (_raw_value, -1.066))  );

            //return 14500/_raw_value;
        }
    }       // read value from analog port and return distance in cm

	int read()
	{
		float raw_value = _analog_source->voltage_average();
		int distance = convert_raw_to_distance(raw_value);

		return distance;
	}

};
#endif
