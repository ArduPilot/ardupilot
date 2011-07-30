#ifndef RangeFinder_h
#define RangeFinder_h

#include <stdlib.h>
#include <inttypes.h>
#include "../AP_ADC/AP_ADC.h"
#include "../ModeFilter/ModeFilter.h" // ArduPilot Mega RC Library

/*
#define AP_RANGEFINDER_ORIENTATION_FRONT		  0, 10,  0
#define AP_RANGEFINDER_ORIENTATION_RIGHT		-10,  0,  0
#define AP_RANGEFINDER_ORIENTATION_BACK			  0,-10,  0
#define AP_RANGEFINDER_ORIENTATION_LEFT			 10,  0,  0
#define AP_RANGEFINDER_ORIENTATION_UP			  0,  0,-10
#define AP_RANGEFINDER_ORIENTATION_DOWN			  0,  0, 10
#define AP_RANGEFINDER_ORIENTATION_FRONT_RIGHT 	 -5, -5,  0
#define AP_RANGEFINDER_ORIENTATION_BACK_RIGHT 	 -5, -5,  0
#define AP_RANGEFINDER_ORIENTATION_BACK_LEFT 	  5, -5,  0
#define AP_RANGEFINDER_ORIENTATION_FRONT_LEFT 	  5,  5,  0
*/
//#define AP_RANGEFINDER_PITOT_TUBE 1007
#define AP_RANGEFINDER_PITOT_TUBE_ADC_CHANNEL 7

//#define AP_RANGEFINDER_NUM_AVERAGES 4

class RangeFinder
{
  protected:
	//GPS(Stream *s) : _port(s) {};
	RangeFinder(AP_ADC *adc, ModeFilter *filter) :
		_ap_adc(adc),
		_mode_filter(filter),
		_analogPort(-1)
	{}
  public:

	//int _history[AP_RANGEFINDER_NUM_AVERAGES]; // history of recent distances used for filtering
	//int _num_averages; // filter will return average of this many historic values (must be < AP_RANGEFINDER_NUM_AVERAGES)
	//int _history_ptr;  // pointer to the most recent entry in the history table

	int raw_value;     // raw value from the sensor
	int distance;      // distance in cm
	int max_distance;  // maximum measurable distance (in cm) - should be set in child's constructor
	int min_distance;  // minimum measurable distance (in cm) - should be set in child's constructor
	int orientation_x, orientation_y, orientation_z;

	virtual void set_analog_port(int analogPort);
    virtual void set_orientation(int x, int y, int z);
	//virtual void set_filter(int num_averages) { _num_averages = num_averages; } // allows control of amount of filtering done
	virtual int convert_raw_to_distance(int _raw_value) { return _raw_value; }  // function that each child class should override to convert voltage to distance
	virtual int read();   // read value from sensor and return distance in cm

	int _analogPort;   // the port to which the sensor is connected
	AP_ADC 		*_ap_adc;   // pointer to AP_ADC used for pitot tube
  	ModeFilter  *_mode_filter;
};
#endif
