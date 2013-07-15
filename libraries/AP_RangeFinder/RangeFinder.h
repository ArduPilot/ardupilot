#ifndef __RANGEFINDER_H__
#define __RANGEFINDER_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <Filter.h> // Filter library

/*
 * #define AP_RANGEFINDER_ORIENTATION_FRONT		  0, 10,  0
 * #define AP_RANGEFINDER_ORIENTATION_RIGHT		-10,  0,  0
 * #define AP_RANGEFINDER_ORIENTATION_BACK			  0,-10,  0
 * #define AP_RANGEFINDER_ORIENTATION_LEFT			 10,  0,  0
 * #define AP_RANGEFINDER_ORIENTATION_UP			  0,  0,-10
 * #define AP_RANGEFINDER_ORIENTATION_DOWN			  0,  0, 10
 * #define AP_RANGEFINDER_ORIENTATION_FRONT_RIGHT    -5, -5,  0
 * #define AP_RANGEFINDER_ORIENTATION_BACK_RIGHT     -5, -5,  0
 * #define AP_RANGEFINDER_ORIENTATION_BACK_LEFT       5, -5,  0
 * #define AP_RANGEFINDER_ORIENTATION_FRONT_LEFT      5,  5,  0
 */

class RangeFinder
{
protected:
    RangeFinder(AP_HAL::AnalogSource * source, FilterInt16 *filter) :
        _analog_source(source),
        _mode_filter(filter) {
    }
public:
    // raw_value: read the sensor
    int  raw_value;
    // distance: in cm
    int  distance;
    // maximum measurable distance: in cm
    int  max_distance;
    // minimum measurable distance: in cm
    int  min_distance;

    int  orientation_x, orientation_y, orientation_z;
    void set_orientation(int x, int y, int z);

    /**
     * convert_raw_to_distance:
     * function that each child class should override to convert voltage
     * to distance (in cm)
     */
    virtual int convert_raw_to_distance(int _raw_value) {
        return _raw_value;
    }
    /**
     * read:
     * read value from sensor and return distance in cm
     */
    virtual int read();

    AP_HAL::AnalogSource*       _analog_source;
    FilterInt16 *           _mode_filter;
};
#endif // __RANGEFINDER_H__
