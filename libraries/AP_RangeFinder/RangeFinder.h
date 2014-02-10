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
    // distance: in cm
    int16_t  distance;
    // maximum measurable distance: in cm
    int16_t  max_distance;
    // minimum measurable distance: in cm
    int16_t  min_distance;

    /**
     * convert_raw_to_distance:
     * function that each child class should override to convert voltage
     * to distance (in cm)
     */
    virtual int16_t convert_raw_to_distance(int16_t raw_value) {
        return raw_value;
    }
    /**
     * read:
     * read value from sensor and return distance in cm
     */
    virtual int16_t read();

    AP_HAL::AnalogSource*   _analog_source;
    FilterInt16 *           _mode_filter;
};
#endif // __RANGEFINDER_H__
