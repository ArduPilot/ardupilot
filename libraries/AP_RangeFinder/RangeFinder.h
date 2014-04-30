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

 /**
   maximum number of range finder instances available on this platform. If more
   than 1 then redundant sensors may be available
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define RANGEFINDER_MAX_INSTANCES 2
#else
#define RANGEFINDER_MAX_INSTANCES 1
#endif
 
class RangeFinder
{
protected:
    RangeFinder(AP_HAL::AnalogSource * source, FilterInt16 *filter) :
        _analog_source(source),
        _mode_filter(filter) {
    }
    
    virtual uint8_t _get_primary(void) const { return 0; }
    bool _healthy[RANGEFINDER_MAX_INSTANCES];
    int16_t _distance[RANGEFINDER_MAX_INSTANCES];  // distance: in cm
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
    
    /// Return the number of range finder instances
    virtual uint8_t get_count(void) const { return 1; }

    /// Return the current distance as a int16_t
    int16_t get_distance(uint8_t i) const { return _distance[i]; }
    int16_t get_distance(void) const { return get_distance(_get_primary()); }
    
    /// Return the health of a range finder
    bool healthy(uint8_t i) const { return _healthy[i]; }
    bool healthy(void) const { return healthy(_get_primary()); }
};
#endif // __RANGEFINDER_H__
