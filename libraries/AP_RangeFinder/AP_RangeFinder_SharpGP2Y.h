#ifndef __AP_RangeFinder_SharpGP2Y_H__
#define __AP_RangeFinder_SharpGP2Y_H__

#include "RangeFinder.h"

#define AP_RANGEFINDER_SHARPEGP2Y_MIN_DISTANCE 20
#define AP_RANGEFINDER_SHARPEGP2Y_MAX_DISTANCE 150

class AP_RangeFinder_SharpGP2Y : public RangeFinder
{
public:
    AP_RangeFinder_SharpGP2Y(AP_HAL::AnalogSource *source, FilterInt16 *filter);
    int16_t convert_raw_to_distance(int16_t raw_value) {
        if (raw_value == 0) {
            return max_distance;
        } else {
            return 14500/raw_value;
        }
    }       // read value from analog port and return distance in cm

};
#endif
