#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_RangeFinder_Params {
public:
	static const struct AP_Param::GroupInfo var_info[];
	
	AP_RangeFinder_Params(void);
	
    /* Do not allow copies */
    AP_RangeFinder_Params(const AP_RangeFinder_Params &other) = delete;
    AP_RangeFinder_Params &operator=(const AP_RangeFinder_Params&) = delete;
    
//private:
    AP_Int8  type;
    AP_Int8  pin;
    AP_Int8  ratiometric;
    AP_Int8  stop_pin;
    AP_Int16 settle_time_ms;
    AP_Float scaling;
    AP_Float offset;
    AP_Int8  function;
    AP_Int16 min_distance_cm;
    AP_Int16 max_distance_cm;
    AP_Int8  ground_clearance_cm;
    AP_Int8  address;
    AP_Vector3f pos_offset; // position offset in body frame
    AP_Int8  orientation;
};
