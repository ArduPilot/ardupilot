#pragma once

#include <AP_Param/AP_Param.h>

class AP_Mixing {
public:
    AP_Mixing(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // Individual mix enable parameter
    AP_Int8 individual_mix_enable;
    
    // V-tail mixing parameters
    AP_Float vtail_mgain;
    AP_Int16 vtail_moffset;
    
    // Elevon mixing parameters  
    AP_Float elevon_mgain;
    AP_Int16 elevon_moffset;
}; 