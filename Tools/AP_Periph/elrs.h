#pragma once

#include "elrs_config.h"

#if AP_PERIPH_ELRS_ENABLED

#include <AP_Param/AP_Param.h>

class Parameters_ELRS
{
public:
    Parameters_ELRS();
    void convert_parameters();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Int32 uid1;
    AP_Int32 uid2;
    AP_Int16 model_id;
    AP_Int8 bind;
    AP_Int8 version;
};

#endif // AP_PERIPH_ELRS_ENABLED
