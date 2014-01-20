/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BOARDCONFIG_H__
#define __AP_BOARDCONFIG_H__

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Param.h>

class AP_BoardConfig
{
public:
    // constructor
    AP_BoardConfig(void)
    {
		AP_Param::setup_object_defaults(this, var_info);
    };

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _pwm_count;
};

#endif // __AP_BOARDCONFIG_H__


