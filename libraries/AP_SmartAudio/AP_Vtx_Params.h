#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Vtx_Params
{
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Vtx_Params(void);

    /* Do not allow copies */
    AP_Vtx_Params(const AP_Vtx_Params &other) = delete;
    AP_Vtx_Params &operator=(const AP_Vtx_Params&) = delete;

    AP_Int8  type;          // SMARTAUDIO | TRAMP
    AP_Int8  mode;          // PIT | RACE | FREESTYLE
    AP_Int8  band;
    AP_Int8  channel;
    AP_Int16 frecuency;
    AP_Int8  power;
};
