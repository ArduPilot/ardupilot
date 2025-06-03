#pragma once

/// @file   Eagle.h
/// @brief  Eagle

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

// Add this class as a variable to classes that would like to expose its parameters.
// Example of this is in ArduCopter.
// It's important to add this library as a dependency in `wscript`.

/// @class  Eagle
/// @brief  Eagle
class Eagle{
public:
    Eagle() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(Eagle);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Useful for adding new parameters:
    // https://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html#adding-a-parameter-to-a-library

    // Tested that 64 parameters fit with idx in range 0-63 (more parameters cause quiet runtime errors)

    AP_Float _f0;
    AP_Float _f1;
    AP_Float _f2;
    AP_Float _f3;
    AP_Float _f4;
    AP_Float _f5;
    AP_Float _f6;
    AP_Float _f7;
    AP_Float _f8;
    AP_Float _f9;
    AP_Int32 _i0;
    AP_Int32 _i1;
    AP_Int32 _i2;
    AP_Int32 _i3;
    AP_Int32 _i4;
    AP_Int32 _i5;
};
