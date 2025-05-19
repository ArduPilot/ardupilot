#pragma once

#include <AP_Common/AP_Param.h>

class Eagle {
public:
    Eagle();
    static const AP_Param::GroupInfo var_info[];

private:
    AP_Float eagle_f0;
    AP_Float eagle_f1;
    AP_Float eagle_f2;
    AP_Float eagle_f3;
    AP_Float eagle_f4;
    AP_Float eagle_f5;
    AP_Float eagle_f6;
    AP_Float eagle_f7;
    AP_Float eagle_f8;
    AP_Float eagle_f9;
    AP_Int32 eagle_i0;
    AP_Int32 eagle_i1;
    AP_Int32 eagle_i2;
    AP_Int32 eagle_i3;
    AP_Int32 eagle_i4;
    AP_Int32 eagle_i5;
};

// Define the parameter
const AP_Param::GroupInfo MyLibrary::var_info[] = {
    AP_GROUPINFO("MY_PARAM", 0, MyLibrary, my_param, 42),
    AP_GROUPEND
};

