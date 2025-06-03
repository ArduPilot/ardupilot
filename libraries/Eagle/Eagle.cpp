#include "Eagle.h"

const AP_Param::GroupInfo Eagle::var_info[] = {
    // @Param: _F0
    // @DisplayName: _F0
    // @Description: _F0
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F0", 0, Eagle, _f0, 0),

    // @Param: _F1
    // @DisplayName: _F1
    // @Description: _F1
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F1", 1, Eagle, _f1, 0),

    // @Param: _F2
    // @DisplayName: _F2
    // @Description: _F2
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F2", 2, Eagle, _f2, 0),

    // @Param: _F3
    // @DisplayName: _F3
    // @Description: _F3
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F3", 3, Eagle, _f3, 0),

    // @Param: _F4
    // @DisplayName: _F4
    // @Description: _F4
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F4", 4, Eagle, _f4, 0),

    // @Param: _F5
    // @DisplayName: _F5
    // @Description: _F5
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F5", 5, Eagle, _f5, 0),

    // @Param: _F6
    // @DisplayName: _F6
    // @Description: _F6
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F6", 6, Eagle, _f6, 0),

    // @Param: _F7
    // @DisplayName: _F7
    // @Description: _F7
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F7", 7, Eagle, _f7, 0),

    // @Param: _F8
    // @DisplayName: _F8
    // @Description: _F8
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F8", 8, Eagle, _f8, 0),

    // @Param: _F9
    // @DisplayName: _F9
    // @Description: _F9
    // @Range: -100 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_F9", 9, Eagle, _f9, 0),

    // @Param: _I0
    // @DisplayName: _I0
    // @Description: _I0
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("_I0", 10, Eagle, _i0, 0),

    // @Param: _I1
    // @DisplayName: _I1
    // @Description: _I1
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("_I1", 11, Eagle, _i1, 0),

    // @Param: _I2
    // @DisplayName: _I2
    // @Description: _I2
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("_I2", 12, Eagle, _i2, 0),

    // @Param: _I3
    // @DisplayName: _I3
    // @Description: _I3
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("_I3", 13, Eagle, _i3, 0),

    // @Param: _I4
    // @DisplayName: _I4
    // @Description: _I4
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("_I4", 14, Eagle, _i4, 0),

    // @Param: _I5
    // @DisplayName: _I5
    // @Description: _I5
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("_I5", 15, Eagle, _i5, 0),

    AP_GROUPEND
};
