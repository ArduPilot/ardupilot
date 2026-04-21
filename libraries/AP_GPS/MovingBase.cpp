#include "MovingBase.h"

const AP_Param::GroupInfo MovingBase::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Moving base type
    // @Description: Controls the type of moving base used if using moving base.This is renamed in 4.6 and later to GPSx_MB_TYPE.
    // @Values: 0:Relative to alternate GPS instance,1:RelativeToCustomBase
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, MovingBase, type, int8_t(Type::RelativeToAlternateInstance), AP_PARAM_FLAG_ENABLE),

    // @Param: OFS_X
    // @DisplayName: Base antenna X position offset
    // @Description: X position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive X is forward of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_X.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    
    // @Param: OFS_Y
    // @DisplayName: Base antenna Y position offset    
    // @Description: Y position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Y is to the right of the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Y.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    
    // @Param: OFS_Z
    // @DisplayName: Base antenna Z position offset
    // @Description: Z position of the base (primary) GPS antenna in body frame from the position of the 2nd antenna. Positive Z is down from the 2nd antenna. Use antenna phase centroid location if provided by the manufacturer.This is renamed in 4.6 and later to GPSx_MB_OFS_Z.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("OFS", 2, MovingBase, base_offset, 0.0f),

    AP_GROUPEND

};

MovingBase::MovingBase(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
