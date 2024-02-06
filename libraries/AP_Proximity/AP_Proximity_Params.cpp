#include "AP_Proximity_Params.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity_Params::var_info[] = {

    // 0 should not be used

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,7:LightwareSF40c,2:MAVLink,3:TeraRangerTower,4:RangeFinder,5:RPLidarA2,6:TeraRangerTowerEvo,8:LightwareSF45B,10:SITL,12:AirSimSITL,13:CygbotD1, 14:DroneCAN, 15:Scripting, 16:LD06, 17: MR72_CAN
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE",   1, AP_Proximity_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _ORIENT
    // @DisplayName: Proximity sensor orientation
    // @Description: Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("_ORIENT", 2, AP_Proximity_Params, orientation, 0),

    // @Param: _YAW_CORR
    // @DisplayName: Proximity sensor yaw correction
    // @Description: Proximity sensor yaw correction
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_YAW_CORR", 3, AP_Proximity_Params, yaw_correction, 0),

    // @Param: _IGN_ANG1
    // @DisplayName: Proximity sensor ignore angle 1
    // @Description: Proximity sensor ignore angle 1
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG1", 4, AP_Proximity_Params, ignore_angle_deg[0], 0),

    // @Param: _IGN_WID1
    // @DisplayName: Proximity sensor ignore width 1
    // @Description: Proximity sensor ignore width 1
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID1", 5, AP_Proximity_Params, ignore_width_deg[0], 0),

    // @Param: _IGN_ANG2
    // @DisplayName: Proximity sensor ignore angle 2
    // @Description: Proximity sensor ignore angle 2
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG2", 6, AP_Proximity_Params, ignore_angle_deg[1], 0),

    // @Param: _IGN_WID2
    // @DisplayName: Proximity sensor ignore width 2
    // @Description: Proximity sensor ignore width 2
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID2", 7, AP_Proximity_Params, ignore_width_deg[1], 0),

    // @Param: _IGN_ANG3
    // @DisplayName: Proximity sensor ignore angle 3
    // @Description: Proximity sensor ignore angle 3
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG3", 8, AP_Proximity_Params, ignore_angle_deg[2], 0),

    // @Param: _IGN_WID3
    // @DisplayName: Proximity sensor ignore width 3
    // @Description: Proximity sensor ignore width 3
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID3", 9, AP_Proximity_Params, ignore_width_deg[2], 0),

    // @Param: _IGN_ANG4
    // @DisplayName: Proximity sensor ignore angle 4
    // @Description: Proximity sensor ignore angle 4
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG4", 10, AP_Proximity_Params, ignore_angle_deg[3], 0),

    // @Param: _IGN_WID4
    // @DisplayName: Proximity sensor ignore width 4
    // @Description: Proximity sensor ignore width 4
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID4", 11, AP_Proximity_Params, ignore_width_deg[3], 0),

    // @Param: _MIN
    // @DisplayName: Proximity minimum range
    // @Description: Minimum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.
    // @Units: m
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("_MIN", 16, AP_Proximity_Params, min_m, 0.0f),

    // @Param: _MAX
    // @DisplayName: Proximity maximum range
    // @Description: Maximum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.
    // @Units: m
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("_MAX", 17, AP_Proximity_Params, max_m, 0.0f),

    // @Param: _ADDR
    // @DisplayName: Bus address of sensor
    // @Description: The bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ADDR", 25, AP_Proximity_Params, address, 0),

    AP_GROUPEND
};

AP_Proximity_Params::AP_Proximity_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
