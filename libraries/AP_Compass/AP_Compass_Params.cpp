#include "AP_Compass_Params.h"
#include "AP_Compass.h"

const AP_Param::GroupInfo AP_Compass_Params::var_info[] = {

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    1, AP_Compass_Params, use_for_yaw, 1), // true if used for DCM yaw

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 2, AP_Compass_Params, orientation, ROTATION_NONE),

    // @Param: EXTERN
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on most boards. Set to 1 if the compass is externally connected. When externally connected the COMPASSx_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN", 3, AP_Compass_Params, external, 0),

    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  4, AP_Compass_Params, dev_id, 0),

    // @Param: OFS_X
    // @DisplayName: Compass offsets in milligauss on the X axis
    // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: OFS_Y
    // @DisplayName: Compass offsets in milligauss on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: OFS_Z
    // @DisplayName: Compass offsets in milligauss on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS",    5, AP_Compass_Params, offset, 0),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: DIA_X
    // @DisplayName: Compass soft-iron diagonal X component
    // @Description: DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: DIA_Y
    // @DisplayName: Compass soft-iron diagonal Y component
    // @Description: DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: DIA_Z
    // @DisplayName: Compass soft-iron diagonal Z component
    // @Description: DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA",    6, AP_Compass_Params, diagonals, 0),

    // @Param: ODI_X
    // @DisplayName: Compass soft-iron off-diagonal X component
    // @Description: ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: ODI_Y
    // @DisplayName: Compass soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: ODI_Z
    // @DisplayName: Compass soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI",    7, AP_Compass_Params, offdiagonals, 0),

    // @Param: SCALE
    // @DisplayName: Compass1 scale factor
    // @Description: Scaling factor for first compass to compensate for sensor scaling errors. If this is 0 then no scaling is done
    // @User: Standard
    // @Range: 0 1.3
    AP_GROUPINFO("SCALE", 8, AP_Compass_Params, scale_factor, 0),
#endif // HAL_BUILD_AP_PERIPH

#if COMPASS_MOT_ENABLED
    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT",    9, AP_Compass_Params, motor_compensation, 0),
#endif

    AP_GROUPEND
};

AP_Compass_Params::AP_Compass_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
