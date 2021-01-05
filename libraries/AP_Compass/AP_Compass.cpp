#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/I2CDevice.h>
#endif
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#include "AP_Compass_SITL.h"
#include "AP_Compass_AK8963.h"
#include "AP_Compass_Backend.h"
#include "AP_Compass_BMM150.h"
#include "AP_Compass_HIL.h"
#include "AP_Compass_HMC5843.h"
#include "AP_Compass_IST8308.h"
#include "AP_Compass_IST8310.h"
#include "AP_Compass_LSM303D.h"
#include "AP_Compass_LSM9DS1.h"
#include "AP_Compass_LIS3MDL.h"
#include "AP_Compass_AK09916.h"
#include "AP_Compass_QMC5883L.h"
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_Compass_UAVCAN.h"
#endif
#include "AP_Compass_MMC3416.h"
#include "AP_Compass_MAG3110.h"
#include "AP_Compass_RM3100.h"
#if HAL_MSP_COMPASS_ENABLED
#include "AP_Compass_MSP.h"
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
#include "AP_Compass_ExternalAHRS.h"
#endif
#include "AP_Compass.h"
#include "Compass_learn.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#ifndef COMPASS_LEARN_DEFAULT
#define COMPASS_LEARN_DEFAULT Compass::LEARN_NONE
#endif

#ifndef AP_COMPASS_OFFSETS_MAX_DEFAULT
#define AP_COMPASS_OFFSETS_MAX_DEFAULT 1800
#endif

#ifndef HAL_COMPASS_FILTER_DEFAULT
#define HAL_COMPASS_FILTER_DEFAULT 0 // turned off by default
#endif

#ifndef HAL_COMPASS_AUTO_ROT_DEFAULT
#define HAL_COMPASS_AUTO_ROT_DEFAULT 2
#endif

const AP_Param::GroupInfo Compass::var_info[] = {
    // index 0 was used for the old orientation matrix

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
    AP_GROUPINFO("OFS",    1, Compass, _state._priv_instance[0].offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: rad
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),

#if COMPASS_LEARN_ENABLED
    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle. If InFlight learning is enabled then the compass with automatically start learning once a flight starts (must be armed). While InFlight learning is running you cannot use position control modes.
    // @Values: 0:Disabled,1:Internal-Learning,2:EKF-Learning,3:InFlight-Learning
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, COMPASS_LEARN_DEFAULT),
#endif

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, Compass, _use_for_yaw._priv_instance[0], 1), // true if used for DCM yaw

    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, Compass, _auto_declination, 1),

#if COMPASS_MOT_ENABLED
    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("MOTCT",    6, Compass, _motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

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
    AP_GROUPINFO("MOT",    7, Compass, _state._priv_instance[0].motor_compensation, 0),
#endif

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315,100:Custom
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 8, Compass, _state._priv_instance[0].orientation, ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, Compass, _state._priv_instance[0].external, 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: OFS2_X
    // @DisplayName: Compass2 offsets in milligauss on the X axis
    // @Description: Offset to be added to compass2's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: OFS2_Y
    // @DisplayName: Compass2 offsets in milligauss on the Y axis
    // @Description: Offset to be added to compass2's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: OFS2_Z
    // @DisplayName: Compass2 offsets in milligauss on the Z axis
    // @Description: Offset to be added to compass2's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS2",    10, Compass, _state._priv_instance[1].offset, 0),

    // @Param: MOT2_X
    // @DisplayName: Motor interference compensation to compass2 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: MOT2_Y
    // @DisplayName: Motor interference compensation to compass2 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: MOT2_Z
    // @DisplayName: Motor interference compensation to compass2 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT2",    11, Compass, _state._priv_instance[1].motor_compensation, 0),

#endif // COMPASS_MAX_INSTANCES

#if COMPASS_MAX_INSTANCES > 2
    // @Param: OFS3_X
    // @DisplayName: Compass3 offsets in milligauss on the X axis
    // @Description: Offset to be added to compass3's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: OFS3_Y
    // @DisplayName: Compass3 offsets in milligauss on the Y axis
    // @Description: Offset to be added to compass3's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: OFS3_Z
    // @DisplayName: Compass3 offsets in milligauss on the Z axis
    // @Description: Offset to be added to compass3's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS3",    13, Compass, _state._priv_instance[2].offset, 0),

    // @Param: MOT3_X
    // @DisplayName: Motor interference compensation to compass3 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: MOT3_Y
    // @DisplayName: Motor interference compensation to compass3 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    // @Calibration: 1

    // @Param: MOT3_Z
    // @DisplayName: Motor interference compensation to compass3 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT3",    14, Compass, _state._priv_instance[2].motor_compensation, 0),
#endif // COMPASS_MAX_INSTANCES

    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  15, Compass, _state._priv_instance[0].dev_id, 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: DEV_ID2
    // @DisplayName: Compass2 device id
    // @Description: Second compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID2", 16, Compass, _state._priv_instance[1].dev_id, 0),
#endif // COMPASS_MAX_INSTANCES

#if COMPASS_MAX_INSTANCES > 2
    // @Param: DEV_ID3
    // @DisplayName: Compass3 device id
    // @Description: Third compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID3", 17, Compass, _state._priv_instance[2].dev_id, 0),
#endif // COMPASS_MAX_INSTANCES

#if COMPASS_MAX_INSTANCES > 1
    // @Param: USE2
    // @DisplayName: Compass2 used for yaw
    // @Description: Enable or disable the secondary compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2",    18, Compass, _use_for_yaw._priv_instance[1], 1),

    // @Param: ORIENT2
    // @DisplayName: Compass2 orientation
    // @Description: The orientation of a second external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315,100:Custom
    // @User: Advanced
    AP_GROUPINFO("ORIENT2", 19, Compass, _state._priv_instance[1].orientation, ROTATION_NONE),

    // @Param: EXTERN2
    // @DisplayName: Compass2 is attached via an external cable
    // @Description: Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN2",20, Compass, _state._priv_instance[1].external, 0),
#endif // COMPASS_MAX_INSTANCES

#if COMPASS_MAX_INSTANCES > 2
    // @Param: USE3
    // @DisplayName: Compass3 used for yaw
    // @Description: Enable or disable the tertiary compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3",    21, Compass, _use_for_yaw._priv_instance[2], 1),

    // @Param: ORIENT3
    // @DisplayName: Compass3 orientation
    // @Description: The orientation of a third external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315,100:Custom
    // @User: Advanced
    AP_GROUPINFO("ORIENT3", 22, Compass, _state._priv_instance[2].orientation, ROTATION_NONE),

    // @Param: EXTERN3
    // @DisplayName: Compass3 is attached via an external cable
    // @Description: Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN3",23, Compass, _state._priv_instance[2].external, 0),
#endif // COMPASS_MAX_INSTANCES

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
    AP_GROUPINFO("DIA",    24, Compass, _state._priv_instance[0].diagonals, 0),

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
    AP_GROUPINFO("ODI",    25, Compass, _state._priv_instance[0].offdiagonals, 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: DIA2_X
    // @DisplayName: Compass2 soft-iron diagonal X component
    // @Description: DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: DIA2_Y
    // @DisplayName: Compass2 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: DIA2_Z
    // @DisplayName: Compass2 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA2",    26, Compass, _state._priv_instance[1].diagonals, 0),

    // @Param: ODI2_X
    // @DisplayName: Compass2 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: ODI2_Y
    // @DisplayName: Compass2 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: ODI2_Z
    // @DisplayName: Compass2 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI2",    27, Compass, _state._priv_instance[1].offdiagonals, 0),
#endif // COMPASS_MAX_INSTANCES

#if COMPASS_MAX_INSTANCES > 2
    // @Param: DIA3_X
    // @DisplayName: Compass3 soft-iron diagonal X component
    // @Description: DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: DIA3_Y
    // @DisplayName: Compass3 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: DIA3_Z
    // @DisplayName: Compass3 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA3",    28, Compass, _state._priv_instance[2].diagonals, 0),

    // @Param: ODI3_X
    // @DisplayName: Compass3 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: ODI3_Y
    // @DisplayName: Compass3 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    // @Calibration: 1

    // @Param: ODI3_Z
    // @DisplayName: Compass3 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI3",    29, Compass, _state._priv_instance[2].offdiagonals, 0),
#endif // COMPASS_MAX_INSTANCES

#if COMPASS_CAL_ENABLED
    // @Param: CAL_FIT
    // @DisplayName: Compass calibration fitness
    // @Description: This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.
    // @Range: 4 32
    // @Values: 4:Very Strict,8:Strict,16:Default,32:Relaxed
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("CAL_FIT", 30, Compass, _calibration_threshold, AP_COMPASS_CALIBRATION_FITNESS_DEFAULT),
#endif

    // @Param: OFFS_MAX
    // @DisplayName: Compass maximum offset
    // @Description: This sets the maximum allowed compass offset in calibration and arming checks
    // @Range: 500 3000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFFS_MAX", 31, Compass, _offset_max, AP_COMPASS_OFFSETS_MAX_DEFAULT),

#if COMPASS_MOT_ENABLED
    // @Group: PMOT
    // @Path: Compass_PerMotor.cpp
    AP_SUBGROUPINFO(_per_motor, "PMOT", 32, Compass, Compass_PerMotor),
#endif

    // @Param: TYPEMASK
    // @DisplayName: Compass disable driver type mask
    // @Description: This is a bitmask of driver types to disable. If a driver type is set in this mask then that driver will not try to find a sensor at startup
    // @Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK09916,7:IST8310,8:ICM20948,9:MMC3416,11:UAVCAN,12:QMC5883,14:MAG3110,15:IST8308,16:RM3100,17:MSP,18:ExternalAHRS
    // @User: Advanced
    AP_GROUPINFO("TYPEMASK", 33, Compass, _driver_type_mask, 0),

    // @Param: FLTR_RNG
    // @DisplayName: Range in which sample is accepted
    // @Description: This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    AP_GROUPINFO("FLTR_RNG", 34, Compass, _filter_range, HAL_COMPASS_FILTER_DEFAULT),

#if COMPASS_CAL_ENABLED
    // @Param: AUTO_ROT
    // @DisplayName: Automatically check orientation
    // @Description: When enabled this will automatically check the orientation of compasses on successful completion of compass calibration. If set to 2 then external compasses will have their orientation automatically corrected.
    // @Values: 0:Disabled,1:CheckOnly,2:CheckAndFix
    AP_GROUPINFO("AUTO_ROT", 35, Compass, _rotate_auto, HAL_COMPASS_AUTO_ROT_DEFAULT),
#endif

#if COMPASS_MAX_INSTANCES > 1
    // @Param: PRIO1_ID
    // @DisplayName: Compass device id with 1st order priority
    // @Description: Compass device id with 1st order priority, set automatically if 0. Reboot required after change.
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PRIO1_ID",  36, Compass, _priority_did_stored_list._priv_instance[0], 0),

    // @Param: PRIO2_ID
    // @DisplayName: Compass device id with 2nd order priority
    // @Description: Compass device id with 2nd order priority, set automatically if 0. Reboot required after change.
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PRIO2_ID", 37, Compass, _priority_did_stored_list._priv_instance[1], 0),
#endif // COMPASS_MAX_INSTANCES

#if COMPASS_MAX_INSTANCES > 2
    // @Param: PRIO3_ID
    // @DisplayName: Compass device id with 3rd order priority
    // @Description: Compass device id with 3rd order priority, set automatically if 0. Reboot required after change.
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PRIO3_ID", 38, Compass, _priority_did_stored_list._priv_instance[2], 0),
#endif // COMPASS_MAX_INSTANCES

    // @Param: ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass. Note that this is separate from COMPASS_USE. This will enable the low level senor, and will enable logging of magnetometer data. To use the compass for navigation you must also set COMPASS_USE to 1.
    // @User: Standard
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO("ENABLE", 39, Compass, _enabled, 1),

    // @Param: SCALE
    // @DisplayName: Compass1 scale factor
    // @Description: Scaling factor for first compass to compensate for sensor scaling errors. If this is 0 then no scaling is done
    // @User: Standard
    // @Range: 0 1.3
    AP_GROUPINFO("SCALE", 40, Compass, _state._priv_instance[0].scale_factor, 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: SCALE2
    // @DisplayName: Compass2 scale factor
    // @Description: Scaling factor for 2nd compass to compensate for sensor scaling errors. If this is 0 then no scaling is done
    // @User: Standard
    // @Range: 0 1.3
    AP_GROUPINFO("SCALE2", 41, Compass, _state._priv_instance[1].scale_factor, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: SCALE3
    // @DisplayName: Compass3 scale factor
    // @Description: Scaling factor for 3rd compass to compensate for sensor scaling errors. If this is 0 then no scaling is done
    // @User: Standard
    // @Range: 0 1.3
    AP_GROUPINFO("SCALE3", 42, Compass, _state._priv_instance[2].scale_factor, 0),
#endif

    // @Param: OPTIONS
    // @DisplayName: Compass options
    // @Description: This sets options to change the behaviour of the compass
    // @Bitmask: 0:CalRequireGPS
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 43, Compass, _options, 0),

#if COMPASS_MAX_UNREG_DEV > 0
    // @Param: DEV_ID4
    // @DisplayName: Compass4 device id
    // @Description: Extra 4th compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID4", 44, Compass, extra_dev_id[0], 0),
#endif // COMPASS_MAX_UNREG_DEV

#if COMPASS_MAX_UNREG_DEV > 1
    // @Param: DEV_ID5
    // @DisplayName: Compass5 device id
    // @Description: Extra 5th compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID5", 45, Compass, extra_dev_id[1], 0),
#endif // COMPASS_MAX_UNREG_DEV

#if COMPASS_MAX_UNREG_DEV > 2
    // @Param: DEV_ID6
    // @DisplayName: Compass6 device id
    // @Description: Extra 6th compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID6", 46, Compass, extra_dev_id[2], 0),
#endif // COMPASS_MAX_UNREG_DEV

#if COMPASS_MAX_UNREG_DEV > 3
    // @Param: DEV_ID7
    // @DisplayName: Compass7 device id
    // @Description: Extra 7th compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID7", 47, Compass, extra_dev_id[3], 0),
#endif // COMPASS_MAX_UNREG_DEV

#if COMPASS_MAX_UNREG_DEV > 4
    // @Param: DEV_ID8
    // @DisplayName: Compass8 device id
    // @Description: Extra 8th compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID8", 48, Compass, extra_dev_id[4], 0),
#endif // COMPASS_MAX_UNREG_DEV

#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
    // @Param: CUS_ROLL
    // @DisplayName: Custom orientation roll offset
    // @Description: Compass mounting position roll offset. Positive values = roll right, negative values = roll left. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CUS_ROLL", 49, Compass, _custom_roll, 0),

    // @Param: CUS_PIT
    // @DisplayName: Custom orientation pitch offset
    // @Description: Compass mounting position pitch offset. Positive values = pitch up, negative values = pitch down. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CUS_PIT", 50, Compass, _custom_pitch, 0),

    // @Param: CUS_YAW
    // @DisplayName: Custom orientation yaw offset
    // @Description: Compass mounting position yaw offset. Positive values = yaw right, negative values = yaw left. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CUS_YAW", 51, Compass, _custom_yaw, 0),
#endif
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void)
{
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Compass must be singleton");
#endif
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Default init method
//
void Compass::init()
{
    if (!AP::compass().enabled()) {
        return;
    }

#if COMPASS_MAX_INSTANCES > 1
    // Look if there was a primary compass setup in previous version
    // if so and the the primary compass is not set in current setup
    // make the devid as primary.
    if (_priority_did_stored_list[Priority(0)] == 0) {
        uint16_t k_param_compass;
        if (AP_Param::find_top_level_key_by_pointer(this, k_param_compass)) {
            const AP_Param::ConversionInfo primary_compass_old_param = {k_param_compass, 12, AP_PARAM_INT8, ""};
            AP_Int8 value;
            value.set(0);
            bool primary_param_exists = AP_Param::find_old_parameter(&primary_compass_old_param, &value);
            int8_t oldvalue = value.get();
            if ((oldvalue!=0) && (oldvalue<COMPASS_MAX_INSTANCES) && primary_param_exists) {
                _priority_did_stored_list[Priority(0)].set_and_save_ifchanged(_state[StateIndex(oldvalue)].dev_id);
            }
        }
    }

    // Load priority list from storage, the changes to priority list
    // by user only take effect post reboot, after this
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_stored_list[i] != 0) {
            _priority_did_list[i] = _priority_did_stored_list[i];
        } else {
            // Maintain a list without gaps and duplicates
            for (Priority j(i+1); j<COMPASS_MAX_INSTANCES; j++) {
                int32_t temp;
                if (_priority_did_stored_list[j] == _priority_did_stored_list[i]) {
                    _priority_did_stored_list[j].set_and_save_ifchanged(0);
                }
                if (_priority_did_stored_list[j] == 0) {
                    continue;
                }
                temp = _priority_did_stored_list[j];
                _priority_did_stored_list[j].set_and_save_ifchanged(0);
                _priority_did_list[i] = temp;
                _priority_did_stored_list[i].set_and_save_ifchanged(temp);
                break;
            }
        }
    }
#endif // COMPASS_MAX_INSTANCES

    // cache expected dev ids for use during runtime detection
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        _state[i].expected_dev_id = _state[i].dev_id;
    }

#if COMPASS_MAX_UNREG_DEV
    // set the dev_id to 0 for undetected compasses. extra_dev_id is just an
    // interface for users to see unreg compasses, we actually never store it
    // in storage.
    for (uint8_t i=_unreg_compass_count; i<COMPASS_MAX_UNREG_DEV; i++) {
        // cache the extra devices detected in last boot
        // for detecting replacement mag
        _previously_unreg_mag[i] = extra_dev_id[i];
        extra_dev_id[i].set_and_save(0);
    }
#endif

#if COMPASS_MAX_INSTANCES > 1
    // This method calls set_and_save_ifchanged on parameters
    // which are set() but not saved() during normal runtime,
    // do not move this call without ensuring that is not happening
    // read comments under set_and_save_ifchanged for details
    _reorder_compass_params();
#endif

    if (_compass_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }

#if COMPASS_MAX_UNREG_DEV
    // We store the list of unregistered mags detected here,
    // We don't do this during runtime, as we don't want to detect
    // compasses connected by user as a replacement while the system
    // is running
    for (uint8_t i=0; i<COMPASS_MAX_UNREG_DEV; i++) {
        extra_dev_id[i].save();
    }
#endif

    if (_compass_count != 0) {
        // get initial health status
        hal.scheduler->delay(100);
        read();
    }
    // set the dev_id to 0 for undetected compasses, to make it easier
    // for users to see how many compasses are detected. We don't do a
    // set_and_save() as the user may have temporarily removed the
    // compass, and we don't want to force a re-cal if they plug it
    // back in again
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (!_state[i].registered) {
            _state[i].dev_id.set(0);
        }
    }

#ifndef HAL_BUILD_AP_PERIPH
    AP::ahrs().set_compass(this);
#endif

    init_done = true;
}

#if COMPASS_MAX_INSTANCES > 1 || COMPASS_MAX_UNREG_DEV
// Update Priority List for Mags, by default, we just
// load them as they come up the first time
Compass::Priority Compass::_update_priority_list(int32_t dev_id)
{
    // Check if already in priority list
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_list[i] == dev_id) {
            if (i >= _compass_count) {
                _compass_count = uint8_t(i)+1;
            }
            return i;
        }
    }

    // We are not in priority list, let's add at first empty
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_stored_list[i] == 0) {
            _priority_did_stored_list[i].set_and_save(dev_id);
            _priority_did_list[i] = dev_id;
            if (i >= _compass_count) {
                _compass_count = uint8_t(i)+1;
            }
            return i;
        }
    }
    return Priority(COMPASS_MAX_INSTANCES);
}
#endif


#if COMPASS_MAX_INSTANCES > 1
// This method reorganises devid list to match
// priority list, only call before detection at boot
void Compass::_reorder_compass_params()
{
    mag_state swap_state;
    StateIndex curr_state_id;
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_list[i] == 0) {
            continue;
        }
        curr_state_id = COMPASS_MAX_INSTANCES;
        for (StateIndex j(0); j<COMPASS_MAX_INSTANCES; j++) {
            if (_priority_did_list[i] == _state[j].dev_id) {
                curr_state_id = j;
                break;
            }
        }
        if (curr_state_id != COMPASS_MAX_INSTANCES && uint8_t(curr_state_id) != uint8_t(i)) {
            //let's swap
            swap_state.copy_from(_state[curr_state_id]);
            _state[curr_state_id].copy_from(_state[StateIndex(uint8_t(i))]);
            _state[StateIndex(uint8_t(i))].copy_from(swap_state);
        }
    }
}
#endif

void Compass::mag_state::copy_from(const Compass::mag_state& state)
{
    external.set_and_save_ifchanged(state.external);
    orientation.set_and_save_ifchanged(state.orientation);
    offset.set_and_save_ifchanged(state.offset);
    diagonals.set_and_save_ifchanged(state.diagonals);
    offdiagonals.set_and_save_ifchanged(state.offdiagonals);
    scale_factor.set_and_save_ifchanged(state.scale_factor);
    dev_id.set_and_save_ifchanged(state.dev_id);
    motor_compensation.set_and_save_ifchanged(state.motor_compensation);
    expected_dev_id = state.expected_dev_id;
    detected_dev_id = state.detected_dev_id;
}
//  Register a new compass instance
//
bool Compass::register_compass(int32_t dev_id, uint8_t& instance)
{

#if COMPASS_MAX_INSTANCES == 1 && !COMPASS_MAX_UNREG_DEV
    // simple single compass setup for AP_Periph
    Priority priority(0);
    StateIndex i(0);
    if (_state[i].registered) {
        return false;
    }
    _state[i].registered = true;
    _state[i].priority = priority;
    instance = uint8_t(i);
    return true;
#else
    Priority priority;
    // Check if we already have this dev_id registered
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        priority = _update_priority_list(dev_id);
        if (_state[i].expected_dev_id == dev_id && priority < COMPASS_MAX_INSTANCES) {
            _state[i].registered = true;
            _state[i].priority = priority;
            instance = uint8_t(i);
            return true;
        }
    }

    // This is an unregistered compass, check if any free slot is available
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        priority = _update_priority_list(dev_id);
        if (_state[i].dev_id == 0 && priority < COMPASS_MAX_INSTANCES) {
            _state[i].registered = true;
            _state[i].priority = priority;
            instance = uint8_t(i);
            return true;
        }
    }

    // This might be a replacement compass module, find any unregistered compass
    // instance and replace that
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        priority = _update_priority_list(dev_id);
        if (!_state[i].registered && priority < COMPASS_MAX_INSTANCES) {
            _state[i].registered = true;
            _state[i].priority = priority;
            instance = uint8_t(i);
            return true;
        }
    }
#endif

#if COMPASS_MAX_UNREG_DEV
    // Set extra dev id
    if (_unreg_compass_count >= COMPASS_MAX_UNREG_DEV) {
        AP_HAL::panic("Too many compass instances");
    }

    for (uint8_t i=0; i<COMPASS_MAX_UNREG_DEV; i++) {
        if (extra_dev_id[i] == dev_id) {
            if (i >= _unreg_compass_count) {
                _unreg_compass_count = i+1;
            }
            instance = i+COMPASS_MAX_INSTANCES;
            return false;
        } else if (extra_dev_id[i] == 0) {
            extra_dev_id[_unreg_compass_count++].set(dev_id);
            instance = i+COMPASS_MAX_INSTANCES;
            return false;
        }
    }
#else
    AP_HAL::panic("Too many compass instances");
#endif

    return false;
}

Compass::StateIndex Compass::_get_state_id(Compass::Priority priority) const
{
#if COMPASS_MAX_INSTANCES > 1
    if (_priority_did_list[priority] == 0) {
        return StateIndex(COMPASS_MAX_INSTANCES);
    }
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_list[priority] == _state[i].detected_dev_id) {
            return i;
        }
    }
    return StateIndex(COMPASS_MAX_INSTANCES);
#else
    return StateIndex(0);
#endif
}

bool Compass::_add_backend(AP_Compass_Backend *backend)
{
    if (!backend) {
        return false;
    }

    if (_backend_count == COMPASS_MAX_BACKEND) {
        return false;
    }

    _backends[_backend_count++] = backend;

    return true;
}

/*
  return true if a driver type is enabled
 */
bool Compass::_driver_enabled(enum DriverType driver_type)
{
    uint32_t mask = (1U<<uint8_t(driver_type));
    return (mask & uint32_t(_driver_type_mask.get())) == 0;
}

/*
  wrapper around hal.i2c_mgr->get_device() that prevents duplicate devices being opened
 */
bool Compass::_have_i2c_driver(uint8_t bus, uint8_t address) const
{
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (!_state[i].registered) {
            continue;
        }
        if (AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_I2C, bus, address, 0) ==
            AP_HAL::Device::change_bus_id(uint32_t(_state[i].dev_id.get()), 0)) {
            // we are already using this device
            return true;
        }
    }
    return false;
}

#if COMPASS_MAX_UNREG_DEV > 0
#define CHECK_UNREG_LIMIT_RETURN  if (_unreg_compass_count == COMPASS_MAX_UNREG_DEV) return
#else
#define CHECK_UNREG_LIMIT_RETURN
#endif

/*
  macro to add a backend with check for too many backends or compass
  instances. We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(driver_type, backend)   \
    do { if (_driver_enabled(driver_type)) { _add_backend(backend); } \
        CHECK_UNREG_LIMIT_RETURN; \
    } while (0)

#define GET_I2C_DEVICE(bus, address) _have_i2c_driver(bus, address)?nullptr:hal.i2c_mgr->get_device(bus, address)

/*
  look for compasses on external i2c buses
 */
void Compass::_probe_external_i2c_compasses(void)
{
    bool all_external = (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2);

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_HMC5843)
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(i, HAL_COMPASS_HMC5843_I2C_ADDR),
                    true, ROTATION_ROLL_180));
    }

    if (AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_MINDPXV2 &&
        AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_AEROFC) {
        // internal i2c bus
        FOREACH_I2C_INTERNAL(i) {
            ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(i, HAL_COMPASS_HMC5843_I2C_ADDR),
                        all_external, all_external?ROTATION_ROLL_180:ROTATION_YAW_270));
        }
    }
#endif

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_QMC5883L)
    //external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_QMC5883L, AP_Compass_QMC5883L::probe(GET_I2C_DEVICE(i, HAL_COMPASS_QMC5883L_I2C_ADDR),
                    true, HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL));
    }

    // internal i2c bus
    if (all_external) {
        // only probe QMC5883L on internal if we are treating internals as externals
        FOREACH_I2C_INTERNAL(i) {
            ADD_BACKEND(DRIVER_QMC5883L, AP_Compass_QMC5883L::probe(GET_I2C_DEVICE(i, HAL_COMPASS_QMC5883L_I2C_ADDR),
                        all_external,
                        all_external?HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL:HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL));
        }
    }
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // AK09916 on ICM20948
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948(GET_I2C_DEVICE(i, HAL_COMPASS_AK09916_I2C_ADDR),
                    GET_I2C_DEVICE(i, HAL_COMPASS_ICM20948_I2C_ADDR),
                    true, ROTATION_PITCH_180_YAW_90));
    }

    FOREACH_I2C_INTERNAL(i) {
        ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948(GET_I2C_DEVICE(i, HAL_COMPASS_AK09916_I2C_ADDR),
                    GET_I2C_DEVICE(i, HAL_COMPASS_ICM20948_I2C_ADDR),
                    all_external, ROTATION_PITCH_180_YAW_90));
    }
#endif // HAL_BUILD_AP_PERIPH

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_LIS3MDL)
    // lis3mdl on bus 0 with default address
    FOREACH_I2C_INTERNAL(i) {
        ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(GET_I2C_DEVICE(i, HAL_COMPASS_LIS3MDL_I2C_ADDR),
                    all_external, all_external?ROTATION_YAW_90:ROTATION_NONE));
    }

    // lis3mdl on bus 0 with alternate address
    FOREACH_I2C_INTERNAL(i) {
        ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(GET_I2C_DEVICE(i, HAL_COMPASS_LIS3MDL_I2C_ADDR2),
                    all_external, all_external?ROTATION_YAW_90:ROTATION_NONE));
    }

    // external lis3mdl on bus 1 with default address
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(GET_I2C_DEVICE(i, HAL_COMPASS_LIS3MDL_I2C_ADDR),
                    true, ROTATION_YAW_90));
    }

    // external lis3mdl on bus 1 with alternate address
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(GET_I2C_DEVICE(i, HAL_COMPASS_LIS3MDL_I2C_ADDR2),
                    true, ROTATION_YAW_90));
    }
#endif

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_AK09916)
    // AK09916. This can be found twice, due to the ICM20948 i2c bus pass-thru, so we need to be careful to avoid that
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe(GET_I2C_DEVICE(i, HAL_COMPASS_AK09916_I2C_ADDR),
                    true, ROTATION_YAW_270));
    }
    FOREACH_I2C_INTERNAL(i) {
        ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe(GET_I2C_DEVICE(i, HAL_COMPASS_AK09916_I2C_ADDR),
                    all_external, all_external?ROTATION_YAW_270:ROTATION_NONE));
    }
#endif

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_IST8310)
    // IST8310 on external and internal bus
    if (AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_FMUV5 &&
        AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_FMUV6) {
        enum Rotation default_rotation;

        if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_AEROFC) {
            default_rotation = ROTATION_PITCH_180_YAW_90;
        } else {
            default_rotation = ROTATION_PITCH_180;
        }
        // probe all 4 possible addresses
        const uint8_t ist8310_addr[] = { 0x0C, 0x0D, 0x0E, 0x0F };

        for (uint8_t a=0; a<ARRAY_SIZE(ist8310_addr); a++) {
            FOREACH_I2C_EXTERNAL(i) {
                ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(i, ist8310_addr[a]),
                            true, default_rotation));
            }
            FOREACH_I2C_INTERNAL(i) {
                ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(i, ist8310_addr[a]),
                            all_external, default_rotation));
            }
        }
    }
#endif

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_IST8308)
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_IST8308, AP_Compass_IST8308::probe(GET_I2C_DEVICE(i, HAL_COMPASS_IST8308_I2C_ADDR),
                    true, ROTATION_NONE));
    }
#endif

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) || defined(HAL_USE_I2C_MAG_RM3100)
#ifdef HAL_COMPASS_RM3100_I2C_ADDR
    const uint8_t rm3100_addresses[] = { HAL_COMPASS_RM3100_I2C_ADDR };
#else
    // RM3100 can be on 4 different addresses
    const uint8_t rm3100_addresses[] = { HAL_COMPASS_RM3100_I2C_ADDR1,
                                         HAL_COMPASS_RM3100_I2C_ADDR2,
                                         HAL_COMPASS_RM3100_I2C_ADDR3,
                                         HAL_COMPASS_RM3100_I2C_ADDR4 };
#endif
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        for (uint8_t j=0; j<ARRAY_SIZE(rm3100_addresses); j++) {
            ADD_BACKEND(DRIVER_RM3100, AP_Compass_RM3100::probe(GET_I2C_DEVICE(i, rm3100_addresses[j]), true, ROTATION_NONE));
        }
    }

    FOREACH_I2C_INTERNAL(i) {
        for (uint8_t j=0; j<ARRAY_SIZE(rm3100_addresses); j++) {
            ADD_BACKEND(DRIVER_RM3100, AP_Compass_RM3100::probe(GET_I2C_DEVICE(i, rm3100_addresses[j]), all_external, ROTATION_NONE));
        }
    }
#endif

#if !defined(HAL_DISABLE_I2C_MAGS_BY_DEFAULT) && !defined(STM32F1)
    // BMM150 on I2C, not on F1 to save flash
    FOREACH_I2C_EXTERNAL(i) {
        for (uint8_t addr=BMM150_I2C_ADDR_MIN; addr <= BMM150_I2C_ADDR_MAX; addr++) {
            ADD_BACKEND(DRIVER_BMM150,
                        AP_Compass_BMM150::probe(GET_I2C_DEVICE(i, addr), true, ROTATION_NONE));
        }
    }
#endif // HAL_BUILD_AP_PERIPH
}

/*
  detect available backends for this board
 */
void Compass::_detect_backends(void)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (_hil_mode) {
        _add_backend(AP_Compass_HIL::detect());
        return;
    }
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    if (int8_t serial_port = AP::externalAHRS().get_port() >= 0) {
        ADD_BACKEND(DRIVER_SERIAL, new AP_Compass_ExternalAHRS(serial_port));
    }
#endif
    
#if AP_FEATURE_BOARD_DETECT
    if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2) {
        // default to disabling LIS3MDL on pixhawk2 due to hardware issue
        _driver_type_mask.set_default(1U<<DRIVER_LIS3MDL);
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    ADD_BACKEND(DRIVER_SITL, new AP_Compass_SITL());
#endif

#ifdef HAL_PROBE_EXTERNAL_I2C_COMPASSES
    // allow boards to ask for external probing of all i2c compass types in hwdef.dat
    _probe_external_i2c_compasses();
    CHECK_UNREG_LIMIT_RETURN;
#endif

#if HAL_MSP_COMPASS_ENABLED
    for (uint8_t i=0; i<8; i++) {
        if (msp_instance_mask & (1U<<i)) {
            ADD_BACKEND(DRIVER_MSP, new AP_Compass_MSP(i));
        }
    }
#endif

#if defined(HAL_MAG_PROBE_LIST)
    // driver probes defined by COMPASS lines in hwdef.dat
    HAL_MAG_PROBE_LIST;
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HIL
    ADD_BACKEND(DRIVER_SITL, AP_Compass_HIL::detect());
#elif AP_FEATURE_BOARD_DETECT
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PX4V1:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
    case AP_BoardConfig::PX4_BOARD_PHMINI:
    case AP_BoardConfig::PX4_BOARD_AUAV21:
    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
    case AP_BoardConfig::PX4_BOARD_MINDPXV2:
    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK_PRO:
    case AP_BoardConfig::PX4_BOARD_AEROFC:
        _probe_external_i2c_compasses();
        CHECK_UNREG_LIMIT_RETURN;
        break;

    case AP_BoardConfig::PX4_BOARD_PCNC1:
        ADD_BACKEND(DRIVER_BMM150,
                    AP_Compass_BMM150::probe(GET_I2C_DEVICE(0, 0x10), false, ROTATION_NONE));
        break;
    case AP_BoardConfig::VRX_BOARD_BRAIN54: {
        // external i2c bus
        ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(1, HAL_COMPASS_HMC5843_I2C_ADDR),
                    true, ROTATION_ROLL_180));
    }
        // internal i2c bus
    ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(0, HAL_COMPASS_HMC5843_I2C_ADDR),
                false, ROTATION_YAW_270));
    break;

    case AP_BoardConfig::VRX_BOARD_BRAIN51:
    case AP_BoardConfig::VRX_BOARD_BRAIN52:
    case AP_BoardConfig::VRX_BOARD_BRAIN52E:
    case AP_BoardConfig::VRX_BOARD_CORE10:
    case AP_BoardConfig::VRX_BOARD_UBRAIN51:
    case AP_BoardConfig::VRX_BOARD_UBRAIN52: {
        // external i2c bus
        ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(1, HAL_COMPASS_HMC5843_I2C_ADDR),
                    true, ROTATION_ROLL_180));
    }
    break;

    default:
        break;
    }
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(hal.spi->get_device(HAL_COMPASS_HMC5843_NAME),
                    false, ROTATION_PITCH_180));
        ADD_BACKEND(DRIVER_LSM303D, AP_Compass_LSM303D::probe(hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME), ROTATION_NONE));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        ADD_BACKEND(DRIVER_LSM303D, AP_Compass_LSM303D::probe(hal.spi->get_device(HAL_INS_LSM9DS0_EXT_A_NAME), ROTATION_YAW_270));
        // we run the AK8963 only on the 2nd MPU9250, which leaves the
        // first MPU9250 to run without disturbance at high rate
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(1, ROTATION_YAW_270));
        ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe_ICM20948(0, ROTATION_ROLL_180_YAW_90));
        break;

    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
        FOREACH_I2C_EXTERNAL(i) {
            ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(i, HAL_COMPASS_IST8310_I2C_ADDR),
                        true, ROTATION_ROLL_180_YAW_90));
        }
        FOREACH_I2C_INTERNAL(i) {
            ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(i, HAL_COMPASS_IST8310_I2C_ADDR),
                        false, ROTATION_ROLL_180_YAW_90));
        }
        break;

    case AP_BoardConfig::PX4_BOARD_SP01:
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(1, ROTATION_NONE));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK_PRO:
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_ROLL_180_YAW_90));
        ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(hal.spi->get_device(HAL_COMPASS_LIS3MDL_NAME),
                    false, ROTATION_NONE));
        break;

    case AP_BoardConfig::PX4_BOARD_PHMINI:
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_ROLL_180));
        break;

    case AP_BoardConfig::PX4_BOARD_AUAV21:
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_ROLL_180_YAW_90));
        break;

    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_YAW_270));
        break;

    case AP_BoardConfig::PX4_BOARD_MINDPXV2:
        ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(0, HAL_COMPASS_HMC5843_I2C_ADDR),
                    false, ROTATION_YAW_90));
        ADD_BACKEND(DRIVER_LSM303D, AP_Compass_LSM303D::probe(hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME), ROTATION_PITCH_180_YAW_270));
        break;

    default:
        break;
    }

#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_NONE
    // no compass, or only external probe
#else
#error Unrecognised HAL_COMPASS_TYPE setting
#endif


#if HAL_ENABLE_LIBUAVCAN_DRIVERS
    if (_driver_enabled(DRIVER_UAVCAN)) {
        for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
            AP_Compass_Backend* _uavcan_backend = AP_Compass_UAVCAN::probe(i);
            if (_uavcan_backend) {
                _add_backend(_uavcan_backend);
            }
#if COMPASS_MAX_UNREG_DEV > 0
            if (_unreg_compass_count == COMPASS_MAX_UNREG_DEV)  {
                break;
            }
#endif
        }

#if COMPASS_MAX_UNREG_DEV > 0
        // check if there's any uavcan compass in prio slot that's not found
        // and replace it if there's a replacement compass
        for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
            if (AP_HAL::Device::devid_get_bus_type(_priority_did_list[i]) != AP_HAL::Device::BUS_TYPE_UAVCAN
                || _get_state(i).registered) {
                continue;
            }
            // There's a UAVCAN compass missing
            // Let's check if there's a replacement
            for (uint8_t j=0; j<COMPASS_MAX_INSTANCES; j++) {
                uint32_t detected_devid = AP_Compass_UAVCAN::get_detected_devid(j);
                // Check if this is a potential replacement mag
                if (!is_replacement_mag(detected_devid)) {
                    continue;
                }
                // We have found a replacement mag, let's replace the existing one
                // with this by setting the priority to zero and calling uavcan probe 
                gcs().send_text(MAV_SEVERITY_ALERT, "Mag: Compass #%d with DEVID %lu replaced", uint8_t(i), (unsigned long)_priority_did_list[i]);
                _priority_did_stored_list[i].set_and_save(0);
                _priority_did_list[i] = 0;

                AP_Compass_Backend* _uavcan_backend = AP_Compass_UAVCAN::probe(j);
                if (_uavcan_backend) {
                    _add_backend(_uavcan_backend);
                    // we also need to remove the id from unreg list
                    remove_unreg_dev_id(detected_devid);
                } else {
                    // the mag has already been allocated,
                    // let's begin the replacement
                    bool found_replacement = false;
                    for (StateIndex k(0); k<COMPASS_MAX_INSTANCES; k++) {
                        if ((uint32_t)_state[k].dev_id == detected_devid) {
                            if (_state[k].priority <= uint8_t(i)) {
                                // we are already on higher priority
                                // nothing to do
                                break;
                            }
                            found_replacement = true;
                            // reset old priority of replacement mag
                            _priority_did_stored_list[_state[k].priority].set_and_save(0);
                            _priority_did_list[_state[k].priority] = 0;
                            // update new priority
                            _state[k].priority = i;
                        }
                    }
                    if (!found_replacement) {
                        continue;
                    }
                    _priority_did_stored_list[i].set_and_save(detected_devid);
                    _priority_did_list[i] = detected_devid;
                }
            }
        }
#endif
    }
#endif

    if (_backend_count == 0 ||
        _compass_count == 0) {
        hal.console->printf("No Compass backends available\n");
    }
}

// Check if the devid is a potential replacement compass
// Following are the checks done to ensure the compass is a replacement
// * The compass is an UAVCAN compass
// * The compass wasn't seen before this boot as additional unreg mag
// * The compass might have been seen before but never setup
bool Compass::is_replacement_mag(uint32_t devid) {
#if COMPASS_MAX_INSTANCES > 1
    // We only do this for UAVCAN mag
    if (devid == 0 || (AP_HAL::Device::devid_get_bus_type(devid) != AP_HAL::Device::BUS_TYPE_UAVCAN)) {
        return false;
    }

    // Check that its not an unused additional mag
    for (uint8_t i = 0; i<COMPASS_MAX_UNREG_DEV; i++) {
        if (_previously_unreg_mag[i] == devid) {
            return false;
        }
    }

    // Check that its not previously setup mag
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if ((uint32_t)_state[i].expected_dev_id == devid) {
            return false;
        }
    }
#endif
    return true;
}

void Compass::remove_unreg_dev_id(uint32_t devid)
{
#if COMPASS_MAX_INSTANCES > 1
    // We only do this for UAVCAN mag
    if (devid == 0 || (AP_HAL::Device::devid_get_bus_type(devid) != AP_HAL::Device::BUS_TYPE_UAVCAN)) {
        return;
    }

    for (uint8_t i = 0; i<COMPASS_MAX_UNREG_DEV; i++) {
        if ((uint32_t)extra_dev_id[i] == devid) {
            extra_dev_id[i].set_and_save(0);
            return;
        }
    }
#endif
}

void Compass::_reset_compass_id()
{
#if COMPASS_MAX_INSTANCES > 1
    // Check if any of the registered devs are not registered
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_stored_list[i] != _priority_did_list[i] ||
            _priority_did_stored_list[i] == 0) {
            //We don't touch priorities that might have been touched by the user
            continue;
        }
        if (!_get_state(i).registered) {
            _priority_did_stored_list[i].set_and_save(0);
            gcs().send_text(MAV_SEVERITY_ALERT, "Mag: Compass #%d with DEVID %lu removed", uint8_t(i), (unsigned long)_priority_did_list[i]);
        }
    }

    // Check if any of the old registered devs are not registered
    // and hence can be removed
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_state[i].dev_id == 0 && _state[i].expected_dev_id != 0) {
            // also hard reset dev_ids that are not detected
            _state[i].dev_id.save();
        }
    }
#endif
}

// Look for devices beyond initialisation
void
Compass::_detect_runtime(void)
{
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
    //Don't try to add device while armed
    if (hal.util->get_soft_armed()) {
        return;
    }
    static uint32_t last_try;
    //Try once every second
    if ((AP_HAL::millis() - last_try) < 1000) {
        return;
    }
    last_try = AP_HAL::millis();
    if (_driver_enabled(DRIVER_UAVCAN)) {
        for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
            AP_Compass_Backend* _uavcan_backend = AP_Compass_UAVCAN::probe(i);
            if (_uavcan_backend) {
                _add_backend(_uavcan_backend);
            }
            CHECK_UNREG_LIMIT_RETURN;
        }
    }
#endif
}

bool
Compass::read(void)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!_initial_location_set) {
        try_set_initial_location();
    }
#endif

    _detect_runtime();

    for (uint8_t i=0; i< _backend_count; i++) {
        // call read on each of the backend. This call updates field[i]
        _backends[i]->read();
    }
    uint32_t time = AP_HAL::millis();
    bool any_healthy = false;
    for (StateIndex i(0); i < COMPASS_MAX_INSTANCES; i++) {
        _state[i].healthy = (time - _state[i].last_update_ms < 500);
        any_healthy |= _state[i].healthy;
    }
#if COMPASS_LEARN_ENABLED
    if (_learn == LEARN_INFLIGHT && !learn_allocated) {
        learn_allocated = true;
        learn = new CompassLearn(*this);
    }
    if (_learn == LEARN_INFLIGHT && learn != nullptr) {
        learn->update();
    }
#endif
#ifndef HAL_NO_LOGGING
    if (any_healthy && _log_bit != (uint32_t)-1 && AP::logger().should_log(_log_bit)) {
        AP::logger().Write_Compass();
    }
#endif
    return healthy();
}

uint8_t
Compass::get_healthy_mask() const
{
    uint8_t healthy_mask = 0;
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (healthy(uint8_t(i))) {
            healthy_mask |= 1 << uint8_t(i);
        }
    }
    return healthy_mask;
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].offset.set(offsets);
    }
}

void
Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].offset.set(offsets);
        save_offsets(i);
    }
}

void
Compass::set_and_save_diagonals(uint8_t i, const Vector3f &diagonals)
{
    // sanity check compass instance provided
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].diagonals.set_and_save(diagonals);
    }
}

void
Compass::set_and_save_offdiagonals(uint8_t i, const Vector3f &offdiagonals)
{
    // sanity check compass instance provided
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].offdiagonals.set_and_save(offdiagonals);
    }
}

void
Compass::set_and_save_scale_factor(uint8_t i, float scale_factor)
{
    StateIndex id = _get_state_id(Priority(i));
    if (i < COMPASS_MAX_INSTANCES) {
        _state[id].scale_factor.set_and_save(scale_factor);
    }
}

void
Compass::save_offsets(uint8_t i)
{
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].offset.save();  // save offsets
        _state[id].dev_id.set_and_save(_state[id].detected_dev_id);
    }
}

void
Compass::set_and_save_orientation(uint8_t i, Rotation orientation)
{
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].orientation.set_and_save_ifchanged(orientation);
    }
}

void
Compass::save_offsets(void)
{
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        save_offsets(uint8_t(i));
    }
}

void
Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    StateIndex id = _get_state_id(Priority(i));
    if (id < COMPASS_MAX_INSTANCES) {
        _state[id].motor_compensation.set(motor_comp_factor);
    }
}

void
Compass::save_motor_compensation()
{
    StateIndex id;
    _motor_comp_type.save();
    for (Priority k(0); k<COMPASS_MAX_INSTANCES; k++) {
        id = _get_state_id(k);
        if (id<COMPASS_MAX_INSTANCES) {
            _state[id].motor_compensation.save();
        }
    }
}

void Compass::try_set_initial_location()
{
    if (!_auto_declination) {
        return;
    }
    if (!_enabled) {
        return;
    }

    Location loc;
    if (!AP::ahrs().get_position(loc)) {
        return;
    }
    _initial_location_set = true;

    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
    // Set the declination based on the lat/lng from GPS
    _declination.set(radians(
                         AP_Declination::get_declination(
                             (float)loc.lat / 10000000,
                             (float)loc.lng / 10000000)));
}

/// return true if the compass should be used for yaw calculations
bool
Compass::use_for_yaw(void) const
{
    return healthy(0) && use_for_yaw(0);
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
    // when we are doing in-flight compass learning the state
    // estimator must not use the compass. The learning code turns off
    // inflight learning when it has converged
    return _use_for_yaw[Priority(i)] && _learn.get() != LEARN_INFLIGHT;
}

/*
  return the number of enabled sensors. Used to determine if
  non-compass operation is desired
 */
uint8_t Compass::get_num_enabled(void) const
{
    if (get_count() == 0) {
        return 0;
    }
    uint8_t count = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (use_for_yaw(i)) {
            count++;
        }
    }
    return count;
}

void
Compass::set_use_for_yaw(uint8_t i, bool use)
{
    _use_for_yaw[Priority(i)].set(use);
}

void
Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        _declination.set_and_save(radians);
    } else {
        _declination.set(radians);
    }
}

float
Compass::get_declination() const
{
    return _declination.get();
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float
Compass::calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    const Vector3f &field = get_field(i);

    float headY = field.y * dcm_matrix.c.z - field.z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = field.x * cos_pitch_sq - dcm_matrix.c.x * (field.y * dcm_matrix.c.y + field.z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if ( fabsf(_declination) > 0.0f ) {
        heading = heading + _declination;
        if (heading > M_PI) {  // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * M_PI);
        } else if (heading < -M_PI) {
            heading += (2.0f * M_PI);
        }
    }

    return heading;
}

/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool Compass::configured(uint8_t i)
{
    // exit immediately if instance is beyond the number of compasses we have available
    if (i > get_count()) {
        return false;
    }

    // exit immediately if all offsets are zero
    if (is_zero(get_offsets(i).length())) {
        return false;
    }

    StateIndex id = _get_state_id(Priority(i));
    // exit immediately if dev_id hasn't been detected
    if (_state[id].detected_dev_id == 0 || 
        id == COMPASS_MAX_INSTANCES) {
        return false;
    }

    // back up cached value of dev_id
    int32_t dev_id_cache_value = _state[id].dev_id;

    // load dev_id from eeprom
    _state[id].dev_id.load();

    // if dev_id loaded from eeprom is different from detected dev id or dev_id loaded from eeprom is different from cached dev_id, compass is unconfigured
    if (_state[id].dev_id != _state[id].detected_dev_id || _state[id].dev_id != dev_id_cache_value) {
        // restore cached value
        _state[id].dev_id = dev_id_cache_value;
        // return failure
        return false;
    }

    // if we got here then it must be configured
    return true;
}

bool Compass::configured(char *failure_msg, uint8_t failure_msg_len)
{
#if COMPASS_MAX_INSTANCES > 1
    // Check if any of the registered devs are not registered
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_priority_did_list[i] != 0 && use_for_yaw(uint8_t(i))) {
            if (!_get_state(i).registered) {
                snprintf(failure_msg, failure_msg_len, "Compass %d not Found", uint8_t(i));
                return false;
            }
            if (_priority_did_list[i] != _priority_did_stored_list[i]) {
                snprintf(failure_msg, failure_msg_len, "Compass order change requires reboot");
                return false;
            }
        }
    }
#endif

    bool all_configured = true;
    for (uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && (!use_for_yaw(i) || configured(i));
    }
    if (!all_configured) {
        snprintf(failure_msg, failure_msg_len, "Compass not calibrated");
    }
    return all_configured;
}

// Update raw magnetometer values from HIL data
//
void Compass::setHIL(uint8_t instance, float roll, float pitch, float yaw)
{
    Matrix3f R;

    // create a rotation matrix for the given attitude
    R.from_euler(roll, pitch, yaw);

    if (!is_equal(_hil.last_declination,get_declination())) {
        _setup_earth_field();
        _hil.last_declination = get_declination();
    }

    // convert the earth frame magnetic vector to body frame, and
    // apply the offsets
    _hil.field[instance] = R.mul_transpose(_hil.Bearth);

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _hil.field[instance].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _hil.field[instance].rotate((enum Rotation)_state[StateIndex(0)].orientation.get());

    if (!_state[StateIndex(0)].external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        if (_board_orientation == ROTATION_CUSTOM && _custom_rotation) {
            _hil.field[instance] = *_custom_rotation * _hil.field[instance];
        } else {
            _hil.field[instance].rotate(_board_orientation);
        }
    }
    _hil.healthy[instance] = true;
}

// Update raw magnetometer values from HIL mag vector
//
void Compass::setHIL(uint8_t instance, const Vector3f &mag, uint32_t update_usec)
{
    _hil.field[instance] = mag;
    _hil.healthy[instance] = true;
    _state[StateIndex(instance)].last_update_usec = update_usec;
}

const Vector3f& Compass::getHIL(uint8_t instance) const
{
    return _hil.field[instance];
}

// setup _Bearth
void Compass::_setup_earth_field(void)
{
    // assume a earth field strength of 400
    _hil.Bearth = {400, 0, 0};

    // rotate _Bearth for inclination and declination. -66 degrees
    // is the inclination in Canberra, Australia
    Matrix3f R;
    R.from_euler(0, ToRad(66), get_declination());
    _hil.Bearth = R * _hil.Bearth;
}

/*
  set the type of motor compensation to use
 */
void Compass::motor_compensation_type(const uint8_t comp_type)
{
    if (_motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
        _motor_comp_type = (int8_t)comp_type;
        _thr = 0; // set current  throttle to zero
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
        }
    }
}

bool Compass::consistent() const
{
    const Vector3f &primary_mag_field = get_field();
    const Vector2f primary_mag_field_xy = Vector2f(primary_mag_field.x,primary_mag_field.y);

    if (primary_mag_field_xy.is_zero()) {
        return false;
    }

    const Vector3f primary_mag_field_norm = primary_mag_field.normalized();
    const Vector2f primary_mag_field_xy_norm = primary_mag_field_xy.normalized();

    for (uint8_t i=0; i<get_count(); i++) {
        if (!use_for_yaw(i)) {
            // configured not-to-be-used
            continue;
        }

        Vector3f mag_field = get_field(i);
        Vector2f mag_field_xy = Vector2f(mag_field.x,mag_field.y);

        if (mag_field_xy.is_zero()) {
            return false;
        }

        const float xy_len_diff  = (primary_mag_field_xy-mag_field_xy).length();

        mag_field.normalize();
        mag_field_xy.normalize();

        const float xyz_ang_diff = acosf(constrain_float(mag_field*primary_mag_field_norm,-1.0f,1.0f));
        const float xy_ang_diff  = acosf(constrain_float(mag_field_xy*primary_mag_field_xy_norm,-1.0f,1.0f));

        // check for gross misalignment on all axes
        if (xyz_ang_diff > AP_COMPASS_MAX_XYZ_ANG_DIFF) {
            return false;
        }

        // check for an unacceptable angle difference on the xy plane
        if (xy_ang_diff > AP_COMPASS_MAX_XY_ANG_DIFF) {
            return false;
        }

        // check for an unacceptable length difference on the xy plane
        if (xy_len_diff > AP_COMPASS_MAX_XY_LENGTH_DIFF) {
            return false;
        }
    }
    return true;
}

/*
  return true if we have a valid scale factor
 */
bool Compass::have_scale_factor(uint8_t i) const
{
    StateIndex id = _get_state_id(Priority(i));
    if (id >= COMPASS_MAX_INSTANCES ||
        _state[id].scale_factor < COMPASS_MIN_SCALE_FACTOR ||
        _state[id].scale_factor > COMPASS_MAX_SCALE_FACTOR) {
        return false;
    }
    return true;
}

#if HAL_MSP_COMPASS_ENABLED
void Compass::handle_msp(const MSP::msp_compass_data_message_t &pkt)
{
    if (!_driver_enabled(DRIVER_MSP)) {
        return;
    }
    if (!init_done) {
        if (pkt.instance < 8) {
            msp_instance_mask |= 1U<<pkt.instance;
        }
    } else {
        for (uint8_t i=0; i<_backend_count; i++) {
            _backends[i]->handle_msp(pkt);
        }
    }
}
#endif // HAL_MSP_COMPASS_ENABLED

#if HAL_EXTERNAL_AHRS_ENABLED
void Compass::handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt)
{
    if (!_driver_enabled(DRIVER_SERIAL)) {
        return;
    }
    for (uint8_t i=0; i<_backend_count; i++) {
        _backends[i]->handle_external(pkt);
    }
}
#endif // HAL_EXTERNAL_AHRS_ENABLED

// singleton instance
Compass *Compass::_singleton;

namespace AP
{

Compass &compass()
{
    return *Compass::get_singleton();
}

}
