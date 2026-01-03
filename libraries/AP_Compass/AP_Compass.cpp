#include "AP_Compass_config.h"

#if AP_COMPASS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/I2CDevice.h>
#endif
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_CustomRotations/AP_CustomRotations.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_Compass_config.h"

#include "AP_Compass_SITL.h"
#include "AP_Compass_AK8963.h"
#include "AP_Compass_Backend.h"
#include "AP_Compass_BMM150.h"
#include "AP_Compass_BMM350.h"
#include "AP_Compass_HMC5843.h"
#include "AP_Compass_IIS2MDC.h"
#include "AP_Compass_IST8308.h"
#include "AP_Compass_IST8310.h"
#include "AP_Compass_LSM303D.h"
#include "AP_Compass_LSM9DS1.h"
#include "AP_Compass_LIS3MDL.h"
#include "AP_Compass_LIS2MDL.h"
#include "AP_Compass_AK09916.h"
#include "AP_Compass_QMC5883L.h"
#if AP_COMPASS_DRONECAN_ENABLED
#include "AP_Compass_DroneCAN.h"
#endif
#include "AP_Compass_QMC5883P.h"
#include "AP_Compass_MMC3416.h"
#include "AP_Compass_MMC5xx3.h"
#include "AP_Compass_MAG3110.h"
#include "AP_Compass_RM3100.h"
#if AP_COMPASS_MSP_ENABLED
#include "AP_Compass_MSP.h"
#endif
#if AP_COMPASS_EXTERNALAHRS_ENABLED
#include "AP_Compass_ExternalAHRS.h"
#endif
#include "AP_Compass.h"
#include "Compass_learn.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_COMPASS_ENABLE_DEFAULT
 #define AP_COMPASS_ENABLE_DEFAULT 1
#endif

#ifndef COMPASS_LEARN_DEFAULT
#define COMPASS_LEARN_DEFAULT Compass::LearnType::NONE
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

#ifndef HAL_BUILD_AP_PERIPH
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
    // @Calibration: 1
    AP_GROUPINFO("OFS",    1, Compass, _state._priv_instance[0].offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: rad
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),
#endif // HAL_BUILD_AP_PERIPH

#if COMPASS_LEARN_ENABLED
    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle. If InFlight learning is enabled then the compass with automatically start learning once a flight starts (must be armed). While InFlight learning is running you cannot use position control modes.
    // @Values: 0:Disabled,2:EKF-Learning,3:InFlight-Learning
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, float(COMPASS_LEARN_DEFAULT)),
#endif

#ifndef HAL_BUILD_AP_PERIPH
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
#endif

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
    // @Calibration: 1
    AP_GROUPINFO("MOT",    7, Compass, _state._priv_instance[0].motor_compensation, 0),
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the COMPASS_CUS_ROLL/PIT/YAW angles for Compass orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_1_ROLL/PIT/YAW or CUST_2_ROLL/PIT/YAW angles.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom 4.1 and older,101:Custom 1,102:Custom 2
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 8, Compass, _state._priv_instance[0].orientation, ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on most boards. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, Compass, _state._priv_instance[0].external, 0),
#endif

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
    // @Calibration: 1
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
    // @Calibration: 1
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
    // @Calibration: 1
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
    // @Calibration: 1
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
    // @Description: The orientation of a second external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the COMPASS_CUS_ROLL/PIT/YAW angles for Compass orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_1_ROLL/PIT/YAW or CUST_2_ROLL/PIT/YAW angles.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom 4.1 and older,101:Custom 1,102:Custom 2
    // @User: Advanced
    AP_GROUPINFO("ORIENT2", 19, Compass, _state._priv_instance[1].orientation, ROTATION_NONE),

    // @Param: EXTERN2
    // @DisplayName: Compass2 is attached via an external cable
    // @Description: Configure second compass so it is attached externally. This is auto-detected on most boards. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
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
    // @Description: The orientation of a third external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used. The label for each option is specified in the order of rotations for that orientation. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the COMPASS_CUS_ROLL/PIT/YAW angles for Compass orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_1_ROLL/PIT/YAW or CUST_2_ROLL/PIT/YAW angles.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom 4.1 and older,101:Custom 1,102:Custom 2
    // @User: Advanced
    AP_GROUPINFO("ORIENT3", 22, Compass, _state._priv_instance[2].orientation, ROTATION_NONE),

    // @Param: EXTERN3
    // @DisplayName: Compass3 is attached via an external cable
    // @Description: Configure third compass so it is attached externally. This is auto-detected on most boards. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN3",23, Compass, _state._priv_instance[2].external, 0),
#endif // COMPASS_MAX_INSTANCES

#if AP_COMPASS_DIAGONALS_ENABLED
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
    // @Calibration: 1
    AP_GROUPINFO("DIA",    24, Compass, _state._priv_instance[0].diagonals, 1.0),

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
    // @Calibration: 1
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
    // @Calibration: 1
    AP_GROUPINFO("DIA2",    26, Compass, _state._priv_instance[1].diagonals, 1.0),

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
    // @Calibration: 1
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
    // @Calibration: 1
    AP_GROUPINFO("DIA3",    28, Compass, _state._priv_instance[2].diagonals, 1.0),

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
    // @Calibration: 1
    AP_GROUPINFO("ODI3",    29, Compass, _state._priv_instance[2].offdiagonals, 0),
#endif // COMPASS_MAX_INSTANCES
#endif // AP_COMPASS_DIAGONALS_ENABLED

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

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: OFFS_MAX
    // @DisplayName: Compass maximum offset
    // @Description: This sets the maximum allowed compass offset in calibration and arming checks
    // @Range: 500 3000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFFS_MAX", 31, Compass, _offset_max, AP_COMPASS_OFFSETS_MAX_DEFAULT),
#endif

#if COMPASS_MOT_ENABLED
    // @Group: PMOT
    // @Path: Compass_PerMotor.cpp
    AP_SUBGROUPINFO(_per_motor, "PMOT", 32, Compass, Compass_PerMotor),
#endif

    // @Param: DISBLMSK
    // @DisplayName: Compass disable driver type mask
    // @Description: This is a bitmask of driver types to disable. If a driver type is set in this mask then that driver will not try to find a sensor at startup
    // @Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK0991x,7:IST8310,8:ICM20948,9:MMC3416,11:DroneCAN,12:QMC5883,14:MAG3110,15:IST8308,16:RM3100,17:MSP,18:ExternalAHRS,19:MMC5XX3,20:QMC5883P,21:BMM350,22:IIS2MDC,23:LIS2MDL
    // @User: Advanced
    AP_GROUPINFO("DISBLMSK", 33, Compass, _driver_type_mask, 0),

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
    // @Values: 0:Disabled,1:CheckOnly,2:CheckAndFix,3:use same tolerance to auto rotate 45 deg rotations
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
    // @RebootRequired: True
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO("ENABLE", 39, Compass, _enabled, AP_COMPASS_ENABLE_DEFAULT),

#ifndef HAL_BUILD_AP_PERIPH
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
#endif // HAL_BUILD_AP_PERIPH

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: OPTIONS
    // @DisplayName: Compass options
    // @Description: This sets options to change the behaviour of the compass
    // @Bitmask: 0:CalRequireGPS
    // @Bitmask: 1: Allow missing DroneCAN compasses to be automaticaly replaced (calibration still required)
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 43, Compass, _options, 0),
#endif

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

    // @Param: CUS_ROLL
    // @DisplayName: Custom orientation roll offset
    // @Description: Compass mounting position roll offset. Positive values = roll right, negative values = roll left. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced

    // index 49

    // @Param: CUS_PIT
    // @DisplayName: Custom orientation pitch offset
    // @Description: Compass mounting position pitch offset. Positive values = pitch up, negative values = pitch down. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced

    // index 50

    // @Param: CUS_YAW
    // @DisplayName: Custom orientation yaw offset
    // @Description: Compass mounting position yaw offset. Positive values = yaw right, negative values = yaw left. This parameter is only used when COMPASS_ORIENT/2/3 is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced

    // index 51

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
    if (!_enabled) {
        return;
    }

    /*
      on init() if any devid is set then we set suppress_devid_save to
      false. This is used to determine if we save device ids during
      the init process.
     */
    suppress_devid_save = true;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_state._priv_instance[i].dev_id != 0) {
            suppress_devid_save = false;
            break;
        }
#if COMPASS_MAX_INSTANCES > 1
        if (_priority_did_stored_list._priv_instance[i] != 0) {
            suppress_devid_save = false;
            break;
        }
#endif
    }

    // convert to new custom rotation method
    // PARAMETER_CONVERSION - Added: Nov-2021
#if AP_CUSTOMROTATIONS_ENABLED
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_state[i].orientation != ROTATION_CUSTOM_OLD) {
            continue;
        }
        _state[i].orientation.set_and_save(ROTATION_CUSTOM_2);
        AP_Param::ConversionInfo info;
        if (AP_Param::find_top_level_key_by_pointer(this, info.old_key)) {
            info.type = AP_PARAM_FLOAT;
            float rpy[3] = {};
            AP_Float rpy_param;
            for (info.old_group_element=49; info.old_group_element<=51; info.old_group_element++) {
                if (AP_Param::find_old_parameter(&info, &rpy_param)) {
                    rpy[info.old_group_element-49] = rpy_param.get();
                }
            }
            AP::custom_rotations().convert(ROTATION_CUSTOM_2, rpy[0], rpy[1], rpy[2]);
        }
        break;
    }
#endif  // AP_CUSTOMROTATIONS_ENABLED

#if COMPASS_MAX_INSTANCES > 1
    // Look if there was a primary compass setup in previous version
    // if so and the primary compass is not set in current setup
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
    if (!suppress_devid_save) {
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
        extra_dev_id[i].set(0);
    }
#endif

#if COMPASS_MAX_INSTANCES > 1
    // This method calls set_and_save_ifchanged on parameters
    // which are set() but not saved() during normal runtime,
    // do not move this call without ensuring that is not happening
    // read comments under set_and_save_ifchanged for details
    if (!suppress_devid_save) {
        _reorder_compass_params();
    }
#endif

    if (_compass_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }

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
    // updating the AHRS orientation updates our own orientation:
    AP::ahrs().update_orientation();
#endif

    init_done = true;
    suppress_devid_save = false;
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
            if (suppress_devid_save) {
                _priority_did_stored_list[i].set(dev_id);
            } else {
                _priority_did_stored_list[i].set_and_save(dev_id);
            }
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
#if AP_COMPASS_DIAGONALS_ENABLED
    diagonals.set_and_save_ifchanged(state.diagonals);
    offdiagonals.set_and_save_ifchanged(state.offdiagonals);
#endif
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
    _compass_count = 1;
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
    priority = _update_priority_list(dev_id);
    // try to match priority and state index if possible, this ensure that compass order
    // to state order while detection is preserved, this ensures that if compasses in priority
    // list show up out of order during detection, it does not replace the state.
    StateIndex priority_index = StateIndex(uint8_t(priority));
    if (!_state[priority_index].registered && priority < COMPASS_MAX_INSTANCES) {
        _state[priority_index].registered = true;
        _state[priority_index].priority = priority;
        instance = uint8_t(priority_index);
        return true;
    }
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
bool Compass::_i2c_sensor_is_registered(uint8_t bus, uint8_t address) const
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
#define RETURN_IF_NO_SPACE  if (_unreg_compass_count == COMPASS_MAX_UNREG_DEV) return
#else
#define RETURN_IF_NO_SPACE
#endif

/*
  macro to add a backend with check for too many backends or compass
  instances. We don't try to start more than the maximum allowed
 */
void Compass::add_backend(Compass::DriverType driver_type, AP_Compass_Backend *backend)
{
    if (!_driver_enabled(driver_type)) {
        return;
    }

    if (!backend) {
        return;
    }

    if (_backend_count == COMPASS_MAX_BACKEND) {
        return;
    }

    _backends[_backend_count++] = backend;
}

#define GET_I2C_DEVICE(bus, address) _i2c_sensor_is_registered(bus, address)?nullptr:hal.i2c_mgr->get_device(bus, address)

/*
  look for compasses on external i2c buses
 */
void Compass::_probe_external_i2c_compasses(void)
{
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    bool all_external = (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2);
    (void)all_external;  // in case all backends using this are compiled out
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED

#if AP_COMPASS_HMC5843_ENABLED
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_HMC5843, AP_Compass_HMC5843::probe, i, HAL_COMPASS_HMC5843_I2C_ADDR, true, ROTATION_ROLL_180);
        RETURN_IF_NO_SPACE;
    }

#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    if (AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_MINDPXV2 &&
        AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_AEROFC) {
        // internal i2c bus
        FOREACH_I2C_INTERNAL(i) {
            probe_i2c_dev(DRIVER_HMC5843, AP_Compass_HMC5843::probe, i, HAL_COMPASS_HMC5843_I2C_ADDR, all_external, all_external?ROTATION_ROLL_180:ROTATION_YAW_270);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_HMC5843_ENABLED

#if AP_COMPASS_QMC5883L_ENABLED
    //external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_QMC5883L, AP_Compass_QMC5883L::probe, i, HAL_COMPASS_QMC5883L_I2C_ADDR, true, HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL);
        RETURN_IF_NO_SPACE;
    }

    // internal i2c bus
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    if (all_external) {
        // only probe QMC5883L on internal if we are treating internals as externals
        FOREACH_I2C_INTERNAL(i) {
            probe_i2c_dev(DRIVER_QMC5883L, AP_Compass_QMC5883L::probe, i, HAL_COMPASS_QMC5883L_I2C_ADDR, all_external, all_external?HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL:HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_QMC5883L_ENABLED

#if AP_COMPASS_QMC5883P_ENABLED
    //external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_QMC5883P, AP_Compass_QMC5883P::probe, i, HAL_COMPASS_QMC5883P_I2C_ADDR, true, HAL_COMPASS_QMC5883P_ORIENTATION_EXTERNAL);
        RETURN_IF_NO_SPACE;
    }

    // internal i2c bus
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    if (all_external) {
        // only probe QMC5883P on internal if we are treating internals as externals
        FOREACH_I2C_INTERNAL(i) {
            probe_i2c_dev(DRIVER_QMC5883P, AP_Compass_QMC5883P::probe, i, HAL_COMPASS_QMC5883P_I2C_ADDR, all_external, all_external?HAL_COMPASS_QMC5883P_ORIENTATION_EXTERNAL:HAL_COMPASS_QMC5883P_ORIENTATION_INTERNAL);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_QMC5883P_ENABLED

#if AP_COMPASS_IIS2MDC_ENABLED
    //external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_IIS2MDC, AP_Compass_IIS2MDC::probe, i, HAL_COMPASS_IIS2MDC_I2C_ADDR, true, HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL);
        RETURN_IF_NO_SPACE;
    }

    // internal i2c bus
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    if (all_external) {
        // only probe IIS2MDC on internal if we are treating internals as externals
        FOREACH_I2C_INTERNAL(i) {
            probe_i2c_dev(DRIVER_IIS2MDC, AP_Compass_IIS2MDC::probe, i, HAL_COMPASS_IIS2MDC_I2C_ADDR, all_external, all_external?HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL:HAL_COMPASS_IIS2MDC_ORIENTATION_INTERNAL);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_QMC5883P_ENABLED

    // AK09916 on ICM20948
#if AP_COMPASS_AK09916_ENABLED && AP_COMPASS_ICM20948_ENABLED
    FOREACH_I2C_EXTERNAL(i) {
        probe_ak09916_via_icm20948(i, HAL_COMPASS_AK09916_I2C_ADDR, HAL_COMPASS_ICM20948_I2C_ADDR, true, ROTATION_PITCH_180_YAW_90);
        RETURN_IF_NO_SPACE;
        probe_ak09916_via_icm20948(i, HAL_COMPASS_AK09916_I2C_ADDR, HAL_COMPASS_ICM20948_I2C_ADDR2, true, ROTATION_PITCH_180_YAW_90);
        RETURN_IF_NO_SPACE;
    }

#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        probe_ak09916_via_icm20948(i, HAL_COMPASS_AK09916_I2C_ADDR, HAL_COMPASS_ICM20948_I2C_ADDR, all_external, ROTATION_PITCH_180_YAW_90);
        RETURN_IF_NO_SPACE;
        probe_ak09916_via_icm20948(i, HAL_COMPASS_AK09916_I2C_ADDR, HAL_COMPASS_ICM20948_I2C_ADDR2, all_external, ROTATION_PITCH_180_YAW_90);
        RETURN_IF_NO_SPACE;
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_AK09916_ENABLED && AP_COMPASS_ICM20948_ENABLED

#if AP_COMPASS_LIS3MDL_ENABLED
    // lis3mdl on bus 0 with default address
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        probe_i2c_dev(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe, i, HAL_COMPASS_LIS3MDL_I2C_ADDR, all_external, all_external?ROTATION_YAW_90:ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }

    // lis3mdl on bus 0 with alternate address
    FOREACH_I2C_INTERNAL(i) {
        probe_i2c_dev(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe, i, HAL_COMPASS_LIS3MDL_I2C_ADDR2, all_external, all_external?ROTATION_YAW_90:ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif
    // external lis3mdl on bus 1 with default address
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe, i, HAL_COMPASS_LIS3MDL_I2C_ADDR, true, ROTATION_YAW_90);
        RETURN_IF_NO_SPACE;
    }

    // external lis3mdl on bus 1 with alternate address
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe, i, HAL_COMPASS_LIS3MDL_I2C_ADDR2, true, ROTATION_YAW_90);
        RETURN_IF_NO_SPACE;
    }
#endif  // AP_COMPASS_LIS3MDL_ENABLED

#if AP_COMPASS_LIS2MDL_ENABLED
    // external lis2mdl
#if AP_COMPASS_LIS2MDL_EXTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_LIS2MDL, AP_Compass_LIS2MDL::probe, i, HAL_COMPASS_LIS2MDL_I2C_ADDR, true, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif
    // internal lis2mdl
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        probe_i2c_dev(DRIVER_LIS2MDL, AP_Compass_LIS2MDL::probe, i, HAL_COMPASS_LIS2MDL_I2C_ADDR, all_external, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif
#endif  // AP_COMPASS_LIS2MDL_ENABLED

#if AP_COMPASS_AK09916_ENABLED
    // AK09916. This can be found twice, due to the ICM20948 i2c bus pass-thru, so we need to be careful to avoid that
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_AK09916, AP_Compass_AK09916::probe, i, HAL_COMPASS_AK09916_I2C_ADDR, true, ROTATION_YAW_270);
        RETURN_IF_NO_SPACE;
    }
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        probe_i2c_dev(DRIVER_AK09916, AP_Compass_AK09916::probe, i, HAL_COMPASS_AK09916_I2C_ADDR, all_external, all_external?ROTATION_YAW_270:ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_AK09916_ENABLED

#if AP_COMPASS_IST8310_ENABLED
    // IST8310 on external and internal bus
    if (AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_FMUV5 &&
        AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_FMUV6) {
        enum Rotation default_rotation = AP_COMPASS_IST8310_DEFAULT_ROTATION;

        if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_AEROFC) {
            default_rotation = ROTATION_PITCH_180_YAW_90;
        }
        // probe all 4 possible addresses
        const uint8_t ist8310_addr[] = { 0x0C, 0x0D, 0x0E, 0x0F };

        for (uint8_t a=0; a<ARRAY_SIZE(ist8310_addr); a++) {
            FOREACH_I2C_EXTERNAL(i) {
                probe_i2c_dev(DRIVER_IST8310, AP_Compass_IST8310::probe, i, ist8310_addr[a], true, default_rotation);
                RETURN_IF_NO_SPACE;
            }
#if AP_COMPASS_IST8310_INTERNAL_BUS_PROBING_ENABLED
            FOREACH_I2C_INTERNAL(i) {
                probe_i2c_dev(DRIVER_IST8310, AP_Compass_IST8310::probe, i, ist8310_addr[a], all_external, default_rotation);
                RETURN_IF_NO_SPACE;
            }
#endif  // AP_COMPASS_IST8310_INTERNAL_BUS_PROBING_ENABLED
        }
    }
#endif  // AP_COMPASS_IST8310_ENABLED

#if AP_COMPASS_IST8308_ENABLED
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_IST8308, AP_Compass_IST8308::probe, i, HAL_COMPASS_IST8308_I2C_ADDR, true, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        probe_i2c_dev(DRIVER_IST8308, AP_Compass_IST8308::probe, i, HAL_COMPASS_IST8308_I2C_ADDR, all_external, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_IST8308_ENABLED

#if AP_COMPASS_MMC3416_ENABLED
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_MMC3416, AP_Compass_MMC3416::probe, i, HAL_COMPASS_MMC3416_I2C_ADDR, true, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        probe_i2c_dev(DRIVER_MMC3416, AP_Compass_MMC3416::probe, i, HAL_COMPASS_MMC3416_I2C_ADDR, all_external, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_MMC3416_ENABLED

#if AP_COMPASS_MMC5XX3_ENABLED
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        probe_i2c_dev(DRIVER_MMC5XX3, AP_Compass_MMC5XX3::probe, i, HAL_COMPASS_MMC5xx3_I2C_ADDR, true, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
    }
#endif  // AP_COMPASS_MMC5XX3_ENABLED (MMC5983MA)

#if AP_COMPASS_RM3100_ENABLED
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
            probe_i2c_dev(DRIVER_RM3100, AP_Compass_RM3100::probe, i, rm3100_addresses[j], true, ROTATION_NONE);
            RETURN_IF_NO_SPACE;
        }
    }

#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        for (uint8_t j=0; j<ARRAY_SIZE(rm3100_addresses); j++) {
            probe_i2c_dev(DRIVER_RM3100, AP_Compass_RM3100::probe, i, rm3100_addresses[j], all_external, ROTATION_NONE);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif  // AP_COMPASS_RM3100_ENABLED

#if AP_COMPASS_BMM150_ENABLED
    // BMM150 on I2C
    FOREACH_I2C_EXTERNAL(i) {
        for (uint8_t addr=BMM150_I2C_ADDR_MIN; addr <= BMM150_I2C_ADDR_MAX; addr++) {
            probe_i2c_dev(DRIVER_BMM150, AP_Compass_BMM150::probe, i, addr, true, ROTATION_NONE);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_BMM150_ENABLED

#if AP_COMPASS_BMM350_ENABLED
    // BMM350 on I2C
    FOREACH_I2C_EXTERNAL(i) {
        for (uint8_t addr=BMM350_I2C_ADDR_MIN; addr <= BMM350_I2C_ADDR_MAX; addr++) {
            probe_i2c_dev(DRIVER_BMM350, AP_Compass_BMM350::probe, i, addr, true, ROTATION_NONE);
            RETURN_IF_NO_SPACE;
        }
    }
#if AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
    FOREACH_I2C_INTERNAL(i) {
        for (uint8_t addr=BMM350_I2C_ADDR_MIN; addr <= BMM350_I2C_ADDR_MAX; addr++) {
            probe_i2c_dev(DRIVER_BMM350, AP_Compass_BMM350::probe, i, addr, all_external, ROTATION_NONE);
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#endif // AP_COMPASS_BMM350_ENABLED
}

/*
  detect available backends for this board
 */
void Compass::_detect_backends(void)
{
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    const int8_t serial_port = AP::externalAHRS().get_port(AP_ExternalAHRS::AvailableSensor::COMPASS);
    if (serial_port >= 0) {
        add_backend(DRIVER_EXTERNALAHRS, AP_Compass_ExternalAHRS::probe(serial_port));
        RETURN_IF_NO_SPACE;
    }
#endif
    
#if AP_FEATURE_BOARD_DETECT
    if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2) {
        // default to disabling LIS3MDL on pixhawk2 due to hardware issue
#if AP_COMPASS_LIS3MDL_ENABLED
    _driver_type_mask.set_default(1U<<DRIVER_LIS3MDL);
#endif
    }
#endif

#if AP_COMPASS_SITL_ENABLED && !AP_TEST_DRONECAN_DRIVERS
    // create several SITL compass backends:
    for (uint8_t i=0; i<MAX_CONNECTED_MAGS; i++) {
        add_backend(DRIVER_SITL, NEW_NOTHROW AP_Compass_SITL(i));
        RETURN_IF_NO_SPACE;
    }
#endif

#if AP_COMPASS_DRONECAN_ENABLED
    // probe DroneCAN before I2C and SPI so that DroneCAN compasses
    // default to first in the list for a new board
    probe_dronecan_compasses();
    RETURN_IF_NO_SPACE;
#endif

#if AP_COMPASS_PROBING_ENABLED
    // allow boards to ask for external probing of all i2c compass types in hwdef.dat
    _probe_external_i2c_compasses();
    RETURN_IF_NO_SPACE;
#endif  // AP_COMPASS_PROBING_ENABLED

#if AP_COMPASS_MSP_ENABLED
    for (uint8_t i=0; i<8; i++) {
        if (msp_instance_mask & (1U<<i)) {
            auto *backend = AP_Compass_MSP::probe(i);
            if (backend == nullptr) {
                continue;
            }
            add_backend(DRIVER_MSP, backend);
            RETURN_IF_NO_SPACE;
        }
    }
#endif

    // finally look for i2c and spi compasses not found yet
    RETURN_IF_NO_SPACE;
    probe_i2c_spi_compasses();

    if (_backend_count == 0 ||
        _compass_count == 0) {
        DEV_PRINTF("No Compass backends available\n");
    }
}

/*
  probe i2c and SPI compasses
 */
void Compass::probe_i2c_spi_compasses(void)
{
#if defined(HAL_MAG_PROBE_LIST)
    // driver probes defined by COMPASS lines in hwdef.dat
    HAL_MAG_PROBE_LIST;
#elif AP_FEATURE_BOARD_DETECT
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PX4V1:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
    case AP_BoardConfig::PX4_BOARD_PHMINI:
    case AP_BoardConfig::PX4_BOARD_AUAV21:
    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
    case AP_BoardConfig::PX4_BOARD_AEROFC:
        _probe_external_i2c_compasses();
        RETURN_IF_NO_SPACE;
        break;

    default:
        break;
    }
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
#if AP_COMPASS_HMC5843_ENABLED
        probe_spi_dev(DRIVER_HMC5843, AP_Compass_HMC5843::probe, HAL_COMPASS_HMC5843_NAME, false, ROTATION_PITCH_180);
        RETURN_IF_NO_SPACE;
#endif
#if AP_COMPASS_LSM303D_ENABLED
        probe_spi_dev(DRIVER_LSM303D, AP_Compass_LSM303D::probe, HAL_INS_LSM9DS0_A_NAME, ROTATION_NONE);
        RETURN_IF_NO_SPACE;
#endif
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
#if AP_COMPASS_LSM303D_ENABLED
        probe_spi_dev(DRIVER_LSM303D, AP_Compass_LSM303D::probe, HAL_INS_LSM9DS0_EXT_A_NAME, ROTATION_YAW_270);
        RETURN_IF_NO_SPACE;
#endif
#if AP_COMPASS_AK8963_ENABLED
        // we run the AK8963 only on the 2nd MPU9250, which leaves the
        // first MPU9250 to run without disturbance at high rate
        probe_ak8963_via_mpu9250(1, ROTATION_YAW_270);
        RETURN_IF_NO_SPACE;
#endif
#if AP_COMPASS_AK09916_ENABLED && AP_COMPASS_ICM20948_ENABLED
        probe_ak09916_via_icm20948(0, ROTATION_ROLL_180_YAW_90);
        RETURN_IF_NO_SPACE;
#endif
        break;

    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
#if AP_COMPASS_IST8310_ENABLED
        FOREACH_I2C_EXTERNAL(i) {
            probe_i2c_dev(DRIVER_IST8310, AP_Compass_IST8310::probe, i, HAL_COMPASS_IST8310_I2C_ADDR, true, ROTATION_ROLL_180_YAW_90);
            RETURN_IF_NO_SPACE;
        }
        FOREACH_I2C_INTERNAL(i) {
            probe_i2c_dev(DRIVER_IST8310, AP_Compass_IST8310::probe, i, HAL_COMPASS_IST8310_I2C_ADDR, false, ROTATION_ROLL_180_YAW_90);
            RETURN_IF_NO_SPACE;
        }
#endif  // AP_COMPASS_IST8310_ENABLED
        break;

    case AP_BoardConfig::PX4_BOARD_PHMINI:
#if AP_COMPASS_AK8963_ENABLED
        probe_ak8963_via_mpu9250(0, ROTATION_ROLL_180);
        RETURN_IF_NO_SPACE;
#endif
        break;

    case AP_BoardConfig::PX4_BOARD_AUAV21:
#if AP_COMPASS_AK8963_ENABLED
        probe_ak8963_via_mpu9250(0, ROTATION_ROLL_180_YAW_90);
        RETURN_IF_NO_SPACE;
#endif
        break;

    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
#if AP_COMPASS_AK8963_ENABLED
        probe_ak8963_via_mpu9250(0, ROTATION_YAW_270);
        RETURN_IF_NO_SPACE;
#endif
        break;

    default:
        break;
    }
#endif
}

#if AP_COMPASS_AK8963_ENABLED
void Compass::probe_ak8963_via_mpu9250(uint8_t imu_instance, Rotation rotation)
{
    if (!_driver_enabled(DRIVER_AK8963)) {
        return;
    }
    auto *backend = AP_Compass_AK8963::probe_mpu9250(imu_instance, rotation);
    add_backend(DRIVER_AK8963, backend);  // add_backend does nullptr check
}
#endif  // AP_COMPASS_AK8963_ENABLED

#if AP_COMPASS_AK09916_ENABLED && AP_COMPASS_ICM20948_ENABLED
void Compass::probe_ak09916_via_icm20948(uint8_t i2c_bus, uint8_t ak09916_addr, uint8_t icm20948_addr, bool external, Rotation rotation)
{
    if (!_driver_enabled(DRIVER_ICM20948)) {
        return;
    }
    auto *backend = AP_Compass_AK09916::probe_ICM20948(
        GET_I2C_DEVICE(i2c_bus, ak09916_addr),
        GET_I2C_DEVICE(i2c_bus, icm20948_addr),
        external,
        rotation
        );

    add_backend(DRIVER_ICM20948, backend);  // add_backend does nullptr check
}

void Compass::probe_ak09916_via_icm20948(uint8_t ins_instance, Rotation rotation)
{
    if (!_driver_enabled(DRIVER_AK09916)) {
        return;
    }

    auto *backend = AP_Compass_AK09916::probe_ICM20948(ins_instance, rotation);

    add_backend(DRIVER_AK09916, backend);
}

#endif  // #if AP_COMPASS_AK09916_ENABLED && AP_COMPASS_ICM20948_ENABLED

void Compass::probe_i2c_dev(DriverType driver_type, probe_i2c_dev_probefn_t probefn, uint8_t i2c_bus, uint8_t i2c_addr, bool external, Rotation rotation)
{
    if (!_driver_enabled(driver_type)) {
        return;
    }
    auto *backend = probefn(
        GET_I2C_DEVICE(i2c_bus, i2c_addr),
        external,
        rotation
        );

    add_backend(driver_type, backend);  // add_backend does nullptr check
}

void Compass::probe_spi_dev(DriverType driver_type, probe_spi_dev_probefn_t probefn, const char *name, bool external, Rotation rotation)
{
    if (!_driver_enabled(driver_type)) {
        return;
    }
    auto *backend = probefn(hal.spi->get_device(name), external, rotation);

    add_backend(driver_type, backend);  // add_backend does nullptr check
}

// short-lived method which expectes a probe function that doesn't
// offer the ability to specify an external rotation:
void Compass::probe_spi_dev(DriverType driver_type, probe_spi_dev_noexternal_probefn_t probefn, const char *name, Rotation rotation)
{
    if (!_driver_enabled(driver_type)) {
        return;
    }
    auto *backend = probefn(hal.spi->get_device(name), rotation);

    add_backend(driver_type, backend);  // add_backend does nullptr check
}

#if AP_COMPASS_DRONECAN_ENABLED
/*
  look for DroneCAN compasses
 */
void Compass::probe_dronecan_compasses(void)
{
    if (!_driver_enabled(DRIVER_UAVCAN)) {
        return;
    }
    for (uint8_t i=0; i<MAX_CONNECTED_MAGS; i++) {
        AP_Compass_Backend* _uavcan_backend = AP_Compass_DroneCAN::probe(i);
        if (_uavcan_backend) {
            add_backend(DRIVER_UAVCAN, _uavcan_backend);
        }
#if COMPASS_MAX_UNREG_DEV > 0
        if (_unreg_compass_count == COMPASS_MAX_UNREG_DEV)  {
            break;
        }
#endif
    }

#if COMPASS_MAX_UNREG_DEV > 0
    if (option_set(Option::ALLOW_DRONECAN_AUTO_REPLACEMENT) && !suppress_devid_save) {
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
                uint32_t detected_devid = AP_Compass_DroneCAN::get_detected_devid(j);
                // Check if this is a potential replacement mag
                if (!is_replacement_mag(detected_devid)) {
                    continue;
                }
                // We have found a replacement mag, let's replace the existing one
                // with this by setting the priority to zero and calling uavcan probe
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Mag: Compass #%d with DEVID %lu replaced", uint8_t(i), (unsigned long)_priority_did_list[i]);
                _priority_did_stored_list[i].set_and_save(0);
                _priority_did_list[i] = 0;

                AP_Compass_Backend* _uavcan_backend = AP_Compass_DroneCAN::probe(j);
                if (_uavcan_backend) {
                    add_backend(DRIVER_UAVCAN, _uavcan_backend);
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
    }
#endif  // #if COMPASS_MAX_UNREG_DEV > 0
}
#endif  // AP_COMPASS_DRONECAN_ENABLED


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

#if COMPASS_MAX_UNREG_DEV > 0
    // Check that its not an unused additional mag
    for (uint8_t i = 0; i<COMPASS_MAX_UNREG_DEV; i++) {
        if (_previously_unreg_mag[i] == devid) {
            return false;
        }
    }
#endif

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

#if COMPASS_MAX_UNREG_DEV > 0
    for (uint8_t i = 0; i<COMPASS_MAX_UNREG_DEV; i++) {
        if ((uint32_t)extra_dev_id[i] == devid) {
            extra_dev_id[i].set(0);
            return;
        }
    }
#endif
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
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Mag: Compass #%d with DEVID %lu removed", uint8_t(i), (unsigned long)_priority_did_list[i]);
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
#if AP_COMPASS_DRONECAN_ENABLED
    if (!available()) {
        return;
    }

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
            AP_Compass_Backend* _uavcan_backend = AP_Compass_DroneCAN::probe(i);
            if (_uavcan_backend) {
                add_backend(DRIVER_UAVCAN, _uavcan_backend);
            }
            RETURN_IF_NO_SPACE;
        }
    }
#endif  // AP_COMPASS_DRONECAN_ENABLED
}

bool
Compass::read(void)
{
    if (!available()) {
        return false;
    }

#if HAL_LOGGING_ENABLED
    const bool old_healthy = healthy();
#endif

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
    if (_learn == LearnType::INFLIGHT && !learn_allocated) {
        learn_allocated = true;
        learn = NEW_NOTHROW CompassLearn(*this);
    }
    if (_learn == LearnType::INFLIGHT && learn != nullptr) {
        learn->update();
    }
#endif
#if HAL_LOGGING_ENABLED
    if (any_healthy && _log_bit != (uint32_t)-1 && AP::logger().should_log(_log_bit)) {
        AP::logger().Write_Compass();
    }
#endif

    // Set _first_usable parameter
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_use_for_yaw[i]) {
            _first_usable = uint8_t(i);
            break;
        }
    }
    const bool new_healthy = healthy();

#if HAL_LOGGING_ENABLED

    #define MASK_LOG_ANY                    0xFFFF

    if (new_healthy != old_healthy) {
        if (AP::logger().should_log(MASK_LOG_ANY)) {
            const LogErrorCode code = new_healthy ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY;
            AP::logger().Write_Error(LogErrorSubsystem::COMPASS, code);
        }
    }
#endif

    return new_healthy;
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

#if AP_COMPASS_DIAGONALS_ENABLED
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
#endif // AP_COMPASS_DIAGONALS_ENABLED

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
    if (!AP::ahrs().get_location(loc)) {
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
    return healthy(_first_usable) && use_for_yaw(_first_usable);
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
    if (!available()) {
        return false;
    }
    // when we are doing in-flight compass learning the state
    // estimator must not use the compass. The learning code turns off
    // inflight learning when it has converged
    return _use_for_yaw[Priority(i)] && _learn != LearnType::INFLIGHT;
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
/*
    This extracts a roll/pitch-only rotation which is then used to rotate the body frame field into earth frame so the heading can be calculated.
    One could do:
        float roll, pitch, yaw;
        dcm_matrix.to_euler(roll, pitch, yaw)
        Matrix3f rp_rot;
        rp_rot.from_euler(roll, pitch, 0)
        Vector3f ef = rp_rot * field

    Because only the X and Y components are needed it's more efficient to manually calculate:

        rp_rot = [ cos(pitch), sin(roll) * sin(pitch),  cos(roll) * sin(pitch)
                            0,              cos(roll),              -sin(roll)]

    If the whole matrix is multiplied by cos(pitch) the required trigonometric values can be extracted directly from the existing dcm matrix.
    This multiplication has no effect on the calculated heading as it changes the length of the North/East vector but not its angle.

        rp_rot = [ cos(pitch)^2, sin(roll) * sin(pitch) * cos(pitch),  cos(roll) * sin(pitch) * cos(pitch)
                              0,              cos(roll) * cos(pitch),              -sin(roll) * cos(pitch)]

    Preexisting values can be substituted in:

        dcm_matrix.c.x = -sin(pitch)
        dcm_matrix.c.y =  sin(roll) * cos(pitch)
        dcm_matrix.c.z =  cos(roll) * cos(pitch)

        rp_rot = [ cos(pitch)^2, dcm_matrix.c.y * -dcm_matrix.c.x,  dcm_matrix.c.z * -dcm_matrix.c.x
                              0,                   dcm_matrix.c.z,                   -dcm_matrix.c.y]

    cos(pitch)^2 is stil needed. This is the same as 1 - sin(pitch)^2.
    sin(pitch) is avalable as dcm_matrix.c.x
*/

    const float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    const Vector3f &field = get_field(i);

    const float headY = field.y * dcm_matrix.c.z - field.z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    const float headX = field.x * cos_pitch_sq - dcm_matrix.c.x * (field.y * dcm_matrix.c.y + field.z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    const float heading = constrain_float(atan2f(-headY,headX), -M_PI, M_PI);

    // Declination correction
    return wrap_PI(heading + _declination);
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

#ifdef HAL_USE_EMPTY_STORAGE
    // the load() call below returns zeroes on empty storage, so the
    // check-stored-value check here will always fail.  Since nobody
    // really cares about the empty-storage case, shortcut out here
    // for simplicity.
    return true;
#endif

    // back up cached value of dev_id
    int32_t dev_id_cache_value = _state[id].dev_id;

    // load dev_id from eeprom
    _state[id].dev_id.load();

    // if dev_id loaded from eeprom is different from detected dev id or dev_id loaded from eeprom is different from cached dev_id, compass is unconfigured
    if (_state[id].dev_id != _state[id].detected_dev_id || _state[id].dev_id != dev_id_cache_value) {
        // restore cached value
        _state[id].dev_id.set(dev_id_cache_value);
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
                snprintf(failure_msg, failure_msg_len, "Compass %d not found", uint8_t(i + 1));
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
        if (configured(i)) {
            continue;
        }
        if (!use_for_yaw(i)) {
            // we're not planning on using this anyway so sure,
            // whatever, it's configured....
            continue;
        }
        all_configured = false;
        break;
    }
    if (!all_configured) {
        snprintf(failure_msg, failure_msg_len, "Compass not calibrated");
    }
    return all_configured;
}

/*
  set the type of motor compensation to use
 */
void Compass::motor_compensation_type(const uint8_t comp_type)
{
    if (_motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
        _motor_comp_type.set((int8_t)comp_type);
        _thr = 0; // set current  throttle to zero
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
        }
    }
}

bool Compass::consistent() const
{
    const Vector3f &primary_mag_field { get_field() };
    const Vector2f &primary_mag_field_xy { primary_mag_field.xy() };

    for (uint8_t i=0; i<get_count(); i++) {
        if (!use_for_yaw(i)) {
            // configured not-to-be-used
            continue;
        }

        const Vector3f &mag_field = get_field(i);
        const Vector2f &mag_field_xy = mag_field.xy();

        if (mag_field_xy.is_zero()) {
            return false;
        }

        // check for gross misalignment on all axes
        const float xyz_ang_diff  = mag_field.angle(primary_mag_field);
        if (xyz_ang_diff > AP_COMPASS_MAX_XYZ_ANG_DIFF) {
            return false;
        }

        // check for an unacceptable angle difference on the xy plane
        const float xy_ang_diff  = mag_field_xy.angle(primary_mag_field_xy);
        if (xy_ang_diff > AP_COMPASS_MAX_XY_ANG_DIFF) {
            return false;
        }

        // check for an unacceptable length difference on the xy plane
        const float xy_len_diff = (primary_mag_field_xy-mag_field_xy).length();
        if (xy_len_diff > AP_COMPASS_MAX_XY_LENGTH_DIFF) {
            return false;
        }
    }
    return true;
}

bool Compass::healthy(uint8_t i) const
{
    return (i < COMPASS_MAX_INSTANCES) ? _get_state(Priority(i)).healthy : false;
}

/*
  return true if we have a valid scale factor
 */
bool Compass::have_scale_factor(uint8_t i) const
{
    if (!available()) {
        return false;
    }
    StateIndex id = _get_state_id(Priority(i));
    if (id >= COMPASS_MAX_INSTANCES ||
        _state[id].scale_factor < COMPASS_MIN_SCALE_FACTOR ||
        _state[id].scale_factor > COMPASS_MAX_SCALE_FACTOR) {
        return false;
    }
    return true;
}

#if AP_COMPASS_MSP_ENABLED
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
#endif // AP_COMPASS_MSP_ENABLED

#if AP_COMPASS_EXTERNALAHRS_ENABLED
void Compass::handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt)
{
    if (!_driver_enabled(DRIVER_EXTERNALAHRS)) {
        return;
    }
    for (uint8_t i=0; i<_backend_count; i++) {
        _backends[i]->handle_external(pkt);
    }
}
#endif // AP_COMPASS_EXTERNALAHRS_ENABLED

// force save of current calibration as valid
void Compass::force_save_calibration(void)
{
    for (StateIndex i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_state[i].dev_id != 0) {
            _state[i].dev_id.save();
        }
    }
}

// singleton instance
Compass *Compass::_singleton;

namespace AP
{

Compass &compass()
{
    return *Compass::get_singleton();
}

}

#endif  // AP_COMPASS_ENABLED
