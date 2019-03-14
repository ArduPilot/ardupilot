#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/I2CDevice.h>
#endif
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

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
#if HAL_WITH_UAVCAN
#include "AP_Compass_UAVCAN.h"
#endif
#include "AP_Compass_MMC3416.h"
#include "AP_Compass_MAG3110.h"
#include "AP_Compass_RM3100.h"
#include "AP_Compass.h"
#include "Compass_learn.h"

extern AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define COMPASS_LEARN_DEFAULT Compass::LEARN_NONE
#else
#define COMPASS_LEARN_DEFAULT Compass::LEARN_INTERNAL
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

    // @Param: OFS_Y
    // @DisplayName: Compass offsets in milligauss on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS_Z
    // @DisplayName: Compass offsets in milligauss on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS",    1, Compass, _state[0].offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: rad
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),

    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle. If InFlight learning is enabled then the compass with automatically start learning once a flight starts (must be armed). While InFlight learning is running you cannot use position control modes.
    // @Values: 0:Disabled,1:Internal-Learning,2:EKF-Learning,3:InFlight-Learning
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, COMPASS_LEARN_DEFAULT),

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, Compass, _state[0].use_for_yaw, 1), // true if used for DCM yaw

    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, Compass, _auto_declination, 1),

    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @User: Advanced
    AP_GROUPINFO("MOTCT",    6, Compass, _motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT",    7, Compass, _state[0].motor_compensation, 0),

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 8, Compass, _state[0].orientation, ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, Compass, _state[0].external, 0),

    // @Param: OFS2_X
    // @DisplayName: Compass2 offsets in milligauss on the X axis
    // @Description: Offset to be added to compass2's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS2_Y
    // @DisplayName: Compass2 offsets in milligauss on the Y axis
    // @Description: Offset to be added to compass2's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS2_Z
    // @DisplayName: Compass2 offsets in milligauss on the Z axis
    // @Description: Offset to be added to compass2's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS2",    10, Compass, _state[1].offset, 0),

    // @Param: MOT2_X
    // @DisplayName: Motor interference compensation to compass2 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT2_Y
    // @DisplayName: Motor interference compensation to compass2 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT2_Z
    // @DisplayName: Motor interference compensation to compass2 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT2",    11, Compass, _state[1].motor_compensation, 0),

    // @Param: PRIMARY
    // @DisplayName: Choose primary compass
    // @Description: If more than one compass is available, this selects which compass is the primary. When external compasses are connected, they will be ordered first. NOTE: If no external compass is attached, this parameter is ignored.
    // @Values: 0:FirstCompass,1:SecondCompass,2:ThirdCompass
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 12, Compass, _primary, 0),

    // @Param: OFS3_X
    // @DisplayName: Compass3 offsets in milligauss on the X axis
    // @Description: Offset to be added to compass3's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS3_Y
    // @DisplayName: Compass3 offsets in milligauss on the Y axis
    // @Description: Offset to be added to compass3's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS3_Z
    // @DisplayName: Compass3 offsets in milligauss on the Z axis
    // @Description: Offset to be added to compass3's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS3",    13, Compass, _state[2].offset, 0),

    // @Param: MOT3_X
    // @DisplayName: Motor interference compensation to compass3 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT3_Y
    // @DisplayName: Motor interference compensation to compass3 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT3_Z
    // @DisplayName: Motor interference compensation to compass3 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT3",    14, Compass, _state[2].motor_compensation, 0),

    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  15, Compass, _state[0].dev_id, 0),

    // @Param: DEV_ID2
    // @DisplayName: Compass2 device id
    // @Description: Second compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID2", 16, Compass, _state[1].dev_id, 0),

    // @Param: DEV_ID3
    // @DisplayName: Compass3 device id
    // @Description: Third compass's device id.  Automatically detected, do not set manually
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("DEV_ID3", 17, Compass, _state[2].dev_id, 0),

    // @Param: USE2
    // @DisplayName: Compass2 used for yaw
    // @Description: Enable or disable the second compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2",    18, Compass, _state[1].use_for_yaw, 1),

    // @Param: ORIENT2
    // @DisplayName: Compass2 orientation
    // @Description: The orientation of a second external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315
    // @User: Advanced
    AP_GROUPINFO("ORIENT2", 19, Compass, _state[1].orientation, ROTATION_NONE),

    // @Param: EXTERN2
    // @DisplayName: Compass2 is attached via an external cable
    // @Description: Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN2",20, Compass, _state[1].external, 0),

    // @Param: USE3
    // @DisplayName: Compass3 used for yaw
    // @Description: Enable or disable the third compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3",    21, Compass, _state[2].use_for_yaw, 1),

    // @Param: ORIENT3
    // @DisplayName: Compass3 orientation
    // @Description: The orientation of a third external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315
    // @User: Advanced
    AP_GROUPINFO("ORIENT3", 22, Compass, _state[2].orientation, ROTATION_NONE),

    // @Param: EXTERN3
    // @DisplayName: Compass3 is attached via an external cable
    // @Description: Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN3",23, Compass, _state[2].external, 0),

    // @Param: DIA_X
    // @DisplayName: Compass soft-iron diagonal X component
    // @Description: DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA_Y
    // @DisplayName: Compass soft-iron diagonal Y component
    // @Description: DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA_Z
    // @DisplayName: Compass soft-iron diagonal Z component
    // @Description: DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA",    24, Compass, _state[0].diagonals, 0),

    // @Param: ODI_X
    // @DisplayName: Compass soft-iron off-diagonal X component
    // @Description: ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI_Y
    // @DisplayName: Compass soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI_Z
    // @DisplayName: Compass soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI",    25, Compass, _state[0].offdiagonals, 0),

    // @Param: DIA2_X
    // @DisplayName: Compass2 soft-iron diagonal X component
    // @Description: DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA2_Y
    // @DisplayName: Compass2 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA2_Z
    // @DisplayName: Compass2 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA2",    26, Compass, _state[1].diagonals, 0),

    // @Param: ODI2_X
    // @DisplayName: Compass2 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI2_Y
    // @DisplayName: Compass2 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI2_Z
    // @DisplayName: Compass2 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI2",    27, Compass, _state[1].offdiagonals, 0),

    // @Param: DIA3_X
    // @DisplayName: Compass3 soft-iron diagonal X component
    // @Description: DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA3_Y
    // @DisplayName: Compass3 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA3_Z
    // @DisplayName: Compass3 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA3",    28, Compass, _state[2].diagonals, 0),

    // @Param: ODI3_X
    // @DisplayName: Compass3 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI3_Y
    // @DisplayName: Compass3 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI3_Z
    // @DisplayName: Compass3 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI3",    29, Compass, _state[2].offdiagonals, 0),

    // @Param: CAL_FIT
    // @DisplayName: Compass calibration fitness
    // @Description: This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.
    // @Range: 4 32
    // @Values: 4:Very Strict,8:Strict,16:Default,32:Relaxed
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("CAL_FIT", 30, Compass, _calibration_threshold, AP_COMPASS_CALIBRATION_FITNESS_DEFAULT),

    // @Param: OFFS_MAX
    // @DisplayName: Compass maximum offset
    // @Description: This sets the maximum allowed compass offset in calibration and arming checks
    // @Range: 500 3000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFFS_MAX", 31, Compass, _offset_max, AP_COMPASS_OFFSETS_MAX_DEFAULT),

    // @Group: PMOT
    // @Path: Compass_PerMotor.cpp
    AP_SUBGROUPINFO(_per_motor, "PMOT", 32, Compass, Compass_PerMotor),

    // @Param: TYPEMASK
    // @DisplayName: Compass disable driver type mask
    // @Description: This is a bitmask of driver types to disable. If a driver type is set in this mask then that driver will not try to find a sensor at startup
    // @Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK09916,7:IST8310,8:ICM20948,9:MMC3416,11:UAVCAN,12:QMC5883,14:MAG3110,15:IST8308
    // @User: Advanced
    AP_GROUPINFO("TYPEMASK", 33, Compass, _driver_type_mask, 0),

    // @Param: FLTR_RNG
    // @DisplayName: Range in which sample is accepted
    // @Description: This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    AP_GROUPINFO("FLTR_RNG", 34, Compass, _filter_range, HAL_COMPASS_FILTER_DEFAULT),

    // @Param: AUTO_ROT
    // @DisplayName: Automatically check orientation
    // @Description: When enabled this will automatically check the orientation of compasses on successful completion of compass calibration. If set to 2 then external compasses will have their orientation automatically corrected.
    // @Values: 0:Disabled,1:CheckOnly,2:CheckAndFix
    AP_GROUPINFO("AUTO_ROT", 35, Compass, _rotate_auto, HAL_COMPASS_AUTO_ROT_DEFAULT),

    // @Param: EXP_DID
    // @DisplayName: Compass device id expected
    // @Description: The expected value of COMPASS_DEV_ID, used by arming checks. Setting this to -1 means "don't care."
    // @User: Advanced
    AP_GROUPINFO("EXP_DID",  36, Compass, _state[0].expected_dev_id, -1),

    // @Param: EXP_DID2
    // @DisplayName: Compass2 device id expected
    // @Description: The expected value of COMPASS_DEV_ID2, used by arming checks. Setting this to -1 means "don't care."
    // @User: Advanced
    AP_GROUPINFO("EXP_DID2", 37, Compass, _state[1].expected_dev_id, -1),

    // @Param: EXP_DID3
    // @DisplayName: Compass3 device id expected
    // @Description: The expected value of COMPASS_DEV_ID3, used by arming checks. Setting this to -1 means "don't care."
    // @User: Advanced
    AP_GROUPINFO("EXP_DID3", 38, Compass, _state[2].expected_dev_id, -1),
    
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
    for (uint8_t i=_compass_count; i<COMPASS_MAX_INSTANCES; i++) {
        _state[i].dev_id.set(0);
    }
}

//  Register a new compass instance
//
uint8_t Compass::register_compass(void)
{
    if (_compass_count == COMPASS_MAX_INSTANCES) {
        AP_HAL::panic("Too many compass instances");
    }
    return _compass_count++;
}

bool Compass::_add_backend(AP_Compass_Backend *backend)
{
    if (!backend) {
        return false;
    }

    if (_backend_count == COMPASS_MAX_BACKEND) {
        AP_HAL::panic("Too many compass backends");
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
    for (uint8_t i=0; i<_compass_count; i++) {
        if (AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_I2C, bus, address, 0) ==
            AP_HAL::Device::change_bus_id(uint32_t(_state[i].dev_id.get()), 0)) {
            // we are already using this device
            return true;
        }
    }
    return false;
}

/*
  macro to add a backend with check for too many backends or compass
  instances. We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(driver_type, backend)   \
    do { if (_driver_enabled(driver_type)) { _add_backend(backend); } \
       if (_backend_count == COMPASS_MAX_BACKEND || \
           _compass_count == COMPASS_MAX_INSTANCES) { \
          return; \
        } \
    } while (0)

#define GET_I2C_DEVICE(bus, address) _have_i2c_driver(bus, address)?nullptr:hal.i2c_mgr->get_device(bus, address)

/*
  look for compasses on external i2c buses
 */
void Compass::_probe_external_i2c_compasses(void)
{
    bool all_external = (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2);
    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(i, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                              true, ROTATION_ROLL_180));
    }
    
    if (AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_MINDPXV2 &&
        AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_AEROFC) {
        // internal i2c bus
        FOREACH_I2C_INTERNAL(i) {
            ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(i, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                                  all_external, all_external?ROTATION_ROLL_180:ROTATION_YAW_270));
        }
    }
    
    //external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_QMC5883, AP_Compass_QMC5883L::probe(GET_I2C_DEVICE(i, HAL_COMPASS_QMC5883L_I2C_ADDR),
                                                               true, HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL));
    }
    
    //internal i2c bus
    FOREACH_I2C_INTERNAL(i) {
        ADD_BACKEND(DRIVER_QMC5883, AP_Compass_QMC5883L::probe(GET_I2C_DEVICE(i, HAL_COMPASS_QMC5883L_I2C_ADDR),
                                                               all_external,
                                                               all_external?HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL:HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL));
    }
    
#if !HAL_MINIMIZE_FEATURES
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
        
    // AK09916. This can be found twice, due to the ICM20948 i2c bus pass-thru, so we need to be careful to avoid that
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe(GET_I2C_DEVICE(i, HAL_COMPASS_AK09916_I2C_ADDR),
                                                              true, ROTATION_YAW_270));
    }
    FOREACH_I2C_INTERNAL(i) {
        ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe(GET_I2C_DEVICE(i, HAL_COMPASS_AK09916_I2C_ADDR),
                                                              all_external, all_external?ROTATION_YAW_270:ROTATION_NONE));
    }
    
    // IST8310 on external and internal bus
    if (AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_FMUV5 &&
        AP_BoardConfig::get_board_type() != AP_BoardConfig::PX4_BOARD_FMUV6) {
        enum Rotation default_rotation;

        if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_AEROFC) {
            default_rotation = ROTATION_PITCH_180_YAW_90;
        } else {
            default_rotation = ROTATION_PITCH_180;
        }

        FOREACH_I2C_EXTERNAL(i) {
            ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(i, HAL_COMPASS_IST8310_I2C_ADDR),
                                                                  true, default_rotation));
        }
        
        FOREACH_I2C_INTERNAL(i) {
            ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(i, HAL_COMPASS_IST8310_I2C_ADDR),
                                                                  all_external, default_rotation));
        }
    }

    // external i2c bus
    FOREACH_I2C_EXTERNAL(i) {
        ADD_BACKEND(DRIVER_IST8308, AP_Compass_IST8308::probe(GET_I2C_DEVICE(i, HAL_COMPASS_IST8308_I2C_ADDR),
                                                              true, ROTATION_NONE));
    }
#endif // HAL_MINIMIZE_FEATURES
}

/*
  detect available backends for this board
 */
void Compass::_detect_backends(void)
{
    if (_hil_mode) {
        _add_backend(AP_Compass_HIL::detect());
        return;
    }

#if AP_FEATURE_BOARD_DETECT
    if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2) {
        // default to disabling LIS3MDL on pixhawk2 due to hardware issue
        _driver_type_mask.set_default(1U<<DRIVER_LIS3MDL);
    }
#endif
    
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    ADD_BACKEND(DRIVER_SITL, new AP_Compass_SITL());
    return;
#endif

#ifdef HAL_PROBE_EXTERNAL_I2C_COMPASSES
    // allow boards to ask for external probing of all i2c compass types in hwdef.dat
    _probe_external_i2c_compasses();
    if (_backend_count == COMPASS_MAX_BACKEND ||
        _compass_count == COMPASS_MAX_INSTANCES) {
        return;
    }
#endif
    
#if HAL_COMPASS_DEFAULT == HAL_COMPASS_HIL
    ADD_BACKEND(DRIVER_SITL, AP_Compass_HIL::detect());
#elif AP_FEATURE_BOARD_DETECT
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PX4V1:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
    case AP_BoardConfig::PX4_BOARD_PHMINI:
    case AP_BoardConfig::PX4_BOARD_AUAV21:
    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
    case AP_BoardConfig::PX4_BOARD_PIXRACER: 
    case AP_BoardConfig::PX4_BOARD_MINDPXV2: 
    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK_PRO:
    case AP_BoardConfig::PX4_BOARD_AEROFC:
        _probe_external_i2c_compasses();
        if (_backend_count == COMPASS_MAX_BACKEND ||
            _compass_count == COMPASS_MAX_INSTANCES) {
            return;
        }
        break;

    case AP_BoardConfig::PX4_BOARD_PCNC1:
        ADD_BACKEND(DRIVER_BMM150,
                    AP_Compass_BMM150::probe(GET_I2C_DEVICE(0, 0x10)));
        break;
    case AP_BoardConfig::VRX_BOARD_BRAIN54: {
        // external i2c bus
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(1, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                              true, ROTATION_ROLL_180));
        }
        // internal i2c bus
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(0, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                              false, ROTATION_YAW_270));
        break;

    case AP_BoardConfig::VRX_BOARD_BRAIN51:
    case AP_BoardConfig::VRX_BOARD_BRAIN52:
    case AP_BoardConfig::VRX_BOARD_BRAIN52E:
    case AP_BoardConfig::VRX_BOARD_CORE10:
    case AP_BoardConfig::VRX_BOARD_UBRAIN51:
    case AP_BoardConfig::VRX_BOARD_UBRAIN52: {
        // external i2c bus
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(1, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                              true, ROTATION_ROLL_180));
        }
        break;

    default:
        break;
    }
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(hal.spi->get_device(HAL_COMPASS_HMC5843_NAME),
                                                              false, ROTATION_PITCH_180));
        ADD_BACKEND(DRIVER_LSM303D, AP_Compass_LSM303D::probe(hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME)));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        ADD_BACKEND(DRIVER_LSM303D, AP_Compass_LSM303D::probe(hal.spi->get_device(HAL_INS_LSM9DS0_EXT_A_NAME), ROTATION_YAW_270));
        // we run the AK8963 only on the 2nd MPU9250, which leaves the
        // first MPU9250 to run without disturbance at high rate
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(1, ROTATION_YAW_270));
        ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe_ICM20948(0, ROTATION_ROLL_180_YAW_270));
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
        
    case AP_BoardConfig::PX4_BOARD_PIXRACER:
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(hal.spi->get_device(HAL_COMPASS_HMC5843_NAME),
                                                              false, ROTATION_PITCH_180));
        // R15 has LIS3MDL on spi bus instead of HMC; same CS pin
        ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(hal.spi->get_device(HAL_COMPASS_LIS3MDL_NAME),
                                                              false, ROTATION_NONE));
        ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_ROLL_180_YAW_90));
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
        ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(0, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                              false, ROTATION_YAW_90));
        ADD_BACKEND(DRIVER_LSM303D, AP_Compass_LSM303D::probe(hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME), ROTATION_PITCH_180_YAW_270));
        break;
        
    default:
        break;
    }

#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_BH
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR)));
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_BBBMINI
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(1));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_NAVIO2
    ADD_BACKEND(DRIVER_LSM9DS1, AP_Compass_LSM9DS1::probe(hal.spi->get_device("lsm9ds1_m"), ROTATION_ROLL_180));
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_NAVIO
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_OCPOC_ZYNQ
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR)));
    ADD_BACKEND(DRIVER_AK8963,AP_Compass_AK8963::probe_mpu9250(0));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_EDGE
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(GET_I2C_DEVICE(HAL_COMPASS_AK8963_I2C_BUS, HAL_COMPASS_AK8963_I2C_ADDR)));
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_AK8963_MPU9250
#ifndef HAL_COMPASS_AK8963_MPU9250_ROTATION
#define HAL_COMPASS_AK8963_MPU9250_ROTATION ROTATION_NONE
#endif
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, HAL_COMPASS_AK8963_MPU9250_ROTATION));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HMC5843
#ifndef HAL_COMPASS_HMC5843_ROTATION
# define HAL_COMPASS_HMC5843_ROTATION ROTATION_NONE
#endif
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                          false, HAL_COMPASS_HMC5843_ROTATION));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HMC5843_MPU6000
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe_mpu6000());
#elif  HAL_COMPASS_DEFAULT == HAL_COMPASS_AK8963_I2C
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe(GET_I2C_DEVICE(HAL_COMPASS_AK8963_I2C_BUS, HAL_COMPASS_AK8963_I2C_ADDR)));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_AK8963_MPU9250_I2C
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(GET_I2C_DEVICE(HAL_COMPASS_AK8963_I2C_BUS, HAL_COMPASS_AK8963_I2C_ADDR)));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_AERO
    ADD_BACKEND(DRIVER_BMM150, AP_Compass_BMM150::probe(GET_I2C_DEVICE(HAL_COMPASS_BMM150_I2C_BUS, HAL_COMPASS_BMM150_I2C_ADDR)));
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR), true));
    ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(HAL_COMPASS_IST8310_I2C_BUS, HAL_COMPASS_IST8310_I2C_ADDR),
                                                          true, ROTATION_PITCH_180_YAW_90));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_LIS3MDL
    ADD_BACKEND(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(hal.spi->get_device(HAL_COMPASS_LIS3MDL_NAME), false, ROTATION_ROLL_180_YAW_90));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_MAG3110                
    ADD_BACKEND(DRIVER_MAG3110, AP_Compass_MAG3110::probe(GET_I2C_DEVICE(HAL_MAG3110_I2C_BUS, HAL_MAG3110_I2C_ADDR), ROTATION_NONE));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_IST8310
    ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(HAL_COMPASS_IST8310_I2C_BUS, HAL_COMPASS_IST8310_I2C_ADDR),
                                                           true, ROTATION_PITCH_180_YAW_90));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_QMC5883L
    ADD_BACKEND(DRIVER_QMC5883, AP_Compass_QMC5883L::probe(GET_I2C_DEVICE(1, HAL_COMPASS_QMC5883L_I2C_ADDR),
                                                           true, HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL));
    ADD_BACKEND(DRIVER_QMC5883, AP_Compass_QMC5883L::probe(GET_I2C_DEVICE(0, HAL_COMPASS_QMC5883L_I2C_ADDR),
                                                           false, HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL));
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR)));
    ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0));
    ADD_BACKEND(DRIVER_LSM9DS1, AP_Compass_LSM9DS1::probe(hal.spi->get_device("lsm9ds1_m")));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_BMM150_I2C
    ADD_BACKEND(DRIVER_BMM150, AP_Compass_BMM150::probe(GET_I2C_DEVICE(HAL_COMPASS_BMM150_I2C_BUS, HAL_COMPASS_BMM150_I2C_ADDR)));
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_NONE
    // no compass
#else
    #error Unrecognised HAL_COMPASS_TYPE setting
#endif


/* for chibios external board coniguration */
#ifdef HAL_EXT_COMPASS_HMC5843_I2C_BUS
    ADD_BACKEND(DRIVER_HMC5883, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(HAL_EXT_COMPASS_HMC5843_I2C_BUS, HAL_COMPASS_HMC5843_I2C_ADDR),
                                                          true, ROTATION_ROLL_180));
#endif

#if HAL_WITH_UAVCAN
    for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
        ADD_BACKEND(DRIVER_UAVCAN, AP_Compass_UAVCAN::probe());
    }
#endif

    if (_backend_count == 0 ||
        _compass_count == 0) {
        hal.console->printf("No Compass backends available\n");
    }
}

bool
Compass::read(void)
{
    for (uint8_t i=0; i< _backend_count; i++) {
        // call read on each of the backend. This call updates field[i]
        _backends[i]->read();
    }
    uint32_t time = AP_HAL::millis();
    for (uint8_t i=0; i < COMPASS_MAX_INSTANCES; i++) {
        _state[i].healthy = (time - _state[i].last_update_ms < 500);
    }
    if (_learn == LEARN_INFLIGHT && !learn_allocated) {
        learn_allocated = true;
        learn = new CompassLearn(*this);
    }
    if (_learn == LEARN_INFLIGHT && learn != nullptr) {
        learn->update();
    }
    return healthy();
}

uint8_t
Compass::get_healthy_mask() const
{
    uint8_t healthy_mask = 0;
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if(healthy(i)) {
            healthy_mask |= 1 << i;
        }
    }
    return healthy_mask;
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset.set(offsets);
    }
}

void
Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset.set(offsets);
        save_offsets(i);
    }
}

void
Compass::set_and_save_diagonals(uint8_t i, const Vector3f &diagonals)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].diagonals.set_and_save(diagonals);
    }
}

void
Compass::set_and_save_offdiagonals(uint8_t i, const Vector3f &offdiagonals)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offdiagonals.set_and_save(offdiagonals);
    }
}

void
Compass::save_offsets(uint8_t i)
{
    _state[i].offset.save();  // save offsets
    _state[i].dev_id.set_and_save(_state[i].detected_dev_id);
}

void
Compass::save_offsets(void)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        save_offsets(i);
    }
}

void
Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    _state[i].motor_compensation.set(motor_comp_factor);
}

void
Compass::save_motor_compensation()
{
    _motor_comp_type.save();
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        _state[k].motor_compensation.save();
    }
}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
    if (_auto_declination) {
        // Set the declination based on the lat/lng from GPS
        _declination.set(radians(
                AP_Declination::get_declination(
                    (float)latitude / 10000000,
                    (float)longitude / 10000000)));
    }
}

/// return true if the compass should be used for yaw calculations
bool
Compass::use_for_yaw(void) const
{
    uint8_t prim = get_primary();
    return healthy(prim) && use_for_yaw(prim);
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
    // when we are doing in-flight compass learning the state
    // estimator must not use the compass. The learning code turns off
    // inflight learning when it has converged
    return _state[i].use_for_yaw && _learn.get() != LEARN_INFLIGHT;
}

void
Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        _declination.set_and_save(radians);
    }else{
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
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * M_PI);
        else if (heading < -M_PI)
            heading += (2.0f * M_PI);
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

    // exit immediately if dev_id hasn't been detected
    if (_state[i].detected_dev_id == 0) {
        return false;
    }

    // back up cached value of dev_id
    int32_t dev_id_cache_value = _state[i].dev_id;

    // load dev_id from eeprom
    _state[i].dev_id.load();

    // if dev_id loaded from eeprom is different from detected dev id or dev_id loaded from eeprom is different from cached dev_id, compass is unconfigured
    if (_state[i].dev_id != _state[i].detected_dev_id || _state[i].dev_id != dev_id_cache_value) {
        // restore cached value
        _state[i].dev_id = dev_id_cache_value;
        // return failure
        return false;
    }

    // if expected_dev_id is configured and the detected dev_id is different, return false
    if (_state[i].expected_dev_id != -1 && _state[i].expected_dev_id != _state[i].dev_id) {
        return false;
    }

    // if we got here then it must be configured
    return true;
}

bool Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && (!use_for_yaw(i) || configured(i));
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
    _hil.field[instance].rotate((enum Rotation)_state[0].orientation.get());

    if (!_state[0].external) {
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
    _state[instance].last_update_usec = update_usec;
}

const Vector3f& Compass::getHIL(uint8_t instance) const
{
    return _hil.field[instance];
}

// setup _Bearth
void Compass::_setup_earth_field(void)
{
    // assume a earth field strength of 400
    _hil.Bearth(400, 0, 0);

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


// singleton instance
Compass *Compass::_singleton;

namespace AP {

Compass &compass()
{
    return *Compass::get_singleton();
}

}
