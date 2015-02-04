/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_Progmem.h>
#include "Compass.h"

const AP_Param::GroupInfo Compass::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: OFS_X
    // @DisplayName: Compass offsets on the X axis
    // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Y
    // @DisplayName: Compass offsets on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Z
    // @DisplayName: Compass offsets on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS",    1, Compass, _offset[0], 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: Radians
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),

    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, 1), // true if learning calibration

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, Compass, _use_for_yaw[0], 1), // true if used for DCM yaw

#if !defined( __AVR_ATmega1280__ )
    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, Compass, _auto_declination, 1),
#endif

    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @Increment: 1
    AP_GROUPINFO("MOTCT",    6, Compass, _motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT",    7, Compass, _motor_compensation[0], 0),

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll90
    AP_GROUPINFO("ORIENT", 8, Compass, _orientation[0], ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk, but must be set correctly on an APM2. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, Compass, _external[0], 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: OFS2_X
    // @DisplayName: Compass2 offsets on the X axis
    // @Description: Offset to be added to compass2's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS2_Y
    // @DisplayName: Compass2 offsets on the Y axis
    // @Description: Offset to be added to compass2's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS2_Z
    // @DisplayName: Compass2 offsets on the Z axis
    // @Description: Offset to be added to compass2's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS2",    10, Compass, _offset[1], 0),

    // @Param: MOT2_X
    // @DisplayName: Motor interference compensation to compass2 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT2_Y
    // @DisplayName: Motor interference compensation to compass2 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT2_Z
    // @DisplayName: Motor interference compensation to compass2 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT2",    11, Compass, _motor_compensation[1], 0),

    // @Param: PRIMARY
    // @DisplayName: Choose primary compass
    // @Description: If more than one compass is available this selects which compass is the primary. Normally 0=External, 1=Internal. If no External compass is attached this parameter is ignored
    // @Values: 0:FirstCompass,1:SecondCompass
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 12, Compass, _primary, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: OFS3_X
    // @DisplayName: Compass3 offsets on the X axis
    // @Description: Offset to be added to compass3's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS3_Y
    // @DisplayName: Compass3 offsets on the Y axis
    // @Description: Offset to be added to compass3's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS3_Z
    // @DisplayName: Compass3 offsets on the Z axis
    // @Description: Offset to be added to compass3's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS3",    13, Compass, _offset[2], 0),

    // @Param: MOT3_X
    // @DisplayName: Motor interference compensation to compass3 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT3_Y
    // @DisplayName: Motor interference compensation to compass3 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT3_Z
    // @DisplayName: Motor interference compensation to compass3 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT3",    14, Compass, _motor_compensation[2], 0),
#endif

#if COMPASS_MAX_INSTANCES > 1
    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  15, Compass, _dev_id[0], 0),

    // @Param: DEV_ID2
    // @DisplayName: Compass2 device id
    // @Description: Second compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID2", 16, Compass, _dev_id[1], 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: DEV_ID3
    // @DisplayName: Compass3 device id
    // @Description: Third compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID3", 17, Compass, _dev_id[2], 0),
#endif

#if COMPASS_MAX_INSTANCES > 1
    // @Param: USE2
    // @DisplayName: Compass2 used for yaw
    // @Description: Enable or disable the second compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2",    18, Compass, _use_for_yaw[1], 1),

    // @Param: ORIENT2
    // @DisplayName: Compass2 orientation
    // @Description: The orientation of the second compass relative to the frame (if external) or autopilot board (if internal).
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll90
    AP_GROUPINFO("ORIENT2", 19, Compass, _orientation[1], ROTATION_NONE),

    // @Param: EXTERN2
    // @DisplayName: Compass2 is attached via an external cable
    // @Description: Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk.
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERN2",20, Compass, _external[1], 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: USE3
    // @DisplayName: Compass3 used for yaw
    // @Description: Enable or disable the third compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3",    21, Compass, _use_for_yaw[2], 1),

    // @Param: ORIENT3
    // @DisplayName: Compass3 orientation
    // @Description: The orientation of the third compass relative to the frame (if external) or autopilot board (if internal).
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll90
    AP_GROUPINFO("ORIENT3", 22, Compass, _orientation[2], ROTATION_NONE),

    // @Param: EXTERN3
    // @DisplayName: Compass3 is attached via an external cable
    // @Description: Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk.
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERN3",23, Compass, _external[2], 0),
#endif

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    product_id(AP_COMPASS_TYPE_UNKNOWN),
    last_update(0),
    _null_init_done(false),
    _thr_or_curr(0.0f),
    _board_orientation(ROTATION_NONE)
{
    AP_Param::setup_object_defaults(this, var_info);

#if COMPASS_MAX_INSTANCES > 1
    // default device ids to zero.  init() method will overwrite with the actual device ids
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        _dev_id[i] = 0;
    }
#endif
}

// Default init method, just returns success.
//
bool
Compass::init()
{
    return true;
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _offset[i].set(offsets);
    }
}

void
Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _offset[i].set(offsets);
        save_offsets(i);
    }
}

void
Compass::save_offsets(uint8_t i)
{
    _offset[i].save();  // save offsets
#if COMPASS_MAX_INSTANCES > 1
    _dev_id[i].save();  // save device id corresponding to these offsets
#endif
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
    _motor_compensation[i].set(motor_comp_factor);
}

void
Compass::save_motor_compensation()
{
    _motor_comp_type.save();
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        _motor_compensation[k].save();
    }
}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
#if !defined( __AVR_ATmega1280__ )
    if (_auto_declination) {
        // Set the declination based on the lat/lng from GPS
        _declination.set(radians(
                AP_Declination::get_declination(
                    (float)latitude / 10000000,
                    (float)longitude / 10000000)));
    }
#endif
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
    return _use_for_yaw[i];
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
Compass::calculate_heading(const Matrix3f &dcm_matrix) const
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    float headY = _field[0].y * dcm_matrix.c.z - _field[0].z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = _field[0].x * cos_pitch_sq - dcm_matrix.c.x * (_field[0].y * dcm_matrix.c.y + _field[0].z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
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
    if (get_offsets(i).length() == 0.0f) {
        return false;
    }

#if COMPASS_MAX_INSTANCES > 1
    // backup detected dev_id
    int32_t dev_id_orig = _dev_id[i];

    // load dev_id from eeprom
    _dev_id[i].load();

    // if different then the device has not been configured
    if (_dev_id[i] != dev_id_orig) {
        // restore device id
        _dev_id[i] = dev_id_orig;
        // return failure
        return false;
    }
#endif

    // if we got here then it must be configured
    return true;
}

bool Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && configured(i);
    }
    return all_configured;
}

/*
  apply offset and motor compensation corrections
 */
void Compass::apply_corrections(Vector3f &mag, uint8_t i)
{
    const Vector3f &offsets = _offset[i].get();
    const Vector3f &mot = _motor_compensation[i].get();

    /*
      note that _motor_offset[] is kept even if compensation is not
      being applied so it can be logged correctly
     */
    mag += offsets;
    if(_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset[i] = mot * _thr_or_curr;
        mag += _motor_offset[i];
    } else {
        _motor_offset[i].zero();
    }
}
