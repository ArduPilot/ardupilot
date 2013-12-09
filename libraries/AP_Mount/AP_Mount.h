// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_mount -- library to control a 2 or 3 axis mount.		*
*															*
* Author:  Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
*		   Gregory Fletcher;								*
*															*
* Purpose:  Move a 2 or 3 axis mount attached to vehicle,	*
*			Used for mount to track targets or stabilise	*
*			camera plus	other modes.						*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
* Comments: All angles in degrees * 100, distances in meters*
*			unless otherwise stated.						*
************************************************************/
#ifndef __AP_MOUNT_H__
#define __AP_MOUNT_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>

class AP_Mount
{
public:
    //Constructor
    AP_Mount(const struct Location *current_loc, GPS *&gps, const AP_AHRS &ahrs, uint8_t id);

    //enums
    enum MountType {
        k_unknown = 0,                  ///< unknown type
        k_pan_tilt = 1,                 ///< yaw-pitch
        k_tilt_roll = 2,                ///< pitch-roll
        k_pan_tilt_roll = 3,            ///< yaw-pitch-roll
    };

    // MAVLink methods
    void                    configure_msg(mavlink_message_t* msg);
    void                    control_msg(mavlink_message_t* msg);
    void                    status_msg(mavlink_message_t* msg);
    void                    set_roi_cmd(const struct Location *target_loc);
    void                    configure_cmd();
    void                    control_cmd();

    // should be called periodically
    void                    update_mount_position();
    void                    update_mount_type(); ///< Auto-detect the mount gimbal type depending on the functions assigned to the servos
    void                    debug_output();      ///< For testing and development. Called in the medium loop.
    // Accessors
    enum MountType          get_mount_type() {
        return _mount_type;
    }
    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:

    //methods
    void                            set_mode(enum MAV_MOUNT_MODE mode);

    void                            set_retract_angles(float roll, float tilt, float pan); ///< set mount retracted position
    void                            set_neutral_angles(float roll, float tilt, float pan);
    void                            set_control_angles(float roll, float tilt, float pan);
    void                            set_GPS_target_location(Location targetGPSLocation); ///< used to tell the mount to track GPS location

    // internal methods
    void                            calc_GPS_target_angle(const struct Location *target);
    void                            stabilize();
    int16_t                         closest_limit(int16_t angle, int16_t* angle_min, int16_t* angle_max);
    void                            move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);
    int32_t                         angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max);
    float                           angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max);

    //members
    const AP_AHRS                   &_ahrs; ///< Rotation matrix from earth to plane.
    GPS *&                          _gps;
    const struct Location *         _current_loc;
    struct Location                 _target_GPS_location;
    MountType                       _mount_type;

    uint8_t                         _roll_idx; ///< RC_Channel_aux mount roll function index
    uint8_t                         _tilt_idx; ///< RC_Channel_aux mount tilt function index
    uint8_t                         _pan_idx;  ///< RC_Channel_aux mount pan  function index
    uint8_t                         _open_idx; ///< RC_Channel_aux mount open function index

    float                           _roll_control_angle; ///< radians
    float                           _tilt_control_angle; ///< radians
    float                           _pan_control_angle;  ///< radians

    float                           _roll_angle; ///< degrees
    float                           _tilt_angle; ///< degrees
    float                           _pan_angle;  ///< degrees

    // EEPROM parameters
    AP_Int8                         _stab_roll; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_tilt; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_pan;  ///< (1 = yes, 0 = no)

    AP_Int8                         _mount_mode;
    // RC_Channel for providing direct angular input from pilot
    AP_Int8                         _roll_rc_in;
    AP_Int8                         _tilt_rc_in;
    AP_Int8                         _pan_rc_in;

    AP_Int16                        _roll_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _roll_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_min;  ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_max;  ///< max angle limit of actuated surface in 0.01 degree units

    AP_Int8                         _joystick_speed;

    AP_Vector3f                     _retract_angles; ///< retracted position for mount, vector.x = roll vector.y = tilt, vector.z=pan
    AP_Vector3f                     _neutral_angles; ///< neutral position for mount, vector.x = roll vector.y = tilt, vector.z=pan
    AP_Vector3f                     _control_angles; ///< GCS controlled position for mount, vector.x = roll vector.y = tilt, vector.z=pan
};

#endif // __AP_MOUNT_H__
