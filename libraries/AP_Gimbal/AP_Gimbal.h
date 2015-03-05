// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_Gimbal -- library to control a 3 axis rate gimbal.		*
*															*
* Author:  Arthur Benemann, Paul Riseborough;									*
*															*
* Purpose:                          						*
*															*
* Usage:	           										*
*															*
* Comments:                         						*
************************************************************/
#ifndef __AP_GIMBAL_H__
#define __AP_GIMBAL_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <AP_SmallEKF.h>
#include <AP_NavEKF.h>


class AP_Gimbal
{
public:
    //Constructor
    AP_Gimbal(const AP_AHRS_NavEKF &ahrs, uint8_t sysid, uint8_t compid) :
        _ahrs(ahrs),
        _ekf(ahrs),
        _joint_offsets(0.0f,0.0f,0.0f),
        vehicleYawRateFilt(0.0f),
        yawRateFiltPole(10.0f),
        yawErrorLimit(0.1f),
        tilt_rc_in(6),
        _tilt_angle_min(-45.0f),
        _tilt_angle_max(0.0f),
        _max_tilt_rate(0.5f)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _sysid = sysid;
        _compid = compid;
    }
        
    // MAVLink methods
    void    receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg);

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

    struct Measurament {
        float delta_time;
        Vector3f delta_angles;
        Vector3f delta_velocity;
        Vector3f joint_angles;
    } _measurament;

    SmallEKF    _ekf;                   // state of small EKF for gimbal
    Vector3f    gimbalRateDemVec;       // degrees/s   
    Vector3f    _angle_ef_target_rad;   // desired earth-frame roll, tilt and pan angles in radians

private:  
    // filtered yaw rate from the vehicle
    float vehicleYawRateFilt;

    // circular frequency (rad/sec) constant of filter applied to forward path vehicle yaw rate
    // this frequency must not be larger than the update rate (Hz).
    // reducing it makes the gimbal yaw less responsive to vehicle yaw
    // increasing it makes the gimbal yawe more responsive to vehicle yaw
    float const yawRateFiltPole;

    // amount of yaw angle that we permit the gimbal to lag the vehicle when operating in slave mode
    // reducing this makes the gimbal respond more to vehicle yaw disturbances
    float const yawErrorLimit;

    // These are corrections (in radians) applied to the to the gimbal joint (x,y,z = roll,pitch,yaw) measurements
    Vector3f const _joint_offsets;


    uint8_t const tilt_rc_in; 
    float const _tilt_angle_min;   // min tilt in 0.01 degree units
    float const _tilt_angle_max;     // max tilt in 0.01 degree units
    float const _max_tilt_rate;          // max tilt rate in rad/s
    
    const AP_AHRS_NavEKF    &_ahrs;     //  Main EKF    
    uint8_t _sysid;                     
    uint8_t _compid;


    void send_control(mavlink_channel_t chan);
    void update_state();
    void decode_feedback(mavlink_message_t *msg);
    void update_targets_from_rc();

    // Control loop functions
    Vector3f getGimbalRateDemVecYaw(Quaternion quatEst);
    Vector3f getGimbalRateDemVecTilt(Quaternion quatEst);
    Vector3f getGimbalRateDemVecForward(Quaternion quatEst);
    Vector3f getGimbalRateDemVecGyroBias();

};

#endif // __AP_MOUNT_H__
