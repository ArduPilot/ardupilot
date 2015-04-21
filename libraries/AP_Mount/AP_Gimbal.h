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
#include <AP_Mount.h>


class AP_Gimbal
{
public:
    //Constructor
    AP_Gimbal(const AP_AHRS_NavEKF &ahrs, uint8_t sysid, uint8_t compid, const AP_Mount::gimbal_params &gimbalParams) :
        _ekf(ahrs),
        _ahrs(ahrs),
        vehicleYawRateFilt(0.0f),
        yawRateFiltPole(10.0f),
        yawErrorLimit(0.1f),
        _gimbalParams(gimbalParams)
    {
        _sysid = sysid;
        _compid = compid;
    }

    void    update_target(Vector3f newTarget);
    void    receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg);

    Vector3f getGimbalEstimateEF();

    struct Measurament {
        float delta_time;
        Vector3f delta_angles;
        Vector3f delta_velocity;
        Vector3f joint_angles;
    } _measurement;

    SmallEKF    _ekf;                   // state of small EKF for gimbal
    const AP_AHRS_NavEKF    &_ahrs;     //  Main EKF    
    Vector3f    gimbalRateDemVec;       // degrees/s   
    Vector3f    _angle_ef_target_rad;   // desired earth-frame roll, tilt and pan angles in radians    

private:  
    const AP_Mount::gimbal_params &_gimbalParams;

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

    uint8_t _sysid;                     
    uint8_t _compid;    

    void send_control(mavlink_channel_t chan);
    void update_state();
    void decode_feedback(mavlink_message_t *msg);

    uint8_t isCopterFlipped();

    // Control loop functions
    Vector3f getGimbalRateDemVecYaw(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecTilt(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecForward(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecGyroBias();

};

#endif // __AP_MOUNT_H__
