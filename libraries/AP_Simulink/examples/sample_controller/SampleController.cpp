// @file    SampleController.cpp
// @brief   This is an example controller code that will utilize the different classes in AP_Simulink library.

#include "SampleController.h"
#include <GCS_MAVLink/GCS.h>
#include <AC_PID/AC_PID.h>

// PID objects
AC_PID pid_rate_roll_sl(0.035,0,0.0025,0,0,0,0,50);
AC_PID pid_rate_pitch_sl(0.035,0,0.0025,0,0,0,0,50);
AC_PID pid_rate_yaw_sl(0.035,0,0,0,0,0,0,100);

AC_PID pid_alt_sl(1.5,0.01,0.01,0,0,0,0,10);
AC_PID pid_vel_sl(0.5,0.1,0.05,0,0,0,0,10);

void SampleController_init(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Simulink Custom controller is initialized ");;
}
void SampleController_loop(void)
{
    Quaternion currAttQuat,desAttQuat;
    currAttQuat.initialise();
    desAttQuat.initialise();
    Vector3f currAngVel;
    // get current attitude
    sl_ahrs_handle->getAttitudeQuat(currAttQuat);
    Vector3f currAtt;
    currAttQuat.to_euler(currAtt);
    // get current ang velocity
    currAngVel = sl_ahrs_handle->getAngularVelocity();
    // get desired attitude
    desAttQuat = sl_attControl_handle->getAttitudeSetpointQuat();
    Vector3f desAtt;
    desAttQuat.to_euler(desAtt);
    // calculate attitude error
    Vector3f attError;
    for (int i = 0; i < 3; i++) {
        attError[i] = desAtt[i]-currAtt[i];
    }
    // P controller
    Vector3f targetRate;
    targetRate[0] = 4 * attError[0];
    targetRate[1] = 4 * attError[1];
    targetRate[2] = 2 * attError[2];
    //PID controller
    Vector3f motorOut;
    motorOut[0] = pid_rate_roll_sl.update_all(targetRate[0], currAngVel[0], 0.0025, false);
    motorOut[1] = pid_rate_pitch_sl.update_all(targetRate[1], currAngVel[1], 0.0025, false);
    motorOut[2] = pid_rate_yaw_sl.update_all(targetRate[2], currAngVel[2], 0.0025, false);
    // set torque value
    sl_motors_handle->setTorque(motorOut[0],motorOut[1],motorOut[2]);

    // get current altitude
    Vector3f currPOS;
    float currAlt;
    sl_ahrs_handle->getPositionNED(currPOS);
    //NED->NEU
    currAlt = -currPOS[2];
    currAlt = (currAlt<0)? 0:currAlt;
    // get desired altitude
    Vector3p desPos;
    float desAlt;
    desPos = sl_posControl_handle->getPositionSetPointNEU();
    // NEU and cm->m
    desAlt=static_cast<float>(desPos[2]*0.01);
    // get current velocity
    Vector3f currVel;
    sl_ahrs_handle->getVelocityNED(currVel);
    float velZ = -currVel[2];
    // altitude PID controller
    float targetAltRate = pid_alt_sl.update_all(desAlt,currAlt,0.0025, false);
    float tauThrust = pid_vel_sl.update_all(targetAltRate, velZ, 0.0025, false);
    tauThrust = sl_motors_handle->getHandle_AP_Motors()->get_throttle()+0.2*tauThrust;
    // set thrust
    sl_motors_handle->setThrust(tauThrust);
}