/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

/*
  optionally turn down optimisation for debugging
 */
// #pragma GCC optimize("O0")

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


// Control filter mode transitions
void NavEKF2_core::controlFilterModes()
{
    // Determine motor arm status
    prevMotorsArmed = motorsArmed;
    motorsArmed = hal.util->get_soft_armed();
    if (motorsArmed && !prevMotorsArmed) {
        // set the time at which we arm to assist with checks
        timeAtArming_ms =  imuSampleTime_ms;
    }

    // Detect if we are in flight on or ground
    detectFlight();

    // Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
    // avoid unnecessary operations
    setWindMagStateLearningMode();

    // Check the alignmnent status of the tilt and yaw attitude
    // Used during initial bootstrap alignment of the filter
    checkAttitudeAlignmentStatus();

    // Control reset of yaw and magnetic field states
    controlMagYawReset();

    // Set the type of inertial navigation aiding used
    setAidingMode();

}

// Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
// avoid unnecessary operations
void NavEKF2_core::setWindMagStateLearningMode()
{
    // If we are on ground, or in constant position mode, or don't have the right vehicle and sensing to estimate wind, inhibit wind states
    inhibitWindStates = ((!useAirspeed() && !assume_zero_sideslip()) || onGround || (PV_AidingMode == AID_NONE));

    // determine if the vehicle is manoevring
    if (accNavMagHoriz > 0.5f) {
        manoeuvring = true;
    } else {
        manoeuvring = false;
    }

    // Determine if learning of magnetic field states has been requested by the user
    bool magCalRequested = ((frontend._magCal == 0) && !onGround) || ((frontend._magCal == 1) && manoeuvring)  || (frontend._magCal == 3);

    // Deny mag calibration request if we aren't using the compass, are in the pre-arm constant position mode or it has been inhibited by the user
    bool magCalDenied = !use_compass() || (PV_AidingMode == AID_NONE) || (frontend._magCal == 2);

    // Inhibit the magnetic field calibration if not requested or denied
    inhibitMagStates = (!magCalRequested || magCalDenied);

    // Adjust the indexing limits used to address the covariance, states and other EKF arrays to avoid unnecessary operations
    // if we are not using those states
    if (inhibitMagStates && inhibitWindStates) {
        stateIndexLim = 15;
    } else if (inhibitWindStates) {
        stateIndexLim = 21;
    } else {
        stateIndexLim = 23;
    }
}

// Set inertial navigation aiding mode
void NavEKF2_core::setAidingMode()
{
    // Determine when to commence aiding for inertial navigation
    // Save the previous status so we can detect when it has changed
    prevIsAiding = isAiding;
    // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
    bool filterIsStable = tiltAlignComplete && yawAlignComplete;
    // If GPS useage has been prohiited then we use flow aiding provided optical flow data is present
    bool useFlowAiding = (frontend._fusionModeGPS == 3) && optFlowDataPresent();
    // Start aiding if we have a source of aiding data and the filter attitude algnment is complete
    // Latch to on. Aiding can be turned off by setting both
    isAiding = ((readyToUseGPS() || useFlowAiding) && filterIsStable) || isAiding;

    // check to see if we are starting or stopping aiding and set states and modes as required
    if (isAiding != prevIsAiding) {
        // We have transitioned either into or out of aiding
        // zero stored velocities used to do dead-reckoning
        heldVelNE.zero();
        // reset the flag that indicates takeoff for use by optical flow navigation
        takeOffDetected = false;
        // set various  useage modes based on the condition when we start aiding. These are then held until aiding is stopped.
        if (!isAiding) {
            // We have ceased aiding
            // When not aiding, estimate orientation & height fusing synthetic constant position and zero velocity measurement to constrain tilt errors
            PV_AidingMode = AID_NONE;
            posTimeout = true;
            velTimeout = true;
             // store the current position to be used to keep reporting the last known position
            lastKnownPositionNE.x = stateStruct.position.x;
            lastKnownPositionNE.y = stateStruct.position.y;
            // initialise filtered altitude used to provide a takeoff reference to current baro on disarm
            // this reduces the time required for the baro noise filter to settle before the filtered baro data can be used
            meaHgtAtTakeOff = baroDataDelayed.hgt;
            // reset the vertical position state to faster recover from baro errors experienced during touchdown
            stateStruct.position.z = -meaHgtAtTakeOff;
        } else if (frontend._fusionModeGPS == 3) {
            // We have commenced aiding, but GPS useage has been prohibited so use optical flow only
            hal.console->printf("EKF is using optical flow\n");
            PV_AidingMode = AID_RELATIVE; // we have optical flow data and can estimate all vehicle states
            posTimeout = true;
            velTimeout = true;
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
        } else {
            // We have commenced aiding and GPS useage is allowed
            hal.console->printf("EKF is using GPS\n");
            PV_AidingMode = AID_ABSOLUTE; // we have GPS data and can estimate all vehicle states
            posTimeout = false;
            velTimeout = false;
            // we need to reset the GPS timers to prevent GPS timeout logic being invoked on entry into GPS aiding
            // this is becasue the EKF can be interrupted for an arbitrary amount of time during vehicle arming checks
            lastTimeGpsReceived_ms = imuSampleTime_ms;
            secondLastGpsTime_ms = imuSampleTime_ms;
            // reset the last valid position fix time to prevent unwanted activation of GPS glitch logic
            lastPosPassTime_ms = imuSampleTime_ms;
        }
        // Reset all position, velocity and covariance
        ResetVelocity();
        ResetPosition();
        CovarianceInit();

    }

    // Always turn aiding off when the vehicle is disarmed
    if (!isAiding) {
        PV_AidingMode = AID_NONE;
        posTimeout = true;
        velTimeout = true;
    }
}

// Check the alignmnent status of the tilt and yaw attitude
// Used during initial bootstrap alignment of the filter
void NavEKF2_core::checkAttitudeAlignmentStatus()
{
    // Check for tilt convergence - used during initial alignment
    float alpha = 1.0f*dtIMUavg;
    float temp=tiltErrVec.length();
    tiltErrFilt = alpha*temp + (1.0f-alpha)*tiltErrFilt;
    if (tiltErrFilt < 0.005f && !tiltAlignComplete) {
        tiltAlignComplete = true;
        hal.console->printf("EKF tilt alignment complete\n");
    }

    // Once tilt has converged, align yaw using magnetic field measurements
    if (tiltAlignComplete && !yawAlignComplete) {
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        StoreQuatReset();
        yawAlignComplete = true;
        hal.console->printf("EKF yaw alignment complete\n");
    }
}

// return true if we should use the airspeed sensor
bool NavEKF2_core::useAirspeed(void) const
{
    return _ahrs->airspeed_sensor_enabled();
}

// return true if we should use the range finder sensor
bool NavEKF2_core::useRngFinder(void) const
{
    // TO-DO add code to set this based in setting of optical flow use parameter and presence of sensor
    return true;
}

// return true if optical flow data is available
bool NavEKF2_core::optFlowDataPresent(void) const
{
    return (imuSampleTime_ms - flowMeaTime_ms < 200);
}

// return true if the filter to be ready to use gps
bool NavEKF2_core::readyToUseGPS(void) const
{
    return validOrigin && tiltAlignComplete && yawAlignComplete && gpsQualGood;
}

// return true if we should use the compass
bool NavEKF2_core::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw();
}

/*
  should we assume zero sideslip?
 */
bool NavEKF2_core::assume_zero_sideslip(void) const
{
    // we don't assume zero sideslip for ground vehicles as EKF could
    // be quite sensitive to a rapid spin of the ground vehicle if
    // traction is lost
    return _ahrs->get_fly_forward() && _ahrs->get_vehicle_class() != AHRS_VEHICLE_GROUND;
}

#endif // HAL_CPU_CLASS