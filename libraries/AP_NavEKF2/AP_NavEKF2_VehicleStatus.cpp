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


/* Monitor GPS data to see if quality is good enough to initialise the EKF
   Monitor magnetometer innovations to to see if the heading is good enough to use GPS
   Return true if all criteria pass for 10 seconds

   We also record the failure reason so that prearm_failure_reason()
   can give a good report to the user on why arming is failing
*/
bool NavEKF2_core::calcGpsGoodToAlign(void)
{
    // calculate absolute difference between GPS vert vel and inertial vert vel
    float velDiffAbs;
    if (_ahrs->get_gps().have_vertical_velocity()) {
        velDiffAbs = fabsf(gpsDataDelayed.vel.z - stateStruct.velocity.z);
    } else {
        velDiffAbs = 0.0f;
    }

    // fail if velocity difference or reported speed accuracy greater than threshold
    bool gpsVelFail = (velDiffAbs > 1.0f) || (gpsSpdAccuracy > 1.0f);

    if (velDiffAbs > 1.0f) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vert vel error %.1f", (double)velDiffAbs);
    }
    if (gpsSpdAccuracy > 1.0f) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS speed error %.1f", (double)gpsSpdAccuracy);
    }

    // fail if horiziontal position accuracy not sufficient
    float hAcc = 0.0f;
    bool hAccFail;
    if (_ahrs->get_gps().horizontal_accuracy(hAcc)) {
        hAccFail = hAcc > 5.0f;
    } else {
        hAccFail =  false;
    }
    if (hAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horiz error %.1f", (double)hAcc);
    }

    // If we have good magnetometer consistency and bad innovations for longer than 5 seconds then we reset heading and field states
    // This enables us to handle large changes to the external magnetic field environment that occur before arming
    if ((magTestRatio.x <= 1.0f && magTestRatio.y <= 1.0f) || !consistentMagData) {
        magYawResetTimer_ms = imuSampleTime_ms;
    }
    if (imuSampleTime_ms - magYawResetTimer_ms > 5000) {
        // reset heading and field states
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        // reset timer to ensure that bad magnetometer data cannot cause a heading reset more often than every 5 seconds
        magYawResetTimer_ms = imuSampleTime_ms;
    }

    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    bool yawFail;
    if (magTestRatio.x > 1.0f || magTestRatio.y > 1.0f) {
        yawFail = true;
    } else {
        yawFail = false;
    }
    if (yawFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Mag yaw error x=%.1f y=%.1f",
                           (double)magTestRatio.x,
                           (double)magTestRatio.y);
    }

    // fail if not enough sats
    bool numSatsFail = _ahrs->get_gps().num_sats() < 6;
    if (numSatsFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS numsats %u (needs 6)", _ahrs->get_gps().num_sats());
    }

    // record time of fail if failing
    if (gpsVelFail || numSatsFail || hAccFail || yawFail) {
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    if (lastGpsVelFail_ms == 0) {
        // first time through, start with a failure
        lastGpsVelFail_ms = imuSampleTime_ms;
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string), "EKF warmup");
    }

    // DEBUG PRINT
    //hal.console->printf("velDiff = %5.2f, nSats = %i, hAcc = %5.2f, sAcc = %5.2f, delTime = %i\n", velDiffAbs, _ahrs->get_gps().num_sats(), hAcc, gpsSpdAccuracy, imuSampleTime_ms - lastGpsVelFail_ms);
    // continuous period without fail required to return healthy

    if (imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        // we have not failed in the last 10 seconds
        return true;
    }

    return false;
}

// Detect if we are in flight or on ground
void NavEKF2_core::detectFlight()
{
    /*
        If we are a fly forward type vehicle (eg plane), then in-air status can be determined through a combination of speed and height criteria.
        Because of the differing certainty requirements of algorithms that need the in-flight / on-ground status we use two booleans where
        onGround indicates a high certainty we are not flying and inFlight indicates a high certainty we are flying. It is possible for
        both onGround and inFlight to be false if the status is uncertain, but they cannot both be true.

        If we are a plane as indicated by the assume_zero_sideslip() status, then different logic is used

        TODO - this logic should be moved out of the EKF and into the flight vehicle code.
    */

    if (assume_zero_sideslip()) {
        // To be confident we are in the air we use a criteria which combines arm status, ground speed, airspeed and height change
        float gndSpdSq = sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 8 m/s airspeed
        if (_ahrs->airspeed_sensor_enabled()) {
            const AP_Airspeed *airspeed = _ahrs->get_airspeed();
            if (airspeed->get_airspeed() * airspeed->get_EAS2TAS() > 10.0f) {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f && gpsSpdAccuracy < 1.0f) {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(baroDataDelayed.hgt) > 10.0f) {
            largeHgtChange = true;
        }

        // Determine to a high certainty we are flying
        if (motorsArmed && highGndSpd && (highAirSpd || largeHgtChange)) {
            onGround = false;
            inFlight = true;
        }

        // if is possible we are in flight, set the time this condition was last detected
        if (motorsArmed && (highGndSpd || highAirSpd || largeHgtChange)) {
            airborneDetectTime_ms = imuSampleTime_ms;
            onGround = false;
        }

        // Determine if is is possible we are on the ground
        if (highGndSpd || highAirSpd || largeHgtChange) {
            inFlight = false;
        }

        // Determine to a high certainty we are not flying
        // after 5 seconds of not detecting a possible flight condition or we are disarmed, we transition to on-ground mode
        if(!motorsArmed || ((imuSampleTime_ms - airborneDetectTime_ms) > 5000)) {
            onGround = true;
            inFlight = false;
        }
    } else {
        // Non fly forward vehicle, so can only use height and motor arm status

        // If the motors are armed then we could be flying and if they are not armed then we are definitely not flying
        if (motorsArmed) {
            onGround = false;
        } else {
            inFlight = false;
            onGround = true;
        }

        // If height has increased since exiting on-ground, then we definitely are flying
        if (!onGround && ((stateStruct.position.z - posDownAtTakeoff) < -1.5f)) {
            inFlight = true;
        }

        // If rangefinder has increased since exiting on-ground, then we definitely are flying
        if (!onGround && ((rngMea - rngAtStartOfFlight) > 0.5f)) {
            inFlight = true;
        }

    }

    // store current on-ground  and in-air status for next time
    prevOnGround = onGround;
    prevInFlight = inFlight;

    // Store vehicle height and range prior to takeoff for use in post takeoff checks
    if (!onGround && !prevOnGround) {
        // store vertical position at start of flight to use as a reference for ground relative checks
        posDownAtTakeoff = stateStruct.position.z;
        // store the range finder measurement which will be used as a reference to detect when we have taken off
        rngAtStartOfFlight = rngMea;
    }

}


// determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF2_core::getTakeoffExpected()
{
    if (expectGndEffectTakeoff && imuSampleTime_ms - takeoffExpectedSet_ms > frontend.gndEffectTimeout_ms) {
        expectGndEffectTakeoff = false;
    }

    return expectGndEffectTakeoff;
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2_core::setTakeoffExpected(bool val)
{
    takeoffExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTakeoff = val;
}


// determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF2_core::getTouchdownExpected()
{
    if (expectGndEffectTouchdown && imuSampleTime_ms - touchdownExpectedSet_ms > frontend.gndEffectTimeout_ms) {
        expectGndEffectTouchdown = false;
    }

    return expectGndEffectTouchdown;
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2_core::setTouchdownExpected(bool val)
{
    touchdownExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTouchdown = val;
}

// Detect takeoff for optical flow navigation
void NavEKF2_core::detectOptFlowTakeoff(void)
{
    if (motorsArmed && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000) {
        const AP_InertialSensor &ins = _ahrs->get_ins();
        Vector3f angRateVec;
        Vector3f gyroBias;
        getGyroBias(gyroBias);
        bool dual_ins = ins.get_gyro_health(0) && ins.get_gyro_health(1);
        if (dual_ins) {
            angRateVec = (ins.get_gyro(0) + ins.get_gyro(1)) * 0.5f - gyroBias;
        } else {
            angRateVec = ins.get_gyro() - gyroBias;
        }

        takeOffDetected = (takeOffDetected || (angRateVec.length() > 0.1f) || (rngMea > (rngAtStartOfFlight + 0.1f)));
    }
}


#endif // HAL_CPU_CLASS