#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3_core.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>

/* Monitor GPS data to see if quality is good enough to initialise the EKF
   Monitor magnetometer innovations to see if the heading is good enough to use GPS
   Return true if all criteria pass for 10 seconds

   We also record the failure reason so that pre_arm_check()
   can give a good report to the user on why arming is failing
*/
void NavEKF3_core::calcGpsGoodToAlign(void)
{
    if (inFlight && assume_zero_sideslip() && !use_compass()) {
        // this is a special case where a plane has launched without magnetometer
        // is now in the air and needs to align yaw to the GPS and start navigating as soon as possible
        gpsGoodToAlign = true;
        return;
    }

    // User defined multiplier to be applied to check thresholds
    ftype checkScaler = 0.01f*(ftype)frontend->_gpsCheckScaler;

    if (gpsGoodToAlign) {
        /*
          if we have already passed GPS alignment checks then raise
          the check threshold so that we have some hysterisis and
          don't continuously change from able to arm to not able to
          arm
         */
        checkScaler *= 1.3f;
    }
    
    // If we have good magnetometer consistency and bad innovations for longer than 5 seconds then we reset heading and field states
    // This enables us to handle large changes to the external magnetic field environment that occur before arming
    if ((magTestRatio.x <= 1.0f && magTestRatio.y <= 1.0f && yawTestRatio <= 1.0f) || !consistentMagData) {
        magYawResetTimer_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - magYawResetTimer_ms > 5000) && !motorsArmed) {
        // request reset of heading and magnetic field states
        magYawResetRequest = true;
        // reset timer to ensure that bad magnetometer data cannot cause a heading reset more often than every 5 seconds
        magYawResetTimer_ms = imuSampleTime_ms;
    }

    // Check for significant change in GPS position if disarmed which indicates bad GPS
    // This check can only be used when the vehicle is stationary
    const auto &gps = dal.gps();

    const struct Location &gpsloc = gps.location(preferred_gps); // Current location
    const ftype posFiltTimeConst = 10.0; // time constant used to decay position drift
    // calculate time lapsed since last update and limit to prevent numerical errors
    ftype deltaTime = constrain_ftype(ftype(imuDataDelayed.time_ms - lastPreAlignGpsCheckTime_ms)*0.001f,0.01f,posFiltTimeConst);
    lastPreAlignGpsCheckTime_ms = imuDataDelayed.time_ms;
    // Sum distance moved
    gpsDriftNE += gpsloc_prev.get_distance(gpsloc);
    gpsloc_prev = gpsloc;
    // Decay distance moved exponentially to zero
    gpsDriftNE *= (1.0f - deltaTime/posFiltTimeConst);
    // Clamp the filter state to prevent excessive persistence of large transients
    gpsDriftNE = MIN(gpsDriftNE,10.0f);
    // Fail if more than 3 metres drift after filtering whilst on-ground
    // This corresponds to a maximum acceptable average drift rate of 0.3 m/s or single glitch event of 3m
    bool gpsDriftFail = (gpsDriftNE > 3.0f*checkScaler) && onGround && (frontend->_gpsCheck & MASK_GPS_POS_DRIFT);

    // Report check result as a text string and bitmask
    if (gpsDriftFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS drift %.1fm (needs %.1f)", (double)gpsDriftNE, (double)(3.0f*checkScaler));
        gpsCheckStatus.bad_horiz_drift = true;
    } else {
        gpsCheckStatus.bad_horiz_drift = false;
    }

    // Check that the vertical GPS vertical velocity is reasonable after noise filtering
    bool gpsVertVelFail;
    if (gpsDataNew.have_vz && onGround) {
        // check that the average vertical GPS velocity is close to zero
        gpsVertVelFilt = 0.1f * gpsDataNew.vel.z + 0.9f * gpsVertVelFilt;
        gpsVertVelFilt = constrain_ftype(gpsVertVelFilt,-10.0f,10.0f);
        gpsVertVelFail = (fabsF(gpsVertVelFilt) > 0.3f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_VERT_SPD);
    } else {
        gpsVertVelFail = false;
    }

    // Report check result as a text string and bitmask
    if (gpsVertVelFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vertical speed %.2fm/s (needs %.2f)", (double)fabsF(gpsVertVelFilt), (double)(0.3f*checkScaler));
        gpsCheckStatus.bad_vert_vel = true;
    } else {
        gpsCheckStatus.bad_vert_vel = false;
    }

    // Check that the horizontal GPS vertical velocity is reasonable after noise filtering
    // This check can only be used if the vehicle is stationary
    bool gpsHorizVelFail;
    if (onGround) {
        gpsHorizVelFilt = 0.1f * norm(gpsDataDelayed.vel.x,gpsDataDelayed.vel.y) + 0.9f * gpsHorizVelFilt;
        gpsHorizVelFilt = constrain_ftype(gpsHorizVelFilt,-10.0f,10.0f);
        gpsHorizVelFail = (fabsF(gpsHorizVelFilt) > 0.3f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_HORIZ_SPD);
    } else {
        gpsHorizVelFail = false;
    }

    // Report check result as a text string and bitmask
    if (gpsHorizVelFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horizontal speed %.2fm/s (needs %.2f)", (double)gpsDriftNE, (double)(0.3f*checkScaler));
        gpsCheckStatus.bad_horiz_vel = true;
    } else {
        gpsCheckStatus.bad_horiz_vel = false;
    }

    // fail if horiziontal position accuracy not sufficient
    float hAcc = 0.0f;
    bool hAccFail;
    if (gps.horizontal_accuracy(preferred_gps, hAcc)) {
        hAccFail = (hAcc > 5.0f*checkScaler)  && (frontend->_gpsCheck & MASK_GPS_POS_ERR);
    } else {
        hAccFail =  false;
    }

    // Report check result as a text string and bitmask
    if (hAccFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horiz error %.1fm (needs %.1f)", (double)hAcc, (double)(5.0f*checkScaler));
        gpsCheckStatus.bad_hAcc = true;
    } else {
        gpsCheckStatus.bad_hAcc = false;
    }


    // Check for vertical GPS accuracy
    float vAcc = 0.0f;
    bool vAccFail = false;
    if (gps.vertical_accuracy(preferred_gps, vAcc)) {
        vAccFail = (vAcc > 7.5f * checkScaler)  && (frontend->_gpsCheck & MASK_GPS_POS_ERR);
    }
    // Report check result as a text string and bitmask
    if (vAccFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vert error %.1fm (needs < %.1f)", (double)vAcc, (double)(7.5f * checkScaler));
        gpsCheckStatus.bad_vAcc = true;
    } else {
        gpsCheckStatus.bad_vAcc = false;
    }

    // fail if reported speed accuracy greater than threshold
    bool gpsSpdAccFail = (gpsSpdAccuracy > 1.0f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_SPD_ERR);

    // Report check result as a text string and bitmask
    if (gpsSpdAccFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS speed error %.1f (needs < %.1f)", (double)gpsSpdAccuracy, (double)(1.0f*checkScaler));
        gpsCheckStatus.bad_sAcc = true;
    } else {
        gpsCheckStatus.bad_sAcc = false;
    }

    // fail if satellite geometry is poor
    bool hdopFail = (gps.get_hdop(preferred_gps) > 250)  && (frontend->_gpsCheck & MASK_GPS_HDOP);

    // Report check result as a text string and bitmask
    if (hdopFail) {
        dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS HDOP %.1f (needs 2.5)", (double)(0.01f * gps.get_hdop(preferred_gps)));
        gpsCheckStatus.bad_hdop = true;
    } else {
        gpsCheckStatus.bad_hdop = false;
    }

    // fail if not enough sats
    bool numSatsFail = (gps.num_sats(preferred_gps) < 6) && (frontend->_gpsCheck & MASK_GPS_NSATS);

    // Report check result as a text string and bitmask
    if (numSatsFail) {
        dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS numsats %u (needs 6)", gps.num_sats(preferred_gps));
        gpsCheckStatus.bad_sats = true;
    } else {
        gpsCheckStatus.bad_sats = false;
    }

    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    bool yawFail;
    if ((magTestRatio.x > 1.0f || magTestRatio.y > 1.0f || yawTestRatio > 1.0f) && (frontend->_gpsCheck & MASK_GPS_YAW_ERR)) {
        yawFail = true;
    } else {
        yawFail = false;
    }

    // Report check result as a text string and bitmask
    if (yawFail) {
        dal.snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Mag yaw error x=%.1f y=%.1f",
                           (double)magTestRatio.x,
                           (double)magTestRatio.y);
        gpsCheckStatus.bad_yaw = true;
    } else {
        gpsCheckStatus.bad_yaw = false;
    }

    // assume failed first time through and notify user checks have started
    if (lastGpsVelFail_ms == 0) {
        dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string), "EKF starting GPS checks");
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // record time of pass or fail
    if (gpsSpdAccFail || numSatsFail || hdopFail || hAccFail || vAccFail || yawFail || gpsDriftFail || gpsVertVelFail || gpsHorizVelFail) {
        lastGpsVelFail_ms = imuSampleTime_ms;
    } else {
        lastGpsVelPass_ms = imuSampleTime_ms;
    }

    // continuous period of 10s without fail required to set healthy
    // continuous period of 5s without pass required to set unhealthy
    if (!gpsGoodToAlign && imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        gpsGoodToAlign = true;
    } else if (gpsGoodToAlign && imuSampleTime_ms - lastGpsVelPass_ms > 5000) {
        gpsGoodToAlign = false;
    }
}

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
void NavEKF3_core::calcGpsGoodForFlight(void)
{
    // use a simple criteria based on the GPS receivers claimed speed accuracy and the EKF innovation consistency checks

    // set up varaibles and constants used by filter that is applied to GPS speed accuracy
    const ftype alpha1 = 0.2f; // coefficient for first stage LPF applied to raw speed accuracy data
    const ftype tau = 10.0f; // time constant (sec) of peak hold decay
    if (lastGpsCheckTime_ms == 0) {
        lastGpsCheckTime_ms =  imuSampleTime_ms;
    }
    ftype dtLPF = (imuSampleTime_ms - lastGpsCheckTime_ms) * 1e-3f;
    lastGpsCheckTime_ms = imuSampleTime_ms;
    ftype alpha2 = constrain_ftype(dtLPF/tau,0.0f,1.0f);

    // get the receivers reported speed accuracy
    float gpsSpdAccRaw;
    if (!dal.gps().speed_accuracy(preferred_gps, gpsSpdAccRaw)) {
        gpsSpdAccRaw = 0.0f;
    }

    // filter the raw speed accuracy using a LPF
    sAccFilterState1 = constrain_ftype((alpha1 * gpsSpdAccRaw + (1.0f - alpha1) * sAccFilterState1),0.0f,10.0f);

    // apply a peak hold filter to the LPF output
    sAccFilterState2 = MAX(sAccFilterState1,((1.0f - alpha2) * sAccFilterState2));

    // Apply a threshold test with hysteresis to the filtered GPS speed accuracy data
    if (sAccFilterState2 > 1.5f ) {
        gpsSpdAccPass = false;
    } else if(sAccFilterState2 < 1.0f) {
        gpsSpdAccPass = true;
    }

    // Apply a threshold test with hysteresis to the normalised position and velocity innovations
    // Require a fail for one second and a pass for 10 seconds to transition
    if (lastInnovFailTime_ms == 0) {
        lastInnovFailTime_ms = imuSampleTime_ms;
        lastInnovPassTime_ms = imuSampleTime_ms;
    }
    if (velTestRatio < 1.0f && posTestRatio < 1.0f) {
        lastInnovPassTime_ms = imuSampleTime_ms;
    } else if (velTestRatio > 0.7f || posTestRatio > 0.7f) {
        lastInnovFailTime_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - lastInnovPassTime_ms) > 1000) {
        ekfInnovationsPass = false;
    } else if ((imuSampleTime_ms - lastInnovFailTime_ms) > 10000) {
        ekfInnovationsPass = true;
    }

    // both GPS speed accuracy and EKF innovations must pass
    gpsAccuracyGood = gpsSpdAccPass && ekfInnovationsPass;
}

// Detect if we are in flight or on ground
void NavEKF3_core::detectFlight()
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
        ftype gndSpdSq = sq(gpsDataNew.vel.x) + sq(gpsDataNew.vel.y);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 8 m/s airspeed
        const auto *arsp = dal.airspeed();
        if (arsp && arsp->healthy(selected_airspeed) && arsp->use(selected_airspeed)) {
            if (arsp->get_airspeed(selected_airspeed) * dal.get_EAS2TAS() > 10.0f) {
                highAirSpd = true;
            }
        }

        // trigger on ground speed
        const ftype gndSpdThresholdSq = sq(5.0f);
        if (gndSpdSq > gndSpdThresholdSq + sq(gpsSpdAccuracy)) {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsF(hgtMea) > 10.0f) {
            largeHgtChange = true;
        }

        if (motorsArmed) {
            onGround = false;
            if (highGndSpd && (dal.get_takeoff_expected() || highAirSpd || largeHgtChange)) {
                // to a high certainty we are flying
                inFlight = true;
            }
        } else {
            // to a high certainty we are not flying
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

        if (!onGround) {
            // If height has increased since exiting on-ground, then we definitely are flying
            if ((stateStruct.position.z - posDownAtTakeoff) < -1.5f) {
                inFlight = true;
            }

            // If rangefinder has increased since exiting on-ground, then we definitely are flying
            if ((rangeDataNew.rng - rngAtStartOfFlight) > 0.5f) {
                inFlight = true;
            }

            // If more than 5 seconds since likely_flying was set
            // true, then set inFlight true
            if (dal.get_time_flying_ms() > 5000) {
                inFlight = true;
            }
        }

    }

    // Store vehicle height and range prior to takeoff for use in post takeoff checks
    if (onGround) {
        // store vertical position at start of flight to use as a reference for ground relative checks
        posDownAtTakeoff = stateStruct.position.z;
        // store the range finder measurement which will be used as a reference to detect when we have taken off
        rngAtStartOfFlight = rangeDataNew.rng;
        // if the magnetic field states have been set, then continue to update the vertical position
        // quaternion and yaw innovation snapshots to use as a reference when we start to fly.
        if (magStateInitComplete) {
            posDownAtLastMagReset = stateStruct.position.z;
            quatAtLastMagReset = stateStruct.quat;
            yawInnovAtLastMagReset = innovYaw;
        }
    }

    // handle reset of counters used to control how many times we will try to reset the yaw to the EKF-GSF value per flight
    if ((!prevOnGround && onGround) || !gpsSpdAccPass) {
        // disable filter bank
        EKFGSF_run_filterbank = false;
    } else if (yawEstimator != nullptr && !EKFGSF_run_filterbank && (inFlight || dal.get_takeoff_expected()) && gpsSpdAccPass) {
        // flying or about to fly so reset counters and enable filter bank when GPS is good
        EKFGSF_yaw_reset_ms = 0;
        EKFGSF_yaw_reset_request_ms = 0;
        EKFGSF_yaw_reset_count = 0;
        EKFGSF_yaw_valid_count = 0;
        EKFGSF_run_filterbank = true;
        Vector3f gyroBias;
        getGyroBias(gyroBias);
        yawEstimator->setGyroBias(gyroBias);
    }

    // store current on-ground  and in-air status for next time
    prevOnGround = onGround;
    prevInFlight = inFlight;

}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference. Use to prevent range finder operation otherwise
// enabled by the combination of EK3_RNG_USE_HGT and EK3_RNG_USE_SPD parameters.
void NavEKF3_core::setTerrainHgtStable(bool val)
{
    terrainHgtStable = val;
}

// Detect takeoff for optical flow navigation
void NavEKF3_core::detectOptFlowTakeoff(void)
{
    if (!onGround && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000) {
        // we are no longer confidently on the ground so check the range finder and gyro for signs of takeoff
        const auto &ins = dal.ins();
        Vector3f angRateVec;
        Vector3f gyroBias;
        getGyroBias(gyroBias);
        angRateVec = ins.get_gyro(gyro_index_active) - gyroBias;

        takeOffDetected = (takeOffDetected || (angRateVec.length() > 0.1f) || (rangeDataNew.rng > (rngAtStartOfFlight + 0.1f)));
    } else if (onGround) {
        // we are confidently on the ground so set the takeoff detected status to false
        takeOffDetected = false;
    }
}

