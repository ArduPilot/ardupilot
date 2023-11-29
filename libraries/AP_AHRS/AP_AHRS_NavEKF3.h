#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  SITL-based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF3_ENABLED

#include <AP_NavEKF3/AP_NavEKF3.h>
#include "AP_AHRS_Backend.h"

class AP_AHRS_NavEKF3 : public AP_AHRS_Backend {
public:

    bool started() const { return _ekf3_started; }

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override { EKF3.resetGyroBias(); };

    // Methods
    void update() override;
    void get_results(Estimates &results) override;
    void reset() override {
        if (_ekf3_started) {
            _ekf3_started = EKF3.InitialiseFilter();
        }
    }

    // set the EKF's origin location in 10e7 degrees.  This should only
    // be called when the EKF has no absolute position reference (i.e. GPS)
    // from which to decide the origin on its own
    bool set_origin(const Location &loc) override {
        return EKF3.setOriginLLH(loc);
    }

    // Set the EKF's NE horizontal position states and their corresponding variances from a supplied WGS-84 location and uncertainty
    // The altitude element of the location is not used.
    // Returns true if the set was successful
    bool setLatLng(const Location &loc, float posErr, uint32_t timestamp_ms) {
        return EKF3.setLatLng(loc, posErr, timestamp_ms);
    }

    bool use_compass() override { return EKF3.use_compass(); }

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const override {
        EKF3.getFilterStatus(status);
        return true;
    }

    uint32_t getLastYawResetAngle(float &yawAng) override {
        return EKF3.getLastYawResetAngle(yawAng);
    };
    uint32_t getLastPosNorthEastReset(Vector2f &pos) override {
        return EKF3.getLastPosNorthEastReset(pos);
    }
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const override {
        return EKF3.getLastVelNorthEastReset(vel);
    }
    uint32_t getLastPosDownReset(float &posDelta) override {
        return EKF3.getLastPosDownReset(posDelta);
    }
    bool resetHeightDatum() override {
        return EKF3.resetHeightDatum();;
    }

    // get_filter_status - returns filter status as a series of flags
    void get_filter_faults(uint16_t &faults) const {
        EKF3.getFilterFaults(faults);
    }

    void send_ekf_status_report(class GCS_MAVLINK &link) const override {
        EKF3.send_status_report(link);
    }

    void set_terrain_hgt_stable(bool stable) override {
        EKF3.setTerrainHgtStable(stable);
    }

    bool is_vibration_affected() const {
        return EKF3.isVibrationAffected();
    }

    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override {
        return EKF3.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const override {
        return EKF3.getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }

    // start methods which are not inheritted from AP_AHRS_Backend:
    // allow the enable flag to be set by Replay
    void set_enable(bool enable) { EKF3.set_enable(enable); }

    // get the enable parameter
    bool get_enable(void) const { return EKF3.get_enable(); }

    bool configuredToUseGPSForPosXY() const { return EKF3.configuredToUseGPSForPosXY(); }
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride) {
        EKF3.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset, heightOverride);
    }
    bool getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const {
        return EKF3.getOptFlowSample(timeStamp_ms, flowRate, bodyRate, losPred);
    }
    void  writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset) {
        EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset);
    }
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms) {
        EKF3.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
    }
    void writeDefaultAirSpeed(float airspeed, float uncertainty) {
        EKF3.writeDefaultAirSpeed(airspeed, uncertainty);
    }
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms) {
        EKF3.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
    }
    void getAccelBias(Vector3f& ret, uint8_t idx) const {
        EKF3.getAccelBias(idx, ret);
    }
    void getQuaternionBodyToNED(uint8_t i, Quaternion &ekf3_quat) const {
        EKF3.getQuaternionBodyToNED(i, ekf3_quat);
    }
    uint8_t activeCores() const {
        return EKF3.activeCores();
    }
    uint8_t get_active_airspeed_index() const {
        return EKF3.getActiveAirspeed();
    }
    void check_lane_switch(void) override {
        EKF3.checkLaneSwitch();
    }
    void request_yaw_reset(void) override{
        EKF3.requestYawReset();
    }
    void set_posvelyaw_source_set(uint8_t source_set_idx) override {
        EKF3.setPosVelYawSourceSet(source_set_idx);
    }
    uint8_t get_posvelyaw_source_set() const {
        return EKF3.get_active_source_set();
    }
    void Log_Write() {
        return EKF3.Log_Write();
    }
    const EKFGSF_yaw *get_yaw_estimator(void) const {
        return EKF3.get_yawEstimator();
    }
    void set_alt_measurement_noise(float noise) {
        EKF3.set_baro_alt_noise(noise);
    }
    bool using_extnav_for_yaw(void) const override {
        return EKF3.using_extnav_for_yaw();
    }
    bool using_noncompass_for_yaw(void) const override {
        return EKF3.using_noncompass_for_yaw();
    }
    bool airspeed_health_data(float &innovation, float &innovationVariance, uint32_t &age_ms) const {
        return EKF3.getAirSpdHealthData(innovation, innovationVariance, age_ms);
    }

    // this is out here so parameters can be poked into it
    NavEKF3 EKF3;

private:

    bool get_relative_position_NED_origin(Vector3f &vec) const {
        Vector2f posNE;
        float posD;
        if (EKF3.getPosNE(posNE) && EKF3.getPosD(posD)) {
            // position is valid
            vec.x = posNE.x;
            vec.y = posNE.y;
            vec.z = posD;
            return true;
        }
        return false;
    }

    bool _ekf3_started;
    uint32_t start_time_ms;
    const uint16_t startup_delay_ms = 1000;
};

#endif  // AP_AHRS_NAVEKF3_ENABLED
