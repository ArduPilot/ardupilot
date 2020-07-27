/*
  Definition of functions used to provide a backup estimate of yaw angle
  that doesn't use a magnetometer. Uses a bank of 3-state EKF's organised
  as a Guassian sum filter where states are velocity N,E (m/s) and yaw angle (rad)

  Written by Paul Riseborough <p_riseborough@live.com.au>

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

#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF/EKFGSF_yaw.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

EKFGSF_yaw::EKFGSF_yaw() {};

void EKFGSF_yaw::update(const Vector3f &delAng,
                        const Vector3f &delVel,
                        const float delAngDT,
                        const float delVelDT,
                        bool runEKF,
                        float TAS)
{

    // copy to class variables
    delta_angle = delAng;
    delta_velocity = delVel;
    angle_dt = delAngDT;
    velocity_dt = delVelDT;
    run_ekf_gsf = runEKF;
    true_airspeed = TAS;

    // Calculate a low pass filtered acceleration vector that will be used to keep the AHRS tilt aligned
    // The time constant of the filter is a fixed ratio relative to the time constant of the AHRS tilt correction loop
    const float filter_coef = fminf(EKFGSF_accelFiltRatio * delVelDT * EKFGSF_tiltGain, 1.0f);
    const Vector3f accel = delVel / fmaxf(delVelDT, 0.001f);
    ahrs_accel = ahrs_accel * (1.0f - filter_coef) + accel * filter_coef;

    // Iniitialise states and only when acceleration is close to 1g to prevent vehicle movement casuing a large initial tilt error
    if (!ahrs_tilt_aligned) {
        const float accel_norm_sq = accel.length_squared();
        const float upper_accel_limit = GRAVITY_MSS * 1.1f;
        const float lower_accel_limit = GRAVITY_MSS * 0.9f;
        const bool ok_to_align = ((accel_norm_sq > lower_accel_limit * lower_accel_limit &&
                                   accel_norm_sq < upper_accel_limit * upper_accel_limit));
        if (ok_to_align) {
            alignTilt();
            ahrs_tilt_aligned = true;
            ahrs_accel = accel;
        }
        return;
    }

    // Calculate common variables used by the AHRS prediction models
    ahrs_accel_norm = ahrs_accel.length();

    // Calculate AHRS acceleration fusion gain using a quadratic weighting function that is unity at 1g
    // and zero at the min and max g limits. This reduces the effect of large g transients on the attitude
    // esitmates.
    float EKFGSF_ahrs_ng = ahrs_accel_norm / GRAVITY_MSS;
    if (EKFGSF_ahrs_ng > 1.0f) {
        if (is_positive(true_airspeed)) {
            // When flying in fixed wing mode we need to allow for more positive g due to coordinated turns
            // Gain varies from unity at 1g to zero at 2g
            accel_gain = EKFGSF_tiltGain * sq(2.0f - EKFGSF_ahrs_ng);
        } else if (accel_gain <= 1.5f) {
            // Gain varies from unity at 1g to zero at 1.5g
            accel_gain = EKFGSF_tiltGain * sq(3.0f - 2.0f * EKFGSF_ahrs_ng);
        } else {
            // Gain is zero above max g
            accel_gain = 0.0f;
        }
    } else if (accel_gain > 0.5f) {
        // Gain varies from zero at 0.5g to unity at 1g
        accel_gain = EKFGSF_tiltGain * sq(2.0f * EKFGSF_ahrs_ng - 1.0f);
    } else {
        // Gain is zero below min g
        accel_gain = 0.0f;
    }

    // Always run the AHRS prediction cycle for each model
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
        predict(mdl_idx);
    }

    if (vel_fuse_running && !run_ekf_gsf) {
        vel_fuse_running = false;
    }

    // Calculate a composite yaw as a weighted average of the states for each model.
    // To avoid issues with angle wrapping, the yaw state is converted to a vector with legnth
    // equal to the weighting value before it is summed.
    Vector2f yaw_vector = {};
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
        yaw_vector[0] += GSF.weights[mdl_idx] * cosf(EKF[mdl_idx].X[2]);
        yaw_vector[1] += GSF.weights[mdl_idx] * sinf(EKF[mdl_idx].X[2]);
    }
    GSF.yaw = atan2f(yaw_vector[1],yaw_vector[0]);

    // Example for future reference showing how a full GSF covariance matrix could be calculated if required
    /*
    memset(&GSF.P, 0, sizeof(GSF.P));
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
        float delta[3];
        for (uint8_t row = 0; row < 3; row++) {
            delta[row] = EKF[mdl_idx].X[row] - GSF.X[row];
        }
        for (uint8_t row = 0; row < 3; row++) {
            for (uint8_t col = 0; col < 3; col++) {
                GSF.P[row][col] +=  GSF.weights[mdl_idx] * (EKF[mdl_idx].P[row][col] + delta[row] * delta[col]);
            }
        }
    }
    */

    GSF.yaw_variance = 0.0f;
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
        float yawDelta = wrap_PI(EKF[mdl_idx].X[2] - GSF.yaw);
        GSF.yaw_variance +=  GSF.weights[mdl_idx] * (EKF[mdl_idx].P[2][2] + sq(yawDelta));
    }
}

void EKFGSF_yaw::fuseVelData(const Vector2f &vel, const float velAcc)
{
    // convert reported accuracy to a variance, but limit lower value to protect algorithm stability
    const float velObsVar = sq(fmaxf(velAcc, 0.5f));

    // The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
    if (run_ekf_gsf) {
        if (!vel_fuse_running) {
            // Perform in-flight alignment
            resetEKFGSF();
            for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
                // Use the firstGPS  measurement to set the velocities and corresponding variances
                EKF[mdl_idx].X[0] = vel[0];
                EKF[mdl_idx].X[1] = vel[1];
                EKF[mdl_idx].P[0][0] = velObsVar;
                EKF[mdl_idx].P[1][1] = velObsVar;
            }
            alignYaw();
            vel_fuse_running = true;
        } else {
            float total_w = 0.0f;
            float newWeight[(uint8_t)N_MODELS_EKFGSF];
            bool state_update_failed = false;
            for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
                // Update states and covariances using GPS NE velocity measurements fused as direct state observations
                if (!correct(mdl_idx, vel, velObsVar)) {
                    state_update_failed = true;
                }
            }

            if (!state_update_failed) {
                // Calculate weighting for each model assuming a normal error distribution
                for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
                    newWeight[mdl_idx]= fmaxf(gaussianDensity(mdl_idx) * GSF.weights[mdl_idx], 0.0f);
                    total_w += newWeight[mdl_idx];
                }

                // Normalise the sum of weights to unity
                if (vel_fuse_running && is_positive(total_w)) {
                    float total_w_inv = 1.0f / total_w;
                    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
                        GSF.weights[mdl_idx]  = newWeight[mdl_idx] * total_w_inv;
                    }
                }
            }
        }
    }
}

void EKFGSF_yaw::predictAHRS(const uint8_t mdl_idx)
{
    // Generate attitude solution using simple complementary filter for the selected model

    // Calculate 'k' unit vector of earth frame rotated into body frame
    const Vector3f k(AHRS[mdl_idx].R[2][0], AHRS[mdl_idx].R[2][1], AHRS[mdl_idx].R[2][2]);

    // Calculate angular rate vector in rad/sec averaged across last sample interval
    Vector3f ang_rate_delayed_raw = delta_angle / angle_dt;

    // Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
    // During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward

    Vector3f tilt_error_gyro_correction = {}; // (rad/sec)

    if (accel_gain > 0.0f) {

        Vector3f accel = ahrs_accel;

        if (is_positive(true_airspeed)) {
            // Calculate centripetal acceleration in body frame from cross product of body rate and body frame airspeed vector
            // NOTE: this assumes X axis is aligned with airspeed vector
            Vector3f centripetal_accel_vec_bf = Vector3f(0.0f, ang_rate_delayed_raw[2] * true_airspeed, - ang_rate_delayed_raw[1] * true_airspeed);

            // Correct measured accel for centripetal acceleration
            accel -= centripetal_accel_vec_bf;
        }

        tilt_error_gyro_correction = (k % accel) * (accel_gain / ahrs_accel_norm);

    }

    // Gyro bias estimation
    const float gyro_bias_limit = radians(5.0f);
    const float spinRate = ang_rate_delayed_raw.length();
    if (spinRate < 0.175f) {
        AHRS[mdl_idx].gyro_bias -= tilt_error_gyro_correction * (EKFGSF_gyroBiasGain * angle_dt);

        for (uint8_t i = 0; i < 3; i++) {
            AHRS[mdl_idx].gyro_bias[i] = constrain_float(AHRS[mdl_idx].gyro_bias[i], -gyro_bias_limit, gyro_bias_limit);
        }
    }

    // Calculate the corrected body frame rotation vector for the last sample interval and apply to the rotation matrix
    const Vector3f ahrs_delta_angle = delta_angle + (tilt_error_gyro_correction - AHRS[mdl_idx].gyro_bias) * angle_dt;
    AHRS[mdl_idx].R = updateRotMat(AHRS[mdl_idx].R, ahrs_delta_angle);

}

void EKFGSF_yaw::alignTilt()
{
    // Rotation matrix is constructed directly from acceleration measurement and will be the same for
    // all models so only need to calculate it once. Assumptions are:
    // 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
    // 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.

    // Calculate earth frame Down axis unit vector rotated into body frame
    Vector3f down_in_bf = -delta_velocity;
    down_in_bf.normalize();

    // Calculate earth frame North axis unit vector rotated into body frame, orthogonal to 'down_in_bf'
    // * operator is overloaded to provide a dot product
    const Vector3f i_vec_bf(1.0f,0.0f,0.0f);
    Vector3f north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf * down_in_bf);
    north_in_bf.normalize();

    // Calculate earth frame East axis unit vector rotated into body frame, orthogonal to 'down_in_bf' and 'north_in_bf'
    // % operator is overloaded to provide a cross product
    const Vector3f east_in_bf = down_in_bf % north_in_bf;

    // Each column in a rotation matrix from earth frame to body frame represents the projection of the
    // corresponding earth frame unit vector rotated into the body frame, eg 'north_in_bf' would be the first column.
    // We need the rotation matrix from body frame to earth frame so the earth frame unit vectors rotated into body
    // frame are copied into corresponding rows instead to create the transpose.
    Matrix3f R;
    for (uint8_t col=0; col<3; col++) {
        R[0][col] = north_in_bf[col];
        R[1][col] = east_in_bf[col];
        R[2][col] = down_in_bf[col];
    }

    // record alignment
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        AHRS[mdl_idx].R = R;
        AHRS[mdl_idx].aligned = true;
    }
}

void EKFGSF_yaw::alignYaw()
{
    // Align yaw angle for each model
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        if (fabsf(AHRS[mdl_idx].R[2][0]) < fabsf(AHRS[mdl_idx].R[2][1])) {
            // get the roll, pitch, yaw estimates from the rotation matrix using a  321 Tait-Bryan rotation sequence
            float roll,pitch,yaw;
            AHRS[mdl_idx].R.to_euler(&roll, &pitch, &yaw);

            // set the yaw angle
            yaw = wrap_PI(EKF[mdl_idx].X[2]);

            // update the body to earth frame rotation matrix
            AHRS[mdl_idx].R.from_euler(roll, pitch, yaw);

        } else {
            // Calculate the 312 Tait-Bryan rotation sequence that rotates from earth to body frame
            Vector3f euler312 = AHRS[mdl_idx].R.to_euler312();
            euler312[2] = wrap_PI(EKF[mdl_idx].X[2]); // first rotation (yaw) taken from EKF model state

            // update the body to earth frame rotation matrix
            AHRS[mdl_idx].R.from_euler312(euler312[0], euler312[1], euler312[2]);

        }
    }
}

// predict states and covariance for specified model index
void EKFGSF_yaw::predict(const uint8_t mdl_idx)
{
    // generate an attitude reference using IMU data
    predictAHRS(mdl_idx);

    // we don't start running the EKF part of the algorithm until there are regular velocity observations
    if (!vel_fuse_running) {
        return;
    }

    // Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
    if (fabsf(AHRS[mdl_idx].R[2][0]) < fabsf(AHRS[mdl_idx].R[2][1])) {
        // use 321 Tait-Bryan rotation to define yaw state
        EKF[mdl_idx].X[2] = atan2f(AHRS[mdl_idx].R[1][0], AHRS[mdl_idx].R[0][0]);
    } else {
        // use 312 Tait-Bryan rotation to define yaw state
        EKF[mdl_idx].X[2] = atan2f(-AHRS[mdl_idx].R[0][1], AHRS[mdl_idx].R[1][1]); // first rotation (yaw)
    }

    // calculate delta velocity in a horizontal front-right frame
    const Vector3f del_vel_NED = AHRS[mdl_idx].R * delta_velocity;
    const float dvx =   del_vel_NED[0] * cosf(EKF[mdl_idx].X[2]) + del_vel_NED[1] * sinf(EKF[mdl_idx].X[2]);
    const float dvy = - del_vel_NED[0] * sinf(EKF[mdl_idx].X[2]) + del_vel_NED[1] * cosf(EKF[mdl_idx].X[2]);

    // sum delta velocities in earth frame:
    EKF[mdl_idx].X[0] += del_vel_NED[0];
    EKF[mdl_idx].X[1] += del_vel_NED[1];

    // predict covariance - autocode from https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcPupdate.txt

    // Local short variable name copies required for readability
    // Compiler might be smart enough to optimise these out
    const float P00 = EKF[mdl_idx].P[0][0];
    const float P01 = EKF[mdl_idx].P[0][1];
    const float P02 = EKF[mdl_idx].P[0][2];
    const float P10 = EKF[mdl_idx].P[1][0];
    const float P11 = EKF[mdl_idx].P[1][1];
    const float P12 = EKF[mdl_idx].P[1][2];
    const float P20 = EKF[mdl_idx].P[2][0];
    const float P21 = EKF[mdl_idx].P[2][1];
    const float P22 = EKF[mdl_idx].P[2][2];

    // Use fixed values for delta velocity and delta angle process noise variances
    const float dvxVar = sq(EKFGSF_accelNoise * velocity_dt); // variance of forward delta velocity - (m/s)^2
    const float dvyVar = dvxVar; // variance of right delta velocity - (m/s)^2
    const float dazVar = sq(EKFGSF_gyroNoise * angle_dt); // variance of yaw delta angle - rad^2

    const float t2 = sinf(EKF[mdl_idx].X[2]);
    const float t3 = cosf(EKF[mdl_idx].X[2]);
    const float t4 = dvy*t3;
    const float t5 = dvx*t2;
    const float t6 = t4+t5;
    const float t8 = P22*t6;
    const float t7 = P02-t8;
    const float t9 = dvx*t3;
    const float t11 = dvy*t2;
    const float t10 = t9-t11;
    const float t12 = dvxVar*t2*t3;
    const float t13 = t2*t2;
    const float t14 = t3*t3;
    const float t15 = P22*t10;
    const float t16 = P12+t15;

    const float min_var = 1e-6f;
    EKF[mdl_idx].P[0][0] = fmaxf(P00-P20*t6+dvxVar*t14+dvyVar*t13-t6*t7, min_var);
    EKF[mdl_idx].P[0][1] = P01+t12-P21*t6+t7*t10-dvyVar*t2*t3;
    EKF[mdl_idx].P[0][2] = t7;
    EKF[mdl_idx].P[1][0] = P10+t12+P20*t10-t6*t16-dvyVar*t2*t3;
    EKF[mdl_idx].P[1][1] = fmaxf(P11+P21*t10+dvxVar*t13+dvyVar*t14+t10*t16, min_var);
    EKF[mdl_idx].P[1][2] = t16;
    EKF[mdl_idx].P[2][0] = P20-t8;
    EKF[mdl_idx].P[2][1] = P21+t15;
    EKF[mdl_idx].P[2][2] = fmaxf(P22+dazVar, min_var);

    // force symmetry
    forceSymmetry(mdl_idx);
}

// Update EKF states and covariance for specified model index using velocity measurement
// Returns false if the sttae and covariance correction failed
bool EKFGSF_yaw::correct(const uint8_t mdl_idx, const Vector2f &vel, const float velObsVar)
{
    // calculate velocity observation innovations
    EKF[mdl_idx].innov[0] = EKF[mdl_idx].X[0] - vel[0];
    EKF[mdl_idx].innov[1] = EKF[mdl_idx].X[1] - vel[1];

    // copy covariance matrix to temporary variables
    const float P00 = EKF[mdl_idx].P[0][0];
    const float P01 = EKF[mdl_idx].P[0][1];
    const float P02 = EKF[mdl_idx].P[0][2];
    const float P10 = EKF[mdl_idx].P[1][0];
    const float P11 = EKF[mdl_idx].P[1][1];
    const float P12 = EKF[mdl_idx].P[1][2];
    const float P20 = EKF[mdl_idx].P[2][0];
    const float P21 = EKF[mdl_idx].P[2][1];
    const float P22 = EKF[mdl_idx].P[2][2];

    // calculate innovation variance
    EKF[mdl_idx].S[0][0] = P00 + velObsVar;
    EKF[mdl_idx].S[1][1] = P11 + velObsVar;
    EKF[mdl_idx].S[0][1] = P01;
    EKF[mdl_idx].S[1][0] = P10;

    // Perform a chi-square innovation consistency test and calculate a compression scale factor that limits the magnitude of innovations to 5-sigma
    float S_det_inv = (EKF[mdl_idx].S[0][0]*EKF[mdl_idx].S[1][1] - EKF[mdl_idx].S[0][1]*EKF[mdl_idx].S[1][0]);
    float innov_comp_scale_factor = 1.0f;
    if (fabsf(S_det_inv) > 1E-6f) {
        // Calculate elements for innovation covariance inverse matrix assuming symmetry
        S_det_inv = 1.0f / S_det_inv;
        const float S_inv_NN = EKF[mdl_idx].S[1][1] * S_det_inv;
        const float S_inv_EE = EKF[mdl_idx].S[0][0] * S_det_inv;
        const float S_inv_NE = EKF[mdl_idx].S[0][1] * S_det_inv;

        // The following expression was derived symbolically from test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
        const float test_ratio = EKF[mdl_idx].innov[0]*(EKF[mdl_idx].innov[0]*S_inv_NN + EKF[mdl_idx].innov[1]*S_inv_NE) + EKF[mdl_idx].innov[1]*(EKF[mdl_idx].innov[0]*S_inv_NE + EKF[mdl_idx].innov[1]*S_inv_EE);

        // If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
        // This protects from large measurement spikes
        if (test_ratio > 25.0f) {
            innov_comp_scale_factor = sqrtf(25.0f / test_ratio);
        }
    } else {
        // skip this fusion step because calculation is badly conditioned
        return false;
    }

    // calculate Kalman gain K  and covariance matrix P
    // autocode from https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcK.txt
    // and https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcPmat.txt
    const float t2 = P00*velObsVar;
    const float t3 = P11*velObsVar;
    const float t4 = velObsVar*velObsVar;
    const float t5 = P00*P11;
    const float t9 = P01*P10;
    const float t6 = t2+t3+t4+t5-t9;
    float t7;
    if (fabsf(t6) > 1e-6f) {
        t7 = 1.0f/t6;
    } else {
        // skip this fusion step
        return false;
    }
    const float t8 = P11+velObsVar;
    const float t10 = P00+velObsVar;
    float K[3][2];

    K[0][0] = -P01*P10*t7+P00*t7*t8;
    K[0][1] = -P00*P01*t7+P01*t7*t10;
    K[1][0] = -P10*P11*t7+P10*t7*t8;
    K[1][1] = -P01*P10*t7+P11*t7*t10;
    K[2][0] = -P10*P21*t7+P20*t7*t8;
    K[2][1] = -P01*P20*t7+P21*t7*t10;

    const float t11 = P00*P01*t7;
    const float t15 = P01*t7*t10;
    const float t12 = t11-t15;
    const float t13 = P01*P10*t7;
    const float t16 = P00*t7*t8;
    const float t14 = t13-t16;
    const float t17 = t8*t12;
    const float t18 = P01*t14;
    const float t19 = t17+t18;
    const float t20 = t10*t14;
    const float t21 = P10*t12;
    const float t22 = t20+t21;
    const float t27 = P11*t7*t10;
    const float t23 = t13-t27;
    const float t24 = P10*P11*t7;
    const float t26 = P10*t7*t8;
    const float t25 = t24-t26;
    const float t28 = t8*t23;
    const float t29 = P01*t25;
    const float t30 = t28+t29;
    const float t31 = t10*t25;
    const float t32 = P10*t23;
    const float t33 = t31+t32;
    const float t34 = P01*P20*t7;
    const float t38 = P21*t7*t10;
    const float t35 = t34-t38;
    const float t36 = P10*P21*t7;
    const float t39 = P20*t7*t8;
    const float t37 = t36-t39;
    const float t40 = t8*t35;
    const float t41 = P01*t37;
    const float t42 = t40+t41;
    const float t43 = t10*t37;
    const float t44 = P10*t35;
    const float t45 = t43+t44;

    const float min_var = 1e-6f;
    EKF[mdl_idx].P[0][0] = fmaxf(P00-t12*t19-t14*t22, min_var);
    EKF[mdl_idx].P[0][1] = P01-t19*t23-t22*t25;
    EKF[mdl_idx].P[0][2] = P02-t19*t35-t22*t37;
    EKF[mdl_idx].P[1][0] = P10-t12*t30-t14*t33;
    EKF[mdl_idx].P[1][1] = fmaxf(P11-t23*t30-t25*t33, min_var);
    EKF[mdl_idx].P[1][2] = P12-t30*t35-t33*t37;
    EKF[mdl_idx].P[2][0] = P20-t12*t42-t14*t45;
    EKF[mdl_idx].P[2][1] = P21-t23*t42-t25*t45;
    EKF[mdl_idx].P[2][2] = fmaxf(P22-t35*t42-t37*t45, min_var);

    // force symmetry
    forceSymmetry(mdl_idx);

    // Apply state corrections and capture change in yaw angle
    const float yaw_prev = EKF[mdl_idx].X[2];
    for (uint8_t obs_index = 0; obs_index < 2; obs_index++) {
        // apply the state corrections including the compression scale factor
        for (unsigned row = 0; row < 3; row++) {
            EKF[mdl_idx].X[row] -= K[row][obs_index] * EKF[mdl_idx].innov[obs_index] * innov_comp_scale_factor;
        }
    }
    const float yaw_delta = EKF[mdl_idx].X[2] - yaw_prev;

    // apply the change in yaw angle to the AHRS taking advantage of sparseness in the yaw rotation matrix
    const float cos_yaw = cosf(yaw_delta);
    const float sin_yaw = sinf(yaw_delta);
    float  R_prev[2][3];
    memcpy(&R_prev, &AHRS[mdl_idx].R, sizeof(R_prev)); // copy first two rows from 3x3
    AHRS[mdl_idx].R[0][0] = R_prev[0][0] * cos_yaw - R_prev[1][0] * sin_yaw;
    AHRS[mdl_idx].R[0][1] = R_prev[0][1] * cos_yaw - R_prev[1][1] * sin_yaw;
    AHRS[mdl_idx].R[0][2] = R_prev[0][2] * cos_yaw - R_prev[1][2] * sin_yaw;
    AHRS[mdl_idx].R[1][0] = R_prev[0][0] * sin_yaw + R_prev[1][0] * cos_yaw;
    AHRS[mdl_idx].R[1][1] = R_prev[0][1] * sin_yaw + R_prev[1][1] * cos_yaw;
    AHRS[mdl_idx].R[1][2] = R_prev[0][2] * sin_yaw + R_prev[1][2] * cos_yaw;

    return true;
}

void EKFGSF_yaw::resetEKFGSF()
{
    memset(&GSF, 0, sizeof(GSF));
    vel_fuse_running = false;
    run_ekf_gsf = false;

    memset(&EKF, 0, sizeof(EKF));
    const float yaw_increment = M_2PI / (float)N_MODELS_EKFGSF;
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        // evenly space initial yaw estimates in the region between +-Pi
        EKF[mdl_idx].X[2] = -M_PI + (0.5f * yaw_increment) + ((float)mdl_idx * yaw_increment);

        // All filter models start with the same weight
        GSF.weights[mdl_idx] = 1.0f / (float)N_MODELS_EKFGSF;

        // Use half yaw interval for yaw uncertainty as that is the maximum that the best model can be away from truth
        GSF.yaw_variance = sq(0.5f * yaw_increment);
        EKF[mdl_idx].P[2][2] = GSF.yaw_variance;
    }
}

// returns the probability of a selected model output assuming a gaussian error distribution
float EKFGSF_yaw::gaussianDensity(const uint8_t mdl_idx) const
{
    const float t2 = EKF[mdl_idx].S[0][0] * EKF[mdl_idx].S[1][1];
    const float t5 = EKF[mdl_idx].S[0][1] * EKF[mdl_idx].S[1][0];
    const float t3 = t2 - t5; // determinant
    const float t4 = 1.0f / MAX(t3, 1e-12f); // determinant inverse

    // inv(S)
    float invMat[2][2];
    invMat[0][0] =   t4 * EKF[mdl_idx].S[1][1];
    invMat[1][1] =   t4 * EKF[mdl_idx].S[0][0];
    invMat[0][1] = - t4 * EKF[mdl_idx].S[0][1];
    invMat[1][0] = - t4 * EKF[mdl_idx].S[1][0];

    // inv(S) * innovation
    float tempVec[2];
    tempVec[0] = invMat[0][0] * EKF[mdl_idx].innov[0] + invMat[0][1] * EKF[mdl_idx].innov[1];
    tempVec[1] = invMat[1][0] * EKF[mdl_idx].innov[0] + invMat[1][1] * EKF[mdl_idx].innov[1];

    // transpose(innovation) * inv(S) * innovation
    float normDist = tempVec[0] * EKF[mdl_idx].innov[0] + tempVec[1] * EKF[mdl_idx].innov[1];

    // convert from a normalised variance to a probability assuming a Gaussian distribution
    normDist = expf(-0.5f * normDist);
    normDist *= sqrtf(t4)/ M_2PI;
    return normDist;
}

bool EKFGSF_yaw::getLogData(float &yaw_composite, float &yaw_composite_variance, float yaw[N_MODELS_EKFGSF], float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
    if (vel_fuse_running) {
        yaw_composite = GSF.yaw;
        yaw_composite_variance = GSF.yaw_variance;
        for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
            yaw[mdl_idx] = EKF[mdl_idx].X[2];
            innov_VN[mdl_idx] = EKF[mdl_idx].innov[0];
            innov_VE[mdl_idx] = EKF[mdl_idx].innov[1];
            weight[mdl_idx] = GSF.weights[mdl_idx];
        }
        return true;
    }
    return false;
}

void EKFGSF_yaw::forceSymmetry(const uint8_t mdl_idx)
{
    float P01 = 0.5f * (EKF[mdl_idx].P[0][1] + EKF[mdl_idx].P[1][0]);
    float P02 = 0.5f * (EKF[mdl_idx].P[0][2] + EKF[mdl_idx].P[2][0]);
    float P12 = 0.5f * (EKF[mdl_idx].P[1][2] + EKF[mdl_idx].P[2][1]);
    EKF[mdl_idx].P[0][1] = EKF[mdl_idx].P[1][0] = P01;
    EKF[mdl_idx].P[0][2] = EKF[mdl_idx].P[2][0] = P02;
    EKF[mdl_idx].P[1][2] = EKF[mdl_idx].P[2][1] = P12;
}

// Apply a body frame delta angle to the body to earth frame rotation matrix using a small angle approximation
Matrix3f EKFGSF_yaw::updateRotMat(const Matrix3f &R, const Vector3f &g)
{
    Matrix3f ret = R;
    ret[0][0] += R[0][1] * g[2] - R[0][2] * g[1];
    ret[0][1] += R[0][2] * g[0] - R[0][0] * g[2];
    ret[0][2] += R[0][0] * g[1] - R[0][1] * g[0];
    ret[1][0] += R[1][1] * g[2] - R[1][2] * g[1];
    ret[1][1] += R[1][2] * g[0] - R[1][0] * g[2];
    ret[1][2] += R[1][0] * g[1] - R[1][1] * g[0];
    ret[2][0] += R[2][1] * g[2] - R[2][2] * g[1];
    ret[2][1] += R[2][2] * g[0] - R[2][0] * g[2];
    ret[2][2] += R[2][0] * g[1] - R[2][1] * g[0];

    // Renormalise rows
    float rowLengthSq;
    for (uint8_t r = 0; r < 3; r++) {
        rowLengthSq = ret[r][0] * ret[r][0] + ret[r][1] * ret[r][1] + ret[r][2] * ret[r][2];
        if (is_positive(rowLengthSq)) {
            // Use linear approximation for inverse sqrt taking advantage of the row length being close to 1.0
            const float rowLengthInv = 1.5f - 0.5f * rowLengthSq;
            Vector3f &row = ret[r];
            row *= rowLengthInv;
        }
    }

    return ret;
}

bool EKFGSF_yaw::getYawData(float &yaw, float &yawVariance)
{
    if (!vel_fuse_running) {
        return false;
    }
    yaw = GSF.yaw;
    yawVariance = GSF.yaw_variance;
    return true;
}

void EKFGSF_yaw::setGyroBias(Vector3f &gyroBias)
{
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        AHRS[mdl_idx].gyro_bias = gyroBias;
    }
}