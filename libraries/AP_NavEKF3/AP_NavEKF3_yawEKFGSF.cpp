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

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

void NavEKF3_core::EKFGSF_predictQuat(const uint8_t mdl_idx)
{
	// Generate attitude solution using simple complementary filter for the selected model

	// Calculate 'k' unit vector of earth frame rotated into body frame
	const Vector3f k(
		2.0f * (EKFGSF_ahrs[mdl_idx].quat[1] * EKFGSF_ahrs[mdl_idx].quat[3] - EKFGSF_ahrs[mdl_idx].quat[0] * EKFGSF_ahrs[mdl_idx].quat[2]),
		2.0f * (EKFGSF_ahrs[mdl_idx].quat[2] * EKFGSF_ahrs[mdl_idx].quat[3] + EKFGSF_ahrs[mdl_idx].quat[0] * EKFGSF_ahrs[mdl_idx].quat[1]),
		(EKFGSF_ahrs[mdl_idx].quat[0] * EKFGSF_ahrs[mdl_idx].quat[0] - EKFGSF_ahrs[mdl_idx].quat[1] * EKFGSF_ahrs[mdl_idx].quat[1] - EKFGSF_ahrs[mdl_idx].quat[2] * EKFGSF_ahrs[mdl_idx].quat[2] + EKFGSF_ahrs[mdl_idx].quat[3] * EKFGSF_ahrs[mdl_idx].quat[3])
	);

    // Calculate angular rate vector in rad/sec averaged across last sample interval
    Vector3f ang_rate_delayed_raw = imuDataDelayed.delAng / imuDataDelayed.delAngDT;

	// Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
	// During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward
	Vector3f tilt_error_gyro_correction = {}; // (rad/sec)
	Vector3f accel = EKFGSF_ahrs_accel;
	if (EKFGSF_ahrs_accel_norm > 0.5f * GRAVITY_MSS && (EKFGSF_ahrs_accel_norm < 1.5f * GRAVITY_MSS || EKFGSF_ahrs_turn_comp_enabled)) {
		if (EKFGSF_ahrs_turn_comp_enabled) {
			// Turn rate is component of gyro rate about vertical (down) axis
			const float turn_rate = EKFGSF_ahrs[mdl_idx].R[2][0] * ang_rate_delayed_raw[0]
					  + EKFGSF_ahrs[mdl_idx].R[2][1] * ang_rate_delayed_raw[1]
					  + EKFGSF_ahrs[mdl_idx].R[2][2] * ang_rate_delayed_raw[2];

			// Use measured airspeed to calculate centripetal acceleration if available
			float centripetal_accel;
			if (useAirspeed() && imuDataDelayed.time_ms < (tasDataDelayed.time_ms + 1000)) {
				centripetal_accel = tasDataDelayed.tas * turn_rate;
			} else {
				// Use default airspeed value scaled for density altitude
				centripetal_accel = frontend->EKFGSF_easDefault * AP::ahrs().get_EAS2TAS() * turn_rate;
			}

			// Project Y body axis onto horizontal and multiply by centripetal acceleration to give estimated
			// centripetal acceleration vector in earth frame due to coordinated turn
			Vector3f centripetal_accel_vec_ef = {EKFGSF_ahrs[mdl_idx].R[0][1], EKFGSF_ahrs[mdl_idx].R[1][1], 0.0f};
			if (EKFGSF_ahrs[mdl_idx].R[2][2] > 0.0f) {
				// Vehicle is upright
				centripetal_accel_vec_ef *= centripetal_accel;
			} else {
				// Vehicle is inverted
				centripetal_accel_vec_ef *= - centripetal_accel;
			}

			// Rotate into body frame
			Vector3f centripetal_accel_vec_bf = EKFGSF_ahrs[mdl_idx].R.transposed() * centripetal_accel_vec_ef;

			// Correct measured accel for centripetal acceleration
			accel -= centripetal_accel_vec_bf;
		}

		tilt_error_gyro_correction = (k % accel) * (EKFGSF_accel_gain / EKFGSF_ahrs_accel_norm);

	}

	// Gyro bias estimation
	const float gyro_bias_limit = radians(5.0f);
	const float spinRate = ang_rate_delayed_raw.length();
	if (spinRate < 0.175f) {
		EKFGSF_ahrs[mdl_idx].gyro_bias -= tilt_error_gyro_correction * (frontend->EKFGSF_gyroBiasGain * imuDataDelayed.delAngDT);

		for (int i = 0; i < 3; i++) {
			EKFGSF_ahrs[mdl_idx].gyro_bias[i] = constrain_float(EKFGSF_ahrs[mdl_idx].gyro_bias[i], -gyro_bias_limit, gyro_bias_limit);
		}
	}

	// Calculate the corrected body frame rotation vector for the last sample interval
	const Vector3f delta_angle = imuDataDelayed.delAng + (tilt_error_gyro_correction - EKFGSF_ahrs[mdl_idx].gyro_bias) * imuDataDelayed.delAngDT;

	// Rotate quaternion from previous to current time index
    Quaternion deltaQuat;
    EKFGSF_ahrs[mdl_idx].quat.rotate(delta_angle);

	// Normalize quaternion
	EKFGSF_ahrs[mdl_idx].quat.normalize();

	// Uodate body to earth frame rotation matrix
    EKFGSF_ahrs[mdl_idx].quat.rotation_matrix(EKFGSF_ahrs[mdl_idx].R);

}

void NavEKF3_core::EKFGSF_alignQuatTilt()
{
	// Rotation matrix is constructed directly from acceleration measurement and will be the same for
	// all models so only need to calculate it once. Assumptions are:
	// 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
	// 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.

	// Calculate earth frame Down axis unit vector rotated into body frame
	Vector3f down_in_bf = -imuDataDelayed.delVel;
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

	// Convert to quaternion
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
		EKFGSF_ahrs[mdl_idx].quat.from_rotation_matrix(R);
		EKFGSF_ahrs[mdl_idx].quat.normalize();
		EKFGSF_ahrs[mdl_idx].quat_initialised = true;
	}
}

void NavEKF3_core::EKFGSF_alignQuatYaw()
{
	// Align yaw angle for each model
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
		if (fabsf(EKFGSF_ahrs[mdl_idx].R[2][0]) < fabsf(EKFGSF_ahrs[mdl_idx].R[2][1])) {
			// get the roll, pitch, yaw estimates from the rotation matrix using a  321 Tait-Bryan rotation sequence
            float roll,pitch,yaw;
            EKFGSF_ahrs[mdl_idx].R.to_euler(&roll, &pitch, &yaw);

			// set the yaw angle
			yaw = wrap_PI(EKFGSF_mdl[mdl_idx].X[2]);

			// update the body to earth frame rotation matrix
            EKFGSF_ahrs[mdl_idx].R.from_euler(roll, pitch, yaw);

		} else {
			// Calculate the 312 Tait-Bryan rotation sequence that rotates from earth to body frame
            Vector3f euler312 = EKFGSF_ahrs[mdl_idx].R.to_euler312();
			euler312[2] = wrap_PI(EKFGSF_mdl[mdl_idx].X[2]); // first rotation (yaw) taken from EKF model state

			// update the body to earth frame rotation matrix
            EKFGSF_ahrs[mdl_idx].R.from_euler312(euler312[0], euler312[1], euler312[2]);

		}

        // update the quaternion
        EKFGSF_ahrs[mdl_idx].quat.from_rotation_matrix(EKFGSF_ahrs[mdl_idx].R);
		EKFGSF_ahrs[mdl_idx].quat_initialised = true;

	}
}

// predict states and covariance for specified model index
void NavEKF3_core::EKFGSF_predict(const uint8_t mdl_idx)
{
	// generate an attitude reference using IMU data
	EKFGSF_predictQuat(mdl_idx);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	if (!EKFGSF_vel_fuse_started) {
		return;
	}

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	if (fabsf(EKFGSF_ahrs[mdl_idx].R[2][0]) < fabsf(EKFGSF_ahrs[mdl_idx].R[2][1])) {
		// use 321 Tait-Bryan rotation to define yaw state
		EKFGSF_mdl[mdl_idx].X[2] = atan2f(EKFGSF_ahrs[mdl_idx].R[1][0], EKFGSF_ahrs[mdl_idx].R[0][0]);
		EKFGSF_mdl[mdl_idx].use_312 = false;
	} else {
		// use 312 Tait-Bryan rotation to define yaw state
		EKFGSF_mdl[mdl_idx].X[2] = atan2f(-EKFGSF_ahrs[mdl_idx].R[0][1], EKFGSF_ahrs[mdl_idx].R[1][1]); // first rotation (yaw)
		EKFGSF_mdl[mdl_idx].use_312 = true;
	}

	// calculate delta velocity in a horizontal front-right frame
	const Vector3f del_vel_NED = EKFGSF_ahrs[mdl_idx].R * imuDataDelayed.delVel;
	const float dvx =   del_vel_NED[0] * cosf(EKFGSF_mdl[mdl_idx].X[2]) + del_vel_NED[1] * sinf(EKFGSF_mdl[mdl_idx].X[2]);
	const float dvy = - del_vel_NED[0] * sinf(EKFGSF_mdl[mdl_idx].X[2]) + del_vel_NED[1] * cosf(EKFGSF_mdl[mdl_idx].X[2]);

	// sum delta velocities in earth frame:
	EKFGSF_mdl[mdl_idx].X[0] += del_vel_NED[0];
	EKFGSF_mdl[mdl_idx].X[1] += del_vel_NED[1];

	// predict covariance - autocode from https://github.com/priseborough/3_state_filter/blob/flightLogReplay-wip/calcPupdate.txt

	// Local short variable name copies required for readability
	// Compiler might be smart enough to optimise these out
	const float P00 = EKFGSF_mdl[mdl_idx].P[0][0];
	const float P01 = EKFGSF_mdl[mdl_idx].P[0][1];
	const float P02 = EKFGSF_mdl[mdl_idx].P[0][2];
	const float P10 = EKFGSF_mdl[mdl_idx].P[1][0];
	const float P11 = EKFGSF_mdl[mdl_idx].P[1][1];
	const float P12 = EKFGSF_mdl[mdl_idx].P[1][2];
	const float P20 = EKFGSF_mdl[mdl_idx].P[2][0];
	const float P21 = EKFGSF_mdl[mdl_idx].P[2][1];
	const float P22 = EKFGSF_mdl[mdl_idx].P[2][2];

	// Use fixed values for delta velocity and delta angle process noise variances
	const float dvxVar = sq(frontend->EKFGSF_accelNoise * imuDataDelayed.delVelDT); // variance of forward delta velocity - (m/s)^2
	const float dvyVar = dvxVar; // variance of right delta velocity - (m/s)^2
	const float dazVar = sq(frontend->EKFGSF_gyroNoise * imuDataDelayed.delAngDT); // variance of yaw delta angle - rad^2

	const float t2 = sinf(EKFGSF_mdl[mdl_idx].X[2]);
	const float t3 = cosf(EKFGSF_mdl[mdl_idx].X[2]);
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
	EKFGSF_mdl[mdl_idx].P[0][0] = fmaxf(P00-P20*t6+dvxVar*t14+dvyVar*t13-t6*t7 , min_var);
	EKFGSF_mdl[mdl_idx].P[0][1] = P01+t12-P21*t6+t7*t10-dvyVar*t2*t3;
	EKFGSF_mdl[mdl_idx].P[0][2] = t7;
	EKFGSF_mdl[mdl_idx].P[1][0] = P10+t12+P20*t10-t6*t16-dvyVar*t2*t3;
	EKFGSF_mdl[mdl_idx].P[1][1] = fmaxf(P11+P21*t10+dvxVar*t13+dvyVar*t14+t10*t16 , min_var);
	EKFGSF_mdl[mdl_idx].P[1][2] = t16;
	EKFGSF_mdl[mdl_idx].P[2][0] = P20-t8;
	EKFGSF_mdl[mdl_idx].P[2][1] = P21+t15;
	EKFGSF_mdl[mdl_idx].P[2][2] = fmaxf(P22+dazVar , min_var);

	// force symmetry
	EKFGSF_forceSymmetry(mdl_idx);
}

// Update EKF states and covariance for specified model index using velocity measurement
void NavEKF3_core::EKFGSF_correct(const uint8_t mdl_idx)
{
	// set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	const float velObsVar = sq(fmaxf(gpsSpdAccuracy, frontend->_gpsHorizVelNoise));

	// calculate velocity observation innovations
	EKFGSF_mdl[mdl_idx].innov[0] = EKFGSF_mdl[mdl_idx].X[0] - gpsDataDelayed.vel[0];
	EKFGSF_mdl[mdl_idx].innov[1] = EKFGSF_mdl[mdl_idx].X[1] - gpsDataDelayed.vel[1];

	// copy covariance matrix to temporary variables
	const float P00 = EKFGSF_mdl[mdl_idx].P[0][0];
	const float P01 = EKFGSF_mdl[mdl_idx].P[0][1];
	const float P02 = EKFGSF_mdl[mdl_idx].P[0][2];
	const float P10 = EKFGSF_mdl[mdl_idx].P[1][0];
	const float P11 = EKFGSF_mdl[mdl_idx].P[1][1];
	const float P12 = EKFGSF_mdl[mdl_idx].P[1][2];
	const float P20 = EKFGSF_mdl[mdl_idx].P[2][0];
	const float P21 = EKFGSF_mdl[mdl_idx].P[2][1];
	const float P22 = EKFGSF_mdl[mdl_idx].P[2][2];

	// calculate innovation variance
	EKFGSF_mdl[mdl_idx].S[0][0] = P00 + velObsVar;
	EKFGSF_mdl[mdl_idx].S[1][1] = P11 + velObsVar;
	EKFGSF_mdl[mdl_idx].S[0][1] = P01;
	EKFGSF_mdl[mdl_idx].S[1][0] = P10;

	// Perform a chi-square innovation consistency test and calculate a compression scale factor that limits the magnitude of innovations to 5-sigma
	float S_det_inv = (EKFGSF_mdl[mdl_idx].S[0][0]*EKFGSF_mdl[mdl_idx].S[1][1] - EKFGSF_mdl[mdl_idx].S[0][1]*EKFGSF_mdl[mdl_idx].S[1][0]);
	float innov_comp_scale_factor = 1.0f;
	if (fabsf(S_det_inv) > 1E-6f) {
		// Calculate elements for innovation covariance inverse matrix assuming symmetry
		S_det_inv = 1.0f / S_det_inv;
		const float S_inv_NN = EKFGSF_mdl[mdl_idx].S[1][1] * S_det_inv;
		const float S_inv_EE = EKFGSF_mdl[mdl_idx].S[0][0] * S_det_inv;
		const float S_inv_NE = EKFGSF_mdl[mdl_idx].S[0][1] * S_det_inv;

		// The following expression was derived symbolically from test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
		const float test_ratio = EKFGSF_mdl[mdl_idx].innov[0]*(EKFGSF_mdl[mdl_idx].innov[0]*S_inv_NN + EKFGSF_mdl[mdl_idx].innov[1]*S_inv_NE) + EKFGSF_mdl[mdl_idx].innov[1]*(EKFGSF_mdl[mdl_idx].innov[0]*S_inv_NE + EKFGSF_mdl[mdl_idx].innov[1]*S_inv_EE);

		// If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
		// This protects from large measurement spikes
		if (test_ratio > 25.0f) {
			innov_comp_scale_factor = sqrtf(25.0f / test_ratio);
		}
	} else {
		// skip this fusion step because calculation is badly conditioned
		return;
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
		return;
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
	EKFGSF_mdl[mdl_idx].P[0][0] = fmaxf(P00-t12*t19-t14*t22 , min_var);
	EKFGSF_mdl[mdl_idx].P[0][1] = P01-t19*t23-t22*t25;
	EKFGSF_mdl[mdl_idx].P[0][2] = P02-t19*t35-t22*t37;
	EKFGSF_mdl[mdl_idx].P[1][0] = P10-t12*t30-t14*t33;
	EKFGSF_mdl[mdl_idx].P[1][1] = fmaxf(P11-t23*t30-t25*t33 , min_var);
	EKFGSF_mdl[mdl_idx].P[1][2] = P12-t30*t35-t33*t37;
	EKFGSF_mdl[mdl_idx].P[2][0] = P20-t12*t42-t14*t45;
	EKFGSF_mdl[mdl_idx].P[2][1] = P21-t23*t42-t25*t45;
	EKFGSF_mdl[mdl_idx].P[2][2] = fmaxf(P22-t35*t42-t37*t45 , min_var);

	// force symmetry
	EKFGSF_forceSymmetry(mdl_idx);

	for (uint8_t obs_index = 0; obs_index < 2; obs_index++) {
		// apply the state corrections including the compression scale factor
		for (unsigned row = 0; row < 3; row++) {
			EKFGSF_mdl[mdl_idx].X[row] -= K[row][obs_index] * EKFGSF_mdl[mdl_idx].innov[obs_index] * innov_comp_scale_factor;
		}
	}

	// Apply yaw correction to AHRS quaternion sing the same rotation sequence as was used by the prediction step
	// TODO - This is an  expensive process due to the number of trig operations so a method of doing it more efficiently,
	// eg storing rotation matrix from the state prediction that doesn't include the yaw rotation should be investigated.
	// This matrix could then be multiplied with the yaw rotation to obtain the updated R matrix from which the updated
	// quaternion is calculated
	if (EKFGSF_mdl[mdl_idx].use_312) {
		// Calculate the 312 Tait-Bryan rotation sequence that rotates from earth to body frame
		// We use a 312 sequence as an alternate when there is more pitch tilt than roll tilt
		// to avoid gimbal lock
		Vector3f euler312 =EKFGSF_ahrs[mdl_idx].R.to_euler312();
		euler312[2] = EKFGSF_mdl[mdl_idx].X[2]; // first rotation is about Z and is taken from 3-state EKF

		// Calculate the body to earth frame rotation matrix
       EKFGSF_ahrs[mdl_idx].R.from_euler312(euler312[0], euler312[1], euler312[2]);


	} else {
		// using a 321 Tait-Bryan rotation to define yaw state
		// take roll pitch yaw from AHRS prediction
        float roll,pitch,yaw;
        EKFGSF_ahrs[mdl_idx].R.to_euler(&roll, &pitch, &yaw);

		// replace the yaw angle using the EKF state estimate
		yaw = EKFGSF_mdl[mdl_idx].X[2];

		// Calculate the body to earth frame rotation matrix
       EKFGSF_ahrs[mdl_idx].R.from_euler(roll, pitch, yaw);

	}

    // update the quaternion used by the AHRS prediction algorithm
    EKFGSF_ahrs[mdl_idx].quat.from_rotation_matrix(EKFGSF_ahrs[mdl_idx].R);

}

void NavEKF3_core::EKFGSF_initialise()
{
	memset(&EKFGSF_GSF, 0, sizeof(EKFGSF_GSF));
	EKFGSF_vel_fuse_started = false;
	EKFGSF_yaw_reset_time_ms = 0;
    EKFGSF_yaw_reset_count = 0;
	memset(&EKFGSF_mdl, 0, sizeof(EKFGSF_mdl));
	const float yaw_increment = M_2PI / (float)N_MODELS_EKFGSF;
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
		// evenly space initial yaw estimates in the region between +-Pi
		EKFGSF_mdl[mdl_idx].X[2] = -M_PI + (0.5f * yaw_increment) + ((float)mdl_idx * yaw_increment);

		// All filter models start with the same weight
		EKFGSF_GSF.weights[mdl_idx] = 1.0f / (float)N_MODELS_EKFGSF;

		if (frontend->_fusionModeGPS <= 1) {
			// Take state and variance estimates direct from GPS
			EKFGSF_mdl[mdl_idx].X[0] = gpsDataDelayed.vel[0];
			EKFGSF_mdl[mdl_idx].X[1] = gpsDataDelayed.vel[1];
			EKFGSF_mdl[mdl_idx].P[0][0] = sq(fmaxf(gpsSpdAccuracy, frontend->_gpsHorizVelNoise));
			EKFGSF_mdl[mdl_idx].P[1][1] = EKFGSF_mdl[mdl_idx].P[0][0];
		} else {
			// No velocity data so assume initial alignment conditions
			EKFGSF_mdl[mdl_idx].P[0][0] = sq(0.5f);
			EKFGSF_mdl[mdl_idx].P[1][1] = sq(0.5f);
		}

		// Use half yaw interval for yaw uncertainty as that is the maximum that the best model can be away from truth
        EKFGSF_GSF.yaw_variance = sq(0.5f * yaw_increment);
		EKFGSF_mdl[mdl_idx].P[2][2] = EKFGSF_GSF.yaw_variance;
	}
}

void NavEKF3_core::EKFGSF_run()
{
	// Iniitialise states and only when acceleration is close to 1g to rpevent vehicle movement casuing a large initial tilt error
	EKFGSF_ahrs_accel = imuDataDelayed.delVel / fmaxf(imuDataDelayed.delVelDT, dtEkfAvg / 4);
	if (!EKFGSF_ahrs_tilt_aligned) {
		const float accel_norm_sq = EKFGSF_ahrs_accel.length_squared();
		const float upper_accel_limit = GRAVITY_MSS * 1.1f;
		const float lower_accel_limit = GRAVITY_MSS * 0.9f;
		const bool ok_to_align = ((accel_norm_sq > lower_accel_limit * lower_accel_limit &&
			  accel_norm_sq < upper_accel_limit * upper_accel_limit));
		if (ok_to_align) {
			EKFGSF_initialise();
			EKFGSF_alignQuatTilt();
			EKFGSF_ahrs_tilt_aligned = true;
		}
		return;
	}

	// Calculate common varaibles used by the AHRS prediction models
	EKFGSF_ahrs_accel_norm = EKFGSF_ahrs_accel.length();
	EKFGSF_ahrs_turn_comp_enabled = assume_zero_sideslip() && frontend->EKFGSF_easDefault > FLT_EPSILON;
	if (EKFGSF_ahrs_accel_norm > GRAVITY_MSS) {
		if (EKFGSF_ahrs_turn_comp_enabled && EKFGSF_ahrs_accel_norm <= 2.0f * GRAVITY_MSS) {
			EKFGSF_accel_gain = frontend->EKFGSF_tiltGain * sq(1.0f - (EKFGSF_ahrs_accel_norm - GRAVITY_MSS)/GRAVITY_MSS);
		} else if (EKFGSF_ahrs_accel_norm <= 1.5f * GRAVITY_MSS) {
			EKFGSF_accel_gain = frontend->EKFGSF_tiltGain * sq(1.0f - 2.0f * (EKFGSF_ahrs_accel_norm - GRAVITY_MSS)/GRAVITY_MSS);
		}
	} else if (EKFGSF_ahrs_accel_norm > 0.5f * GRAVITY_MSS) {
		EKFGSF_accel_gain = frontend->EKFGSF_tiltGain * sq(1.0f + 2.0f * (EKFGSF_ahrs_accel_norm - GRAVITY_MSS)/GRAVITY_MSS);
	}

	// Always run the AHRS prediction cycle for each model
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
		EKFGSF_predict(mdl_idx);
	}

	// The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
	if (filterStatus.flags.horiz_pos_abs && gpsDataToFuse && inFlight) {
		if (!EKFGSF_vel_fuse_started) {
			// Perform in-flight alignment
			for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
				// Use the firstGPS  measurement to set the velocities and corresponding variances
				EKFGSF_mdl[mdl_idx].X[0] = gpsDataDelayed.vel[0];
				EKFGSF_mdl[mdl_idx].X[1] = gpsDataDelayed.vel[1];
				EKFGSF_mdl[mdl_idx].P[0][0] = sq(fmaxf(gpsSpdAccuracy, frontend->_gpsHorizVelNoise));
				EKFGSF_mdl[mdl_idx].P[1][1] = EKFGSF_mdl[mdl_idx].P[0][0];
			}
			EKFGSF_alignQuatYaw();
			EKFGSF_vel_fuse_started = true;
		} else {
			float total_w = 0.0f;
			float newWeight[N_MODELS_EKFGSF];
			for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
				// Update states and covariances using GPS NE velocity measurements fused as direct state observations
				EKFGSF_correct(mdl_idx);

				// Calculate weighting for each model assuming a normal distribution
				newWeight[mdl_idx]= fmaxf(EKFGSF_gaussianDensity(mdl_idx) * EKFGSF_GSF.weights[mdl_idx], 0.0f);
				total_w += newWeight[mdl_idx];
			}

			// Normalise the sume of weights to unity
			if (EKFGSF_vel_fuse_started && total_w > 0.0f) {
				float total_w_inv = 1.0f / total_w;
				for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
					EKFGSF_GSF.weights[mdl_idx]  = newWeight[mdl_idx] * total_w_inv;
				}
			}
		}
	} else if (EKFGSF_vel_fuse_started && !inFlight) {
		// We have landed so reset EKF-GSF states and wait to fly again
		// Do not reset the AHRS quaternions as the AHRS continues to run when on ground
		EKFGSF_initialise();
		EKFGSF_vel_fuse_started = false;
	}

	// Calculate a composite state vector as a weighted average of the states for each model.
	// To avoid issues with angle wrapping, the yaw state is converted to a vector with legnth
	// equal to the weighting value before it is summed.
	memset(&EKFGSF_GSF.state, 0, sizeof(EKFGSF_GSF.state));
	Vector2f yaw_vector = {};
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
		for (uint8_t state_index = 0; state_index < 2; state_index++) {
			EKFGSF_GSF.state[state_index] += EKFGSF_mdl[mdl_idx].X[state_index] * EKFGSF_GSF.weights[mdl_idx];
		}
		yaw_vector[0] += EKFGSF_GSF.weights[mdl_idx] * cosf(EKFGSF_mdl[mdl_idx].X[2]);
		yaw_vector[1] += EKFGSF_GSF.weights[mdl_idx] * sinf(EKFGSF_mdl[mdl_idx].X[2]);
	}
	EKFGSF_GSF.state[2] = atan2f(yaw_vector[1],yaw_vector[0]);

	/*
	// calculate a composite covariance matrix from a weighted average of the covariance for each model
	// models with larger innovations are weighted less
	memset(&P_GSF, 0, sizeof(P_GSF));
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
		float Xdelta[3];
		for (uint8_t row = 0; row < 3; row++) {
			Xdelta[row] = EKFGSF_mdl[mdl_idx].X[row] - X_GSF[row];
		}
		for (uint8_t row = 0; row < 3; row++) {
			for (uint8_t col = 0; col < 3; col++) {
				P_GSF[row][col] +=  EKFGSF_GSF.weights[mdl_idx] * (EKFGSF_mdl[mdl_idx].P[row][col] + Xdelta[row] * Xdelta[col]);
			}
		}
	}
	*/
    EKFGSF_GSF.yaw_variance = 0.0f;
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx ++) {
		float yawDelta = wrap_PI(EKFGSF_mdl[mdl_idx].X[2] - EKFGSF_GSF.state[2]);
		EKFGSF_GSF.yaw_variance +=  EKFGSF_GSF.weights[mdl_idx] * (EKFGSF_mdl[mdl_idx].P[2][2] + sq(yawDelta));
	}

}

// returns the probability of a selected model output assuming a gaussian error distribution
float NavEKF3_core::EKFGSF_gaussianDensity(const uint8_t mdl_idx) const
{
	const float t2 = EKFGSF_mdl[mdl_idx].S[0][0] * EKFGSF_mdl[mdl_idx].S[1][1];
	const float t5 = EKFGSF_mdl[mdl_idx].S[0][1] * EKFGSF_mdl[mdl_idx].S[1][0];
	const float t3 = t2 - t5; // determinant
	float t4; // determinant inverse
	if (fabsf(t3) > 1e-6f) {
		t4 = 1.0f/t3;
	} else {
		t4 = 1.0f/t2;
	}

	// inv(S)
	float invMat[2][2];
	invMat[0][0] =   t4 * EKFGSF_mdl[mdl_idx].S[1][1];
	invMat[1][1] =   t4 * EKFGSF_mdl[mdl_idx].S[0][0];
	invMat[0][1] = - t4 * EKFGSF_mdl[mdl_idx].S[0][1];
	invMat[1][0] = - t4 * EKFGSF_mdl[mdl_idx].S[1][0];

 	// inv(S) * innovation
	float tempVec[2];
	tempVec[0] = invMat[0][0] * EKFGSF_mdl[mdl_idx].innov[0] + invMat[0][1] * EKFGSF_mdl[mdl_idx].innov[1];
	tempVec[1] = invMat[1][0] * EKFGSF_mdl[mdl_idx].innov[0] + invMat[1][1] * EKFGSF_mdl[mdl_idx].innov[1];

	// transpose(innovation) * inv(S) * innovation
	float normDist = tempVec[0] * EKFGSF_mdl[mdl_idx].innov[0] + tempVec[1] * EKFGSF_mdl[mdl_idx].innov[1];

    // convert from a normalised variance to a probability assuming a Gaussian distribution
	normDist = expf(-0.5f * normDist);
	normDist *= sqrtf(t4)/ M_2PI;
	return normDist;
}

void NavEKF3_core::getDataEKFGSF(float *yaw_composite, float *yaw_composite_variance, float yaw[N_MODELS_EKFGSF], float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	memcpy(yaw_composite, &EKFGSF_GSF.state[2], sizeof(EKFGSF_GSF.state[2]));
	memcpy(yaw_composite_variance, &EKFGSF_GSF.yaw_variance, sizeof(EKFGSF_GSF.yaw_variance));
	for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
		yaw[mdl_idx] = EKFGSF_mdl[mdl_idx].X[2];
		innov_VN[mdl_idx] = EKFGSF_mdl[mdl_idx].innov[0];
		innov_VE[mdl_idx] = EKFGSF_mdl[mdl_idx].innov[1];
		weight[mdl_idx] = EKFGSF_GSF.weights[mdl_idx];
	}
}

void NavEKF3_core::EKFGSF_forceSymmetry(const uint8_t mdl_idx)
{
	float P01 = 0.5f * (EKFGSF_mdl[mdl_idx].P[0][1] + EKFGSF_mdl[mdl_idx].P[1][0]);
	float P02 = 0.5f * (EKFGSF_mdl[mdl_idx].P[0][2] + EKFGSF_mdl[mdl_idx].P[2][0]);
	float P12 = 0.5f * (EKFGSF_mdl[mdl_idx].P[1][2] + EKFGSF_mdl[mdl_idx].P[2][1]);
	EKFGSF_mdl[mdl_idx].P[0][1] = EKFGSF_mdl[mdl_idx].P[1][0] = P01;
	EKFGSF_mdl[mdl_idx].P[0][2] = EKFGSF_mdl[mdl_idx].P[2][0] = P02;
	EKFGSF_mdl[mdl_idx].P[1][2] = EKFGSF_mdl[mdl_idx].P[2][1] = P12;
}

// Reset quaternion states using yaw from EKF-GSF
bool NavEKF3_core::EKFGSF_resetMainFilterYaw()
{
    if (EKFGSF_yaw_reset_count < frontend->EKFGSF_n_reset_max &&
        EKFGSF_vel_fuse_started &&
        EKFGSF_GSF.yaw_variance < sq(radians(15.0f)) &&
        (imuSampleTime_ms - EKFGSF_yaw_reset_time_ms) > 5000) {

        // save a copy of the quaternion state for later use in calculating the amount of reset change
        Quaternion quat_before_reset = stateStruct.quat;

        // calculate the variance for the rotation estimate expressed as a rotation vector
        // this will be used later to reset the quaternion state covariances
        Vector3f angleErrVarVec = calcRotVecVariances();

        // update transformation matrix from body to world frame using the current estimate
        stateStruct.quat.inverse().rotation_matrix(prevTnb);

        // calculate the initial quaternion
        // determine if a 321 or 312 Euler sequence is best
        float deltaYaw;
        if (fabsf(prevTnb[2][0]) < fabsf(prevTnb[2][1])) {
            // using a 321 Tait-Bryan rotation to define yaw state
            // take roll pitch yaw from AHRS prediction
            float roll,pitch,yaw;
            prevTnb.to_euler(&roll, &pitch, &yaw);

            // record change in yaw
            deltaYaw = wrap_PI(EKFGSF_GSF.state[2] - yaw);

            // Calculate the body to earth frame rotation matrix
            prevTnb.from_euler(roll, pitch, EKFGSF_GSF.state[2]);

        } else {
            // Calculate the 312 Tait-Bryan rotation sequence that rotates from earth to body frame
            // We use a 312 sequence as an alternate when there is more pitch tilt than roll tilt
            // to avoid gimbal lock
            Vector3f euler312 = prevTnb.to_euler312();

            // record change in yaw
            deltaYaw = wrap_PI(EKFGSF_GSF.state[2] - euler312[2]);

            // Calculate the body to earth frame rotation matrix
            prevTnb.from_euler312(euler312[0], euler312[1], EKFGSF_GSF.state[2]);

        }

        // quaternion states for the main EKF with yaw reset applied
        Quaternion quat_after_reset;
        quat_after_reset.from_rotation_matrix(prevTnb);

        // update quaternion states
        stateStruct.quat = quat_after_reset;

        // calculate the change in the quaternion state and apply it to the output history buffer
        Quaternion quat_delta = stateStruct.quat / quat_before_reset;
        StoreQuatRotate(quat_delta);

        // update the yaw angle variance using the variance of the EKF-GSF estimate
        angleErrVarVec.z = EKFGSF_GSF.yaw_variance;

        // reset the quaternion covariances using the rotation vector variances
        initialiseQuatCovariances(angleErrVarVec);

        // record the yaw reset event
        yawResetAngle += deltaYaw;
        lastYawReset_ms = imuSampleTime_ms;
        EKFGSF_yaw_reset_time_ms = imuSampleTime_ms;
        EKFGSF_yaw_reset_count++;

        gcs().send_text(MAV_SEVERITY_WARNING, "EKF3 IMU%u emergency yaw reset - mag sensor stopped",(unsigned)imu_index);

        // Fail the magnetomer so it doesn't get used and pull the yaw away from the correct value
        allMagSensorsFailed = true;

        // reset velocity and position states to GPS - if yaw is fixed then the filter should start to operate correctly
        ResetVelocity();
        ResetPosition();

		// reset test ratios that are reported to prevent a race condition with the external state machine requesting the reset
		velTestRatio = 0.0f;
		posTestRatio = 0.0f;

        return true;

    }
    
    return false;

}