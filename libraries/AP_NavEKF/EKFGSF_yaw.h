#pragma once

#pragma GCC optimize("O2")

#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>

#define IMU_DT_MIN_SEC 0.001f // Minimum delta time between IMU samples (sec)

class EKFGSF_yaw
{
public:
    // Constructor
    EKFGSF_yaw();

    // Update Filter States - this should be called whenever new IMU data is available
    void update(const Vector3f &delAng,// IMU delta angle rotation vector meassured in body frame (rad)
                const Vector3f &delVel,// IMU delta velocity vector meassured in body frame (m/s)
                const float delAngDT, // time interval that delAng was integrated over (sec) - must be no less than IMU_DT_MIN_SEC
                const float delVelDT, // time interval that delVel was integrated over (sec) - must be no less than IMU_DT_MIN_SEC
                bool runEKF,          // set to true when flying or movement suitable for yaw estimation
                float TAS);           // true airspeed used for centripetal accel compensation - set to 0 when not required.

    // Fuse NE velocty mesurements and update the EKF's and GSF state and covariance estimates
    // Should be called after update(...) whenever new velocity data is available
    void fuseVelData(const Vector2f &vel,    // NE velocity measurement (m/s)
                     const float velAcc);   // 1-sigma accuracy of velocity measurement (m/s)

    // set the gyro bias in rad/sec
    void setGyroBias(Vector3f &gyroBias);

    // get solution data for logging
    // return false if yaw estimation is inactive
    bool getLogData(float &yaw_composite, float &yaw_composite_variance, float yaw[N_MODELS_EKFGSF], float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]);

    // get yaw estimated and corresponding variance
    // return false if yaw estimation is inactive
    bool getYawData(float &yaw, float &yawVariance);

private:

    typedef float ftype;
#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
#else
    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Matrix3[3][3];

#endif

    // Parameters
    const float EKFGSF_gyroNoise{1.0e-1f};  // yaw rate noise used for covariance prediction (rad/sec)
    const float EKFGSF_accelNoise{2.0f};    // horizontal accel noise used for covariance prediction (m/sec**2)
    const float EKFGSF_tiltGain{0.2f};      // gain from tilt error to gyro correction for complementary filter (1/sec)
    const float EKFGSF_gyroBiasGain{0.04f}; // gain applied to integral of gyro correction for complementary filter (1/sec)
    const float EKFGSF_accelFiltRatio{10.0f}; // ratio  of time constant of AHRS tilt correction to time constant of first order LPF applied to accel data used by ahrs

    // Declarations used by the bank of AHRS complementary filters that use IMU data augmented by true
    // airspeed data when in fixed wing mode to estimate the quaternions that are used to rotate IMU data into a
    // Front, Right, Yaw frame of reference.
    Vector3f delta_angle;
    Vector3f delta_velocity;
    float angle_dt;
    float velocity_dt;
    struct ahrs_struct {
        Matrix3f R;             // matrix that rotates a vector from body to earth frame
        Vector3f gyro_bias;     // gyro bias learned and used by the quaternion calculation
        bool aligned{false};    // true when AHRS has been aligned
        float accel_FR[2];      // front-right acceleration vector in a horizontal plane (m/s/s)
        float vel_NE[2];        // NE velocity vector from last GPS measurement (m/s)
        bool fuse_gps;          // true when GPS should be fused on that frame
        float accel_dt;         // time step used when generating _simple_accel_FR data (sec)
    };
    ahrs_struct AHRS[N_MODELS_EKFGSF];
    bool ahrs_tilt_aligned;         // true the initial tilt alignment has been calculated
    float accel_gain;               // gain from accel vector tilt error to rate gyro correction used by AHRS calculation
    Vector3f ahrs_accel;            // filtered body frame specific force vector used by AHRS calculation (m/s/s)
    float ahrs_accel_norm;          // length of body frame specific force vector used by AHRS calculation (m/s/s)
    bool ahrs_turn_comp_enabled;    // true when compensation for centripetal acceleration in coordinated turns using true airspeed is being used.
    float true_airspeed;            // true airspeed used to correct for centripetal acceleratoin in coordinated turns (m/s)

    // Runs quaternion prediction for the selected AHRS using IMU (and optionally true airspeed) data
    void predictAHRS(const uint8_t mdl_idx);

    // Applies a body frame delta angle to a body to earth frame rotation matrix using a small angle approximation
    Matrix3f updateRotMat(const Matrix3f &R, const Vector3f &g);

    // Initialises the tilt (roll and pitch) for all AHRS using IMU acceleration data
    void alignTilt();

    // Initialises the yaw angle for all AHRS using a uniform distribution of yaw angles between -180 and +180 deg
    void alignYaw();

    // The Following declarations are used by bank of EKF's that estimate yaw angle starting from a different yaw hypothesis for each filter.

    struct EKF_struct {
        float X[3];     // Vel North (m/s),  Vel East (m/s), yaw (rad)
        float P[3][3];  // covariance matrix
        float S[2][2];  // N,E velocity innovation variance (m/s)^2
        float innov[2]; // Velocity N,E innovation (m/s)
    };
    EKF_struct EKF[N_MODELS_EKFGSF];
    bool vel_fuse_running;  // true when the bank of EKF's has started fusing GPS velocity data
    bool run_ekf_gsf;       // true when operating condition is suitable for to run the GSF and EKF models and fuse velocity data

    // Resets states and covariances for the EKF's and GSF including GSF weights, but not the AHRS complementary filters
    void resetEKFGSF();

    // Runs the state and covariance prediction for the selected EKF
    void predict(const uint8_t mdl_idx);

    // Runs the state and covariance update for the selected EKF using the GPS NE velocity measurement
    // Returns false if the sttae and covariance correction failed
    bool correct(const uint8_t mdl_idx, const Vector2f &vel, const float velObsVar);

    // Forces symmetry on the covariance matrix for the selected EKF
    void forceSymmetry(const uint8_t mdl_idx);

    // The following declarations are used  by the Gaussian Sum Filter that combines the state estimates from the bank of
    // EKF's to form a single state estimate.

    struct GSF_struct {
        float yaw;                      // yaw (rad)
        float yaw_variance;             // Yaw state variance (rad^2)
        float weights[N_MODELS_EKFGSF]; // Weighting applied to each EKF model. Sum of weights is unity.
    };
    GSF_struct GSF;

    // Returns the probability for a selected model assuming a Gaussian error distribution
    // Used by the Guassian Sum Filter to calculate the weightings when combining the outputs from the bank of EKF's
    float gaussianDensity(const uint8_t mdl_idx) const;
};
