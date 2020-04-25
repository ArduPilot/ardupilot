#include "EKF_Polar.h"
#include "AP_Math/matrixN.h"


float EKF_Polar::measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U)
{
    // This function calculates the Jacobian using equations of theoretical sinkrate as
    // a function of the glide polar parameters.
    // Vz = CD0 * Vx^3 / K + B*K/Vx, where K = (2*m*g)/(1.225*S)

    float V = U[0];
    // float roll = U[1];
    float k = U[2];

    A[0] = powf(V,3) / k;
    A[1] = k / V;

    // Expected measurement
    float w = X[0] * A[0] + X[1] * A[1];
    
    return w;
}


void EKF_Polar::reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r)
{
    P = p;
    X = x;
    Q = q;
    R = r;
}


void EKF_Polar::update(float z, const VectorN<float,M> &U)
{
    MatrixN<float,N> tempM;
    VectorN<float,N> H;
    VectorN<float,N> P12;
    VectorN<float,N> K;
    
    // U: [driftX,driftY,Px,Py]

    // LINE 28
    state_update(U);

    // LINE 33
    // Update the covariance matrix
    // P = A*ekf.P*A'+ekf.Q;
    // We know A is identity so
    // P = ekf.P+ekf.Q;
    P += Q;

    // What measurement do we expect to receive in the estimated
    // state
    // LINE 37
    // [z1,H] = ekf.jacobian_h(x1);
    float z1 = measurementpredandjacobian(H, U);

    // LINE 40
    // P12 = P * H';
    P12.mult(P, H); //cross covariance 
    
    // LINE 41
    // Calculate the KALMAN GAIN
    // K = P12 * inv(H*P12 + ekf.R);                     %Kalman filter gain
    K = P12 * 1.0 / (H * P12 + R);

    // Correct the state estimate using the measurement residual.
    // LINE 44
    // X = x1 + K * (z - z1);
    X += K * (z - z1);

    // Make sure values stay sensible.
    X[0] = X[0]>0.02 ? X[0]: 0.02;
    X[1] = X[1]>0.02 ? X[1]: 0.02;
    X[0] = X[0]<0.10 ? X[0]: 0.10;
    X[1] = X[1]<0.10 ? X[1]: 0.10;


    // Correct the covariance too.
    // LINE 46
    // NB should be altered to reflect Stengel
    // P = P_predict - K * P12';
    tempM.mult(K, P12);
    P -= tempM;
    
    P.force_symmetry();
}

void EKF_Polar::state_update(const VectorN<float,M> &U)
{
    // Estimate new state from old.
}