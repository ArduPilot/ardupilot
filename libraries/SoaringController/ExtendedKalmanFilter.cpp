#include "ExtendedKalmanFilter.h"

float ExtendedKalmanFilter::measurementpredandjacobian(float* A){
    // This function computes the Jacobian using equations from
    // analytical derivation of Gaussian updraft distribution
    // This expression gets used lots
    float expon = exp(- (pow(X[2], 2) + pow(X[3], 2)) / pow(X[1], 2));
    // Expected measurement
    float w = X[0] * expon;
    
    // Elements of the Jacobian
    A[0] = expon;
    A[1] = 2 * X[0] * ((pow(X[2],2) + pow(X[3],2)) / pow(X[1],3)) * expon;
    A[2] = -2 * (X[0] * X[2] / pow(X[1],2)) * expon;
    A[3] = A[2] * X[3] / X[2];
    return w;
}


void ExtendedKalmanFilter::reset(float x[N], float p[N][N],float q[N][N], float r[1][1]){
    MatrixMath::MatrixCopy((float*)p, N, N, (float*)P);
    MatrixMath::MatrixCopy((float*)x, N, 1, (float*)X);
    MatrixMath::MatrixCopy((float*)q, N, N, (float*)Q);
    MatrixMath::MatrixCopy((float*)r, 1, 1, (float*)R);
}


void ExtendedKalmanFilter::update(float z,float Vx, float Vy)
{   
    float temp1[N][N] ;
    float H[1][N];
    float P12[N][1];
    float K[N][1];
    // LINE 28  
    // Estimate new state from old. 
    X[2] -= Vx;
    X[3] -= Vy;
    
    // LINE 33
    // Update the covariance matrix 
    // P = A*ekf.P*A'+ekf.Q;              
    // We know A is identity so
    // P = ekf.P+ekf.Q;
    MatrixMath::MatrixAdd((float*)P, (float*)Q, N, N, (float*)P);
    
    // What measurement do we expect to receive in the estimated
    // state
    // LINE 37
    // [z1,H] = ekf.jacobian_h(x1);
    float z1 = measurementpredandjacobian((float*)H);    
    
    // LINE 40
    // P12 = P * H';
    MatrixMath::MatrixMultTranspose((float*)P, (float*)H, N, N, 1, (float*)P12);      //cross covariance
    
    // LINE 41
    // Calculate the KALMAN GAIN
    // K = P12 * inv(H*P12 + ekf.R);                     %Kalman filter gain
    MatrixMath::MatrixMult((float*)H, (float*)P12, 1, N, 1, (float*)temp1);
    float temp = 1.0 / (temp1[0][0] + R[0][0]);
    MatrixMath::MatrixMultScalar((float*)P12, temp, N, 1, (float*)K);
    
    // Correct the state estimate using the measurement residual.
    // LINE 44
    // X = x1 + K * (z - z1);
    float residual = z - z1;
    MatrixMath::MatrixMultScalar((float*)K, residual, N, 1, (float*)temp1);
    MatrixMath::MatrixAdd((float*)temp1, (float*)X, N, 1, (float*)X);
    
    // Correct the covariance too.
    // LINE 46
    // NB should be altered to reflect Stengel
    // P = P_predict - K * P12'; 
    MatrixMath::MatrixMultTranspose((float*)K, (float*)P12, N, 1, N, (float*)temp1);
    MatrixMath::MatrixSubtract((float*)P,(float*)temp1, N, N, (float*)P);
    MatrixMath::MatrixForceSymmetry((float*)P, N);
}