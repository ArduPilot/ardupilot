/* 
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other 
* purposes by adapting the equations for the jacobians.
*/

#ifndef ExtendedKalmanFilter_h
#define ExtendedKalmanFilter_h

#include "AP_Math/matrixN.h"

// Kalman filter dimensionality
#define N 4

class ExtendedKalmanFilter
{   
    float measurementpredandjacobian(float* A);
    
public:
    float X[N];
    float P[N][N];
    float Q[N][N];
    float R[1][1];
    void reset(float x[N], float p[N][N], float q[N][N], float r[1][1]);
    void update(float z, float Vx, float Vy);
};


#endif