/* 
Extended Kalman Filter class by Sam Tabor, 2013.
http://diydrones.com/forum/topics/autonomous-soaring
Set up for identifying thermals of Gaussian form, but could be adapted to other 
purposes by adapting the equations for the jacobians.
*/
#ifndef ExtendedKalmanFilter_h
#define ExtendedKalmanFilter_h

#include "MatrixMath.h"

#define N (4)

class ExtendedKalmanFilter
{   
    float measurementpredandjacobian (float* A);
    
public:
    float X[4];
    float P[4][4];
    float Q[4][4];
    float R[1][1];
    void reset(float x[N], float p[N][N], float q[4][4], float r[1][1]);
    void update(float z,float Vx, float Vy);
};


#endif