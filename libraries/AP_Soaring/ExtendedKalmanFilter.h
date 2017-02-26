/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <AP_Math/matrixN.h>

#define N 4

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(void) {}
    
    VectorN<float,N> X;
    MatrixN<N> P;
    MatrixN<N> Q;
    float R;
    void reset(const VectorN<float,N> &x, const MatrixN<N> &p, const MatrixN<N> q, float r);
    void update(float z, float Vx, float Vy);

private:
    float measurementpredandjacobian(VectorN<float,N> &A);
};
