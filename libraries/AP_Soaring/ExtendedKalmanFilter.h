/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <AP_Math/matrixN.h>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(void) {}

    static constexpr const uint8_t N = 4;
    static constexpr const uint8_t M = 4;

    VectorN<float,N> X;
    MatrixN<float,N> P;
    MatrixN<float,N> Q;
    float R;
    void reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r);
    void update(float z, const VectorN<float,M> &U);

private:
    float measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U);
};
