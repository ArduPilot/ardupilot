/*
   2-state EKF for estimating drag polar parameters with single measurement and
   no state dynamics. By Samuel Tabor 2020.

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

#pragma once

#include <AP_Math/matrixN.h>

class EKF_Polar {
public:
    EKF_Polar(void) {}

    static constexpr const uint8_t N = 2;
    static constexpr const uint8_t M = 3;

    VectorN<float,N> X;
    MatrixN<float,N> P;
    MatrixN<float,N> Q;
    float R;
    void reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r);
    void update(float z, const VectorN<float,M> &U);

private:
    float measurementpredandjacobian(VectorN<float,N> &A, const VectorN<float,M> &U);

    void state_update(const VectorN<float,M> &U);
};
