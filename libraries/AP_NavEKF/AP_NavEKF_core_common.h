/*
  NavEKF_core_common holds scratch data shared by EKF2 and EKF3

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

#include <stdint.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include "AP_Nav_Common.h"

/*
  this declares a common parent class for AP_NavEKF2 and
  AP_NavEKF3. The purpose of this class is to hold common static
  scratch space variables. These variables do not hold anything that
  matters between iterations, they are only intermediate variables. By
  placing these in a common parent class we save a lot of memory, but
  we also save a lot of CPU (approx 10% on STM32F427) as the compiler
  is able to resolve the address of these variables at compile time,
  which means significantly faster code
 */
class NavEKF_core_common {
public:
#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,28> Vector28;
    typedef VectorN<VectorN<ftype,24>,24> Matrix24;
#else
    typedef ftype Vector28[28];
    typedef ftype Matrix24[24][24];
#endif

protected:
    static Matrix24 KH;                   // intermediate result used for covariance updates
    static Matrix24 KHP;                  // intermediate result used for covariance updates
    static Matrix24 nextP;                // Predicted covariance matrix before addition of process noise to diagonals
    static Vector28 Kfusion;              // intermediate fusion vector

    // fill all the common scratch variables with NaN on SITL
    void fill_scratch_variables(void);

    // zero part of an array for index range [n1,n2]
    static void zero_range(ftype *v, uint8_t n1, uint8_t n2) {
        memset(&v[n1], 0, sizeof(ftype)*(1+(n2-n1)));
    }
};

#if HAL_WITH_EKF_DOUBLE
// stack frames are larger with double EKF
#if MATH_CHECK_INDEXES
#pragma GCC diagnostic error "-Wframe-larger-than=4000"
#else
#pragma GCC diagnostic error "-Wframe-larger-than=2500"
#endif
#endif

