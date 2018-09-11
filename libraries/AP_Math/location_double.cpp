/*
 * location_double.cpp
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  this is for double precision functions related to the location structure
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_HAL/AP_HAL.h>
#include <cstdlib>
#include "AP_Math.h"
#include "location.h"

/*
  these are not currently used. They should be moved to location_double.cpp if we do enable them in the future
 */
void wgsllh2ecef(const Vector3d &llh, Vector3d &ecef) {
  double d = WGS84_E * sin(llh[0]);
  double N = WGS84_A / sqrt(1 - d*d);

  ecef[0] = (N + llh[2]) * cos(llh[0]) * cos(llh[1]);
  ecef[1] = (N + llh[2]) * cos(llh[0]) * sin(llh[1]);
  ecef[2] = ((1 - WGS84_E*WGS84_E)*N + llh[2]) * sin(llh[0]);
}


void wgsecef2llh(const Vector3d &ecef, Vector3d &llh) {
  /* Distance from polar axis. */
  const double p = sqrt(ecef[0]*ecef[0] + ecef[1]*ecef[1]);

  /* Compute longitude first, this can be done exactly. */
  if (!is_zero(p))
    llh[1] = atan2(ecef[1], ecef[0]);
  else
    llh[1] = 0;

  /* If we are close to the pole then convergence is very slow, treat this is a
   * special case. */
  if (p < WGS84_A * double(1e-16)) {
    llh[0] = copysign(M_PI_2, ecef[2]);
    llh[2] = fabs(ecef[2]) - WGS84_B;
    return;
  }

  /* Calculate some other constants as defined in the Fukushima paper. */
  const double P = p / WGS84_A;
  const double e_c = sqrt(1 - WGS84_E*WGS84_E);
  const double Z = fabs(ecef[2]) * e_c / WGS84_A;

  /* Initial values for S and C correspond to a zero height solution. */
  double S = Z;
  double C = e_c * P;

  /* Neither S nor C can be negative on the first iteration so
   * starting prev = -1 will not cause and early exit. */
  double prev_C = -1;
  double prev_S = -1;

  double A_n, B_n, D_n, F_n;

  /* Iterate a maximum of 10 times. This should be way more than enough for all
   * sane inputs */
  for (int i=0; i<10; i++)
  {
    /* Calculate some intermmediate variables used in the update step based on
     * the current state. */
    A_n = sqrt(S*S + C*C);
    D_n = Z*A_n*A_n*A_n + WGS84_E*WGS84_E*S*S*S;
    F_n = P*A_n*A_n*A_n - WGS84_E*WGS84_E*C*C*C;
    B_n = double(1.5) * WGS84_E*S*C*C*(A_n*(P*S - Z*C) - WGS84_E*S*C);

    /* Update step. */
    S = D_n*F_n - B_n*S;
    C = F_n*F_n - B_n*C;

    /* The original algorithm as presented in the paper by Fukushima has a
     * problem with numerical stability. S and C can grow very large or small
     * and over or underflow a double. In the paper this is acknowledged and
     * the proposed resolution is to non-dimensionalise the equations for S and
     * C. However, this does not completely solve the problem. The author caps
     * the solution to only a couple of iterations and in this period over or
     * underflow is unlikely but as we require a bit more precision and hence
     * more iterations so this is still a concern for us.
     *
     * As the only thing that is important is the ratio T = S/C, my solution is
     * to divide both S and C by either S or C. The scaling is chosen such that
     * one of S or C is scaled to unity whilst the other is scaled to a value
     * less than one. By dividing by the larger of S or C we ensure that we do
     * not divide by zero as only one of S or C should ever be zero.
     *
     * This incurs an extra division each iteration which the author was
     * explicityl trying to avoid and it may be that this solution is just
     * reverting back to the method of iterating on T directly, perhaps this
     * bears more thought?
     */

    if (S > C) {
      C = C / S;
      S = 1;
    } else {
      S = S / C;
      C = 1;
    }

    /* Check for convergence and exit early if we have converged. */
    if (fabs(S - prev_S) < double(1e-16) && fabs(C - prev_C) < double(1e-16)) {
      break;
    } else {
      prev_S = S;
      prev_C = C;
    }
  }

  A_n = sqrt(S*S + C*C);
  llh[0] = copysign(1.0, ecef[2]) * atan(S / (e_c*C));
  llh[2] = (p*e_c*C + fabs(ecef[2])*S - WGS84_A*e_c*A_n) / sqrt(e_c*e_c*C*C + S*S);
}
