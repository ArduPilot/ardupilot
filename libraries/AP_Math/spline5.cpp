/*
 * spline5.cpp
 *
 * Created by William Geyer and Chris Olson modified for ardupilot
 * Original work by Ryan Muller
 * https://gist.github.com/ryanthejuggler/4132103
 * released under the Creative Commons CC0 License
 * http://creativecommons.org/publicdomain/zero/1.0/
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

#include <stdint.h>
#include "spline5.h"

void splinterp5(const float x[5], float out[4][4])
{

    // number of spline points
    const uint8_t n = 5;

    // working variables
    float u[n] {};

    // second derivative
    // additional element in array necessary for back substitution loop.
    float z[n+1] {};

    // set the second derivative to 0 at the ends
    z[0] = u[0] = 0;
    z[n-1] = 0;

    // decomposition loop
    for (uint8_t i=1; i<n-1; i++) {
        float p = 0.5f * z[i-1] + 2.0f;
        // keep p from ever becoming zero
        if (p < 0.01f && p >= 0.0f) {
            p = 0.01f;
        } else if (p > -0.01f && p < 0.0f) {
            p = -0.01f;
        }
        const float p_inv = 1.0f / p;
        z[i] = -0.5f * p_inv;
        u[i] = x[i+1] + x[i-1] - 2.0f * x[i];
        u[i] = (3.0f * u[i] - 0.5f * u[i-1]) * p_inv;
    }

    // back-substitution loop
    for (uint8_t i=n-1; i>0; i--) {
        z[i] = z[i] * z[i+1] + u[i];
    }

    for (uint8_t i=0; i<n-1; i++) {
        out[i][0] = x[i+1];
        out[i][1] = x[i];
        out[i][2] = z[i+1];
        out[i][3] = z[i];
    }

}
