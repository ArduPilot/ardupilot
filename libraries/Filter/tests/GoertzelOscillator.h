#pragma once

#include <cmath>
#include <AP_Math/definitions.h>

/*
  Goertzel_Oscillator generates a sine wave sample-by-sample using the
  "inverse Goertzel" / coupled-form digital resonator trick: once the
  recurrence coefficient c = 2*cos(omega) is computed at init() time,
  each subsequent sample is produced with one multiply and one
  subtract instead of a sin() call:

      y[n+1] = c*y[n] - y[n-1]

  which reproduces y[n] = sin(omega*n + phase) for all n, with sin()
  called only twice (to seed y[-2] and y[-1]) regardless of how many
  samples are generated afterwards.

  Floating-point rounding leaves a tiny error in c, equivalent to
  generating a frequency infinitesimally off from the requested one.
 */
class Goertzel_Oscillator
{
public:
    void init(double freq_hz, double sample_rate_hz, double phase_rad = 0.0)
    {
        const double omega = M_2PI * freq_hz / sample_rate_hz;
        c = 2.0 * cos(omega);

        // Seed one step further back so the first next() call works as expected.
        y1 = sin(phase_rad - 2.0*omega);
        y0 = sin(phase_rad - omega);
    }

    // returns sin(omega*n + phase_rad) for n=0,1,2,... advancing by
    // one sample on every call
    double next()
    {
        const double y2 = c * y0 - y1;
        y1 = y0;
        y0 = y2;
        return y0;
    }

private:
    double c = 0;
    double y0 = 0;
    double y1 = 0;
};
