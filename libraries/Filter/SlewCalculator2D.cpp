/*
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
/*
  The SlewCalculator2D filter calculates a slew rate useful for detecting
  oscillations in a 2-axis PID controller.
 */
#include "SlewCalculator2D.h"

SlewCalculator2D::SlewCalculator2D() :
    xlimiter(slew_rate_max, slew_rate_tau),
    ylimiter(slew_rate_max, slew_rate_tau)
{
}

// apply filter to sample and calculate slewrate
void SlewCalculator2D::update(const Vector2f& sample, float dt)
{
    if (!is_positive(dt)) {
        return;
    }

    // call x and y slew rate limiter
    xlimiter.modifier(sample.x, dt);
    ylimiter.modifier(sample.y, dt);
}

// get last oscillation slew rate
float SlewCalculator2D::get_slew_rate() const
{
    return safe_sqrt(sq(xlimiter.get_slew_rate()) + sq(xlimiter.get_slew_rate()));
}
