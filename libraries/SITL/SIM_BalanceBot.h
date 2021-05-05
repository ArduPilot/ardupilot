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
  BalanceBot simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

class BalanceBot : public Aircraft {
public:
    BalanceBot(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new BalanceBot(frame_str);
    }

private:
    // vehicle frame x velocity
    float velocity_vf_x;

    float skid_turn_rate;

    float calc_yaw_rate(float steering) const;
};

} // namespace SITL
