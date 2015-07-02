// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "AC_BoosterSingle.h"

extern const AP_HAL::HAL& hal;

AC_BoosterSingle::AC_BoosterSingle ()
{
  
};

bool AC_BoosterSingle::is_enabled ()
{
    // check if we have configured an aux channel for the booster
    return RC_Channel_aux::function_assigned(RC_Channel_aux::k_booster_single);
}

void AC_BoosterSingle::set_boost (int16_t boost_in)
{    
    // exit immediately if the booster function has not been set-up for any servo
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_booster_single)) {
        return;
    }

    // set the boost on the aux booster channel
    int16_t boost_out = constrain_int16(boost_in, 0, 1000);
    
    RC_Channel_aux::move_servo(RC_Channel_aux::k_booster_single, boost_out, 0, 1000);
}