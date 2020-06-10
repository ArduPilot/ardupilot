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

#include <AP_HAL/AP_HAL.h>
#include "AP_ADSB_Sagetech.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_ADSB_Sagetech::AP_ADSB_Sagetech(AP_ADSB &adsb) :
        AP_ADSB_Backend(adsb)
{
}



void AP_ADSB_Sagetech::update()
{

}

void AP_ADSB_Sagetech::send_configure(const mavlink_channel_t chan)
{

}


void AP_ADSB_Sagetech::send_dynamic_out(const mavlink_channel_t chan)
{


}

