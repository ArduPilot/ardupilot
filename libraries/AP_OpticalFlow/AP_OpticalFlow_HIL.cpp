/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 * AP_OpticalFlow_HIL.cpp - HIL emulation of optical flow sensor.
 * This is a dummy class, with the work done in setHIL()
 */

#include <AP_HAL.h>
#include "OpticalFlow.h"

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_HIL::AP_OpticalFlow_HIL(OpticalFlow &_frontend) : 
    OpticalFlow_backend(_frontend) 
{}

void AP_OpticalFlow_HIL::init(void)
{
}

void AP_OpticalFlow_HIL::update(void)
{
}
