/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * adsb.cpp
 * Copyright (C) Tom Pittenger 2015
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

#include "Plane.h"

/*
 *  this module deals with ADS-B handling for ArduPlane
 *  ADS-B is an RF based collision avoidance protocol to tell nearby aircraft your location
 *  https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast
 *
 */

/*
  handle periodic adsb database maintenance
 */
void Plane::adsb_update(void)
{
    adsb.update();
}

