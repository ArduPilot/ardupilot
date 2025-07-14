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

// Automatic Identification System, https://gpsd.gitlab.io/gpsd/AIVDM.html

#include "AP_AIS.h"

#if AP_AIS_ENABLED

#include <AP_Math/AP_Math.h>

// Apply dimensions to a vessel
// All dimensions are in meters from the provided location
void AP_AIS::ais_vehicle_t::set_dimensions(uint16_t bow, uint16_t stern, uint8_t port, uint8_t star)
{
    if (bow == 0 || stern == 0 || port == 0 || star == 0) {
        // Incoming dimensions are not valid
        // Never clear existing dimensions
        return;
    }

    // Set valid dimensions flag
    info.flags |= AIS_FLAGS_VALID_DIMENSIONS;

    // clear large dimension flags
    const uint16_t mask = ~(AIS_FLAGS_LARGE_BOW_DIMENSION | AIS_FLAGS_LARGE_STERN_DIMENSION | AIS_FLAGS_LARGE_PORT_DIMENSION | AIS_FLAGS_LARGE_STARBOARD_DIMENSION);
    info.flags &= mask; 

    // Set large dimension flags
    if (bow == 511) {
        info.flags |= AIS_FLAGS_LARGE_BOW_DIMENSION;
    }
    if (stern == 511) {
        info.flags |= AIS_FLAGS_LARGE_STERN_DIMENSION;
    }
    if (port == 63) {
        info.flags |= AIS_FLAGS_LARGE_PORT_DIMENSION;
    }
    if (star == 63) {
        info.flags |= AIS_FLAGS_LARGE_STARBOARD_DIMENSION;
    }

    // Set values
    info.dimension_bow = bow;
    info.dimension_stern = stern;
    info.dimension_port = port;
    info.dimension_starboard = star;
}

// Apply corse over ground to a vessel
// cog in cdeg
void AP_AIS::ais_vehicle_t::set_cog(uint16_t cog)
{
    if (cog < 36000) {
        // Valid
        info.flags |= AIS_FLAGS_VALID_COG;
        info.COG = cog;
        return;
    }

    // Invalid
    info.flags &= ~AIS_FLAGS_VALID_COG;
    info.COG = 0;
}

// Apply speed over ground to a vessel
// sog in 0.1 Knots
void AP_AIS::ais_vehicle_t::set_sog(uint16_t sog)
{
    if (sog >= 1023) {
        // Invalid
        info.flags &= ~(AIS_FLAGS_VALID_VELOCITY | AIS_FLAGS_HIGH_VELOCITY);
        info.velocity = 0;
        return;
    }

    // Valid
    info.flags |= AIS_FLAGS_VALID_VELOCITY;

    // More than 102.2 knots, don't know by how much
    if (sog == 1022) {
        info.flags |= AIS_FLAGS_HIGH_VELOCITY;
    } else {
        info.flags &= ~AIS_FLAGS_HIGH_VELOCITY;
    }

    // Convert 0.1 knots to cm/s
    info.velocity = sog * 0.1 * KNOTS_TO_M_PER_SEC * 100.0;

}

// Apply position accuracy flag to a vessel
void AP_AIS::ais_vehicle_t::set_pos_acc(bool pos_acc)
{
    if (pos_acc) {
        info.flags |= AIS_FLAGS_POSITION_ACCURACY;
        return;
    }

    info.flags &= ~AIS_FLAGS_POSITION_ACCURACY;
}

// Apply rate of turn to a vessel
// rot in none standard units!
void AP_AIS::ais_vehicle_t::set_rot(int8_t rot)
{
    if (rot <= -128) {
        // Invalid
        info.flags &= ~(AIS_FLAGS_VALID_TURN_RATE | AIS_FLAGS_TURN_RATE_SIGN_ONLY);
        info.turn_rate = 0;
        return;
    }

    // Valid value
    info.flags |= AIS_FLAGS_VALID_TURN_RATE;
    const int8_t sign = rot > 0 ? 1 : -1;
    if (rot == 127 || rot == -127) {
        info.flags |= AIS_FLAGS_TURN_RATE_SIGN_ONLY;
        info.turn_rate = sign;
        return;
    }

    // Apply expo formula and convert from deg per min to cdeg per second
    // constrain to avoid overflow, probably need to change the units or increase to int16
    info.turn_rate = sign * MIN(sq(rot / 4.733) * (100.0 / 60.0), INT8_MAX - 1);
}

// Set call sign of vessel
void AP_AIS::ais_vehicle_t::set_callsign(const char* callsign)
{
    if (strlen(callsign) == 0) {
        // Never clear call sign
        return;
    }

    info.flags |= AIS_FLAGS_VALID_CALLSIGN;
    memcpy(info.callsign, callsign, sizeof(info.callsign));
}

// Set name of vessel
void AP_AIS::ais_vehicle_t::set_name(const char* name)
{
    if (strlen(name) == 0) {
        // Never clear name
        return;
    }

    info.flags |= AIS_FLAGS_VALID_NAME;
    memcpy(info.name, name, sizeof(info.name));
}

#endif  // AP_AIS_ENABLED
