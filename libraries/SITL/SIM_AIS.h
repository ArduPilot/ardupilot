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
    Dump logged AIS data to the serial port
    ./Tools/autotest/sim_vehicle.py -v Rover --no-mavproxy -A --serial5=sim:AIS --custom-location 51.58689798356386,-3.9044570193067965,0,0

    param set SERIAL5_PROTOCOL 40
    param set AIS_TYPE 1
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_AIS_ENABLED

#include "SIM_SerialDevice.h"
#include <SITL/SITL.h>

#include <GCS_MAVLink/GCS_MAVLink.h>

namespace SITL {

// Replay saved data file
class AIS_Replay : public SerialDevice {
public:

    AIS_Replay();

    void update();

private:
    FILE* file;

    uint32_t last_sent_ms;

};

// Generate virtual vessels
class AIS : public SerialDevice {
public:
    AIS();

    void update(const class Aircraft &aircraft);

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8 vessel_count;
    AP_Float radius_m;

    struct ais_vessel {
        mavlink_ais_vessel_t info; // Info about the vessel
        uint32_t last_position_report_ms;   // last time a postion report message was sent
        uint32_t last_static_and_voyage_ms; // last time a static and voyage data message was sent
        bool active; // true if this vessel should be reported
    };
    ais_vessel vessels[50];

    // Update a active vessel position and heading
    void update_simulated_vessel(ais_vessel &vessel, const float dt, const Location &vehicle_loc, const float radius, const uint32_t now_ms);
    uint32_t last_sim_update_ms;

    // Generate a new vessel
    void init_vessel(ais_vessel &vessel, const Location &vehicle_loc, const float radius);

    // formatted print of NMEA message, with checksum appended
    void nmea_printf(const char *fmt, ...);

    // Convert to NMEA char
    uint8_t encode_char(uint8_t payload) const;

    // Send a position report message for the passed vessel
    void send_position_report(const mavlink_ais_vessel_t &info);

    // set the specified bits in the payload with 6 bit chars
    void set_bits(uint8_t *payload, const uint16_t low, const uint16_t high, uint32_t value);
    void set_bits_signed(uint8_t *payload, const uint16_t low, const uint16_t high, int32_t value);

    // Get bits from uint32 starting at start for given length
    uint8_t get_bits(uint32_t value, const uint8_t start, const uint8_t len) const;

    // Send a static and voyage data message for the passed vessel
    void send_static_and_voyage(const mavlink_ais_vessel_t &info);

    // Used to match multi part messages
    uint8_t sequence_ID;
};


}

#endif // AP_SIM_AIS_ENABLED
