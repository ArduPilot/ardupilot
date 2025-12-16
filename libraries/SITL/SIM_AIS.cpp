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
    Simulate virtual vessels
    ./Tools/autotest/sim_vehicle.py -v Rover -A --serial5=sim:AIS --map

    param set SERIAL5_PROTOCOL 40
    param set AIS_TYPE 1
    reboot
    param set SIM_AIS_COUNT 10
    module load ais

    Dump logged AIS data to the serial port
    ./Tools/autotest/sim_vehicle.py -v Rover -A --serial5=sim:AISReplay --custom-location 51.58689798356386,-3.9044570193067965,0,0 --map

    param set SERIAL5_PROTOCOL 40
    param set AIS_TYPE 1
    reboot
    module load ais
*/

#include "SIM_config.h"

#if AP_SIM_AIS_ENABLED

#include "SIM_AIS.h"

#include <SITL/SITL.h>

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"

#include "SIM_Aircraft.h"
#include <AP_Common/NMEA.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

const AP_Param::GroupInfo AIS::var_info[] = {

    // @Param: COUNT
    // @DisplayName: Number of AIS vessels
    // @Description: Total number of AIS simulated vessels
    AP_GROUPINFO("COUNT", 1, AIS, vessel_count, -1),

    // @Param: RADIUS
    // @DisplayName: AIS radius stddev of vessels
    // @Description: Simulated standard deviation of radius in AIS of a vessel
    // @Units: m
    AP_GROUPINFO("RADIUS",  2, AIS, radius_m, 10000),

    AP_GROUPEND
};

AIS_Replay::AIS_Replay() : SerialDevice::SerialDevice()
{
    char* file_path;
    IGNORE_RETURN(asprintf(&file_path, AP_BUILD_ROOT "/libraries/SITL/SIM_AIS_data.txt"));

    file = fopen(file_path,"r");

    if (file == nullptr) {
        AP_HAL::panic("AIS could not open data file");
    }

    // seek past the header line
    char line[100];
    IGNORE_RETURN(fgets(line, sizeof(line), file));
}

void AIS_Replay::update()
{
    if (file == nullptr) {
        AP_HAL::panic("AIS lost data file");
    }

    // just send a line of data at 1Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 1000) {
        return;
    }
    last_sent_ms = now;

    char line[100];

    if (!fgets(line, sizeof(line), file)) {
        // got to the end of the file, circle back
        fseek(file,0,SEEK_SET);
        if (!fgets(line, sizeof(line), file)) {
            AP_HAL::panic("AIS lost data file");
        }
        return;
    }

    //hal.console->printf("%s",line);
    write_to_autopilot(line, strlen(line));

}

AIS::AIS()
{
    AP::sitl()->models.ais_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AIS::update(const class Aircraft &aircraft)
{
    const SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    // Disabled
    if (vessel_count <= 0) {
        return;
    }

    // Calculate the time step to take
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = (now_ms - last_sim_update_ms) * 0.001;
    last_sim_update_ms = now_ms;

    // Get current location to remove any vessel that is too far away
    const Location &aircraft_loc = aircraft.get_location();

    // Update all vessels, marking any that move too far from the aircraft as in-active
    for (uint8_t i=0; i<ARRAY_SIZE(vessels); i++) {
        update_simulated_vessel(vessels[i], dt, aircraft_loc, radius_m, now_ms);
    }

    // Add new vessels until we have the correct number
    for (uint8_t i=0; i<ARRAY_SIZE(vessels); i++) {
        // Disable any extra vessels over the set count
        if (i >= vessel_count) {
            vessels[i].active = false;
            continue;
        }

        // Vessel is already active
        if (vessels[i].active) {
            continue;
        }

        // Generate new vessel
        init_vessel(vessels[i], aircraft_loc, radius_m);
    }

}

// Update a active vessel position and heading
void AIS::update_simulated_vessel(ais_vessel &vessel, const float dt, const Location &vehicle_loc, const float radius, const uint32_t now_ms)
{
    // Check if this vessel is active
    if (!vessel.active) {
        return;
    }

    // Update heading from turn rate, both turn rate and heading are in cdeg
    vessel.info.heading += vessel.info.turn_rate * dt;

    // Extract location
    Location loc { vessel.info.lat, vessel.info.lon, 0, Location::AltFrame::ABOVE_ORIGIN };

    // Update location from heading and velocity
    loc.offset_bearing(vessel.info.heading * 0.01, vessel.info.velocity * 0.01 * dt);

    // Repopulate location
    vessel.info.lat = loc.lat;
    vessel.info.lon = loc.lng;

    // Check if location is still valid
    if (vehicle_loc.get_distance(loc) > radius) {
        vessel.active = false;
        return;
    }

    // Report, update rate depends on speed
    uint32_t postion_interval;

    // Convert speed over ground from 0.1 knots to cm/s
    if (vessel.info.velocity > 23.0 * 0.1 * KNOTS_TO_M_PER_SEC * 100.0) {
        postion_interval = 2000; // every 2 seconds if faster than 23 knots

    } else if (vessel.info.velocity > 14.0 * 0.1 * KNOTS_TO_M_PER_SEC * 100.0) {
        postion_interval = 6000; // every 6 seconds if faster than 14 knots

    } else if (vessel.info.velocity > 0.0) {
        postion_interval = 10000; // every 10 seconds if moving

    } else {
        postion_interval = 3 * 60 * 1000; // every 3 mins if anchored or moored and moving at less than 3 knots

    }

    // Send position report at internal
    if ((vessel.last_position_report_ms == 0) || (now_ms - vessel.last_position_report_ms > postion_interval)) {
        send_position_report(vessel.info);
        vessel.last_position_report_ms = now_ms;
    }

    // Send static and voyage at internal
    const uint32_t static_and_voyage_interval = 6 * 60 * 1000; // every 6 mins
    if ((vessel.last_static_and_voyage_ms == 0) || (now_ms - vessel.last_static_and_voyage_ms > static_and_voyage_interval)) {
        send_static_and_voyage(vessel.info);
        vessel.last_static_and_voyage_ms = now_ms;
    }

}

// Generate a new vessel
void AIS::init_vessel(ais_vessel &vessel, const Location &vehicle_loc, const float radius)
{
    // Clear any existing data
    memset(&vessel, 0, sizeof(vessel));

    // Set flags for valid data
    vessel.info.flags |= AIS_FLAGS_VALID_VELOCITY | AIS_FLAGS_VALID_TURN_RATE;

    // Generate random data for static info, mask to 30 bit
    vessel.info.MMSI = rand() & 0x3FFFFFFF;

    // Set nav status and type
    vessel.info.navigational_status = AIS_NAV_STATUS_UNDER_WAY;
    vessel.info.type = AIS_TYPE_CARGO;

    // 90% chance of having valid dimensions
    if (rand() > RAND_MAX * 0.1) {
        vessel.info.flags |= AIS_FLAGS_VALID_DIMENSIONS;

        // Generate each dimension
        vessel.info.dimension_bow = abs(Aircraft::rand_normal(0, 300.0));
        if (vessel.info.dimension_bow > 511) {
            vessel.info.dimension_bow = 511;
            vessel.info.flags |= AIS_FLAGS_LARGE_BOW_DIMENSION;
        }

        vessel.info.dimension_stern = abs(Aircraft::rand_normal(0, 300.0));
        if (vessel.info.dimension_stern > 511) {
            vessel.info.dimension_stern = 511;
            vessel.info.flags |= AIS_FLAGS_LARGE_STERN_DIMENSION;
        }

        vessel.info.dimension_port = abs(Aircraft::rand_normal(0, 30.0));
        if (vessel.info.dimension_port > 63) {
            vessel.info.dimension_port = 63;
            vessel.info.flags |= AIS_FLAGS_LARGE_PORT_DIMENSION;
        }

        vessel.info.dimension_starboard = abs(Aircraft::rand_normal(0, 30.0));
        if (vessel.info.dimension_starboard > 63) {
            vessel.info.dimension_starboard = 63;
            vessel.info.flags |= AIS_FLAGS_LARGE_STARBOARD_DIMENSION;
        }
    }

    // 25% chance of turning
    if (rand() > RAND_MAX * 0.75) {
        vessel.info.turn_rate = Aircraft::rand_normal(0, 100.0);
    }

    // 95% chance of moving
    if (rand() > RAND_MAX * 0.05) {
        vessel.info.velocity = abs(Aircraft::rand_normal(0, 500.0));

        // Speed threshold for high velocity flag
        // Convert speed over ground from 0.1 knots to cm/s
        const uint16_t high_velocity = 1022 * 0.1 * KNOTS_TO_M_PER_SEC * 100.0;

        if (vessel.info.velocity >= high_velocity) {
            vessel.info.velocity = high_velocity;
            vessel.info.flags |= AIS_FLAGS_HIGH_VELOCITY;
        }
    }

    // Generate a heading
    vessel.info.heading = wrap_360_cd(rand());

    // Generate a random location
    Location loc = vehicle_loc;
    loc.offset( Vector3p {
        Aircraft::rand_normal(0, radius),
        Aircraft::rand_normal(0, radius),
        0
    });
    vessel.info.lat = loc.lat;
    vessel.info.lon = loc.lng;

    // mark active
    vessel.active = true;
}

/*
  formatted print of NMEA message, with checksum appended
 */
void AIS::nmea_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s != nullptr) {
        write_to_autopilot((const char*)s, strlen(s));
        free(s);
    }
}


// Convert to NMEA char
uint8_t AIS::encode_char(uint8_t payload) const
{
    if (payload + 8 > 40) {
        return payload + 8 + 48;
    }
    return payload + 48;
}

uint8_t AIS::get_bits(uint32_t value, const uint8_t start, const uint8_t len) const
{
    value = value >> start;
    const uint8_t mask = 0b111111 >> (6 - len);
    return value & mask;
}

// set the specified bits in the payload with 6 bit chars
void AIS::set_bits(uint8_t *payload, const uint16_t low, const uint16_t high, uint32_t value)
{
    // Bits are inclusive, so add one
    const uint8_t bit_len = high - low + 1;
    const uint32_t value_mask = 0xFFFFFFFF >> (32 - bit_len);

    if ((value & ~value_mask) != 0) {
        AP_HAL::panic("AIS value passed too big to fit (%u > %u bits)", value, bit_len);
    }

    const uint8_t char_low = low / 6;
    const uint8_t bit_low = low % 6;

    const uint8_t char_high = high / 6;
    const uint8_t bit_high = (high % 6) + 1;

    const uint8_t char_range = char_high - char_low;
    uint8_t bits = bit_len;
    for (uint8_t index = 0; index <= char_range; index++) {
        if (bits == 0) {
            AP_HAL::panic("AIS packing fail A");
        }
        if (index == char_range) {
            // last char uses high bit
            const uint8_t bits_needed = MIN(bit_high, bit_len);
            bits -= bits_needed;
            payload[char_low + index] |= get_bits(value, bits, bits_needed) << (6 - bit_high);

        } else if (index == 0) {
            // First char uses low bit
            const uint8_t bits_needed = MIN(6 - bit_low, bit_len);
            bits -= bits_needed;
            payload[char_low + index] |= get_bits(value, bits, bits_needed);

        } else {
            // Others need full length
            bits -= 6;
            payload[char_low + index] |= get_bits(value, bits, 6);

        }
    }

    if (bits != 0) {
        AP_HAL::panic("AIS packing fail B");
    }

}

void AIS::set_bits_signed(uint8_t *payload, const uint16_t low, const uint16_t high, int32_t value)
{
    if (value >= 0) {
        // Positive number is the same as unsigned
        set_bits(payload, low, high, value);
        return;
    }

    // Convert to unsigned
    uint32_t val = value;

    // Mask to correct length
    const uint8_t bit_len = high - low + 1;
    val &= 0xFFFFFFFF >> (32 - bit_len);

    // Set sign bit
    val |= 1 << (bit_len - 1);

    set_bits(payload, low, high, val);
}

void AIS::send_position_report(const mavlink_ais_vessel_t &info) {

    // Turn rate, default to not available
    int8_t rot = -128;
    if ((info.flags & AIS_FLAGS_VALID_TURN_RATE) != 0) {
        if ((info.flags & AIS_FLAGS_TURN_RATE_SIGN_ONLY) != 0) {
            // Only sign available
            rot = info.turn_rate >= 0 ? 127 : -127;

        } else {
            // cdeg/s to deg/min, take abs to allow sqrt
            const float turn_rate_deg_per_min = fabsf(info.turn_rate) * 60.0 * 0.01;

            // Apply scaling, recovering sign
            rot = 4.733 * sqrtf(turn_rate_deg_per_min) * info.turn_rate >= 0 ? 1.0 : -1.0;
        }
    }

    // Corse over ground, default to not available
    uint16_t cog = 3600;
    if ((info.flags & AIS_FLAGS_VALID_COG) != 0) {
        // cdeg to 0.1 deg
        cog = info.COG * 0.1;
    }

    // Accuracy bit from flag
    const bool accuracy = (info.flags & AIS_FLAGS_POSITION_ACCURACY) != 0;

    // Speed over ground, default to not available
    uint16_t sog = 1023;
    if ((info.flags & AIS_FLAGS_VALID_VELOCITY) != 0) {
        if ((info.flags & AIS_FLAGS_HIGH_VELOCITY) != 0) {
            sog = 1022;
        } else {
            sog = MIN((float(info.velocity) * 0.01 * M_PER_SEC_TO_KNOTS * 10), 1022);
        }
    }

    // storage for payload
    uint8_t payload[28] {};
    set_bits(payload, 0,  5,  1); // type: Position Report Class A
    set_bits(payload, 6,  7,  3); // repeat: Do not repeat
    set_bits(payload, 8,  37, info.MMSI); // mmsi
    set_bits(payload, 38, 41, info.navigational_status); // status
    set_bits_signed(payload, 42, 49, rot); // rate of turn
    set_bits(payload, 50, 59, sog); // speed
    set_bits(payload, 60, 60, accuracy); // accuracy
    set_bits_signed(payload, 61, 88, int32_t(double(info.lon) * 0.06)); // lon
    set_bits_signed(payload, 89, 115, int32_t(double(info.lat) * 0.06)); // lat
    set_bits(payload, 116, 127, cog); // course over ground
    set_bits(payload, 128, 136, uint32_t(info.heading * 0.01)); // heading convert from deg to centi-deg
    set_bits(payload, 137, 142, 60); // second: not available
    set_bits(payload, 143, 144, 0); // maneuver: not available
    // 145 - 147: spare
    set_bits(payload, 148, 148, 0); // raim not used
    set_bits(payload, 149, 167, 0); // Radio status

    // Convert to chars
    uint8_t encoded[sizeof(payload) + 1] {};
    for (uint8_t i = 0; i < sizeof(payload); i++) {
        encoded[i] = encode_char(payload[i]);
    }

    const uint8_t total_fragments = 1;
    const uint8_t fragment = 1;
    // sequence ID is not used for a single fragment message
    const char channel_code = 'A';
    const uint8_t fill_bits = 0;

    // Send
    nmea_printf("!AIVDM,%u,%u,,%c,%s,%u", total_fragments, fragment, channel_code, encoded, fill_bits);

}

void AIS::send_static_and_voyage(const mavlink_ais_vessel_t &info) {

    // Fill in dimensions if valid
    uint16_t bow_dim = 0;
    uint16_t stern_dim = 0;
    uint8_t port_dim = 0;
    uint8_t star_dim = 0;

    if ((info.flags & AIS_FLAGS_VALID_DIMENSIONS) != 0) {
        if ((info.flags & AIS_FLAGS_LARGE_BOW_DIMENSION) != 0) {
            bow_dim = 511;
        } else {
            bow_dim = MIN(info.dimension_bow, 511);
        }
        if ((info.flags & AIS_FLAGS_LARGE_STERN_DIMENSION) != 0) {
            stern_dim = 511;
        } else {
            stern_dim = MIN(info.dimension_stern, 511);
        }
        if ((info.flags & AIS_FLAGS_LARGE_PORT_DIMENSION) != 0) {
            port_dim = 63;
        } else {
            port_dim = MIN(info.dimension_port, 63);
        }
        if ((info.flags & AIS_FLAGS_LARGE_STARBOARD_DIMENSION) != 0) {
            star_dim = 63;
        } else {
            star_dim = MIN(info.dimension_starboard, 63);
        }
    }

    // storage for payload
    uint8_t payload[71] {};
    set_bits(payload, 0,  5,  5); // type: Static and Voyage Related Data
    set_bits(payload, 6,  7,  3); // repeat: Do not repeat
    set_bits(payload, 8,  37, info.MMSI); // mmsi
    set_bits(payload, 38, 39, 1); // AIS version
    set_bits(payload, 40, 69, 0); // imo number
    set_bits(payload, 70, 111, '@'); // call_sign: empty
    set_bits(payload, 112, 231, '@'); // name: empty
    set_bits(payload, 232, 239, info.type);
    set_bits(payload, 240, 248, bow_dim); // bow_dim
    set_bits(payload, 249, 257, stern_dim); // stern_dim
    set_bits(payload, 258, 263, port_dim); // port_dim
    set_bits(payload, 264, 269, star_dim); // star_dim
    set_bits(payload, 270, 273, 1); // fix: GPS
    set_bits(payload, 274, 277, 0); // ETA month: not available
    set_bits(payload, 278, 282, 0); // ETA day: not available
    set_bits(payload, 283, 287, 0); // ETA hour: not available
    set_bits(payload, 288, 293, 0); // ETA minute: not available
    set_bits(payload, 294, 301, 0); // draught: 0 decimeters
    set_bits(payload, 302, 421, '@'); // destination: empty
    set_bits(payload, 422, 422, 1); // dte: Data terminal not ready
    // 423: not used

    // Split and encode
    const uint8_t msg1_len = 50;
    const uint8_t msg2_len = sizeof(payload) - msg1_len;

    uint8_t msg1_encoded[msg1_len + 1] {};
    for (uint8_t i = 0; i < msg1_len; i++) {
        msg1_encoded[i] = encode_char(payload[i]);
    }

    uint8_t msg2_encoded[msg2_len + 1] {};
    for (uint8_t i = 0; i < msg2_len; i++) {
        msg2_encoded[i] = encode_char(payload[msg1_len + i]);
    }

    const uint8_t total_fragments = 2;
    uint8_t fragment = 1;
    const char channel_code = 'A';
    const uint8_t fill_bits = 0;

    // Send
    nmea_printf("!AIVDM,%u,%u,%u,%c,%s,%u", total_fragments, fragment, sequence_ID, channel_code, msg1_encoded, fill_bits);
    fragment++;

    nmea_printf("!AIVDM,%u,%u,%u,%c,%s,%u", total_fragments, fragment, sequence_ID, channel_code, msg2_encoded, fill_bits);

    sequence_ID++;
}

#endif  // AP_SIM_AIS_ENABLED
