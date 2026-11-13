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
#pragma once

#include "SIM_config.h"

#if AP_SIM_JSON_ENABLED

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"
#include <AP_JSON/AP_JSON_FieldParser.h>

#define SITL_JSON_DEBUG 0

namespace SITL {

class JSON : public Aircraft {
public:
    JSON(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW JSON(frame_str);
    }

    /* Create and set in/out socket for JSON generic simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:

    struct servo_packet_16 {
        uint16_t magic = 18458; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[16];
    };

    struct servo_packet_32 {
        uint16_t magic = 29569; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[32];
    };

    // default connection_info_.ip_address
    const char *target_ip = "127.0.0.1";

    // default connection_info_.sitl_ip_port
    uint16_t control_port = 9002;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SocketAPM_native sock;
#else
    // sim-on-hardware
    SocketAPM sock;
#endif

    uint32_t frame_counter;
    double last_timestamp_s;
    double base_timestamp_s;
    uint64_t base_time_us;

    void output_servos(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);

    // parse received bytes, returning the fields of the last complete
    // valid packet in them, or 0 if there was none
    uint64_t parse_sensors(const uint8_t *data, size_t len);

    uint64_t handle_parse_result(AP_JSON_FieldParser::Result result);
    uint64_t non_finite_fields(uint64_t mask) const;
    void print_fields(const char *msg, uint64_t mask) const;

    // one datagram of sensor data. The headroom over the largest packet
    // lets a datagram that is too large be told from one that fits
    uint8_t recv_buffer[AP_JSON_FieldParser::MAX_PACKET_LEN + 64];

    struct SensorState {
        double timestamp_s;
        double latitude;
        double longitude;
        double altitude;
        struct {
            Vector3f gyro;
            Vector3f accel_body;
        } imu;
        Vector3d position;
        Vector3f attitude;
        Quaternion quaternion;
        Vector3f velocity;
        Vector3f velocity_wind;
        float rng[6];
        float rc[12];
        float bat_volt;
        float bat_amp;
        struct {
            float direction;
            float speed;
        } wind_vane_apparent;
        float airspeed;
        bool no_time_sync;
        bool no_lockstep;
    };
    // state from the last valid packet
    SensorState state;
    // the packet being received, copied to state when it is complete and
    // valid, so a bad packet never changes state
    SensorState state_rx;

    // fields of a JSON sensor packet, written to state_rx while parsing.
    // Bit i of a received bitmask is fields[i], see DataKey below
    typedef AP_JSON_FieldParser::Type FT;
    const AP_JSON_FieldParser::Field fields[36] {
        { "timestamp", nullptr, FT::DOUBLE, 1, &state_rx.timestamp_s },
        { "latitude", nullptr, FT::DOUBLE, 1, &state_rx.latitude },
        { "longitude", nullptr, FT::DOUBLE, 1, &state_rx.longitude },
        { "altitude", nullptr, FT::DOUBLE, 1, &state_rx.altitude },
        { "imu.gyro", nullptr, FT::FLOAT_ARRAY, 3, &state_rx.imu.gyro },
        { "imu.accel_body", nullptr, FT::FLOAT_ARRAY, 3, &state_rx.imu.accel_body },
        { "position", nullptr, FT::DOUBLE_ARRAY, 3, &state_rx.position },
        { "attitude", nullptr, FT::FLOAT_ARRAY, 3, &state_rx.attitude },
        { "quaternion", nullptr, FT::FLOAT_ARRAY, 4, &state_rx.quaternion.q1 },
        { "velocity", nullptr, FT::FLOAT_ARRAY, 3, &state_rx.velocity },
        { "rng_1", nullptr, FT::FLOAT, 1, &state_rx.rng[0] },
        { "rng_2", nullptr, FT::FLOAT, 1, &state_rx.rng[1] },
        { "rng_3", nullptr, FT::FLOAT, 1, &state_rx.rng[2] },
        { "rng_4", nullptr, FT::FLOAT, 1, &state_rx.rng[3] },
        { "rng_5", nullptr, FT::FLOAT, 1, &state_rx.rng[4] },
        { "rng_6", nullptr, FT::FLOAT, 1, &state_rx.rng[5] },
        { "velocity_wind", nullptr, FT::FLOAT_ARRAY, 3, &state_rx.velocity_wind },
        { "windvane.direction", nullptr, FT::FLOAT, 1, &state_rx.wind_vane_apparent.direction },
        { "windvane.speed", nullptr, FT::FLOAT, 1, &state_rx.wind_vane_apparent.speed },
        { "airspeed", nullptr, FT::FLOAT, 1, &state_rx.airspeed },
        { "no_time_sync", nullptr, FT::BOOL, 1, &state_rx.no_time_sync },
        { "no_lockstep", nullptr, FT::BOOL, 1, &state_rx.no_lockstep },
        { "rc.rc_1", nullptr, FT::FLOAT, 1, &state_rx.rc[0] },
        { "rc.rc_2", nullptr, FT::FLOAT, 1, &state_rx.rc[1] },
        { "rc.rc_3", nullptr, FT::FLOAT, 1, &state_rx.rc[2] },
        { "rc.rc_4", nullptr, FT::FLOAT, 1, &state_rx.rc[3] },
        { "rc.rc_5", nullptr, FT::FLOAT, 1, &state_rx.rc[4] },
        { "rc.rc_6", nullptr, FT::FLOAT, 1, &state_rx.rc[5] },
        { "rc.rc_7", nullptr, FT::FLOAT, 1, &state_rx.rc[6] },
        { "rc.rc_8", nullptr, FT::FLOAT, 1, &state_rx.rc[7] },
        { "rc.rc_9", nullptr, FT::FLOAT, 1, &state_rx.rc[8] },
        { "rc.rc_10", nullptr, FT::FLOAT, 1, &state_rx.rc[9] },
        { "rc.rc_11", nullptr, FT::FLOAT, 1, &state_rx.rc[10] },
        { "rc.rc_12", nullptr, FT::FLOAT, 1, &state_rx.rc[11] },
        { "battery.voltage", nullptr, FT::FLOAT, 1, &state_rx.bat_volt },
        { "battery.current", nullptr, FT::FLOAT, 1, &state_rx.bat_amp },
    };
    AP_JSON_FieldParser parser{fields, ARRAY_SIZE(fields)};

    // Enum corresponding to the ordering of entries in fields[]
    enum DataKey : uint64_t {
        TIMESTAMP   = 0x0000000000000001ULL, // 1ULL << 0
        LATITUDE    = 0x0000000000000002ULL, // 1ULL << 1
        LONGITUDE   = 0x0000000000000004ULL, // 1ULL << 2
        ALTITUDE    = 0x0000000000000008ULL, // 1ULL << 3
        GYRO        = 0x0000000000000010ULL, // 1ULL << 4
        ACCEL_BODY  = 0x0000000000000020ULL, // 1ULL << 5
        POSITION    = 0x0000000000000040ULL, // 1ULL << 6
        EULER_ATT   = 0x0000000000000080ULL, // 1ULL << 7
        QUAT_ATT    = 0x0000000000000100ULL, // 1ULL << 8
        VELOCITY    = 0x0000000000000200ULL, // 1ULL << 9
        RNG_1       = 0x0000000000000400ULL, // 1ULL << 10
        RNG_2       = 0x0000000000000800ULL, // 1ULL << 11
        RNG_3       = 0x0000000000001000ULL, // 1ULL << 12
        RNG_4       = 0x0000000000002000ULL, // 1ULL << 13
        RNG_5       = 0x0000000000004000ULL, // 1ULL << 14
        RNG_6       = 0x0000000000008000ULL, // 1ULL << 15
        WIND_VEL    = 0x0000000000010000ULL, // 1ULL << 16
        WIND_DIR    = 0x0000000000020000ULL, // 1ULL << 17
        WIND_SPD    = 0x0000000000040000ULL, // 1ULL << 18
        AIRSPEED    = 0x0000000000080000ULL, // 1ULL << 19
        TIME_SYNC   = 0x0000000000100000ULL, // 1ULL << 20
        LOCKSTEP    = 0x0000000000200000ULL, // 1ULL << 21
        RC_1        = 0x0000000000400000ULL, // 1ULL << 22
        RC_2        = 0x0000000000800000ULL, // 1ULL << 23
        RC_3        = 0x0000000001000000ULL, // 1ULL << 24
        RC_4        = 0x0000000002000000ULL, // 1ULL << 25
        RC_5        = 0x0000000004000000ULL, // 1ULL << 26
        RC_6        = 0x0000000008000000ULL, // 1ULL << 27
        RC_7        = 0x0000000010000000ULL, // 1ULL << 28
        RC_8        = 0x0000000020000000ULL, // 1ULL << 29
        RC_9        = 0x0000000040000000ULL, // 1ULL << 30
        RC_10       = 0x0000000080000000ULL, // 1ULL << 31
        RC_11       = 0x0000000100000000ULL, // 1ULL << 32
        RC_12       = 0x0000000200000000ULL, // 1ULL << 33
        BAT_VOLT    = 0x0000000400000000ULL, // 1ULL << 34
        BAT_AMP     = 0x0000000800000000ULL, // 1ULL << 35
    };
    uint64_t last_received_bitmask;

    // packets missing any of these are rejected
    static const uint64_t REQUIRED_FIELDS = TIMESTAMP | GYRO | ACCEL_BODY | VELOCITY;

    // rangefinders may send infinity or NaN for having no reading, which
    // SITL reports as no data; packets with them in any other field are
    // rejected, as the rest of SITL uses those values as they are
    static const uint64_t NON_FINITE_ALLOWED = RNG_1 | RNG_2 | RNG_3 | RNG_4 | RNG_5 | RNG_6;

    uint32_t rejected_packets;

    // rate limits reports of rejected sensor packets
    uint64_t last_parse_error_us;
    bool report_parse_error();

#if SITL_JSON_DEBUG
    uint32_t last_debug_ms;
#endif

    bool last_no_lockstep;
};

}

#endif  // AP_SIM_JSON_ENABLED