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

    ATH- Vinayak Pundhir (VP)
 */
/*
    Simulate Aeron Systems PLX3 INS Serially
    Usage Example:
    param set AHRS_EKF_TYPE 11
    param set EAHRS_TYPE 10
    param set SERIAL1_PROTOCOL 36
    param set SERIAL1_BAUD 921600
    param set GPS1_TYPE 21
    
    param set EAHRS_RATE 200

    sim_vehicle.py -v ArduPlane -D --console --map -A --serial1=sim:Aeron-PLX3

*/

#pragma once

#include "SITL/SIM_config.h"

#if AP_SIM_AERON_ENABLED

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class Aeron : public SerialDevice {
public:

    Aeron();

    // update — called from SIMState at the simulator step rate
    void update(void);

private:

    // packet identifiers (PRP_ID, transmitted big-endian)
    enum class PktID : uint16_t {
        NAV_PARA1       = 0xAAA1,
        NAV_PARA2       = 0xA2A2,
        SENS_PARA       = 0xA2A3,
        GPS_PARA        = 0xA2A4,
        EXTD_GNSS       = 0xA2A8,
        // Replies sent in response to the autopilot's external-sensor
        // presence queries.  Single-byte payload (0 or 1).
        EXT_SENSORS_MAG = 0x0117,
        EXT_SENSORS_ASP = 0x0362,
    };

    // payload structures (must match the on-wire layout exactly) 

    struct PACKED NavPara1Payload {
        uint32_t epoch_time;        // seconds since Unix epoch
        uint32_t microseconds;      // sub-second component
        uint32_t ins_status;        // 0 → no error
        uint32_t hw_status;         // 0 → no error
        float    euler[3];          // roll, pitch, yaw [rad]
        float    course;            // ground track [deg]
        float    velocity_ned[3];   // [m/s]
        double   position[2];       // lat, lon [deg]
        float    height_abv_ellip;  // height above WGS84 ellipsoid [m]
    };

    struct PACKED NavPara2Payload {
        uint32_t epoch_time;
        uint32_t microseconds;
        uint32_t ins_status;
        uint32_t hw_status;
        float    quat[4];           // w, x, y, z
        float    body_vel[3];       // [m/s] in body frame
        double   ecef_pos[3];       // [m] WGS84 ECEF
        float    altitude;          // [m]
    };

    struct PACKED SensParaPayload {
        uint32_t epoch_time;
        uint32_t microseconds;
        float    temperature;       // [°C]
        float    gyro[3];           // [deg/s] body frame
        float    accel[3];          // [m/s²] body frame
        float    mag[3];            // [mGauss] body frame
        float    euler_rates[3];    // [deg/s]
        float    baro_pressure;     // [Pa]
        float    baro_temperature;  // [°C]
        float    baro_altitude;     // [m]
    };

    struct PACKED GpsParaPayload {
        uint32_t epoch_time;
        uint32_t microseconds;
        uint32_t gps_status;        // packed: see build_gps_status()
        double   gps_position[2];   // lat, lon [deg]
        float    gps_height_abv_ellip;
        float    gps_undulation;    // [m] geoid undulation
        float    pdop;
        float    hdop;
        float    gdop;
    };

    struct PACKED ExtdGnssPayload {
        float    hpa;               // horizontal position accuracy [m]
        float    vpa;               // vertical   position accuracy [m]
        float    hva;               // horizontal velocity accuracy [m/s]
        float    vdop;
        double   gnss_vned[3];      // [m/s]
    };

    // catch any drift between the on-wire spec and the C++ structs at
    // compile time
    static_assert(sizeof(NavPara1Payload) == 64, "NAV_PARA1 payload size mismatch");
    static_assert(sizeof(NavPara2Payload) == 72, "NAV_PARA2 payload size mismatch");
    static_assert(sizeof(SensParaPayload) == 72, "SENS_PARA payload size mismatch");
    static_assert(sizeof(GpsParaPayload)  == 48, "GPS_PARA  payload size mismatch");
    static_assert(sizeof(ExtdGnssPayload) == 40, "EXTD_GNSS payload size mismatch");

    // -------- packet construction & transmission --------

    void send_aeron_packet(PktID prp_id, const void *payload, uint16_t payload_len);

    void send_nav_para1();
    void send_nav_para2();
    void send_sens_para();
    void send_gps_para();
    void send_extd_gnss();

    // build the packed gps_status word consumed by AP_ExternalAHRS_Aeron's
    // format_and_push_gps() routine
    static uint32_t build_gps_status(uint16_t num_sats, bool fix_present,
                                     uint8_t fix_raw, uint8_t jam, uint8_t spoof);

    // produce a Unix timeval that follows the simulation clock
    static void simulation_timeval(struct timeval *tv);

    // -------- inbound (autopilot -> SIM) parser --------
    // Drives EXT_SENSORS_MAG / EXT_SENSORS_ASP responses so the SITL
    // run exercises the driver's external-sensor query path.
    enum class RxParseState : uint8_t {
        SYNC = 0,
        LEN_HIGH,
        LEN_LOW,
        PAYLOAD,
        CRC_BYTE,
        RESET,
    };

    void process_inbound();
    void handle_inbound_byte(uint8_t byte);
    void handle_inbound_packet(uint16_t prp_id, const uint8_t *payload, uint16_t payload_len);

    RxParseState rx_state;
    uint8_t      rx_sync_count;
    uint16_t     rx_pkt_len;
    uint16_t     rx_write_idx;
    uint8_t      rx_buf[128];

    // PRP_ID used to request an external-sensor
    // presence reply.  The 2-byte payload identifies which response
    // packet to send (e.g. 0x0117 -> reply with EXT_SENSORS_MAG).
    static constexpr uint16_t QUERY_PRP_ID = 0x00A1;

    // -------- timing state --------
    // 1 kHz tick counter — caps update() to one execution per
    // simulated millisecond.
    uint64_t _tick;

    // Per-packet schedule — period and phase (both in milliseconds of simulated time).
    // Sent at SENS @50Hz, NAV1 @50Hz, NAV2 @50Hz, GPS & EXTD @10Hz. (Respected the Scheduler of Fixed Wing) (Bare Min Requirement)
    static constexpr uint32_t SENS_PERIOD_MS = 20;
    static constexpr uint32_t SENS_PHASE_MS  = 0;
    static constexpr uint32_t NAV1_PERIOD_MS = 20;
    static constexpr uint32_t NAV1_PHASE_MS  = 4;
    static constexpr uint32_t NAV2_PERIOD_MS = 20;
    static constexpr uint32_t NAV2_PHASE_MS  = 8;
    static constexpr uint32_t GPS_PERIOD_MS  = 100;
    static constexpr uint32_t GPS_PHASE_MS   = 12;
    static constexpr uint32_t EXTD_PERIOD_MS = 100;
    static constexpr uint32_t EXTD_PHASE_MS  = 16;

    // framing constants
    static constexpr uint8_t  SYNC_BYTE  = 0x05;
    static constexpr uint8_t  SYNC_COUNT = 4;

    // simulated GNSS quality (driver decode in format_and_push_gps())
    static constexpr uint16_t SIM_NUM_SATS    = 19;
    static constexpr uint8_t  SIM_FIX_RAW     = 1;  // GNSS_FIX → 3D fix
    static constexpr uint8_t  SIM_JAM_STATUS  = 1;  // NOT_DETECTED
    static constexpr uint8_t  SIM_SPOOF_LEVEL = 1;  // NOT_DETECTED

    // simulated accuracies
    static constexpr float SIM_HPA_M   = 0.6f;
    static constexpr float SIM_VPA_M   = 1.0f;
    static constexpr float SIM_HVA_MPS = 0.05f;
    static constexpr float SIM_PDOP    = 1.4f;
    static constexpr float SIM_HDOP    = 0.8f;
    static constexpr float SIM_VDOP    = 1.1f;
    static constexpr float SIM_GDOP    = 1.6f;
};

}  // namespace SITL

#endif // AP_SIM_AERON_ENABLED
