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
  Aeron Systems PLX series INS — External AHRS backend.

  Supports NAV_PARA1 (0xAAA1), NAV_PARA2 (0xA2A2), SENS_PARA (0xA2A3),
  GPS_PARA (0xA2A4), DEV_INFO (0xA2A6), EXTD_GNSS (0xA2A8)

  The driver runs a dedicated update thread that reads bytes from the
  UART, runs them through a state-machine parser, decodes valid frames
  into stack-local structs, and then briefly takes state.sem to publish
  into the AP_ExternalAHRS state and the INS / Baro / Compass / GPS
  frontends.

  Recommended usage (Plane / Copter / Quadplane):

      param set AHRS_EKF_TYPE     11
      param set EAHRS_TYPE        10
      param set EAHRS_RATE        <SENS_PARA rate, e.g. 200>
      param set SERIAL*_PROTOCOL  36
      param set SERIAL*_BAUD      921
      param set GPS1_TYPE	      21

  EAHRS_RATE must match the actual SENS_PARA rate, and SENS_PARA rate
  must be at least the vehicle's SCHED_LOOP_RATE (400 Hz copter,
  300 Hz VTOL, 50 Hz plane) when using IMU data.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_AERON_PLX_ENABLED

#include "AP_ExternalAHRS_backend.h"

// packet identifiers (PRP_ID field, big-endian on the wire)
enum class AeronPacketID : uint16_t {
    NAV_PARA1       = 0xAAA1,
    NAV_PARA2       = 0xA2A2,
    SENS_PARA       = 0xA2A3,
    GPS_PARA        = 0xA2A4,
    DEV_INFO        = 0xA2A6,
    EXTD_GNSS       = 0xA2A8,
    HEADING_S       = 0x0165,
    H_SPEED_S       = 0x0166,
    V_SPEED_S       = 0x0167,
    POSITION_S      = 0x0168,
    ALTITUDE_S      = 0x0169,
    GNSS_PKT        = 0x0212,
    GNSS_FIX_STATUS = 0x0067,
    EXT_SENSORS_MAG = 0x0117,
    EXT_SENSORS_ASP = 0x0362,
};

enum class AeronPacketLen : uint16_t {
    LEN_NAV_PARA1 = 64,
    LEN_NAV_PARA2 = 72,
    LEN_SENS_PARA = 72,
    LEN_GPS_PARA  = 48,
    LEN_DEV_INFO  = 62,
    LEN_EXTD_GNSS = 40,
};

// GNSS fix types reported by PLX
enum class AeronGnssFix : uint8_t {
    NO_FIX    = 0,
    GNSS_FIX  = 1,
    SBAS_FIX  = 2,
    RTK_FIX   = 3,
    RTK_FLOAT = 4,
};

// jamming/spoofing detection levels
enum class AeronJamSpoof : uint8_t {
    NOT_DETECTED = 1,
    WARNING      = 2,
    CRITICAL     = 3,
};

// hardware status bit positions inside NAV_PARA2.hw_status
enum class AeronHwBit : uint8_t {
    GNSS     = 0,
    ACC      = 1,
    GYR      = 2,
    MAG      = 3,
    BARO     = 4,
    SUP_VLTG = 5,
    CM_PRT   = 10,
    RAM      = 11,
    FIRMWARE = 12,
    CONFIG   = 13,
    EXT_MAG  = 22,
    EXT_ASP  = 23,
};

// Local decode structures
struct AeronDecNavPara1 {
    uint32_t epoch_time;
    uint32_t microseconds;
    uint32_t ins_status;
    uint32_t hw_status;
    float    euler[3];
    float    course;
    float    velocity_ned[3];
    float    height_abv_ellip;
    double   position[2];
    bool     fresh;
};

struct AeronDecNavPara2 {
    uint32_t epoch_time;
    uint32_t microseconds;
    uint32_t ins_status;
    uint32_t hw_status;
    float    quat[4];
    float    body_vel[3];
    float    altitude;
    double   ecef_pos[3];
    bool     fresh;
};

struct AeronDecSens {
    uint32_t epoch_time;
    uint32_t microseconds;
    float    temperature;
    float    gyro[3];
    float    accel[3];
    float    mag[3];
    float    euler_rates[3];
    float    baro_pressure;
    float    baro_temperature;
    float    baro_altitude;
    bool     fresh;
};

struct AeronDecGps {
    uint32_t epoch_time;
    uint32_t microseconds;
    uint32_t gps_status;
    double   gps_position[2];
    float    gps_height_abv_ellip;
    float    gps_undulation;
    float    pdop;
    float    hdop;
    float    gdop;
    bool     fresh;
};

struct AeronDecCust {
    float    hpa;
    float    vpa;
    float    hva;
    float    vdop;
    double   gnss_vned[3];
    bool     fresh;
};

// deferred GCS messages - collected during decode, emitted once per
// drain so GCS_SEND_TEXT calls aren't sprinkled through the parser
struct AeronDeferredMsgs {
    bool     crc_fail;
    bool     garbage;
    bool     ext_mag_checked;
    bool     ext_asp_checked;
    uint16_t unknown_id;
};

class AP_ExternalAHRS_Aeron_plx : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_Aeron_plx(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // AP_ExternalAHRS_backend interface
    int8_t      get_port() const override;
    const char *get_name() const override;
    bool        healthy() const override;
    bool        initialised() const override;
    bool        pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void        get_filter_status(nav_filter_status &status) const override;
    bool        get_variances(float &velVar, float &posVar, float &hgtVar,
                              Vector3f &magVar, float &tasVar) const override;
    void        update() override { check_and_decode(); }

protected:
    uint8_t num_gps_sensors() const override
    {
        return 1;
    }

private:

    mutable HAL_Semaphore arn_sem;
    // hw_status bits we treat as errors. Built from AeronHwBit so the
    // mask, the case-statement coverage and the loop bound stay in sync.
    static constexpr uint32_t HW_ERROR_MASK =
        (1U << uint8_t(AeronHwBit::GNSS))     |
        (1U << uint8_t(AeronHwBit::ACC))      |
        (1U << uint8_t(AeronHwBit::GYR))      |
        (1U << uint8_t(AeronHwBit::MAG))      |
        (1U << uint8_t(AeronHwBit::BARO))     |
        (1U << uint8_t(AeronHwBit::SUP_VLTG)) |
        (1U << uint8_t(AeronHwBit::CM_PRT))   |
        (1U << uint8_t(AeronHwBit::RAM))      |
        (1U << uint8_t(AeronHwBit::FIRMWARE)) |
        (1U << uint8_t(AeronHwBit::CONFIG))   |
        (1U << uint8_t(AeronHwBit::EXT_MAG))  |
        (1U << uint8_t(AeronHwBit::EXT_ASP));

    // highest bit position we ever inspect, +1 - used as the loop bound
    static constexpr uint8_t HW_BIT_COUNT = uint8_t(AeronHwBit::EXT_ASP) + 1;

    void update_thread();
    bool check_and_decode();
    void report_hw_status();
    bool parse_byte(uint8_t byte);
    uint16_t decode_to_local(AeronDecNavPara1 *nav1,
                             AeronDecNavPara2 *nav2,
                             AeronDecSens *sens,
                             AeronDecGps *gps,
                             AeronDecCust *cust,
                             AeronDeferredMsgs *msgs);

    void publish_nav_para1(const AeronDecNavPara1 &data);
    void publish_nav_para2(const AeronDecNavPara2 &data);
    void publish_sens_para(const AeronDecSens &data);
    void publish_gps_para(const AeronDecGps &data);
    void publish_cust_pkt(const AeronDecCust &data);

    // GPS formatting - combines GPS_PARA + EXTD_GNSS + NAV_PARA1 fields
    void format_and_push_gps();

    // utility
    void send_deferred_messages(const AeronDeferredMsgs &msgs);
    void query_ext_sensors();

    // UART interface (read-only after construction)
    AP_HAL::UARTDriver *uart;
    uint32_t            baudrate;
    int8_t              port_num;
    bool                setup_complete;

    // Shared sensor data
    struct SharedData {
        // NAV_PARA1
        float    nav1_height_m;
        double   nav1_pos_deg[2];
        // NAV_PARA2
        uint32_t nav2_hw_status;
        // GPS_PARA
        uint32_t gps_epoch_s;
        uint32_t gps_status;
        uint32_t gps_microseconds;
        float    gps_undulation_m;
        float    gps_hdop;
        // EXTD_GNSS
        float    cust_hpa_m;
        float    cust_vpa_m;
        float    cust_hva_mps;
        float    cust_vdop;
        double   cust_gnss_vel_ned_mps[3];
        // external sensor presence / response-received flags
        uint8_t  ext_mag_present;
        uint8_t  ext_asp_present;
        bool     ext_mag_checked;
        bool     ext_asp_checked;
    };

    SharedData shared;

    // parser state
    enum class ParseState : uint8_t {
        SYNC = 0,
        LEN_HIGH,
        LEN_LOW,
        PAYLOAD,
        CRC,
    };

    ParseState parse_state;
    uint8_t    sync_count;
    uint16_t   pkt_length;
    uint16_t   write_idx;
    uint16_t   decoded_pkt_len;
    uint8_t    rx_buf[512];
    uint8_t    decode_buf[512];
    uint8_t    chunk_buf[512];

    // Per-packet-group staleness timestamps.
    volatile uint32_t last_sens_ms;
    volatile uint32_t last_nav_ms;
    volatile uint32_t last_gnss_ms;

    // GNSS state
    bool     gnss_fix;
    uint8_t  jam_status;
    uint8_t  spoof_status;
    uint32_t last_jam_warn_ms;

    // one-time init flags
    bool ext_sensor_query_done;
    bool crc_fail_pending;
    bool nav1_valid;          // gates the GPS message construction
    bool has_variance_data;   // gates get_variances() reporting

    // throttled-warning state for hw_status bits. Rising edges fire
    // immediately; persistent faults re-warn every HEALTH_REPEAT_INTERVAL_MS.
    uint32_t last_warned_bits;
    uint32_t last_warned_ms[32];

    // rate diagnostics
    uint32_t sens_count;
    uint32_t nav1_count;
    uint32_t nav2_count;
    uint32_t gps_count;
    uint32_t extd_gnss_count;
    uint32_t last_rate_ms;

    static constexpr uint32_t RATE_LOG_INTERVAL_MS = 60000;

    // Per-group staleness deadlines for healthy().
    static constexpr uint32_t SENS_TIMEOUT_MS = 40;
    static constexpr uint32_t NAV_TIMEOUT_MS  = 200;
    static constexpr uint32_t GNSS_TIMEOUT_MS = 500;

    static constexpr uint32_t HEALTH_REPEAT_INTERVAL_MS = 10000;

    static constexpr uint8_t  SYNC_BYTE                 = 0x05;
    static constexpr uint8_t  SYNC_REQUIRED             = 4;
    static constexpr uint16_t PKT_LEN_MIN               = 8;
    static constexpr uint16_t PKT_LEN_MAX               = 255;

    // Unix epoch -> GPS epoch (Jan 6 1980) offset
    static constexpr uint32_t GPS_UNIX_OFFSET_S = 315964800;
    static constexpr uint32_t SECONDS_PER_WEEK  = 604800;
};

#endif  // AP_EXTERNAL_AHRS_AERON_PLX_ENABLED
