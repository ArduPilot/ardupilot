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
  UART, runs them through a state-machine parser, then dispatches each
  CRC-valid frame to a publisher that re-casts the payload as a packed
  on-wire struct and pushes the values into the AP_ExternalAHRS state
  plus the INS / Baro / Compass / GPS frontends.

  Recommended usage (Plane / Copter / Quadplane):

      param set AHRS_EKF_TYPE     11
      param set EAHRS_TYPE        10
      param set EAHRS_RATE        <SENS_PARA rate, e.g. 200>
      param set SERIAL*_PROTOCOL  36
      param set SERIAL*_BAUD      921
      param set GPS1_TYPE         21

  EAHRS_RATE must match the actual SENS_PARA rate, and SENS_PARA rate
  must be at least the vehicle's SCHED_LOOP_RATE (400 Hz copter,
  300 Hz VTOL, 50 Hz plane) when using IMU data.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_AERON_PLX_ENABLED

#include "AP_ExternalAHRS_backend.h"

// Deferred GCS messages - collected during one drain pass and emitted
// once at the end, so GCS_SEND_TEXT() calls don't get sprinkled through
// the byte-by-byte parser loop.
struct AeronDeferredMsgs {
    bool     crc_fail;
    bool     garbage;
    bool     ext_mag_checked;
    bool     ext_asp_checked;
    bool     stale_data;
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
    void        update() override {}    // all work happens in update_thread()

protected:
    uint8_t num_gps_sensors() const override
    {
        return 1;
    }

private:

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

    // GNSS fix types reported by PLX (raw value field inside gps_status)
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

    // hardware status flag positions inside NAV_PARA2.hw_status
    enum class AeronHwStatus : uint8_t {
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

    // ---- packed on-wire payload layouts ----
    // These are cast directly over rx_buf[8 .. payload_end] once the CRC
    // has validated the frame, so the layout must exactly match the
    // PLX3 wire format. The matching SIM_Aeron structs carry the same
    // static_asserts.

    struct PACKED NavPara1Payload {
        uint32_t epoch_time;
        uint32_t microseconds;
        uint32_t ins_status;
        uint32_t hw_status;
        float    euler[3];
        float    course;
        float    velocity_ned[3];
        double   position[2];
        float    height_abv_ellip;
    };
    static_assert(sizeof(NavPara1Payload) == 64, "NavPara1Payload size mismatch");

    struct PACKED NavPara2Payload {
        uint32_t epoch_time;
        uint32_t microseconds;
        uint32_t ins_status;
        uint32_t hw_status;
        float    quat[4];
        float    body_vel[3];
        double   ecef_pos[3];
        float    altitude;
    };
    static_assert(sizeof(NavPara2Payload) == 72, "NavPara2Payload size mismatch");

    struct PACKED SensParaPayload {
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
    };
    static_assert(sizeof(SensParaPayload) == 72, "SensParaPayload size mismatch");

    struct PACKED GpsParaPayload {
        uint32_t epoch_time;
        uint32_t microseconds;
        uint32_t gps_status;
        double   gps_position[2];
        float    gps_height_abv_ellip;
        float    gps_undulation;
        float    pdop;
        float    hdop;
        float    gdop;
    };
    static_assert(sizeof(GpsParaPayload) == 48, "GpsParaPayload size mismatch");

    struct PACKED ExtdGnssPayload {
        float    hpa;
        float    vpa;
        float    hva;
        float    vdop;
        double   gnss_vned[3];
    };
    static_assert(sizeof(ExtdGnssPayload) == 40, "ExtdGnssPayload size mismatch");

    // ---- hw_status reporting table ----
    // One row per status flag we track. Used by report_hw_status() to
    // emit both the fault and the matching "okay" message, so the value
    // -> name mapping lives in exactly one place.
    struct HwStatusEntry {
        AeronHwStatus  status;
        uint8_t        severity;    // a MAV_SEVERITY_* value
        const char    *fault_msg;
        const char    *clear_msg;   // nullptr -> never emit a recovery message
    };
    static const HwStatusEntry hw_status_table[];

    // Protects every field of `shared` (the cross-thread cache written by
    // the parser thread's publish_* handlers and read by the const
    // accessors get_filter_status()/report_hw_status() on other threads).
    // The frontend `state` struct has its own lock (state.sem, owned by
    // the base class); the two are always taken sequentially, never
    // nested, so there is no lock-ordering hazard.
    mutable HAL_Semaphore arn_sem;

    void update_thread();
    void check_and_decode();
    void report_hw_status();
    void parse_byte(uint8_t byte, AeronDeferredMsgs &msgs);
    void handle_chunk_buf_packet(uint16_t prp_id, const uint8_t *payload,
                                 uint16_t payload_len, AeronDeferredMsgs &msgs);

    void publish_nav_para1(const NavPara1Payload &p);
    void publish_nav_para2(const NavPara2Payload &p);
    void publish_sens_para(const SensParaPayload &p);
    void publish_gps_para(const GpsParaPayload &p, AeronDeferredMsgs &msgs);
    void publish_extd_gnss(const ExtdGnssPayload &p);

    // GPS formatting - combines GPS_PARA + EXTD_GNSS + NAV_PARA1 fields
    void format_and_push_gps(AeronDeferredMsgs &msgs);

    // utility
    void send_deferred_messages(const AeronDeferredMsgs &msgs);
    void query_ext_sensors();
    void check_for_query_ext_sensors();

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
        RESET = 0,
        SYNC,
        LEN_HIGH,
        LEN_LOW,
        PAYLOAD,
        CRC_BYTE,
    };

    ParseState parse_state;
    uint8_t    sync_count;
    uint16_t   pkt_length;
    uint16_t   write_idx;
    // A frame is bounded by the LEN sanity check to pkt_length <=
    // PKT_LEN_MAX (255) plus one CRC byte, so the largest index written
    // is 255 - rx_buf[256] holds any valid frame exactly.
    uint8_t    rx_buf[256];

    // Per-packet-group staleness timestamps.
    volatile uint32_t last_sens_ms;
    volatile uint32_t last_nav1_ms;
    volatile uint32_t last_nav2_ms;
    volatile uint32_t last_gnss_ms;
    volatile uint32_t last_extd_ms;

    // GNSS state derived on demand from shared.gps_status (fix bit),
    // not stored - see get_filter_status().
    uint32_t last_jam_warn_ms;

    // init / gating flags
    bool ext_sensor_query_done;
    bool nav1_valid;          // gates the GPS message construction
    bool has_variance_data;   // gates get_variances() reporting
    uint32_t prev_nav_ms;     // last NAV timestamp seen by check_for_query_ext_sensors

    // throttled-warning state for hw_status flags. Rising edges fire
    // immediately; persistent faults re-warn every HEALTH_REPEAT_INTERVAL_MS.
    // Indexed by hw_status BIT POSITION (AeronHwStatus value), not table
    // row - so it must span the highest bit we track (EXT_ASP = 23).
    static constexpr uint8_t HW_STATUS_BITS = uint8_t(AeronHwStatus::EXT_ASP) + 1;
    uint32_t last_warned_stat;
    uint32_t last_warned_ms[HW_STATUS_BITS];

    // last_warned_ms is indexed by hw_status bit position; EXT_ASP is the
    // highest bit tracked. If a higher bit is added to AeronHwStatus this
    // fires so the array gets resized.
    static_assert(uint8_t(AeronHwStatus::EXT_ASP) < HW_STATUS_BITS,
                  "last_warned_ms too small for highest hw_status bit");

    // Throttle state for the noisy parser-side GCS messages.
    uint32_t last_crc_warn_ms;
    uint32_t last_garbage_warn_ms;
    uint32_t last_stale_data_ms;

    // Per-message log decimation timestamps (high-rate messages only).
    uint32_t last_aern_log_ms;
    uint32_t last_aers_log_ms;

    // Per-group staleness deadlines for healthy().
    static constexpr uint32_t SENS_TIMEOUT_MS = 40;
    static constexpr uint32_t NAV_TIMEOUT_MS  = 200;
    static constexpr uint32_t GNSS_TIMEOUT_MS = 500;

    // EXTD_GNSS validity window. Past this, the GPS message is not
    // published at all (see format_and_push_gps) so no stale accuracy or
    // velocity reaches AP_GPS.
    static constexpr uint32_t EXTD_VALIDITY_MS = 500;

    // NAV gap beyond which the PLX is assumed to have reset (re-query
    // external sensors on resumption).
    static constexpr uint32_t DEVICE_RESET_GAP_MS = 2000;

    // Max Logging time gate, currently limited to 100Hz
    static constexpr uint32_t LOG_THROTTLE_MS = 10;

    // At how much interval in ms should we shout health messages?
    static constexpr uint32_t HEALTH_REPEAT_INTERVAL_MS = 10000;

    // Minimum spacing between repeated parser-side telemetry warnings.
    static constexpr uint32_t WARN_THROTTLE_MS = 5000;

    // State Machine gears
    static constexpr uint8_t  SYNC_BYTE                 = 0x05;
    static constexpr uint8_t  SYNC_REQUIRED             = 4;
    static constexpr uint16_t PKT_LEN_MIN               = 8;
    static constexpr uint16_t PKT_LEN_MAX               = 255;

    // GPS_PARA status word: bit 10 is the "GNSS fix present" flag.
    static constexpr uint32_t GPS_STATUS_FIX_BIT = 0x00000400U;
};

#endif  // AP_EXTERNAL_AHRS_AERON_PLX_ENABLED
