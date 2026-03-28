#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>
#include <cstring>

#define AP_DOPPLER_BAUD 115200
#define AP_DOPPLER_BUFSIZE_RX 512
#define AP_DOPPLER_BUFSIZE_TX 512
#define AP_DOPPLER_START_BYTE ':'
#define AP_DOPPLER_CMD_LENGTH 16
#define AP_DOPPLER_SET_EPD6   "DF 3\r000000000000"
#define AP_DOPPLER_LAUNCH     "CS \r000000000000"

enum class DVL_LockState : uint8_t {
    NO_LOCK = 0,
    BOTTOM_LOCK = 1,
    WATER_TRACK = 2,
};

struct EPD6VelocitySample {
    Vector3f vel_body_mps {};
    uint32_t update_ms = 0;
    float quality = 0.0f;
    DVL_LockState lock = DVL_LockState::NO_LOCK;
    bool valid = false;
};

struct DVL_BI_Msg {
    uint64_t time_usec = 0;
    uint32_t sequence = 0;
    bool valid = false;
    uint8_t status = 0;
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;
    float vz_mps = 0.0f;
    float vel_error_mps = 0.0f;
};

struct DVL_BD_Msg {
    uint64_t time_usec = 0;
    uint32_t sequence = 0;
    bool valid = false;
    uint8_t status = 0;
    float east_m = 0.0f;
    float north_m = 0.0f;
    float up_m = 0.0f;
    float bottom_m = 0.0f;
    float time_since_valid_s = 0.0f;
};

struct DVL_WI_Msg {
    uint64_t time_usec = 0;
    uint32_t sequence = 0;
    bool valid = false;
    uint8_t status = 0;
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;
    float vz_mps = 0.0f;
    float vel_error_mps = 0.0f;
};

struct DVL_U_Msg {
    uint64_t time_usec = 0;
    uint32_t sequence = 0;
    bool valid = false;
    uint8_t status = 0;
    float velocity_mps = 0.0f;
    float distance_m = 0.0f;
    float rssi = 0.0f;
    float nsd = 0.0f;
};

class AP_Doppler_Backend
{
public:
    AP_Doppler_Backend(AP_HAL::UARTDriver *port) :
        _port(port) { }

    virtual ~AP_Doppler_Backend() {}

    virtual bool init();
    virtual void send();
    virtual void loop();
    bool get_velocity_body(Vector3f &vel_body_mps, uint32_t &t_ms, float &quality, DVL_LockState &lock) const;
    bool get_bi_msg(DVL_BI_Msg &msg) const;
    bool get_bd_msg(DVL_BD_Msg &msg) const;
    bool get_wi_msg(DVL_WI_Msg &msg) const;
    bool get_ua_msg(DVL_U_Msg &msg) const;
    bool get_ub_msg(DVL_U_Msg &msg) const;
    bool get_uc_msg(DVL_U_Msg &msg) const;
    bool get_ud_msg(DVL_U_Msg &msg) const;
    typedef union {
        struct PACKED {
            uint8_t sensor;
            uint8_t frame;
            uint16_t appid;
            uint32_t data;
        };
        uint8_t raw[8];
    } sport_packet_t;

    virtual uint32_t initial_baud() const
    {
        return AP_DOPPLER_BAUD;
    }

protected:
    AP_HAL::UARTDriver *_port;

    virtual bool init_serial_port();

    enum EPD6_Status : char {
        EPD6_STATUS_INVALID = 'V',
        EPD6_STATUS_ACQUIRING = 'A',
        EPD6_ZERO_ERROR = '0',
        EPD6_ONE_ERROR = '1',
        EPD6_TWO_ERROR = '2',
        EPD6_THREE_ERROR = '3',
        EPD6_FOUR_ERROR = '4',
    };

    struct EPD6BeamSample {
        float velocity_mm_s = 0.0f;
        float distance_m = 0.0f;
        float rssi = 0.0f;
        float nsd = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
        uint32_t update_ms = 0;
    };

    struct EPD6TDData {
        uint32_t period_ms = 0;
        uint64_t echo_unix_ms = 0;
        uint64_t report_unix_ms = 0;
        bool valid = false;
        uint32_t update_ms = 0;
    };

    struct EPD6BEData {
        float east_velocity_mm_s = 0.0f;
        float north_velocity_mm_s = 0.0f;
        float up_velocity_mm_s = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
    } _epd6_be;

    struct EPD6SAData {
        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
    } _epd6_sa;

    struct EPD6TSData {
        char time[17] {};
        float satellites_PPT = 0.0f;
        float temperature_C = 0.0f;
        float deep_m = 0.0f;
        float sound_speed_m_s = 0.0f;
        uint8_t fault_code = 0;
        char protocol_version[3] {};
    } _epd6_ts;

    struct EPD6BIData {
        float X_velocity_mm_s = 0.0f;
        float Y_velocity_mm_s = 0.0f;
        float Z_velocity_mm_s = 0.0f;
        float velocity_error_mm_s = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
    } _epd6_bi;

    struct EPD6BSData {
        float x_velocity_mm_s = 0.0f;
        float y_velocity_mm_s = 0.0f;
        float z_velocity_mm_s = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
    } _epd6_bs;

    struct EPD6BDData {
        float east_distance_m = 0.0f;
        float north_distance_m = 0.0f;
        float up_distance_m = 0.0f;
        float bottom_distance_m = 0.0f;
        float time_since_valid_s = 0.0f;
    } _epd6_bd;

    struct EPD6WIData {
        float x_velocity_mm_s = 0.0f;
        float y_velocity_mm_s = 0.0f;
        float z_velocity_mm_s = 0.0f;
        float velocity_error_mm_s = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
    } _epd6_wi;

    struct EPD6WSData {
        float x_velocity_mm_s = 0.0f;
        float y_velocity_mm_s = 0.0f;
        float z_velocity_mm_s = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
    } _epd6_ws;

    struct EPD6WEData {
        float east_velocity_mm_s = 0.0f;
        float north_velocity_mm_s = 0.0f;
        float up_velocity_mm_s = 0.0f;
        EPD6_Status status = EPD6_STATUS_INVALID;
    } _epd6_we;

    struct EPD6WDData {
        float east_distance_m = 0.0f;
        float north_distance_m = 0.0f;
        float up_distance_m = 0.0f;
        float center_distance_m = 0.0f;
        float time_since_valid_s = 0.0f;
    } _epd6_wd;

    enum Doppler_options_e : uint8_t {
        OPTION_AIRSPEED_AND_GROUNDSPEED = 1U<<0,
    };

private:
    void parse_epd6_sa(const char *payload);
    void parse_epd6_ts(const char *payload);
    void parse_epd6_bi(const char *payload);
    void parse_epd6_bs(const char *payload);
    void parse_epd6_be(const char *payload);
    void parse_epd6_bd(const char *payload);
    void parse_epd6_wi(const char *payload);
    void parse_epd6_ws(const char *payload);
    void parse_epd6_we(const char *payload);
    void parse_epd6_wd(const char *payload);
    void parse_epd6_ua(const char *payload);
    void parse_epd6_ub(const char *payload);
    void parse_epd6_uc(const char *payload);
    void parse_epd6_ud(const char *payload);
    void parse_epd6_td(const char *payload);
    void send_epd6_startup_commands();
    void update_epd6_velocity_sample(EPD6VelocitySample &sample, float x_velocity_mm_s, float y_velocity_mm_s, float z_velocity_mm_s, DVL_LockState lock);
    void update_epd6_beam_sample(EPD6BeamSample &sample, float velocity_mm_s, float distance_m, float rssi, float nsd, EPD6_Status status);

    static constexpr uint32_t DVL_TIMEOUT_MS = 500;

    mutable HAL_Semaphore _sample_sem;
    EPD6VelocitySample _epd6_bottom_track_velocity_sample;
    EPD6VelocitySample _epd6_water_track_velocity_sample;
    EPD6BeamSample _epd6_beam_samples[4];
    EPD6TDData _epd6_td;
    DVL_BI_Msg bi_msg {};
    DVL_BD_Msg bd_msg {};
    DVL_WI_Msg wi_msg {};
    DVL_U_Msg ua_msg {};
    DVL_U_Msg ub_msg {};
    DVL_U_Msg uc_msg {};
    DVL_U_Msg ud_msg {};
};
