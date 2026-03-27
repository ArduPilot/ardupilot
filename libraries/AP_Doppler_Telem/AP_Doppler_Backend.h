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
#define AP_DOPPLER_LAUNCH      "CS \r000000000000"

enum class DVL_LockState : uint8_t {
    NO_LOCK = 0,
    BOTTOM_LOCK = 1,
    WATER_TRACK = 2,
};

struct DVLVelocitySample {
    Vector3f vel_body_mps {};
    uint32_t update_ms = 0;
    float quality = 0.0f;
    DVL_LockState lock = DVL_LockState::NO_LOCK;
    bool valid = false;
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

    enum Message_Status : char {
        STATUS_INVALID = 'V',
        STATUS_ACQUIRING = 'A',
        ZERO_ERROR = '0',
        ONE_ERROR = '1',
        TWO_ERROR = '2',
        THREE_ERROR = '3',
        FOUR_ERROR = '4',
    };

    struct DVLBeamSample {
        float velocity_mm_s = 0.0f;
        float distance_m = 0.0f;
        float rssi = 0.0f;
        float nsd = 0.0f;
        Message_Status status = STATUS_INVALID;
        uint32_t update_ms = 0;
    };

    struct DVLExtendedTimeSample {
        uint32_t period_ms = 0;
        uint64_t echo_unix_ms = 0;
        uint64_t report_unix_ms = 0;
        bool valid = false;
        uint32_t update_ms = 0;
    };

    struct {
        const char *FLAGS = "BE";
        float east_velocity_mm_s = 0.0f;
        float north_velocity_mm_s = 0.0f;
        float up_velocity_mm_s = 0.0f;
        Message_Status status = STATUS_INVALID;
    } BottomTrackEarthVel;

    struct {
        const char *FLAGS = "SA";
        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
    } Posture_data;

    struct {
        const char *FLAGS = "TS";
        char time[20] {};
        float satellites_PPT = 0.0f;
        float temperature_C = 0.0f;
        float deep_m = 0.0f;
        float voltage_m_s = 0.0f;
        Message_Status status = STATUS_INVALID;
        uint8_t version = 0;
    } Parameters_data;

    struct {
        const char *FLAGS = "BI";
        float X_velocity_m_s = 0.0f;
        float Y_velocity_m_s = 0.0f;
        float Z_velocity_m_s = 0.0f;
        float velocity_error_mm_s = 0.0f;
        Message_Status status = STATUS_INVALID;
    } velocity_data;

    struct {
        const char *FLAGS = "BS";
        float x_velocity_mm_s = 0.0f;
        float y_velocity_mm_s = 0.0f;
        float z_velocity_mm_s = 0.0f;
        Message_Status status = STATUS_INVALID;
    } BottomTrackShipVel;

    struct {
        const char *FLAGS = "BD";
        float east_distance_m = 0.0f;
        float north_distance_m = 0.0f;
        float up_distance_m = 0.0f;
        float bottom_distance_m = 0.0f;
        float time_since_valid_s = 0.0f;
    } BottomTrackDistance;

    struct {
        const char *FLAGS = "WI";
        float x_velocity_mm_s = 0.0f;
        float y_velocity_mm_s = 0.0f;
        float z_velocity_mm_s = 0.0f;
        float velocity_error_mm_s = 0.0f;
        Message_Status status = STATUS_INVALID;
    } WaterTrackInstrumentVel;

    struct {
        const char *FLAGS = "WS";
        float x_velocity_mm_s = 0.0f;
        float y_velocity_mm_s = 0.0f;
        float z_velocity_mm_s = 0.0f;
        Message_Status status = STATUS_INVALID;
    } WaterTrackShipVel;

    struct {
        const char *FLAGS = "WE";
        float east_velocity_mm_s = 0.0f;
        float north_velocity_mm_s = 0.0f;
        float up_velocity_mm_s = 0.0f;
        Message_Status status = STATUS_INVALID;
    } WaterTrackEarthVel;

    struct {
        const char *FLAGS = "WD";
        float east_distance_m = 0.0f;
        float north_distance_m = 0.0f;
        float up_distance_m = 0.0f;
        float center_distance_m = 0.0f;
        float time_since_valid_s = 0.0f;
    } WaterTrackEarthDist;

    enum Doppler_options_e : uint8_t {
        OPTION_AIRSPEED_AND_GROUNDSPEED = 1U<<0,
    };

private:
    void parse_SA(const char *payload);
    void parse_TS(const char *payload);
    void parse_BI(const char *payload);
    void parse_BS(const char *payload);
    void parse_BE(const char *payload);
    void parse_BD(const char *payload);
    void parse_WI(const char *payload);
    void parse_WS(const char *payload);
    void parse_WE(const char *payload);
    void parse_WD(const char *payload);
    void parse_UA(const char *payload);
    void parse_UB(const char *payload);
    void parse_UC(const char *payload);
    void parse_UD(const char *payload);
    void parse_TD(const char *payload);
    void update_velocity_sample(DVLVelocitySample &sample, float x_velocity_mm_s, float y_velocity_mm_s, float z_velocity_mm_s, DVL_LockState lock);
    void update_beam_sample(DVLBeamSample &sample, float velocity_mm_s, float distance_m, float rssi, float nsd, Message_Status status);

    static constexpr uint32_t DVL_TIMEOUT_MS = 500;

    mutable HAL_Semaphore _sample_sem;
    DVLVelocitySample _bottom_track_sample;
    DVLVelocitySample _water_track_sample;
    DVLBeamSample _beam_samples[4];
    DVLExtendedTimeSample _extended_time_sample;
};
