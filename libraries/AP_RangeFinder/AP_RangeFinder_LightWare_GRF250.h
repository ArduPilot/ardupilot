#pragma once

#include <AP_RangeFinder/AP_RangeFinder_Backend_Serial.h>

#ifndef AP_RANGEFINDER_GRF250_ENABLED
#define AP_RANGEFINDER_GRF250_ENABLED ENABLED
#endif

#define GRF250_START_BYTE 0xAA
#define GRF250_MAX_PAYLOAD 32
#define GRF250_BUFFER_SIZE 1280

class AP_RangeFinder_GRF250 : public AP_RangeFinder_Backend_Serial {
public:
    static AP_RangeFinder_Backend_Serial *create(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_GRF250(_state, _params);
    }

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }
    bool get_reading(float &reading_m) override;
    uint16_t read_timeout_ms() const override { return 500; }
    float max_distance() const override { return MIN(AP_RangeFinder_Backend::max_distance(), 250.0f); }
    float min_distance() const override { return MAX(AP_RangeFinder_Backend::min_distance(), 0.2f); }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    enum InitStage {
        INIT_STAGE_IDLE,
        INIT_STAGE_SENT_DUMMY,
        INIT_STAGE_SENT_HANDSHAKE1,
        INIT_STAGE_SENT_HANDSHAKE2,
        INIT_STAGE_WAITING_RESPONSE,
        INIT_STAGE_STREAMING
    };

    struct {
        uint8_t parse_buffer[GRF250_BUFFER_SIZE];
        uint16_t parse_ofs = 0;
        InitStage init_stage = INIT_STAGE_IDLE;
        uint32_t last_init_ms = 0;
        uint32_t init_started_ms = 0;
        bool product_verified = false;
    } grf;

    void update_init();
    void move_preamble();
    bool check_grf250_response();
    bool try_parse_stream_packet(float &reading_m);

    uint16_t calculate_crc(const uint8_t *data, uint16_t length);
    uint16_t build_packet(uint8_t cmd_id, bool is_write, const uint8_t *payload, uint16_t payload_len, uint8_t *out_buf);

    bool parse_packet(const uint8_t *packet, uint16_t packet_len, uint8_t &cmd_id, uint8_t *payload_buf, uint16_t &payload_len);
    bool parse_from_buffer(uint8_t *buf, uint32_t len, uint8_t &cmd_id, uint8_t *payload, uint16_t &payload_len, uint16_t &consumed);
};
