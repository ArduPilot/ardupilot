#pragma once

#include "elrs_protocol.h"

namespace ELRS
{

class Tracker
{
public:
    static constexpr uint8_t RATE = 21; // RATE_LORA_2G4_50HZ
    static constexpr uint32_t INTERVAL_US = 20000;
    static constexpr uint32_t SLACK_US = 200; // max(interval - 2 * 10798, 200)
    static constexpr uint32_t SERVICE_LIMIT_US = 2000; // diagnostic service/phase bound
    static constexpr uint32_t TIMEOUT_US = 4000000; // upstream 50 Hz disconnect timeout

    enum class State : uint8_t { DISABLED, SEARCHING, TRACKING };
    struct Stats {
        uint32_t sync = 0;
        uint32_t rc = 0;
        uint32_t data = 0;
        uint32_t transfers = 0;
        uint32_t rejected = 0;
        uint32_t late = 0;
        uint32_t lost = 0;
        uint32_t max_service_us = 0;
        uint32_t max_slot_late_us = 0;
    };

    static constexpr uint8_t RATE_COUNT = 10;
    struct Rate {
        uint8_t wire;
        uint8_t payload_size;
        uint8_t spreading_factor;
        uint8_t hop_interval;
        uint32_t interval_us;
        uint32_t timeout_us;
        uint32_t scan_us;
        uint16_t airtime_us;
        uint8_t coding_rate;
        uint8_t preamble;
        uint8_t sends;
        bool flrc;
        uint32_t slack_us() const
        {
            const int32_t slack = int32_t(interval_us) - 2 * airtime_us;
            return slack > 200 ? uint32_t(slack) : 200U;
        }
    };
    const Rate &rate() const;
    static const Rate &rate_at(uint8_t index);
    uint32_t frame_sequence() const
    {
        return _frame_sequence;
    }
    uint32_t service_limit_us() const
    {
        return rate().interval_us / 2 < SERVICE_LIMIT_US ? rate().interval_us / 2 : SERVICE_LIMIT_US;
    }
    uint8_t rate_index() const
    {
        return _rate_index;
    }
    uint32_t output_max_age_us() const
    {
        return 2 * rate().interval_us * rate().sends;
    }
    bool start(const uint8_t uid[UID_LENGTH], uint8_t model_id);
    void stop();
    void receive(const uint8_t *packet, uint32_t timestamp_us, uint32_t now_us);
    void advance(uint32_t now_us);
    uint32_t time_until_slot(uint32_t now_us) const;
    void lose();
    State state() const
    {
        return _state;
    }
    uint32_t frequency() const;
    uint8_t nonce() const
    {
        return _nonce;
    }
    uint8_t hop_index() const
    {
        return _fhss.index();
    }
    uint16_t seen_channels() const
    {
        return _seen_channels;
    }
    const Stats &stats() const
    {
        return _stats;
    }
    const Channels &channels() const
    {
        return _channels;
    }
    static constexpr uint32_t OUTPUT_MAX_AGE_US = 2 * INTERVAL_US;
    bool read_primary_channels(uint32_t now_us, uint16_t values[4]) const;
    uint32_t last_rc_us() const
    {
        return _frame_rc_us;
    }
    uint32_t last_input_us() const;
    static constexpr uint8_t OUTPUT_CHANNELS = 16; // maximum TX channels
    uint8_t read_channels(uint32_t now_us, uint16_t values[OUTPUT_CHANNELS], uint32_t &valid_for_us) const;
    bool take_telemetry(uint32_t now_us, uint8_t packet[STANDARD_SIZE], int8_t rssi, int8_t snr);
    uint8_t telemetry_denom() const
    {
        return _telemetry_denom;
    }
    uint8_t link_quality() const
    {
        return _lq_count == 0 ? 0 : uint16_t(_lq_good) * 100U / _lq_count;
    }
    SwitchMode switch_mode() const
    {
        return _mode;
    }

private:
    uint32_t _frame_sequence = 0;
    uint32_t _frame_rc_us = 0;
    uint16_t _frame_seen = 0;
    uint16_t _pending_full_channels = 0;
    Channels _frame_channels;
    uint32_t _frame_updated_us[OUTPUT_CHANNELS] {};
    bool _group_rc = false;
    bool _group_uplink = false;
    bool _group_good = false;
    void publish_frame();
    void expire_full_channels(uint32_t now_us);
    bool full16() const
    {
        return rate().payload_size == FULL_SIZE && _mode == SwitchMode::HYBRID;
    }
    void record_lq(bool good);
    uint8_t _rate_index = 0;
    uint32_t _scan_started_us = 0;
    bool _scan_started = false;
    State _state = State::DISABLED;
    FHSS _fhss;
    Channels _channels;
    UplinkSink _uplink;
    Stats _stats;
    uint8_t _uid[UID_LENGTH] {};
    uint8_t _model_id = 255;
    uint16_t _init = 0;
    uint16_t _seen_channels = 0;
    uint8_t _nonce = 0;
    SwitchMode _mode = SwitchMode::WIDE;
    uint32_t _next_slot_us = 0;
    uint32_t _last_valid_us = 0;
    uint32_t _last_rc_us = 0;
    uint32_t _channel_updated_us[OUTPUT_CHANNELS] {};
    uint8_t _telemetry_denom = 1;
    bool _telemetry_pending = false;
    bool _lq_history[100] {};
    uint8_t _lq_position = 0;
    uint8_t _lq_count = 0;
    uint8_t _lq_good = 0;
    bool telemetry_slot() const
    {
        return _telemetry_denom > 1 && _nonce % _telemetry_denom == 0;
    }
    bool _received_in_slot = false;
};

// Main-thread publication state. Empty FAILSAFE reports never carry old channels.
class InputStatus
{
public:
    void valid(uint32_t timestamp_us)
    {
        _last_valid_us = timestamp_us;
        _seen_valid = true;
        _sent_failsafe = false;
    }
    bool failsafe_due(uint32_t now_us)
    {
        if (!_seen_valid || (!_sent_failsafe && now_us - _last_valid_us <= Tracker::TIMEOUT_US)) {
            return false;
        }
        if (_sent_failsafe && now_us - _last_report_us < 500000U) {
            return false;
        }
        _sent_failsafe = true;
        _last_report_us = now_us;
        return true;
    }
private:
    uint32_t _last_valid_us = 0;
    uint32_t _last_report_us = 0;
    bool _seen_valid = false;
    bool _sent_failsafe = false;
};

} // namespace ELRS
