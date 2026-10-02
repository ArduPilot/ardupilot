#include "AP_ELRS_config.h"
#include "elrs_tracker.h"
#include <string.h>

#if AP_ELRS_ENABLED

namespace ELRS
{

const Tracker::Rate &Tracker::rate() const
{
    return rate_at(_rate_index);
}

const Tracker::Rate &Tracker::rate_at(uint8_t index)
{
    // Complete official SX1280 table, searched from the longest-range LoRa profile.
    // Coding-rate values are SX1280 register encodings; scan dwell follows upstream.
    static const Rate rates[RATE_COUNT] {
        {21, STANDARD_SIZE, 8, 2, 20000, 4000000, 3520000, 10798, 7, 12, 1, false},
        {23, FULL_SIZE,     7, 4, 10000, 3500000, 3520000,  7605, 7, 12, 1, false},
        {24, STANDARD_SIZE, 7, 4,  6666, 3500000, 2346000,  5871, 7, 12, 1, false},
        {27, STANDARD_SIZE, 6, 4,  4000, 3000000, 1408000,  3300, 7, 14, 1, false},
        {28, FULL_SIZE,     5, 4,  3003, 2500000, 1057000,  2374, 7, 12, 1, false},
        {29, STANDARD_SIZE, 5, 4,  2000, 2500000,  704000,  1507, 6, 12, 1, false},
        {32, STANDARD_SIZE, 0, 2,  2000, 2500000,  352000,   389, 0, 32, 1, true},
        {33, STANDARD_SIZE, 0, 2,  1000, 2500000,  176000,   389, 0, 32, 1, true},
        {31, STANDARD_SIZE, 0, 2,  1000, 2500000,  176000,   389, 0, 32, 2, true},
        {30, STANDARD_SIZE, 0, 2,  1000, 2500000,  176000,   389, 0, 32, 4, true},
    };
    return rates[index < RATE_COUNT ? index : 0];
}


bool Tracker::start(const uint8_t uid[UID_LENGTH], uint8_t model_id)
{
    stop();
    _rate_index = 0;
    _stats = {};
    if (uid == nullptr || !uid_is_bound(uid)) {
        return false;
    }
    memcpy(_uid, uid, sizeof(_uid));
    _model_id = model_id;
    _init = crc_initializer(uid);
    _fhss.randomise(uid_seed(uid));
    _state = State::SEARCHING;
    return true;
}

void Tracker::stop()
{
    _state = State::DISABLED;
    _scan_started = false;
    _frame_seen = 0;
    _pending_full_channels = 0;
    _group_rc = _group_uplink = _group_good = false;
    _channels = {};
    _uplink.reset();
    _seen_channels = 0;
    _received_in_slot = false;
    _nonce = 0;
    _telemetry_denom = 1;
    _telemetry_pending = false;
    memset(_lq_history, 0, sizeof(_lq_history));
    _lq_position = _lq_count = _lq_good = 0;
    _fhss.set_index(0);
}

void Tracker::lose()
{
    if (_state == State::DISABLED) {
        return;
    }
    if (_state == State::TRACKING) {
        _stats.lost++;
    }
    stop();
    _state = State::SEARCHING;
}

bool Tracker::read_primary_channels(uint32_t now_us, uint16_t values[4]) const
{
    if (_state != State::TRACKING || (_frame_seen & 0x000F) != 0x000F ||
        now_us - _frame_rc_us > output_max_age_us() ||
        (full16() && now_us - last_input_us() > TIMEOUT_US)) {
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) {
        // Match AP_RCProtocol_CRSF's channel scaling to microseconds.
        values[i] = uint32_t(_frame_channels.crsf[i]) * 5U / 8U + 880U;
    }
    return true;
}

uint32_t Tracker::last_input_us() const
{
    if (!full16()) {
        return _frame_rc_us;
    }
    // Both halves must be new for each Full16 publication. Anchor loss timing
    // to the older half, including when the microsecond clock wraps.
    const uint32_t low_age = _frame_rc_us - _frame_updated_us[0];
    const uint32_t high_age = _frame_rc_us - _frame_updated_us[8];
    return low_age > high_age ? _frame_updated_us[0] : _frame_updated_us[8];
}

uint8_t Tracker::read_channels(uint32_t now_us, uint16_t values[OUTPUT_CHANNELS], uint32_t &valid_for_us) const
{
    valid_for_us = 0;
    const uint8_t count = rate().payload_size == STANDARD_SIZE ? 12 :
                          (_mode == SwitchMode::HYBRID ? 16 : ((_frame_seen & 0x0F00) ? 12 : 8));
    const uint16_t required = uint16_t((1UL << count) - 1);
    if (_state != State::TRACKING || (_frame_seen & required) != required ||
        now_us - _frame_rc_us > output_max_age_us()) {
        return 0;
    }
    uint32_t remaining = output_max_age_us() - (now_us - _frame_rc_us);
    for (uint8_t i = 0; i < count; i++) {
        const uint32_t age = now_us - _frame_updated_us[i];
        // Cap multiplexed channel retention separately from the time allowed
        // to deliver a newly completed frame.
        if (age > TIMEOUT_US) {
            return 0;
        }
        const uint32_t channel_remaining = TIMEOUT_US - age;
        if (channel_remaining < remaining) {
            remaining = channel_remaining;
        }
    }
    for (uint8_t i = 0; i < count; i++) {
        values[i] = uint32_t(_frame_channels.crsf[i]) * 5U / 8U + 880U;
    }
    valid_for_us = remaining;
    return count;
}

void Tracker::publish_frame()
{
    if (full16()) {
        if (_pending_full_channels != 0xFFFF) {
            return;
        }
        _pending_full_channels = 0;
    }
    _frame_sequence++;
    _frame_channels = _channels;
    _frame_seen = _seen_channels;
    _frame_rc_us = _last_rc_us;
    memcpy(_frame_updated_us, _channel_updated_us, sizeof(_frame_updated_us));
}

void Tracker::expire_full_channels(uint32_t now_us)
{
    if (!full16()) {
        return;
    }
    // Expire pending halves while the clock advances, so an old half cannot
    // become apparently fresh again after a complete microsecond-clock wrap.
    if (now_us - _channel_updated_us[0] > TIMEOUT_US) {
        _pending_full_channels &= 0xFF00;
    }
    if (now_us - _channel_updated_us[8] > TIMEOUT_US) {
        _pending_full_channels &= 0x00FF;
    }
    if (now_us - _frame_rc_us > output_max_age_us()) {
        _frame_seen = 0;
    }
}

void Tracker::record_lq(bool good)
{
    _lq_good -= _lq_history[_lq_position];
    _lq_history[_lq_position] = good;
    _lq_good += good;
    _lq_position = (_lq_position + 1) % sizeof(_lq_history);
    if (_lq_count < sizeof(_lq_history)) {
        _lq_count++;
    }
}

uint32_t Tracker::frequency() const
{
    return _state == State::TRACKING ? _fhss.frequency() : FHSS::initial_frequency();
}

uint32_t Tracker::time_until_slot(uint32_t now_us) const
{
    if (_state != State::TRACKING) {
        return rate().interval_us;
    }
    const int32_t remaining = int32_t(_next_slot_us - now_us);
    return remaining > 0 ? uint32_t(remaining) : 0;
}

void Tracker::advance(uint32_t now_us)
{
    if (_state == State::SEARCHING) {
        if (!_scan_started) {
            _scan_started = true;
            _scan_started_us = now_us;
        } else if (now_us - _scan_started_us >= rate().scan_us) {
            _rate_index = (_rate_index + 1) % RATE_COUNT;
            _scan_started_us = now_us;
        }
        return;
    }
    if (_state != State::TRACKING) {
        return;
    }
    expire_full_channels(now_us);
    if (now_us - _last_valid_us > rate().timeout_us) {
        lose();
        return;
    }
    if (int32_t(now_us - _next_slot_us) < 0) {
        return;
    }
    const uint32_t lateness = now_us - _next_slot_us;
    if (lateness > _stats.max_slot_late_us) {
        _stats.max_slot_late_us = lateness;
    }
    if (lateness > service_limit_us()) {
        _stats.late++;
        lose();
        return;
    }
    // D modes report one opportunity and one frame per repetition group.
    _group_uplink |= !telemetry_slot();
    _group_good |= _received_in_slot && !telemetry_slot();
    if (rate().sends == 1 || _nonce % rate().sends == 0) {
        if (_group_uplink) {
            record_lq(_group_good);
        }
        if (rate().sends > 1 && _group_rc) {
            publish_frame();
        }
        _group_rc = _group_uplink = _group_good = false;
    }
    _next_slot_us += rate().interval_us;
    _nonce++;
    if (_nonce % rate().hop_interval == 0) {
        _fhss.next_frequency();
    }
    _received_in_slot = false;
    _telemetry_pending = telemetry_slot();
}

bool Tracker::take_telemetry(uint32_t now_us, uint8_t packet[STANDARD_SIZE], int8_t rssi, int8_t snr)
{
    if (!_telemetry_pending) {
        return false;
    }
    _telemetry_pending = false;
    // Never send outside the freshly opened slot or after stale uplink timing.
    if (_state != State::TRACKING || !telemetry_slot() ||
        now_us - (_next_slot_us - rate().interval_us) > service_limit_us() ||
        now_us - _last_valid_us > output_max_age_us()) {
        return false;
    }
    make_link_stats(packet, _init, _nonce, rssi, snr, link_quality(), _uplink.ack(), rate().payload_size);
    return true;
}

void Tracker::receive(const uint8_t *packet, uint32_t timestamp_us, uint32_t now_us)
{
    if (_state == State::DISABLED) {
        return;
    }
    const uint32_t service_us = now_us - timestamp_us;
    if (service_us > _stats.max_service_us) {
        _stats.max_service_us = service_us;
    }
    if (service_us > service_limit_us()) {
        _stats.late++;
        lose();
        return;
    }
    if (!validate_packet(packet, rate().payload_size, _init, _nonce)) {
        _stats.rejected++;
        return;
    }
    const auto type = PacketType(packet[0] & 3);
    if (type == PacketType::SYNC) {
        Sync sync;
        parse_sync(packet, rate().payload_size, sync);
        const uint8_t model_xor = (~_model_id) & 0x3F;
        // Validate identity and normal RC protocol before following a rate announcement.
        if (sync.gemini || sync.protocol != 0 ||
            sync.uid4 != _uid[4] || sync.uid5 != (_uid[5] ^ model_xor) ||
            sync.fhss_index >= FHSS::SEQUENCE_LENGTH) {
            _stats.rejected++;
            // A valid CRC with a new unsupported mode must invalidate old observations.
            lose();
            return;
        }
        uint8_t proposed_rate = 0;
        while (proposed_rate < RATE_COUNT && rate_at(proposed_rate).wire != sync.rate) {
            proposed_rate++;
        }
        if (proposed_rate == RATE_COUNT) {
            _stats.rejected++;
            lose();
            return;
        }
        if (proposed_rate != _rate_index) {
            const bool same_flrc = rate().flrc && rate_at(proposed_rate).flrc;
            if (!same_flrc) {
                // The TX announces a new profile on the old modulation first.
                // Reconfigure and reacquire instead of interpreting old timing as new.
                lose();
                _rate_index = proposed_rate;
                return;
            }
            // All four FLRC modes share a waveform; SYNC disambiguates their timing.
            _rate_index = proposed_rate;
            _channels = {};
            _seen_channels = _frame_seen = 0;
            _pending_full_channels = 0;
            _group_rc = _group_uplink = _group_good = false;
        }
        FHSS proposed = _fhss;
        proposed.set_index(sync.fhss_index);
        if (proposed.frequency() != frequency()) {
            _stats.rejected++;
            lose();
            return;
        }
        if (_state != State::TRACKING || _nonce != sync.nonce ||
            _fhss.index() != sync.fhss_index || _mode != sync.switch_mode) {
            _channels = {};
            _seen_channels = 0;
            _frame_seen = 0;
            _pending_full_channels = 0;
            _group_rc = _group_uplink = _group_good = false;
        } else if (_received_in_slot) {
            _stats.rejected++;
            return;
        }
        _fhss.set_index(sync.fhss_index);
        _nonce = sync.nonce;
        _mode = sync.switch_mode;
        _telemetry_denom = telemetry_denominator(sync.telemetry_ratio);
        _telemetry_pending = false;
        _state = State::TRACKING;
        _stats.sync++;
    } else {
        if (_state != State::TRACKING || telemetry_slot() || _received_in_slot ||
            now_us - _last_valid_us > rate().timeout_us) {
            _stats.rejected++;
            return;
        }
        // The predicted packet end precedes the boundary by this profile's slack.
        const int32_t phase_us = int32_t(timestamp_us - (_next_slot_us - rate().slack_us()));
        if (phase_us < -int32_t(service_limit_us()) || phase_us > int32_t(service_limit_us())) {
            _stats.late++;
            lose();
            return;
        }
        if (type == PacketType::DATA) {
            _stats.data++;
            if (_uplink.receive(rate().payload_size == FULL_SIZE ? packet[0] >> 3 : packet[1] & 0x7F)) {
                _stats.transfers++;
            }
        }
        if (type == PacketType::RC) {
            expire_full_channels(now_us);
            if (!decode_channels(packet, rate().payload_size, _init, _nonce, _mode, _channels)) {
                _stats.rejected++;
                return;
            }
            _seen_channels |= _channels.updated_mask;
            if (full16()) {
                _pending_full_channels |= _channels.updated_mask;
            }
            _stats.rc++;
            _last_rc_us = timestamp_us;
            for (uint8_t i = 0; i < OUTPUT_CHANNELS; i++) {
                if (_channels.updated_mask & (1U << i)) {
                    _channel_updated_us[i] = timestamp_us;
                }
            }
            if (rate().sends == 1) {
                publish_frame();
            } else {
                _group_rc = true;
            }
        }
    }
    _received_in_slot = true;
    _last_valid_us = timestamp_us;
    // Re-anchor each valid observation; this is not upstream frequency/phase lock.
    _next_slot_us = timestamp_us + rate().slack_us();
}

} // namespace ELRS

#endif // AP_ELRS_ENABLED
