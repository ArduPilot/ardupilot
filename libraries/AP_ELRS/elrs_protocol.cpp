#include "AP_ELRS_config.h"
#include "elrs_protocol.h"

#if AP_ELRS_ENABLED

namespace ELRS
{

uint32_t uid_seed(const uint8_t uid[UID_LENGTH])
{
    return (uint32_t(uid[2]) << 24) | (uint32_t(uid[3]) << 16) |
           (uint32_t(uid[4]) << 8) | (uid[5] ^ OTA_VERSION);
}

uint16_t crc_initializer(const uint8_t uid[UID_LENGTH])
{
    return (uint16_t(uid[4]) << 8 | uid[5]) ^ (uint16_t(OTA_VERSION) << 8);
}

bool uid_is_bound(const uint8_t uid[UID_LENGTH])
{
    return uid[2] != 0 || uid[3] != 0 || uid[4] != 0 || uid[5] != 0;
}

static uint16_t packet_crc(const uint8_t *packet, uint8_t size, uint16_t crc)
{
    const bool standard = size == STANDARD_SIZE;
    const uint8_t bits = standard ? 14 : 16;
    const uint16_t poly = standard ? 0x2E57 : 0x3D65;
    const uint16_t high_bit = uint16_t(1U << (bits - 1));
    const uint8_t length = standard ? STANDARD_SIZE - 1 : FULL_SIZE - 2;
    for (uint8_t i = 0; i < length; i++) {
        const uint8_t byte = (standard && i == 0) ? packet[i] & 3 : packet[i];
        crc ^= uint16_t(byte) << (bits - 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc << 1) ^ ((crc & high_bit) ? poly : 0);
        }
    }
    return crc & (standard ? 0x3FFF : 0xFFFF);
}

bool validate_packet(const uint8_t *packet, uint8_t size, uint16_t init, uint8_t nonce)
{
    if (packet == nullptr || (size != STANDARD_SIZE && size != FULL_SIZE)) {
        return false;
    }
    const uint8_t type = packet[0] & 3;
    if (type > uint8_t(PacketType::SYNC)) {
        return false;
    }
    const uint16_t received = size == STANDARD_SIZE ?
                              (uint16_t(packet[0] >> 2) << 8) | packet[7] :
                              uint16_t(packet[11]) | (uint16_t(packet[12]) << 8);
    const uint16_t nonce_validator = type == uint8_t(PacketType::SYNC) ? 0 : nonce;
    return received == packet_crc(packet, size, init ^ nonce_validator);
}

uint8_t telemetry_denominator(uint8_t ratio)
{
    return ratio > 0 && ratio <= 7 ? uint8_t(1U << (8 - ratio)) : 1;
}

void make_link_stats(uint8_t packet[STANDARD_SIZE], uint16_t init, uint8_t nonce,
                     int8_t rssi_dbm, int8_t snr_raw, uint8_t lq, bool uplink_ack, uint8_t size)
{
    // OTA4 downlink type 0 is LINKSTATS (uplink type 0 is RC).
    // No downlink data transfer: packageIndex=0, payload padding=0.
    for (uint8_t i = 0; i < size; i++) {
        packet[i] = 0;
    }
    const bool full = size == FULL_SIZE;
    const uint8_t offset = full ? 1 : 2;
    packet[full ? 0 : 1] = uplink_ack ? (full ? 4 : 0x80) : 0;
    const int16_t magnitude = -int16_t(rssi_dbm);
    packet[offset] = magnitude < 0 ? 0 : (magnitude > 127 ? 127 : magnitude);
    packet[offset + 1] = 0x80;
    packet[offset + 2] = lq > 100 ? 100 : lq;
    packet[offset + 3] = uint8_t(snr_raw);
    const uint16_t crc = packet_crc(packet, size, init ^ nonce);
    if (full) {
        packet[11] = uint8_t(crc);
        packet[12] = uint8_t(crc >> 8);
    } else {
        packet[0] = uint8_t(crc >> 8) << 2;
        packet[7] = uint8_t(crc);
    }
}

bool UplinkSink::receive(uint8_t index)
{
    // Upstream ELRS_MSP_MAX_PACKAGES = 65 / 5 + 1 = 14 (resync).
    if (index == 14) {
        _ack = !_ack;
        _next = 1;
        return false;
    }
    if (index == 0 && _next > 1) {
        _ack = !_ack;
        _next = 1; // completed unsupported command consumed; upstream Unlock()
        return true;
    }
    if (index == _next || (index == 1 && _next > 1)) {
        _next = index + 1;
        _ack = !_ack;
    }
    return false;
}

bool parse_sync(const uint8_t *packet, uint8_t size, Sync &sync)
{
    if (packet == nullptr || (size != STANDARD_SIZE && size != FULL_SIZE) ||
        (packet[0] & 3) != uint8_t(PacketType::SYNC)) {
        return false;
    }
    sync.fhss_index = packet[1];
    sync.nonce = packet[2];
    sync.rate = packet[3];
    sync.switch_mode = SwitchMode(packet[4] & 1);
    sync.telemetry_ratio = (packet[4] >> 1) & 7;
    sync.gemini = (packet[4] & 0x10) != 0;
    sync.protocol = (packet[4] >> 5) & 3;
    sync.uid4 = packet[5];
    sync.uid5 = packet[6];
    return true;
}

bool parse_binding_uid(const uint8_t *packet, uint8_t size, uint8_t uid[UID_LENGTH])
{
    constexpr uint8_t MSP_ELRS_BIND = 0x09;
    if (uid == nullptr || size != STANDARD_SIZE || !validate_packet(packet, size, OTA_VERSION, 0) ||
        (packet[0] & 3) != uint8_t(PacketType::DATA) || (packet[1] & 0x7F) != 1 ||
        packet[2] != MSP_ELRS_BIND) {
        return false;
    }
    const uint8_t learned[UID_LENGTH] {0, 0, packet[3], packet[4], packet[5], packet[6]};
    if (!uid_is_bound(learned)) {
        return false;
    }
    for (uint8_t i = 0; i < UID_LENGTH; i++) {
        uid[i] = learned[i];
    }
    return true;
}

static uint16_t switch_value(uint8_t value, uint8_t maximum)
{
    return uint16_t(value) * (1792U - 191U) / maximum + 191U;
}

static uint16_t switch_3bit(uint8_t value)
{
    switch (value) {
    case 0:
        return 191;
    case 5:
        return 1792;
    case 6:
    case 7:
        return 992;
    default:
        return value * 240U + 391U;
    }
}

bool decode_channels(const uint8_t *packet, uint8_t size, uint16_t init,
                     uint8_t nonce, SwitchMode mode, Channels &channels)
{
    if (!validate_packet(packet, size, init, nonce) ||
        (packet[0] & 3) != uint8_t(PacketType::RC) ||
        (mode != SwitchMode::WIDE && mode != SwitchMode::HYBRID)) {
        return false;
    }
    if (size == FULL_SIZE) {
        const bool high = (packet[0] & 0x40) != 0;
        const uint8_t low_channel = mode == SwitchMode::HYBRID && high ? 8 : 0;
        const uint8_t high_channel = mode == SwitchMode::HYBRID ? low_channel + 4 : (high ? 8 : 4);
        channels.updated_mask = 0;
        for (uint8_t group = 0; group < 2; group++) {
            uint64_t packed = 0;
            for (uint8_t i = 0; i < 5; i++) {
                packed |= uint64_t(packet[1 + group * 5 + i]) << (8 * i);
            }
            for (uint8_t i = 0; i < 4; i++) {
                const uint8_t channel = (group == 0 ? low_channel : high_channel) + i;
                channels.crsf[channel] = ((packed >> (10 * i)) & 1023U) << 1;
                channels.updated_mask |= 1U << channel;
            }
        }
        channels.armed = (packet[0] & 0x80) != 0;
        channels.telemetry_ack = (packet[0] & 4) != 0;
        channels.uplink_power = ((packet[0] >> 3) & 7) + 1;
        if (mode == SwitchMode::WIDE) {
            channels.crsf[13] = channels.armed ? 1792 : 191;
            channels.updated_mask |= 1U << 13;
        }
        return true;
    }
    uint32_t packed = 0;
    uint8_t bits = 0;
    uint8_t offset = 1;
    for (uint8_t i = 0; i < 4; i++) {
        while (bits < 10) {
            packed |= uint32_t(packet[offset++]) << bits;
            bits += 8;
        }
        const uint16_t raw = packed & 1023;
        // Upstream UINT10_to_CRSF uses fmap with rounding, standard range 172..1811.
        channels.crsf[i] = (uint32_t(raw) * (1811U - 172U) * 2U / 1023U + 345U) / 2U;
        packed >>= 10;
        bits -= 10;
    }
    channels.armed = (packet[6] & 0x80) != 0;
    channels.crsf[4] = channels.armed ? 1792 : 191;
    channels.crsf[13] = channels.crsf[4];
    channels.updated_mask = 0x201F;
    channels.telemetry_ack = (packet[6] & 0x40) != 0;
    const uint8_t switches = packet[6] & 0x7F;
    uint8_t index;
    if (mode == SwitchMode::HYBRID) {
        index = (switches >> 3) & 7;
        if (index >= 6) {
            index = 11;
            channels.crsf[index] = switch_value(switches & 15, 15);
        } else {
            index += 5;
            channels.crsf[index] = switch_3bit(switches & 7);
        }
    } else {
        index = ((nonce & 7) + ((nonce >> 3) & 1)) % 8;
        if (index == 7) {
            channels.uplink_power = switches & 63;
            return true;
        }
        index += 5;
        channels.crsf[index] = switch_value(switches & 63, 63);
    }
    channels.updated_mask |= uint16_t(1U << index);
    return true;
}

void FHSS::randomise(uint32_t seed)
{
    _index = 0;
    for (uint16_t i = 0; i < SEQUENCE_LENGTH; i++) {
        const uint8_t channel = i % CHANNEL_COUNT;
        _sequence[i] = channel == 0 ? SYNC_CHANNEL : (channel == SYNC_CHANNEL ? 0 : channel);
    }
    for (uint16_t i = 0; i < SEQUENCE_LENGTH; i++) {
        if (i % CHANNEL_COUNT == 0) {
            continue;
        }
        seed = (214013U * seed + 2531011U) & 0x7FFFFFFFU;
        const uint8_t other = (i / CHANNEL_COUNT) * CHANNEL_COUNT +
                              ((seed >> 16) % (CHANNEL_COUNT - 1)) + 1;
        const uint8_t saved = _sequence[i];
        _sequence[i] = _sequence[other];
        _sequence[other] = saved;
    }
}

uint32_t FHSS::channel_frequency(uint8_t channel)
{
    // Preserve upstream truncation: convert band endpoints first, then spread.
    constexpr uint32_t start = uint64_t(2400400000U) * 262144U / 52000000U;
    constexpr uint32_t stop = uint64_t(2479400000U) * 262144U / 52000000U;
    constexpr uint32_t spread = (stop - start) * 256U / (CHANNEL_COUNT - 1);
    return start + spread * channel / 256U;
}

uint32_t FHSS::next_frequency()
{
    _index = (_index + 1) % SEQUENCE_LENGTH;
    return frequency();
}

} // namespace ELRS

#endif // AP_ELRS_ENABLED
