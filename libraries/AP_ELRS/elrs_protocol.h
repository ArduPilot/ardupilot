#pragma once

#include <stdint.h>

namespace ELRS
{

static constexpr uint8_t OTA_VERSION = 4;
static constexpr uint8_t UID_LENGTH = 6;
static constexpr uint8_t STANDARD_SIZE = 8;
static constexpr uint8_t FULL_SIZE = 13;
static constexpr uint8_t RC_CHANNELS = 16;

enum class PacketType : uint8_t { RC = 0, DATA = 1, SYNC = 2 };
enum class SwitchMode : uint8_t { WIDE = 0, HYBRID = 1 };

uint32_t uid_seed(const uint8_t uid[UID_LENGTH]);
uint16_t crc_initializer(const uint8_t uid[UID_LENGTH]);
bool uid_is_bound(const uint8_t uid[UID_LENGTH]);

// CRC checks do not authenticate a sender and do not establish synchronization.
// SYNC is nonce independent; other packets require the current slot nonce.
bool validate_packet(const uint8_t *packet, uint8_t size, uint16_t init, uint8_t nonce);

// Wire ratio 0 disables telemetry; 1..7 mean 1:128 through 1:2.
uint8_t telemetry_denominator(uint8_t ratio);
void make_link_stats(uint8_t packet[STANDARD_SIZE], uint16_t init, uint8_t nonce,
                     int8_t rssi_dbm, int8_t snr_raw, uint8_t lq, bool uplink_ack = false, uint8_t size = STANDARD_SIZE);

// Transport-only sink for unsupported uplink commands. Acknowledges receipt,
// not command execution; no payload is forwarded or applied to parameters.
class UplinkSink
{
public:
    void reset()
    {
        _next = 1;
        _ack = false;
    }
    bool receive(uint8_t index); // true when a transfer completes
    bool ack() const
    {
        return _ack;
    }
private:
    uint8_t _next = 1;
    bool _ack = false;
};

struct Sync {
    uint8_t fhss_index;
    uint8_t nonce;
    uint8_t rate;
    SwitchMode switch_mode;
    uint8_t telemetry_ratio;
    bool gemini;
    uint8_t protocol;
    uint8_t uid4;
    uint8_t uid5;
};

// Caller must validate CRC first. Parse flags even if unsupported by the receiver.
bool parse_sync(const uint8_t *packet, uint8_t size, Sync &sync);

// Explicit binding mode only: standard DATA packet, package 1, MSP_ELRS_BIND.
// Output is unchanged on failure; the learned UID has two leading zero bytes.
bool parse_binding_uid(const uint8_t *packet, uint8_t size, uint8_t uid[UID_LENGTH]);

struct Channels {
    uint16_t crsf[RC_CHANNELS] {};
    uint16_t updated_mask = 0;
    bool armed = false;
    bool telemetry_ack = false;
    uint8_t uplink_power = 0;
};

// Validates before modifying output. Unsent multiplexed channels are retained;
// updated_mask describes this packet only, not freshness or link validity.
// Standard Hybrid/Wide and full-resolution 8/16-channel packets.
bool decode_channels(const uint8_t *packet, uint8_t size, uint16_t init,
                     uint8_t nonce, SwitchMode mode, Channels &channels);

class FHSS
{
public:
    static constexpr uint8_t CHANNEL_COUNT = 80;
    static constexpr uint8_t SYNC_CHANNEL = CHANNEL_COUNT / 2;
    static constexpr uint16_t SEQUENCE_LENGTH = 240;

    void randomise(uint32_t seed);
    uint8_t index() const
    {
        return _index;
    }
    void set_index(uint8_t index)
    {
        _index = index % SEQUENCE_LENGTH;
    }
    uint8_t channel() const
    {
        return _sequence[_index];
    }
    uint32_t frequency() const
    {
        return channel_frequency(channel());
    }
    uint32_t next_frequency();
    static uint32_t initial_frequency()
    {
        return channel_frequency(SYNC_CHANNEL);
    }
    static uint32_t channel_frequency(uint8_t channel);

private:
    uint8_t _sequence[SEQUENCE_LENGTH] {};
    uint8_t _index = 0;
};

} // namespace ELRS
