#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED && AP_LOGGER_ENCRYPTION_ENABLED

#include <AP_HAL/utility/RingBuffer.h>

class AP_Logger_Encryption {
public:
    bool start(ByteBuffer &writebuf);
    void stop();

    bool active() const { return _active; }
    uint32_t encrypted_size_for(uint16_t plaintext_size) const;
    bool write(ByteBuffer &writebuf, const void *pBuffer, uint16_t size);

private:
    static constexpr uint8_t header_len = 152;
    static constexpr uint8_t frame_header_len = 16;
    static constexpr uint8_t tag_len = 16;
    static constexpr uint16_t max_frame_plaintext = 192;

    bool _active { false };
    uint64_t _sequence { 0 };
    uint8_t _data_key[32] {};
    uint8_t _stream_nonce_prefix[16] {};

    static void put_le16(uint8_t *buf, uint16_t value);
    static void put_le64(uint8_t *buf, uint64_t value);
    static void make_key_id(uint8_t key_id[16]);
    static void derive_wrap_key(uint8_t wrap_key[32],
                                const uint8_t shared_secret[32],
                                const uint8_t key_id[16],
                                const uint8_t ephemeral_public_key[32]);

    void make_frame_nonce(uint8_t nonce[24], uint64_t sequence) const;
    bool write_frame(ByteBuffer &writebuf, const uint8_t *plaintext, uint16_t size);
};

#endif // HAL_LOGGING_ENABLED && AP_LOGGER_ENCRYPTION_ENABLED
