#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED && AP_LOGGER_ENCRYPTION_ENABLED

#include "AP_Logger_Encryption.h"

#include <AP_CheckFirmware/monocypher.h>
#include <AP_HAL/AP_HAL.h>

#include <string.h>

extern const AP_HAL::HAL& hal;

namespace {

constexpr uint8_t header_magic[8] = { 'A', 'P', 'L', 'O', 'G', 'E', '1', 0 };
constexpr uint8_t frame_magic[4] = { 'A', 'P', 'L', 'F' };
constexpr uint8_t header_version = 1;
constexpr uint8_t aead_xchacha20_poly1305 = 1;
constexpr uint8_t wrap_x25519_blake2b = 1;
constexpr uint8_t recipient_public_key[32] = AP_LOGGER_ENCRYPTION_PUBLIC_KEY;
constexpr char wrap_context[] = "ArduPilot AP_Logger key wrap v1";

bool write_all(ByteBuffer &writebuf, const uint8_t *data, uint32_t len)
{
    return writebuf.write(data, len) == len;
}

} // namespace

void AP_Logger_Encryption::put_le16(uint8_t *buf, uint16_t value)
{
    buf[0] = value & 0xffU;
    buf[1] = (value >> 8) & 0xffU;
}

void AP_Logger_Encryption::put_le64(uint8_t *buf, uint64_t value)
{
    for (uint8_t i = 0; i < 8; i++) {
        buf[i] = (value >> (i * 8U)) & 0xffU;
    }
}

void AP_Logger_Encryption::make_key_id(uint8_t key_id[16])
{
    crypto_blake2b_general(key_id, 16, nullptr, 0, recipient_public_key, sizeof(recipient_public_key));
}

void AP_Logger_Encryption::derive_wrap_key(uint8_t wrap_key[32],
                                           const uint8_t shared_secret[32],
                                           const uint8_t key_id[16],
                                           const uint8_t ephemeral_public_key[32])
{
    uint8_t message[sizeof(wrap_context) - 1 + 16 + 32 + 32];
    uint8_t *p = message;

    memcpy(p, wrap_context, sizeof(wrap_context) - 1);
    p += sizeof(wrap_context) - 1;
    memcpy(p, key_id, 16);
    p += 16;
    memcpy(p, ephemeral_public_key, 32);
    p += 32;
    memcpy(p, recipient_public_key, 32);

    crypto_blake2b_general(wrap_key, 32, shared_secret, 32, message, sizeof(message));
    crypto_wipe(message, sizeof(message));
}

bool AP_Logger_Encryption::start(ByteBuffer &writebuf)
{
    stop();

    uint8_t ephemeral_secret[32];
    uint8_t ephemeral_public_key[32];
    uint8_t shared_secret[32];
    uint8_t wrap_key[32];
    uint8_t wrap_nonce[24];
    uint8_t key_id[16];
    uint8_t header[header_len] {};

    if (writebuf.space() < sizeof(header)) {
        return false;
    }
    if (!hal.util->get_random_vals(_data_key, sizeof(_data_key)) ||
        !hal.util->get_random_vals(ephemeral_secret, sizeof(ephemeral_secret)) ||
        !hal.util->get_random_vals(wrap_nonce, sizeof(wrap_nonce)) ||
        !hal.util->get_random_vals(_stream_nonce_prefix, sizeof(_stream_nonce_prefix))) {
        crypto_wipe(ephemeral_secret, sizeof(ephemeral_secret));
        crypto_wipe(_data_key, sizeof(_data_key));
        return false;
    }

    crypto_x25519_public_key(ephemeral_public_key, ephemeral_secret);
    crypto_x25519(shared_secret, ephemeral_secret, recipient_public_key);
    make_key_id(key_id);
    derive_wrap_key(wrap_key, shared_secret, key_id, ephemeral_public_key);

    memcpy(&header[0], header_magic, sizeof(header_magic));
    header[8] = header_version;
    header[9] = aead_xchacha20_poly1305;
    header[10] = wrap_x25519_blake2b;
    header[11] = 0;
    put_le16(&header[12], header_len);
    put_le16(&header[14], 0);
    memcpy(&header[16], key_id, sizeof(key_id));
    memcpy(&header[32], ephemeral_public_key, sizeof(ephemeral_public_key));
    memcpy(&header[64], wrap_nonce, sizeof(wrap_nonce));
    memcpy(&header[88], _stream_nonce_prefix, sizeof(_stream_nonce_prefix));

    crypto_lock_aead(&header[136], &header[104], wrap_key, wrap_nonce,
                     header, 104, _data_key, sizeof(_data_key));

    const bool wrote = write_all(writebuf, header, sizeof(header));
    _active = wrote;
    _sequence = 0;

    crypto_wipe(ephemeral_secret, sizeof(ephemeral_secret));
    crypto_wipe(shared_secret, sizeof(shared_secret));
    crypto_wipe(wrap_key, sizeof(wrap_key));
    crypto_wipe(wrap_nonce, sizeof(wrap_nonce));
    crypto_wipe(key_id, sizeof(key_id));
    crypto_wipe(ephemeral_public_key, sizeof(ephemeral_public_key));
    crypto_wipe(header, sizeof(header));

    if (!wrote) {
        crypto_wipe(_data_key, sizeof(_data_key));
        crypto_wipe(_stream_nonce_prefix, sizeof(_stream_nonce_prefix));
    }

    return wrote;
}

void AP_Logger_Encryption::stop()
{
    if (_active) {
        crypto_wipe(_data_key, sizeof(_data_key));
        crypto_wipe(_stream_nonce_prefix, sizeof(_stream_nonce_prefix));
    }
    _active = false;
    _sequence = 0;
}

uint32_t AP_Logger_Encryption::encrypted_size_for(uint16_t plaintext_size) const
{
    if (plaintext_size == 0) {
        return 0;
    }
    const uint32_t frames = (uint32_t(plaintext_size) + max_frame_plaintext - 1U) / max_frame_plaintext;
    return plaintext_size + frames * (frame_header_len + tag_len);
}

void AP_Logger_Encryption::make_frame_nonce(uint8_t nonce[24], uint64_t sequence) const
{
    memcpy(nonce, _stream_nonce_prefix, sizeof(_stream_nonce_prefix));
    put_le64(&nonce[16], sequence);
}

bool AP_Logger_Encryption::write_frame(ByteBuffer &writebuf, const uint8_t *plaintext, uint16_t size)
{
    uint8_t frame_header[frame_header_len] {};
    uint8_t frame_tag[tag_len];
    uint8_t frame_nonce[24];
    uint8_t ciphertext[max_frame_plaintext];
    const uint64_t sequence = _sequence++;

    memcpy(&frame_header[0], frame_magic, sizeof(frame_magic));
    put_le64(&frame_header[4], sequence);
    put_le16(&frame_header[12], size);
    frame_header[14] = 0;
    frame_header[15] = 0;
    make_frame_nonce(frame_nonce, sequence);

    crypto_lock_aead(frame_tag, ciphertext, _data_key, frame_nonce,
                     frame_header, sizeof(frame_header), plaintext, size);

    const bool wrote = write_all(writebuf, frame_header, sizeof(frame_header)) &&
                       write_all(writebuf, frame_tag, sizeof(frame_tag)) &&
                       write_all(writebuf, ciphertext, size);

    crypto_wipe(frame_tag, sizeof(frame_tag));
    crypto_wipe(frame_nonce, sizeof(frame_nonce));
    crypto_wipe(ciphertext, sizeof(ciphertext));

    return wrote;
}

bool AP_Logger_Encryption::write(ByteBuffer &writebuf, const void *pBuffer, uint16_t size)
{
    if (!_active) {
        return false;
    }
    if (writebuf.space() < encrypted_size_for(size)) {
        return false;
    }

    const uint8_t *plaintext = static_cast<const uint8_t *>(pBuffer);
    uint16_t remaining = size;
    while (remaining > 0) {
        const uint16_t frame_size = remaining < max_frame_plaintext ? remaining : max_frame_plaintext;
        if (!write_frame(writebuf, plaintext, frame_size)) {
            return false;
        }
        plaintext += frame_size;
        remaining -= frame_size;
    }
    return true;
}

#endif // HAL_LOGGING_ENABLED && AP_LOGGER_ENCRYPTION_ENABLED
