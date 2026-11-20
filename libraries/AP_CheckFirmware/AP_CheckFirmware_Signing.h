#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <AP_Common/AP_Common.h>

#include "monocypher.h"

#define AP_PUBLIC_KEY_LEN 32
#define AP_PUBLIC_KEY_MAX_KEYS 10
#define AP_PUBLIC_KEY_SIGNATURE {0x4e, 0xcf, 0x4e, 0xa5, 0xa6, 0xb6, 0xf7, 0x29}

struct PACKED ap_secure_data {
    uint8_t sig[8] = AP_PUBLIC_KEY_SIGNATURE;
    struct PACKED {
        uint8_t key[AP_PUBLIC_KEY_LEN] = {};
    } public_key[AP_PUBLIC_KEY_MAX_KEYS];
};

struct ap_signature_buffer {
    const uint8_t *data;
    size_t length;
};

static inline bool ap_public_key_is_zero(const uint8_t key[AP_PUBLIC_KEY_LEN])
{
    uint8_t key_bytes = 0;
    for (uint8_t i = 0; i < AP_PUBLIC_KEY_LEN; i++) {
        key_bytes |= key[i];
    }
    return key_bytes == 0;
}

static inline bool ap_public_keys_all_zero(const struct ap_secure_data &keys)
{
    for (const auto &public_key : keys.public_key) {
        if (!ap_public_key_is_zero(public_key.key)) {
            return false;
        }
    }
    return true;
}

static inline void ap_public_keys_compact(struct ap_secure_data &keys)
{
    uint8_t next_key = 0;
    for (uint8_t i = 0; i < AP_PUBLIC_KEY_MAX_KEYS; i++) {
        if (!ap_public_key_is_zero(keys.public_key[i].key)) {
            if (next_key != i) {
                memcpy(keys.public_key[next_key].key, keys.public_key[i].key, AP_PUBLIC_KEY_LEN);
            }
            next_key++;
        }
    }
    if (next_key < AP_PUBLIC_KEY_MAX_KEYS) {
        memset(keys.public_key[next_key].key, 0,
               AP_PUBLIC_KEY_LEN*(AP_PUBLIC_KEY_MAX_KEYS-next_key));
    }
}

static inline bool ap_public_key_sets_equal(const struct ap_secure_data &first,
                                            const struct ap_secure_data &second)
{
    bool second_matched[AP_PUBLIC_KEY_MAX_KEYS] {};
    for (const auto &first_key : first.public_key) {
        if (ap_public_key_is_zero(first_key.key)) {
            continue;
        }
        bool found = false;
        for (uint8_t i = 0; i < AP_PUBLIC_KEY_MAX_KEYS; i++) {
            if (!second_matched[i] &&
                memcmp(first_key.key, second.public_key[i].key, AP_PUBLIC_KEY_LEN) == 0) {
                second_matched[i] = true;
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    for (uint8_t i = 0; i < AP_PUBLIC_KEY_MAX_KEYS; i++) {
        if (!second_matched[i] && !ap_public_key_is_zero(second.public_key[i].key)) {
            return false;
        }
    }
    return true;
}

static inline bool ap_check_signature(const struct ap_secure_data &keys,
                                      const uint8_t signature[64],
                                      const struct ap_signature_buffer *buffers,
                                      uint8_t num_buffers)
{
    for (const auto &public_key : keys.public_key) {
        if (ap_public_key_is_zero(public_key.key)) {
            continue;
        }
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract *)&ctx;
        crypto_check_init(actx, signature, public_key.key);
        for (uint8_t i = 0; i < num_buffers; i++) {
            crypto_check_update(actx, buffers[i].data, buffers[i].length);
        }
        if (crypto_check_final(actx) == 0) {
            return true;
        }
    }
    return false;
}

static inline void ap_extf_signature_check_init(crypto_check_ctx_abstract *ctx,
                                                const uint8_t signature[64],
                                                const uint8_t public_key[AP_PUBLIC_KEY_LEN],
                                                uint32_t len1, uint32_t len2,
                                                uint32_t extf_length)
{
    static const uint8_t signature_domain[] = "ArduPilotFwSig2";
    const uint32_t image_lengths[] { len1, len2, extf_length };
    crypto_check_init(ctx, signature, public_key);
    crypto_check_update(ctx, signature_domain, sizeof(signature_domain)-1);
    crypto_check_update(ctx, (const uint8_t *)image_lengths, sizeof(image_lengths));
}
