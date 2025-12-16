/*
   SHA-256 implementation for AP_Crypto
   
   This is a simple SHA-256 implementation to match Python's hashlib.sha256
   exactly. This ensures compatibility between C++ and Python implementations.
   
   Based on public domain SHA-256 reference implementation.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// SHA-256 context structure
typedef struct {
    uint8_t data[64];
    uint32_t datalen;
    uint64_t bitlen;
    uint32_t state[8];
} sha256_ctx;

// Initialize SHA-256 context
void sha256_init(sha256_ctx *ctx);

// Update SHA-256 with data
void sha256_update(sha256_ctx *ctx, const uint8_t *data, size_t len);

// Finalize SHA-256 and output hash
void sha256_final(sha256_ctx *ctx, uint8_t hash[32]);

// One-shot SHA-256 hash function
void sha256(const uint8_t *data, size_t len, uint8_t hash[32]);

#ifdef __cplusplus
}
#endif

