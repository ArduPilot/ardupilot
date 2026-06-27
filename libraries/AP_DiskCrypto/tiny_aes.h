/*
  Tiny AES in C

  Vendored from https://github.com/kokke/tiny-AES-c (commit 12e7744)

  This is free and unencumbered software released into the public domain.

  Anyone is free to copy, modify, publish, use, compile, sell, or
  distribute this software, either in source code form or as a compiled
  binary, for any purpose, commercial or non-commercial, and by any
  means.

  SPDX-License-Identifier: Unlicense

  For more information, please refer to <https://unlicense.org>

  ArduPilot notes:
    - Trimmed to AES-256 ECB single-block operation only, which is the
      primitive used by the AES-XTS layer in AP_DiskCrypto. CBC/CTR were
      removed as they are not used here.
    - This is third-party code kept close to upstream; it is intentionally
      excluded from ArduPilot's astyle/lint checks.
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AES_BLOCKLEN 16  // block length in bytes - AES is 128b block only

#define AES_KEYLEN 32     // AES-256 key length in bytes
#define AES_keyExpSize 240

struct AES_ctx {
    uint8_t RoundKey[AES_keyExpSize];
};

void AES_init_ctx(struct AES_ctx* ctx, const uint8_t* key);

// buffer size is exactly AES_BLOCKLEN bytes;
// the buffer is encrypted/decrypted in place.
void AES_ECB_encrypt(const struct AES_ctx* ctx, uint8_t* buf);
void AES_ECB_decrypt(const struct AES_ctx* ctx, uint8_t* buf);

#ifdef __cplusplus
}
#endif
