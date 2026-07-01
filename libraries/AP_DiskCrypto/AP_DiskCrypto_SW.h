#pragma once

#include "AP_DiskCrypto_config.h"

#if AP_DISKCRYPTO_ENABLED

#include "AP_DiskCrypto_Backend.h"
#include "tiny_aes.h"

/*
  Software AES-256 block cipher backend, wrapping the vendored tiny-AES core.
  Portable to every target (including SITL) and used as the default and as the
  fallback when no hardware acceleration is available.
 */
class AP_DiskCrypto_SW : public AP_DiskCrypto_Backend {
public:
    void set_key(const uint8_t key[32]) override;
    void encrypt_block(const uint8_t in[16], uint8_t out[16]) override;
    void decrypt_block(const uint8_t in[16], uint8_t out[16]) override;

private:
    struct AES_ctx _ctx;
};

#endif  // AP_DISKCRYPTO_ENABLED
