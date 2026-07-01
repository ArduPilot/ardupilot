#pragma once

#include "AP_DiskCrypto_config.h"

#if AP_DISKCRYPTO_ENABLED

#include <stdint.h>

/*
  Abstract AES-256 single-block (ECB) cipher used as the primitive for the
  AES-XTS layer in AP_DiskCrypto. This is the seam between software and
  hardware-accelerated implementations: the XTS code only ever talks to this
  interface, so a hardware backend (e.g. STM32 CRYP) can be dropped in without
  touching the disk-encryption logic.
 */
class AP_DiskCrypto_Backend {
public:
    virtual ~AP_DiskCrypto_Backend() {}

    // load a 32-byte (256-bit) AES key for subsequent block operations
    virtual void set_key(const uint8_t key[32]) = 0;

    // encrypt/decrypt a single 16-byte block. in and out may alias.
    virtual void encrypt_block(const uint8_t in[16], uint8_t out[16]) = 0;
    virtual void decrypt_block(const uint8_t in[16], uint8_t out[16]) = 0;

    // allocate the best available backend (hardware if present, else software).
    // returns nullptr on allocation failure; caller owns the object.
    static AP_DiskCrypto_Backend *create();
};

#endif  // AP_DISKCRYPTO_ENABLED
