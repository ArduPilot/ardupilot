#pragma once

#include "AP_DiskCrypto_config.h"

/*
  Build-time AES-256-XTS key for SD card full-disk encryption.

  The key is 64 bytes: a 32-byte data key followed by a 32-byte tweak key. The
  two halves must differ (an AES-XTS requirement); init() rejects a key whose
  halves are identical.

  For a real deployment, provide your own key by defining AP_DISKCRYPTO_KEY
  (and AP_DISKCRYPTO_KEY_PROVIDED) ahead of this header from a build secret,
  for example via an extra include configured in the board hwdef. The default
  below is an INSECURE, well-known development/CI key and protects nothing.

  This build-time key model defends a lost or stolen card against casual
  reading. It does NOT defend against an attacker who can dump the firmware or
  read internal flash. Stronger key sourcing (passphrase + KDF, OTP, or a key
  delivered over MAVLink at boot) can be added behind the same init() path.
 */

#ifndef AP_DISKCRYPTO_KEY
// INSECURE default key - development, SITL and CI use only.
#define AP_DISKCRYPTO_KEY { \
    /* data key (32 bytes) */ \
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, \
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, \
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, \
    /* tweak key (32 bytes) */ \
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, \
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, \
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, \
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f }

#if AP_FATFS_CRYPTO_ENABLED && (CONFIG_HAL_BOARD != HAL_BOARD_SITL)
#warning "AP_DiskCrypto: using the INSECURE default key. Define AP_DISKCRYPTO_KEY for production."
#endif

#endif  // AP_DISKCRYPTO_KEY
