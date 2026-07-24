#pragma once

#include "AP_DiskCrypto_config.h"

#if AP_DISKCRYPTO_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>

class AP_DiskCrypto_Backend;

/*
  Transparent full-disk encryption engine for SD card storage.

  Implements AES-256-XTS over fixed 512-byte sectors, using the sector LBA as
  the XTS tweak (the standard length-preserving construction for disk
  encryption). The actual AES block work is delegated to an
  AP_DiskCrypto_Backend, so software and hardware-accelerated ciphers share
  this exact logic.

  It is driven from the FatFS diskio binding via the ap_diskcrypto_encrypt()
  and ap_diskcrypto_decrypt() C shims, but the encrypt_sector()/
  decrypt_sector() methods are usable directly (and are unit tested on SITL).
 */
class AP_DiskCrypto {
public:
    AP_DiskCrypto();
    ~AP_DiskCrypto();

    CLASS_NO_COPY(AP_DiskCrypto);

    // bytes per encrypted sector (the FatFS/SD block size)
    static const uint16_t SECTOR_SIZE = 512;

    // initialise the engine with a 64-byte XTS key (32-byte data key followed
    // by a 32-byte tweak key). The two halves must differ (an XTS requirement).
    // Safe to call more than once; only the first call takes effect. Returns
    // false on a duplicated-half key or on allocation failure.
    bool init(const uint8_t key[AP_DISKCRYPTO_KEY_LEN]);

    bool initialised() const { return _initialised; }

    // AES-256-XTS encrypt one SECTOR_SIZE sector from in to out (may alias),
    // using lba as the XTS tweak.
    bool encrypt_sector(const uint8_t *in, uint8_t *out, uint64_t lba);

    // AES-256-XTS decrypt one SECTOR_SIZE sector in place, using lba as tweak.
    bool decrypt_sector(uint8_t *buf, uint64_t lba);

private:
    AP_DiskCrypto_Backend *_data_cipher;   // AES key 1, encrypts the data blocks
    AP_DiskCrypto_Backend *_tweak_cipher;  // AES key 2, encrypts the tweak
    bool _initialised;

    // multiply the 128-bit tweak by the primitive element alpha in GF(2^128)
    static void gf128_mul_alpha(uint8_t t[16]);
};

#endif  // AP_DISKCRYPTO_ENABLED
