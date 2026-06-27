/*
  Full-disk AES-256-XTS encryption for SD card storage.

  This file is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This file is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_DiskCrypto.h"

#if AP_DISKCRYPTO_ENABLED

#include "AP_DiskCrypto_Backend.h"
#include "AP_DiskCrypto_key.h"
#include <string.h>

AP_DiskCrypto::AP_DiskCrypto() :
    _data_cipher(nullptr),
    _tweak_cipher(nullptr),
    _initialised(false)
{
}

AP_DiskCrypto::~AP_DiskCrypto()
{
    delete _data_cipher;
    delete _tweak_cipher;
}

bool AP_DiskCrypto::init(const uint8_t key[AP_DISKCRYPTO_KEY_LEN])
{
    if (_initialised) {
        return true;
    }
    // XTS requires the data key and the tweak key to differ; identical halves
    // are a known weakness and are rejected by FIPS implementations (e.g. the
    // OpenSSL-based host recovery tool), so refuse them here too.
    if (memcmp(&key[0], &key[32], 32) == 0) {
        return false;
    }
    if (_data_cipher == nullptr) {
        _data_cipher = AP_DiskCrypto_Backend::create();
    }
    if (_tweak_cipher == nullptr) {
        _tweak_cipher = AP_DiskCrypto_Backend::create();
    }
    if (_data_cipher == nullptr || _tweak_cipher == nullptr) {
        return false;
    }
    _data_cipher->set_key(&key[0]);
    _tweak_cipher->set_key(&key[32]);
    _initialised = true;
    return true;
}

/*
  Multiply the 128-bit little-endian tweak by the primitive element alpha (a
  left shift by one bit), reducing modulo the XTS polynomial
  x^128 + x^7 + x^2 + x + 1 (the 0x87 constant) when the high bit overflows.
 */
void AP_DiskCrypto::gf128_mul_alpha(uint8_t t[16])
{
    uint8_t carry = 0;
    for (uint8_t i = 0; i < 16; i++) {
        const uint8_t next_carry = t[i] >> 7;
        t[i] = uint8_t((t[i] << 1) | carry);
        carry = next_carry;
    }
    if (carry != 0) {
        t[0] ^= 0x87;
    }
}

bool AP_DiskCrypto::encrypt_sector(const uint8_t *in, uint8_t *out, uint64_t lba)
{
    if (!_initialised) {
        return false;
    }

    // tweak = AES-encrypt(tweak_key, lba as a 128-bit little-endian integer)
    uint8_t tweak[16];
    memset(tweak, 0, sizeof(tweak));
    for (uint8_t i = 0; i < 8; i++) {
        tweak[i] = uint8_t(lba >> (8 * i));
    }
    _tweak_cipher->encrypt_block(tweak, tweak);

    uint8_t block[16];
    for (uint16_t off = 0; off < SECTOR_SIZE; off += 16) {
        for (uint8_t i = 0; i < 16; i++) {
            block[i] = in[off + i] ^ tweak[i];
        }
        _data_cipher->encrypt_block(block, block);
        for (uint8_t i = 0; i < 16; i++) {
            out[off + i] = block[i] ^ tweak[i];
        }
        gf128_mul_alpha(tweak);
    }
    return true;
}

bool AP_DiskCrypto::decrypt_sector(uint8_t *buf, uint64_t lba)
{
    if (!_initialised) {
        return false;
    }

    uint8_t tweak[16];
    memset(tweak, 0, sizeof(tweak));
    for (uint8_t i = 0; i < 8; i++) {
        tweak[i] = uint8_t(lba >> (8 * i));
    }
    _tweak_cipher->encrypt_block(tweak, tweak);

    uint8_t block[16];
    for (uint16_t off = 0; off < SECTOR_SIZE; off += 16) {
        for (uint8_t i = 0; i < 16; i++) {
            block[i] = buf[off + i] ^ tweak[i];
        }
        _data_cipher->decrypt_block(block, block);
        for (uint8_t i = 0; i < 16; i++) {
            buf[off + i] = block[i] ^ tweak[i];
        }
        gf128_mul_alpha(tweak);
    }
    return true;
}

#if AP_FATFS_CRYPTO_ENABLED
/*
  C shims called from the ChibiOS FatFS diskio binding (fatfs_diskio.c). They
  own the single AP_DiskCrypto instance for the SD card and lazily initialise
  it from the build-time key on first use. SD card I/O is serialised by the
  filesystem layer, so no additional locking is required here.
 */
static AP_DiskCrypto _sdcard_diskcrypto;

static bool diskcrypto_ensure_init(void)
{
    if (_sdcard_diskcrypto.initialised()) {
        return true;
    }
    static const uint8_t key[AP_DISKCRYPTO_KEY_LEN] = AP_DISKCRYPTO_KEY;
    return _sdcard_diskcrypto.init(key);
}

extern "C" {

bool ap_diskcrypto_encrypt(const uint8_t *in, uint8_t *out, uint32_t lba, uint32_t nsectors)
{
    if (!diskcrypto_ensure_init()) {
        return false;
    }
    for (uint32_t s = 0; s < nsectors; s++) {
        const uint32_t off = s * AP_DiskCrypto::SECTOR_SIZE;
        if (!_sdcard_diskcrypto.encrypt_sector(&in[off], &out[off], uint64_t(lba) + s)) {
            return false;
        }
    }
    return true;
}

bool ap_diskcrypto_decrypt(uint8_t *buf, uint32_t lba, uint32_t nsectors)
{
    if (!diskcrypto_ensure_init()) {
        return false;
    }
    for (uint32_t s = 0; s < nsectors; s++) {
        const uint32_t off = s * AP_DiskCrypto::SECTOR_SIZE;
        if (!_sdcard_diskcrypto.decrypt_sector(&buf[off], uint64_t(lba) + s)) {
            return false;
        }
    }
    return true;
}

} // extern "C"
#endif  // AP_FATFS_CRYPTO_ENABLED

#endif  // AP_DISKCRYPTO_ENABLED
