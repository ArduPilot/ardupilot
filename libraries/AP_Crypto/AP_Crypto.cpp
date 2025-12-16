/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Crypto.h"

#if defined(AP_CRYPTO_ENABLED) && AP_CRYPTO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <StorageManager/StorageManager.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// Key storage constants
#define CRYPTO_KEY_MAGIC 0x43525950  // "CRYP" in ASCII
#define CRYPTO_KEY_VERSION 1
#define CRYPTO_KEY_STORAGE_OFFSET 0  // Offset within StorageKeys area
#define CRYPTO_KEY_STORAGE_SIZE 32   // 32 bytes for key

struct crypto_key_header {
    uint32_t magic;      // CRYPTO_KEY_MAGIC
    uint32_t version;    // CRYPTO_KEY_VERSION
    uint8_t key[32];     // The actual key
};

bool AP_Crypto::store_key(const uint8_t key[32])
{
    if (key == nullptr) {
        return false;
    }
    
    StorageAccess storage(StorageManager::StorageKeys);
    if (storage.size() < sizeof(crypto_key_header)) {
        return false; // Not enough storage space
    }
    
    crypto_key_header header;
    header.magic = CRYPTO_KEY_MAGIC;
    header.version = CRYPTO_KEY_VERSION;
    memcpy(header.key, key, 32);
    
    // Write to storage
    if (!storage.write_block(CRYPTO_KEY_STORAGE_OFFSET, &header, sizeof(header))) {
        return false;
    }
    
    return true;
}

bool AP_Crypto::retrieve_key(uint8_t key[32])
{
    if (key == nullptr) {
        return false;
    }
    
    StorageAccess storage(StorageManager::StorageKeys);
    if (storage.size() < sizeof(crypto_key_header)) {
        return false; // Not enough storage space
    }
    
    crypto_key_header header;
    if (!storage.read_block(&header, CRYPTO_KEY_STORAGE_OFFSET, sizeof(header))) {
        return false;
    }
    
    // Verify magic and version
    if (header.magic != CRYPTO_KEY_MAGIC || header.version != CRYPTO_KEY_VERSION) {
        return false; // Invalid or uninitialized key
    }
    
    memcpy(key, header.key, 32);
    return true;
}

bool AP_Crypto::has_stored_key(void)
{
    StorageAccess storage(StorageManager::StorageKeys);
    if (storage.size() < sizeof(crypto_key_header)) {
        return false;
    }
    
    uint32_t magic = storage.read_uint32(CRYPTO_KEY_STORAGE_OFFSET);
    return (magic == CRYPTO_KEY_MAGIC);
}

bool AP_Crypto::generate_and_store_key(uint8_t key[32])
{
    // Generate random key
    uint8_t new_key[32];
    for (size_t i = 0; i < 32; i++) {
        // Use a combination of time and micros for randomness
        // In production, use proper RNG
        uint32_t seed = AP_HAL::millis() ^ (AP_HAL::micros() << (i % 16));
        new_key[i] = (uint8_t)(seed ^ (i * 0x9E3779B9));
        // Mix in more entropy
        seed = (seed << 1) | (seed >> 31);
        new_key[i] ^= (uint8_t)(seed & 0xFF);
    }
    
    // Store the key
    if (!store_key(new_key)) {
        return false;
    }
    
    // Return key if requested
    if (key != nullptr) {
        memcpy(key, new_key, 32);
    }
    
    return true;
}

bool AP_Crypto::derive_key_from_board_id(uint8_t key[32])
{
    // Board ID key derivation is disabled
    // Keys must be explicitly stored via AP_Crypto::store_key() or use the default key
    (void)key; // Suppress unused parameter warning
    return false;
}

#endif  // AP_CRYPTO_ENABLED
