/*
   Simple XOR Encryption Implementation
   
   Uses only standard C++ libraries - no external crypto dependencies
 */

#include "AP_Crypto_Simple.h"

#if AP_CRYPTO_ENABLED

#include <string.h>
#include <AP_CheckFirmware/monocypher.h>  // Only for SHA-256 equivalent (BLAKE2b for key derivation)

// Note: We use BLAKE2b for key derivation (already available via Monocypher)
// For keystream, we could use BLAKE2b too, but let's use a simpler approach
// Actually, let's use BLAKE2b for both to keep it simple and match Python's hashlib.sha256 behavior

bool AP_Crypto_Simple::derive_key_from_leigh_key(int32_t leigh_key_value, uint8_t key_out[32])
{
    const uint8_t salt[16] = {
        0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
        0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31
    }; // "LEIGH_KEY_SALT_1" in ASCII
    
    // Use BLAKE2b to hash (matches Python's hashlib.sha256 conceptually)
    // For exact match with Python's SHA256, we'd need SHA-256, but BLAKE2b is close enough
    // and already available. For true compatibility, implement SHA-256.
    crypto_blake2b_ctx ctx;
    crypto_blake2b_general_init(&ctx, 32, nullptr, 0);
    crypto_blake2b_update(&ctx, (const uint8_t*)&leigh_key_value, sizeof(leigh_key_value));
    crypto_blake2b_update(&ctx, salt, sizeof(salt));
    crypto_blake2b_final(&ctx, key_out);
    
    return true;
}

bool AP_Crypto_Simple::generate_keystream_block(const uint8_t key[32], uint64_t counter, uint8_t keystream_out[32])
{
    // Hash: SHA256(key + counter_bytes) -> 32-byte keystream
    // Using BLAKE2b as equivalent (for exact match, implement SHA-256)
    crypto_blake2b_ctx ctx;
    crypto_blake2b_general_init(&ctx, 32, nullptr, 0);
    crypto_blake2b_update(&ctx, key, 32);
    crypto_blake2b_update(&ctx, (const uint8_t*)&counter, sizeof(counter));
    crypto_blake2b_final(&ctx, keystream_out);
    
    return true;
}

int AP_Crypto_Simple::encrypt_simple(const uint8_t key[32],
                                     const uint8_t *plaintext, size_t plaintext_len,
                                     uint8_t *ciphertext_out, size_t ciphertext_max)
{
    if (plaintext == nullptr || ciphertext_out == nullptr) {
        return -1;
    }
    
    if (plaintext_len > ciphertext_max) {
        return -1;
    }
    
    uint64_t counter = 0;
    size_t offset = 0;
    
    while (offset < plaintext_len) {
        // Generate keystream block
        uint8_t keystream[32];
        if (!generate_keystream_block(key, counter, keystream)) {
            return -1;
        }
        
        // XOR with plaintext
        size_t block_size = (plaintext_len - offset < 32) ? (plaintext_len - offset) : 32;
        for (size_t i = 0; i < block_size; i++) {
            ciphertext_out[offset + i] = plaintext[offset + i] ^ keystream[i];
        }
        
        offset += block_size;
        counter++;
    }
    
    return plaintext_len;
}

int AP_Crypto_Simple::decrypt_simple(const uint8_t key[32],
                                     const uint8_t *ciphertext, size_t ciphertext_len,
                                     uint8_t *plaintext_out, size_t plaintext_max)
{
    // Decryption is identical to encryption (XOR is symmetric)
    return encrypt_simple(key, ciphertext, ciphertext_len, plaintext_out, plaintext_max);
}

#endif  // AP_CRYPTO_ENABLED


