# Simple Encryption Implementation Guide for ArduPilot (Encryption Side)

> **⚠️ IMPORTANT: See `RESPONSE_TO_DECRYPTION_SIDE.md` for review feedback and compatibility analysis.**

## Overview

This document describes how to implement the simple encryption method on the ArduPilot/sensor side. This encryption method is designed to be:
- **Simple**: Easy to implement and debug
- **Pure standard library**: Uses only standard C++ libraries (no external dependencies)
- **Key-based**: Uses LEIGH_KEY parameter (32-bit float) from Mission Planner
- **Authenticated**: Includes HMAC-SHA256 for integrity verification

## Design Goals

- Replace complex XChaCha20-Poly1305 with simpler XOR + HMAC-SHA256
- Use only standard C++ libraries (no monocypher dependency)
- Match Python decryption implementation exactly
- Support streaming encryption for large log files

## Encryption Algorithm

### Process Flow:

```
1. Key Derivation:
   - Read LEIGH_KEY parameter (32-bit float)
   - Convert float to IEEE 754 bytes (little-endian, 4 bytes)
   - PBKDF2(seed_bytes + salt, iterations=10000) → 32-byte key

2. Generate Nonce:
   - Random 16-byte nonce (unique per file)

3. Generate Keystream:
   - HMAC-SHA256(key, nonce + counter) for each 32-byte block
   - Counter starts at 0, increments for each block

4. Encrypt:
   - XOR plaintext with keystream (block by block)

5. Generate MAC:
   - HMAC-SHA256(key, nonce + ciphertext) → 32-byte MAC
   - Store first 16 bytes as MAC

6. File Format:
   [Nonce: 16 bytes][Ciphertext: variable][MAC: 16 bytes]
```

## Implementation Details

### 1. Key Derivation

**Important**: ArduPilot stores parameters as **32-bit floats**, not integers!

```cpp
#include <cstdint>
#include <cstring>
#include <openssl/evp.h>  // For PBKDF2 (or use ArduPilot's crypto library)

/**
 * Derive encryption key from LEIGH_KEY parameter
 * 
 * @param leigh_key_float LEIGH_KEY parameter value (32-bit float)
 * @param salt Salt bytes (16 bytes: "LEIGH_KEY_SALT_1")
 * @param key_out Output buffer for 32-byte key
 * @return true on success, false on failure
 */
bool derive_key_from_leigh_key(float leigh_key_float, 
                                const uint8_t* salt, 
                                uint8_t key_out[32]) {
    // Convert float to IEEE 754 bytes (little-endian, 4 bytes)
    uint8_t seed_bytes[4];
    memcpy(seed_bytes, &leigh_key_float, 4);  // Direct memory copy preserves float format
    
    // Concatenate seed + salt
    uint8_t input[4 + 16];  // 4 bytes seed + 16 bytes salt
    memcpy(input, seed_bytes, 4);
    memcpy(input + 4, salt, 16);
    
    // PBKDF2 with SHA-256, 10000 iterations
    // Using OpenSSL's PBKDF2 (or ArduPilot's equivalent)
    int result = PKCS5_PBKDF2_HMAC(
        (const char*)input, 20,  // input data (seed + salt), length
        salt, 16,                // salt (can reuse salt here or use NULL)
        10000,                    // iterations
        EVP_sha256(),            // hash function
        32,                      // output key length
        key_out                  // output buffer
    );
    
    return (result == 1);
}
```

**Alternative using ArduPilot's crypto library** (if available):

```cpp
// If ArduPilot has BLAKE2b or PBKDF2 functions:
#include "AP_Crypto.h"  // Or appropriate ArduPilot crypto header

bool derive_key_from_leigh_key(float leigh_key_float, 
                                const uint8_t* salt, 
                                uint8_t key_out[32]) {
    uint8_t seed_bytes[4];
    memcpy(seed_bytes, &leigh_key_float, 4);
    
    // Use ArduPilot's PBKDF2 or BLAKE2b if available
    // Otherwise, implement simple PBKDF2 using SHA-256
    // (See implementation section below)
    
    return true;
}
```

### 2. HMAC-SHA256 Implementation

You'll need HMAC-SHA256. Options:

**Option A: Use OpenSSL** (if available):
```cpp
#include <openssl/hmac.h>

void hmac_sha256(const uint8_t* key, size_t key_len,
                 const uint8_t* data, size_t data_len,
                 uint8_t* output) {
    unsigned int len = 32;
    HMAC(EVP_sha256(), key, key_len, data, data_len, output, &len);
}
```

**Option B: Use ArduPilot's crypto library** (if available):
```cpp
// Check if ArduPilot has HMAC-SHA256 functions
// Otherwise, implement using SHA-256 (see below)
```

**Option C: Simple HMAC-SHA256 implementation** (if no library available):
```cpp
#include <cstring>

// You'll need a SHA-256 implementation (ArduPilot may have one)
// Or use a simple public-domain SHA-256 implementation

void hmac_sha256(const uint8_t* key, size_t key_len,
                 const uint8_t* data, size_t data_len,
                 uint8_t* output) {
    // Standard HMAC-SHA256 implementation
    // See RFC 2104 for algorithm
    // Uses SHA-256 as the underlying hash function
}
```

### 3. Keystream Generation

```cpp
/**
 * Generate keystream using HMAC-SHA256
 * 
 * @param key 32-byte encryption key
 * @param nonce 16-byte nonce
 * @param length Desired keystream length
 * @param keystream_out Output buffer
 */
void generate_keystream(const uint8_t key[32],
                        const uint8_t nonce[16],
                        size_t length,
                        uint8_t* keystream_out) {
    uint64_t counter = 0;
    size_t offset = 0;
    
    while (offset < length) {
        // Prepare input: nonce + counter (big-endian 64-bit)
        uint8_t input[16 + 8];
        memcpy(input, nonce, 16);
        
        // Counter as big-endian 64-bit integer
        for (int i = 0; i < 8; i++) {
            input[16 + i] = (counter >> (56 - i * 8)) & 0xFF;
        }
        
        // HMAC-SHA256(key, nonce + counter) → 32 bytes
        uint8_t block[32];
        hmac_sha256(key, 32, input, 24, block);
        
        // Copy to output (may be partial block at end)
        size_t copy_len = (length - offset < 32) ? (length - offset) : 32;
        memcpy(keystream_out + offset, block, copy_len);
        
        offset += copy_len;
        counter++;
    }
}
```

### 4. Encryption Function

```cpp
/**
 * Encrypt data using simple XOR cipher with HMAC-based keystream
 * 
 * @param key 32-byte encryption key
 * @param plaintext Input plaintext data
 * @param plaintext_len Length of plaintext
 * @param nonce_out Output: 16-byte nonce (caller should generate random)
 * @param ciphertext_out Output: encrypted data (same length as plaintext)
 * @param mac_out Output: 16-byte MAC
 * @return true on success
 */
bool encrypt_simple(const uint8_t key[32],
                    const uint8_t* plaintext,
                    size_t plaintext_len,
                    uint8_t nonce_out[16],
                    uint8_t* ciphertext_out,
                    uint8_t mac_out[16]) {
    // 1. Generate random nonce (use ArduPilot's random number generator)
    // For example: hal.util->get_random_vals(nonce_out, 16);
    // Or use a cryptographically secure random source
    
    // 2. Generate keystream
    uint8_t* keystream = (uint8_t*)malloc(plaintext_len);
    if (!keystream) return false;
    
    generate_keystream(key, nonce_out, plaintext_len, keystream);
    
    // 3. XOR encrypt: ciphertext = plaintext XOR keystream
    for (size_t i = 0; i < plaintext_len; i++) {
        ciphertext_out[i] = plaintext[i] ^ keystream[i];
    }
    
    free(keystream);
    
    // 4. Generate MAC: HMAC(key, nonce + ciphertext)
    uint8_t mac_input[16 + plaintext_len];
    memcpy(mac_input, nonce_out, 16);
    memcpy(mac_input + 16, ciphertext_out, plaintext_len);
    
    uint8_t mac_full[32];
    hmac_sha256(key, 32, mac_input, 16 + plaintext_len, mac_full);
    
    // Use first 16 bytes as MAC
    memcpy(mac_out, mac_full, 16);
    
    return true;
}
```

### 5. Streaming Encryption (for Large Files)

For large log files, implement streaming encryption:

```cpp
class SimpleEncryptStream {
private:
    uint8_t key[32];
    uint8_t nonce[16];
    uint64_t counter;
    size_t bytes_encrypted;
    uint8_t* mac_context;  // For incremental MAC computation
    
public:
    bool init(const uint8_t key_in[32]) {
        memcpy(key, key_in, 32);
        
        // Generate random nonce
        // hal.util->get_random_vals(nonce, 16);
        
        counter = 0;
        bytes_encrypted = 0;
        
        // Initialize MAC context (if using incremental HMAC)
        // Otherwise, buffer all ciphertext for final MAC
        
        return true;
    }
    
    bool encrypt_chunk(const uint8_t* plaintext, size_t len, uint8_t* ciphertext_out) {
        // Generate keystream for this chunk
        // XOR encrypt
        // Update MAC incrementally
        
        bytes_encrypted += len;
        return true;
    }
    
    bool finalize(uint8_t mac_out[16]) {
        // Finalize MAC computation
        // Return MAC
        return true;
    }
    
    void get_nonce(uint8_t nonce_out[16]) {
        memcpy(nonce_out, nonce, 16);
    }
};
```

## File Format

Encrypted files use this format:

```
[Nonce: 16 bytes][Ciphertext: variable length][MAC: 16 bytes]
```

**Total overhead**: 32 bytes per file (16 nonce + 16 MAC)

## Integration with ArduPilot

### 1. Parameter Access

```cpp
#include "AP_Param.h"

// Get LEIGH_KEY parameter
AP_Float leigh_key_param;
leigh_key_param.setup("LEIGH_KEY", 0.0f);  // Default 0

float leigh_key = leigh_key_param.get();

// Derive key
uint8_t encryption_key[32];
const uint8_t salt[16] = "LEIGH_KEY_SALT_1";
if (!derive_key_from_leigh_key(leigh_key, salt, encryption_key)) {
    // Error handling
    return false;
}
```

### 2. Log File Encryption

Integrate into log file writer:

```cpp
// In AP_Logger or log file writer
bool write_encrypted_log_chunk(const uint8_t* data, size_t len) {
    static SimpleEncryptStream encrypt_stream;
    static bool initialized = false;
    
    if (!initialized) {
        uint8_t key[32];
        // Derive key from LEIGH_KEY parameter
        if (!get_encryption_key(key)) {
            return false;
        }
        if (!encrypt_stream.init(key)) {
            return false;
        }
        initialized = true;
        
        // Write nonce to file (first 16 bytes)
        uint8_t nonce[16];
        encrypt_stream.get_nonce(nonce);
        file_write(nonce, 16);
    }
    
    // Encrypt chunk
    uint8_t ciphertext[len];
    if (!encrypt_stream.encrypt_chunk(data, len, ciphertext)) {
        return false;
    }
    
    // Write ciphertext
    file_write(ciphertext, len);
    
    return true;
}

bool finalize_encrypted_log() {
    // Finalize encryption stream
    uint8_t mac[16];
    if (!encrypt_stream.finalize(mac)) {
        return false;
    }
    
    // Write MAC to end of file
    file_write(mac, 16);
    
    return true;
}
```

## Testing

### Test Vector

Use this test to verify implementation matches Python:

```cpp
// Test with LEIGH_KEY = 74768360.0
float test_leigh_key = 74768360.0f;
const uint8_t salt[16] = "LEIGH_KEY_SALT_1";
uint8_t key[32];

derive_key_from_leigh_key(test_leigh_key, salt, key);

// Expected key (first 16 bytes in hex):
// Should match Python output when using same LEIGH_KEY
// Verify: printf("Key: ");
// for (int i = 0; i < 16; i++) printf("%02x", key[i]);
```

### Verification Steps

1. **Encrypt a test file** on ArduPilot side
2. **Decrypt with Python** using same LEIGH_KEY
3. **Verify plaintext matches** original
4. **Test with different LEIGH_KEY values**
5. **Test with large files** (streaming)

## Implementation Checklist

- [ ] Implement `derive_key_from_leigh_key()` function
- [ ] Implement or integrate HMAC-SHA256 function
- [ ] Implement `generate_keystream()` function
- [ ] Implement `encrypt_simple()` function
- [ ] Implement streaming encryption class (for large files)
- [ ] Integrate with ArduPilot parameter system (LEIGH_KEY)
- [ ] Integrate with log file writer
- [ ] Add random number generation for nonce
- [ ] Test with known test vectors
- [ ] Verify compatibility with Python decryption

## Dependencies

**Required:**
- SHA-256 implementation (for HMAC)
- PBKDF2 implementation (for key derivation)
- Random number generator (for nonce)

**Optional:**
- OpenSSL (if available, provides PBKDF2 and HMAC-SHA256)
- ArduPilot crypto library (if available)

**If no crypto library available:**
- Implement simple SHA-256 (public domain implementations available)
- Implement simple PBKDF2 using SHA-256
- Implement HMAC using SHA-256 (RFC 2104)

## Performance Considerations

- **PBKDF2 iterations**: 10000 iterations adds ~10-50ms per key derivation (acceptable)
- **HMAC-SHA256**: Fast, ~1-5MB/s on typical embedded processors
- **XOR encryption**: Very fast, minimal overhead
- **Memory**: Streaming implementation uses constant memory (good for embedded)

## Security Notes

- **PBKDF2**: Slows down brute-force attacks (10000 iterations)
- **HMAC-SHA256**: Industry-standard MAC algorithm
- **Random nonce**: Each file gets unique nonce (prevents replay attacks)
- **Key derivation**: Uses salt to prevent rainbow table attacks

## Compatibility

This implementation must match the Python decryption exactly:
- Same key derivation (float → bytes → PBKDF2)
- Same keystream generation (HMAC-SHA256 with counter)
- Same MAC computation (HMAC-SHA256 of nonce + ciphertext)
- Same file format (nonce + ciphertext + MAC)

## Questions or Issues?

If you encounter issues:
1. Verify float-to-bytes conversion matches Python (`struct.pack('<f', value)`)
2. Verify PBKDF2 parameters match (SHA-256, 10000 iterations)
3. Verify HMAC-SHA256 implementation matches standard
4. Test with known test vectors
5. Compare intermediate values (nonce, keystream blocks) with Python

## Reference Implementation

See `simple_encrypt.py` in the Python decryption side for reference implementation that this must match exactly.

