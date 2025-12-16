# C++ and Python Implementation Compatibility

This document ensures the C++ and Python implementations of AP_Crypto are fully compatible.

## Algorithm Compatibility

### Key Derivation

**Python (CryptoTool.py):**
```python
def derive_key_from_leigh_key_simple(leigh_key_value):
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes
    seed_bytes = struct.pack('<i', leigh_key_value)  # INT32 little-endian
    return hashlib.sha256(seed_bytes + salt).digest()
```

**C++ (AP_Crypto_Simple.cpp):**
```cpp
bool AP_Crypto_Simple::derive_key_from_leigh_key(int32_t leigh_key_value, uint8_t key_out[32])
{
    const uint8_t salt[16] = {
        0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
        0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31
    }; // "LEIGH_KEY_SALT_1" in ASCII
    
    // Pack INT32 as little-endian (matches Python struct.pack('<i', ...))
    uint8_t leigh_key_bytes[4];
    leigh_key_bytes[0] = (uint8_t)(leigh_key_value & 0xFF);
    leigh_key_bytes[1] = (uint8_t)((leigh_key_value >> 8) & 0xFF);
    leigh_key_bytes[2] = (uint8_t)((leigh_key_value >> 16) & 0xFF);
    leigh_key_bytes[3] = (uint8_t)((leigh_key_value >> 24) & 0xFF);
    
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, leigh_key_bytes, 4);
    sha256_update(&ctx, salt, sizeof(salt));
    sha256_final(&ctx, key_out);
    
    return true;
}
```

**Compatibility:** ✅ **MATCHES**
- Both use SHA-256 (Python: `hashlib.sha256`, C++: `sha256()`)
- Both pack INT32 as little-endian 4 bytes
- Both concatenate: `seed_bytes + salt`
- Both produce 32-byte output

### Keystream Generation

**Python (CryptoTool.py):**
```python
def generate_keystream_block(key, counter):
    counter_bytes = struct.pack('<Q', counter)  # uint64_t little-endian
    return hashlib.sha256(key + counter_bytes).digest()
```

**C++ (AP_Crypto_Simple.cpp):**
```cpp
bool AP_Crypto_Simple::generate_keystream_block(const uint8_t key[32], uint64_t counter, uint8_t keystream_out[32])
{
    // Pack uint64_t as little-endian (matches Python struct.pack('<Q', ...))
    uint8_t counter_bytes[8];
    counter_bytes[0] = (uint8_t)(counter & 0xFF);
    counter_bytes[1] = (uint8_t)((counter >> 8) & 0xFF);
    counter_bytes[2] = (uint8_t)((counter >> 16) & 0xFF);
    counter_bytes[3] = (uint8_t)((counter >> 24) & 0xFF);
    counter_bytes[4] = (uint8_t)((counter >> 32) & 0xFF);
    counter_bytes[5] = (uint8_t)((counter >> 40) & 0xFF);
    counter_bytes[6] = (uint8_t)((counter >> 48) & 0xFF);
    counter_bytes[7] = (uint8_t)((counter >> 56) & 0xFF);
    
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, key, 32);
    sha256_update(&ctx, counter_bytes, 8);
    sha256_final(&ctx, keystream_out);
    
    return true;
}
```

**Compatibility:** ✅ **MATCHES**
- Both use SHA-256
- Both pack uint64_t counter as little-endian 8 bytes
- Both concatenate: `key + counter_bytes`
- Both produce 32-byte keystream block

### Encryption/Decryption

**Python (CryptoTool.py):**
```python
def encrypt_simple_xor_format(key_bytes, plaintext):
    counter = 0
    i = 0
    ciphertext = bytearray()
    
    while i < len(plaintext):
        keystream = generate_keystream_block(key_bytes, counter)
        block_size = min(32, len(plaintext) - i)
        for j in range(block_size):
            ciphertext.append(plaintext[i + j] ^ keystream[j])
        i += block_size
        counter += 1
    
    return bytes(ciphertext)
```

**C++ (AP_Crypto_Simple.cpp):**
```cpp
int AP_Crypto_Simple::encrypt_simple(const uint8_t key[32],
                                     const uint8_t *plaintext, size_t plaintext_len,
                                     uint8_t *ciphertext_out, size_t ciphertext_max)
{
    uint64_t counter = 0;
    size_t offset = 0;
    
    while (offset < plaintext_len) {
        uint8_t keystream[32];
        if (!generate_keystream_block(key, counter, keystream)) {
            return -1;
        }
        
        size_t block_size = (plaintext_len - offset < 32) ? (plaintext_len - offset) : 32;
        for (size_t i = 0; i < block_size; i++) {
            ciphertext_out[offset + i] = plaintext[offset + i] ^ keystream[i];
        }
        
        offset += block_size;
        counter++;
    }
    
    return plaintext_len;
}
```

**Compatibility:** ✅ **MATCHES**
- Both start counter at 0
- Both process in 32-byte blocks
- Both XOR plaintext with keystream
- Both increment counter after each block
- Decryption is identical (XOR is symmetric)

## Byte Order Verification

### INT32 (LEIGH_KEY)
- **Python:** `struct.pack('<i', value)` → 4 bytes little-endian
- **C++:** Manual packing with bit shifts → 4 bytes little-endian
- **Match:** ✅

### uint64_t (Counter)
- **Python:** `struct.pack('<Q', value)` → 8 bytes little-endian
- **C++:** Manual packing with bit shifts → 8 bytes little-endian
- **Match:** ✅

## Hash Function

- **Python:** `hashlib.sha256()` (standard library)
- **C++:** `sha256()` (custom implementation in sha256.cpp)
- **Compatibility:** ✅ Both implement SHA-256 standard (FIPS 180-4)

## File Format

Both implementations use the same format:
- **No header** (no version, timestamp, or nonce)
- **No MAC** (no authentication tag)
- **Just ciphertext** (XOR-encrypted data)

## Testing Compatibility

To verify compatibility:

1. **Encrypt with Python:**
   ```python
   from CryptoTool import encrypt_simple_xor_format, derive_key_from_leigh_key_simple
   key = derive_key_from_leigh_key_simple(74768361)
   ciphertext = encrypt_simple_xor_format(key, b"Hello, World!")
   ```

2. **Decrypt with C++:**
   ```cpp
   uint8_t key[32];
   AP_Crypto_Simple::derive_key_from_leigh_key(74768361, key);
   uint8_t plaintext[13];
   AP_Crypto_Simple::decrypt_simple(key, ciphertext, 13, plaintext, 13);
   // plaintext should equal "Hello, World!"
   ```

3. **Vice versa:** Encrypt with C++, decrypt with Python

## Summary

✅ **Full Compatibility Achieved:**
- Same hash function (SHA-256)
- Same key derivation (SHA-256 of little-endian INT32 + salt)
- Same keystream generation (SHA-256 of key + little-endian uint64_t counter)
- Same encryption algorithm (XOR with keystream blocks)
- Same file format (ciphertext only, no header/MAC)
- Same byte order (little-endian for all integers)

The C++ and Python implementations are now fully compatible and can encrypt/decrypt each other's files.

