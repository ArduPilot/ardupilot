# AP_Crypto Library Documentation

## Overview

The `AP_Crypto` library provides cryptographic functionality for ArduPilot, enabling encryption and decryption of files, Lua scripts, and log data. The library uses simple XOR encryption based on SHA-256 for lightweight, Python-compatible encryption.

The library is designed to be fully compatible with Python implementations, allowing files encrypted on a host computer to be decrypted by ArduPilot firmware, and vice versa.

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Configuration](#configuration)
- [Parameters](#parameters)
- [API Reference](#api-reference)
- [Encryption Schemes](#encryption-schemes)
- [Usage Examples](#usage-examples)
- [Python Compatibility](#python-compatibility)
- [Integration Guide](#integration-guide)
- [Security Considerations](#security-considerations)

## Features

- ✅ **Simple XOR Encryption**: SHA-256 based encryption for lightweight, Python-compatible operation
- ✅ **Streaming Support**: Process large files in chunks without loading entire file into memory
- ✅ **Key Management**: Secure key storage and retrieval via parameters
- ✅ **Python Compatible**: Full compatibility with Python's `hashlib.sha256`
- ✅ **Lua Script Encryption**: Automatic decryption of encrypted Lua scripts
- ✅ **Log File Encryption**: Optional encryption of log files written to SD card
- ✅ **USB Terminal Encryption**: Optional encryption of log downloads via USB
- ✅ **MAVLink Integration**: Set encryption keys via MAVLink parameters

## Architecture

### Components

1. **AP_Crypto** - Key storage and retrieval functions
2. **AP_Crypto_Simple** - Simple XOR encryption (SHA-256 based, Python-compatible)
3. **AP_Crypto_Simple_Streaming** - Streaming support for large files
4. **AP_Crypto_Params** - Parameter management for encryption keys
5. **sha256** - SHA-256 hash implementation (matches Python's hashlib.sha256)

### File Structure

```
libraries/AP_Crypto/
├── AP_Crypto.h                    # Key storage API
├── AP_Crypto.cpp                  # Key storage implementation
├── AP_Crypto_Simple.h             # Simple XOR encryption API
├── AP_Crypto_Simple.cpp           # Simple XOR implementation
├── AP_Crypto_Simple_Streaming.h  # Streaming encryption API
├── AP_Crypto_Simple_Streaming.cpp # Streaming implementation
├── AP_Crypto_Params.h             # Parameter management
├── AP_Crypto_Params.cpp           # Parameter implementation
├── AP_Crypto_config.h             # Configuration header
├── sha256.h                       # SHA-256 header
├── sha256.cpp                     # SHA-256 implementation
└── README.md                      # This file
```

## Configuration

### Build Configuration

The library is controlled by compile-time defines in `AP_Crypto_config.h`:

```cpp
#ifndef AP_CRYPTO_ENABLED
#define AP_CRYPTO_ENABLED 1  // Enable/disable the entire library
#endif
```

### Build System Integration

The library is automatically included when `AP_Crypto` is added to:
- `ArduPlane/wscript` (or other vehicle wscript files)
- `Tools/ardupilotwaf/ardupilotwaf.py` → `COMMON_VEHICLE_DEPENDENT_LIBRARIES`

## Parameters

### LEIGH_KEY (INT32)

**Description**: Encryption key for file encryption. The INT32 value is used to derive a 32-byte encryption key via SHA-256.

**Range**: -2147483648 to 2147483647

**Behavior**:
- **When set via MAVLink**: The INT32 value is hashed with salt `"LEIGH_KEY_SALT_1"` using SHA-256 to produce a 32-byte key which is stored in secure storage.
- **When read via MAVLink**: Always returns `0` to prevent key disclosure.

**Key Derivation Algorithm**:
```cpp
// C++ implementation
uint8_t leigh_key_bytes[4];  // Little-endian INT32
leigh_key_bytes[0] = (uint8_t)(value & 0xFF);
leigh_key_bytes[1] = (uint8_t)((value >> 8) & 0xFF);
leigh_key_bytes[2] = (uint8_t)((value >> 16) & 0xFF);
leigh_key_bytes[3] = (uint8_t)((value >> 24) & 0xFF);

uint8_t salt[16] = "LEIGH_KEY_SALT_1";  // ASCII
uint8_t key[32] = SHA256(leigh_key_bytes + salt);
```

**Python Equivalent**:
```python
import hashlib
import struct

def derive_key_from_leigh_key(leigh_key_value):
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes
    seed_bytes = struct.pack('<i', leigh_key_value)  # INT32 little-endian
    return hashlib.sha256(seed_bytes + salt).digest()  # 32 bytes
```

**Example**: Setting `LEIGH_KEY = 74768361` derives a unique 32-byte key.

### LEIGH_CRYPT_LVL (INT32, Bitmask)

**Description**: Bitmask to selectively control what gets encrypted.

**Range**: 0 to 3

**Bitmask Values**:
- **Bit 0 (0x01)**: `LEIGH_CRYPT_LOG_FILES` - Enable encryption for log files written to SD card
- **Bit 1 (0x02)**: `LEIGH_CRYPT_USB_TERMINAL` - Enable encryption for USB terminal output (log downloads)
- **Value 0**: Disables all AP_CRYPTO encryption

**Usage**:
```cpp
// Check if log file encryption is enabled
if (AP::crypto_params().is_encryption_enabled(LEIGH_CRYPT_LOG_FILES)) {
    // Encrypt log file
}

// Check if USB terminal encryption is enabled
if (AP::crypto_params().is_encryption_enabled(LEIGH_CRYPT_USB_TERMINAL)) {
    // Encrypt USB terminal output
}
```

**Note**: When `LEIGH_CRYPT_LVL = 0`, the `LEIGH_KEY` parameter value becomes readable via MAVLink for debugging purposes.

## API Reference

### AP_Crypto Class

Key storage and retrieval functions for secure key management.

#### Static Methods

##### `store_key()`
Store encryption key in persistent storage.

```cpp
static bool store_key(const uint8_t key[32]);
```

**Parameters**:
- `key`: 32-byte encryption key (raw bytes)

**Returns**: `true` on success, `false` on failure

##### `retrieve_key()`
Retrieve encryption key from persistent storage.

```cpp
static bool retrieve_key(uint8_t key[32]);
```

**Parameters**:
- `key`: Output buffer for 32-byte key

**Returns**: `true` if key was found and retrieved, `false` otherwise

##### `has_stored_key()`
Check if a key is stored in persistent storage.

```cpp
static bool has_stored_key(void);
```

**Returns**: `true` if key exists, `false` otherwise

##### `generate_and_store_key()`
Generate and store a new encryption key.

```cpp
static bool generate_and_store_key(uint8_t key[32] = nullptr);
```

**Parameters**:
- `key`: Output buffer for generated key (optional, can be `nullptr`)

**Returns**: `true` on success, `false` on failure

##### `derive_key_from_board_id()`
Derive key from board ID (DISABLED - always returns false).

```cpp
static bool derive_key_from_board_id(uint8_t key[32]);
```

**Returns**: Always returns `false` (functionality disabled)

### AP_Crypto_Simple Class

Simple XOR encryption using SHA-256. Designed for Python compatibility.

#### Static Methods

##### `derive_key_from_leigh_key()`
Derive 32-byte key from LEIGH_KEY INT32 value.

```cpp
static bool derive_key_from_leigh_key(int32_t leigh_key_value, uint8_t key_out[32]);
```

**Algorithm**: `SHA256(little-endian INT32 + "LEIGH_KEY_SALT_1")`

##### `generate_keystream_block()`
Generate 32-byte keystream block for given counter.

```cpp
static bool generate_keystream_block(const uint8_t key[32], uint64_t counter, 
                                    uint8_t keystream_out[32]);
```

**Algorithm**: `SHA256(key + little-endian uint64_t counter)`

##### `encrypt_simple()`
Encrypt data using simple XOR cipher.

```cpp
static int encrypt_simple(const uint8_t key[32],
                         const uint8_t *plaintext, size_t plaintext_len,
                         uint8_t *ciphertext_out, size_t ciphertext_max);
```

**Algorithm**: XOR plaintext with keystream blocks (32 bytes per block)

##### `decrypt_simple()`
Decrypt data using simple XOR cipher.

```cpp
static int decrypt_simple(const uint8_t key[32],
                         const uint8_t *ciphertext, size_t ciphertext_len,
                         uint8_t *plaintext_out, size_t plaintext_max);
```

**Note**: Decryption is identical to encryption (XOR is symmetric)

### AP_Crypto_Simple_Streaming Class

Streaming support for large files using simple XOR encryption.

#### Structures

```cpp
struct SimpleXORStreamEncrypt {
    uint8_t key[32];
    uint64_t counter;
    size_t bytes_encrypted;
    bool initialized;
};

struct SimpleXORStreamDecrypt {
    uint8_t key[32];
    uint64_t counter;
    size_t bytes_decrypted;
    bool initialized;
};
```

#### Static Methods

```cpp
// Initialize streaming encryption
static bool streaming_encrypt_init(SimpleXORStreamEncrypt *ctx, const uint8_t key[32]);

// Encrypt chunk
static ssize_t streaming_encrypt_write(SimpleXORStreamEncrypt *ctx,
                                       const uint8_t *plaintext, size_t plaintext_len,
                                       uint8_t *ciphertext_out, size_t ciphertext_max);

// Cleanup
static void streaming_encrypt_cleanup(SimpleXORStreamEncrypt *ctx);

// Initialize streaming decryption
static bool streaming_decrypt_init(SimpleXORStreamDecrypt *ctx, const uint8_t key[32]);

// Decrypt chunk
static ssize_t streaming_decrypt_read(SimpleXORStreamDecrypt *ctx,
                                     const uint8_t *ciphertext, size_t ciphertext_len,
                                     uint8_t *plaintext_out, size_t plaintext_max);

// Cleanup
static void streaming_decrypt_cleanup(SimpleXORStreamDecrypt *ctx);
```

### AP_Crypto_Params Class

Parameter management for encryption keys.

#### Methods

```cpp
// Get singleton instance
static AP_Crypto_Params *get_singleton(void);

// Check if encryption is enabled for feature
bool is_encryption_enabled(uint32_t feature) const;

// Handle key set (called internally when LEIGH_KEY is set)
void handle_key_set(int32_t value);
```

#### Global Access

```cpp
namespace AP {
    AP_Crypto_Params &crypto_params();
}
```

## Encryption Scheme

### Simple XOR Encryption (AP_Crypto_Simple)

**Algorithm**: SHA-256 based XOR cipher

**Format**:
```
[Ciphertext: variable length]
```

**Features**:
- No header, no MAC, no nonce
- Python-compatible (uses SHA-256)
- Lightweight and fast
- Deterministic (same plaintext + key = same ciphertext)

**Algorithm Details**:
1. **Key Derivation**: `SHA256(little-endian INT32 + "LEIGH_KEY_SALT_1")` → 32-byte key
2. **Keystream Generation**: For each 32-byte block, `SHA256(key + little-endian uint64_t counter)` → 32-byte keystream
3. **Encryption**: XOR plaintext with keystream block by block

**Use Cases**:
- Lua script encryption
- Log file encryption
- When Python compatibility is required
- When minimal overhead is desired

## Usage Examples

### Example 1: Encrypt Data with Simple XOR

```cpp
#include <AP_Crypto/AP_Crypto_Simple.h>

// Derive key from LEIGH_KEY parameter
int32_t leigh_key = AP::crypto_params().leigh_key.get();
uint8_t key[32];
if (!AP_Crypto_Simple::derive_key_from_leigh_key(leigh_key, key)) {
    // Error handling
    return;
}

// Encrypt data
const uint8_t plaintext[] = "Hello, World!";
uint8_t ciphertext[256];
int ciphertext_len = AP_Crypto_Simple::encrypt_simple(
    key, plaintext, sizeof(plaintext) - 1, ciphertext, sizeof(ciphertext));

if (ciphertext_len < 0) {
    // Error handling
    return;
}

// Write ciphertext to file
// ...
```

### Example 2: Decrypt Data with Simple XOR

```cpp
#include <AP_Crypto/AP_Crypto_Simple.h>

// Read ciphertext from file
uint8_t ciphertext[256];
size_t ciphertext_len = read_file("encrypted.bin", ciphertext, sizeof(ciphertext));

// Derive key
int32_t leigh_key = AP::crypto_params().leigh_key.get();
uint8_t key[32];
AP_Crypto_Simple::derive_key_from_leigh_key(leigh_key, key);

// Decrypt
uint8_t plaintext[256];
int plaintext_len = AP_Crypto_Simple::decrypt_simple(
    key, ciphertext, ciphertext_len, plaintext, sizeof(plaintext));

if (plaintext_len < 0) {
    // Error handling
    return;
}

// Use plaintext
// ...
```

### Example 3: Streaming Encryption for Large Files

```cpp
#include <AP_Crypto/AP_Crypto_Simple_Streaming.h>

// Initialize streaming encryption
SimpleXORStreamEncrypt ctx;
uint8_t key[32];
// ... derive key ...
if (!AP_Crypto_Simple_Streaming::streaming_encrypt_init(&ctx, key)) {
    // Error handling
    return;
}

// Encrypt file in chunks
FILE *fp = fopen("large_file.bin", "rb");
uint8_t buffer[4096];
uint8_t encrypted_buffer[4096];

while (!feof(fp)) {
    size_t bytes_read = fread(buffer, 1, sizeof(buffer), fp);
    if (bytes_read == 0) break;
    
    ssize_t bytes_encrypted = AP_Crypto_Simple_Streaming::streaming_encrypt_write(
        &ctx, buffer, bytes_read, encrypted_buffer, sizeof(encrypted_buffer));
    
    if (bytes_encrypted < 0) {
        // Error handling
        break;
    }
    
    // Write encrypted_buffer to output file
    fwrite(encrypted_buffer, 1, bytes_encrypted, output_fp);
}

// Cleanup
AP_Crypto_Simple_Streaming::streaming_encrypt_cleanup(&ctx);
fclose(fp);
```

### Example 4: Check Encryption Settings

```cpp
#include <AP_Crypto/AP_Crypto_Params.h>

// Check if log file encryption is enabled
if (AP::crypto_params().is_encryption_enabled(LEIGH_CRYPT_LOG_FILES)) {
    // Encrypt log file before writing to SD card
    encrypt_log_file();
}

// Check if USB terminal encryption is enabled
if (AP::crypto_params().is_encryption_enabled(LEIGH_CRYPT_USB_TERMINAL)) {
    // Encrypt data before sending via USB
    encrypt_usb_output();
}
```

### Example 5: Set Encryption Key via Parameter

```cpp
// When LEIGH_KEY parameter is set via MAVLink, the key is automatically
// derived and stored. The handle_key_set() method is called internally.

// To manually set the key:
int32_t leigh_key_value = 74768361;
AP::crypto_params().handle_key_set(leigh_key_value);
// Key is now derived and stored in secure storage
```

## Python Compatibility

The `AP_Crypto_Simple` implementation is fully compatible with Python's `hashlib.sha256`. Files encrypted with Python can be decrypted by ArduPilot, and vice versa.

### Python Encryption Example

```python
import hashlib
import struct

def derive_key_from_leigh_key(leigh_key_value):
    """Derive 32-byte key from LEIGH_KEY INT32 value."""
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes
    seed_bytes = struct.pack('<i', leigh_key_value)  # INT32 little-endian
    return hashlib.sha256(seed_bytes + salt).digest()  # 32 bytes

def generate_keystream_block(key, counter):
    """Generate 32-byte keystream block for given counter."""
    counter_bytes = struct.pack('<Q', counter)  # uint64_t little-endian
    return hashlib.sha256(key + counter_bytes).digest()  # 32 bytes

def encrypt_simple(plaintext, leigh_key_value):
    """Encrypt plaintext using simple XOR cipher."""
    key = derive_key_from_leigh_key(leigh_key_value)
    
    ciphertext = bytearray()
    counter = 0
    
    for i in range(0, len(plaintext), 32):
        keystream = generate_keystream_block(key, counter)
        block_size = min(32, len(plaintext) - i)
        for j in range(block_size):
            ciphertext.append(plaintext[i + j] ^ keystream[j])
        counter += 1
    
    return bytes(ciphertext)

# Usage
plaintext = b"Hello, World!"
ciphertext = encrypt_simple(plaintext, 74768361)
```

### Compatibility Verification

✅ **Key Derivation**: Both use SHA-256 with identical byte order  
✅ **Keystream Generation**: Both use SHA-256 with identical counter packing  
✅ **Encryption Algorithm**: Both use XOR with 32-byte blocks  
✅ **Byte Order**: Both use little-endian for all integers  
✅ **File Format**: Both use ciphertext-only format (no header/MAC)

## Integration Guide

### Adding AP_Crypto to a Vehicle

1. **Add to vehicle wscript** (e.g., `ArduPlane/wscript`):
```python
bld.ap_stlib(
    name=vehicle + '_libs',
    ap_vehicle=vehicle,
    ap_libraries=bld.ap_common_vehicle_libraries() + [
        # ... other libraries ...
        'AP_Crypto',
    ],
)
```

2. **Add to common libraries** (`Tools/ardupilotwaf/ardupilotwaf.py`):
```python
COMMON_VEHICLE_DEPENDENT_LIBRARIES = [
    # ... existing libraries ...
    'AP_Crypto',
]
```

3. **Create parameter instance** (in vehicle's Parameters class):
```cpp
#include <AP_Crypto/AP_Crypto_Params.h>

class Parameters {
    // ... other parameters ...
    AP_Crypto_Params crypto_params;
};
```

### Using in Log File Writing

```cpp
#include <AP_Crypto/AP_Crypto_Simple.h>
#include <AP_Crypto/AP_Crypto_Params.h>

void write_encrypted_log_file(const uint8_t *data, size_t len) {
    // Check if encryption is enabled
    if (!AP::crypto_params().is_encryption_enabled(LEIGH_CRYPT_LOG_FILES)) {
        // Write unencrypted
        write_file(data, len);
        return;
    }
    
    // Derive key from LEIGH_KEY
    int32_t leigh_key = AP::crypto_params().leigh_key.get();
    uint8_t key[32];
    if (!AP_Crypto_Simple::derive_key_from_leigh_key(leigh_key, key)) {
        // Error: fall back to unencrypted
        write_file(data, len);
        return;
    }
    
    // Encrypt data
    uint8_t ciphertext[4096];
    int ciphertext_len = AP_Crypto_Simple::encrypt_simple(
        key, data, len, ciphertext, sizeof(ciphertext));
    
    if (ciphertext_len < 0) {
        // Error: fall back to unencrypted
        write_file(data, len);
        return;
    }
    
    // Write encrypted data
    write_file(ciphertext, ciphertext_len);
}
```

### Using in Lua Script Loading

The Lua script loader automatically detects `.lua.enc` files and decrypts them using `AP_Crypto_Simple` before execution. No additional code is required.

## Security Considerations

### Key Management

- **LEIGH_KEY Parameter**: When read via MAVLink, always returns `0` to prevent key disclosure
- **Key Storage**: Keys are stored in secure storage (EEPROM/Flash) and are not accessible via normal parameter reads
- **Key Derivation**: Uses SHA-256 with salt to derive keys from INT32 values

### Encryption Strength

- **Simple XOR**: Uses SHA-256 for key derivation and keystream generation
  - **Note**: Simple XOR does not provide authentication (no MAC). Suitable for scenarios where authentication is not required.

### Limitations

1. **Simple XOR Encryption**:
   - No authentication (no MAC)
   - Deterministic (same plaintext + key = same ciphertext)
   - Not suitable for scenarios requiring authentication or nonce-based security

2. **Key Derivation**:
   - INT32 range limits key space to 2^32 possible keys
   - For stronger security, use `AP_Crypto::generate_and_store_key()` to create random keys

3. **Performance**:
   - Simple XOR is fast and lightweight
   - Suitable for embedded systems with limited resources

### Best Practices

1. **Use Simple XOR encryption** when:
   - Python compatibility is required
   - Performance is critical
   - Authentication is not required
   - Files are stored in secure locations

3. **Key Management**:
   - Use strong LEIGH_KEY values (avoid common numbers)
   - Store LEIGH_KEY securely (not in source code)
   - Rotate keys periodically if possible

4. **Error Handling**:
   - Always check return values
   - Fall back to unencrypted mode on errors (if appropriate)
   - Log encryption errors for debugging

## Dependencies

- **SHA-256**: Custom implementation in `sha256.cpp` (matches Python's hashlib.sha256)
- **AP_Param**: For parameter management
- **StorageManager**: For secure key storage

## Testing

### Compatibility Test

To verify C++ and Python implementations are compatible:

```python
# Python: Encrypt
import hashlib
import struct

def derive_key(leigh_key):
    salt = b'LEIGH_KEY_SALT_1'
    seed = struct.pack('<i', leigh_key)
    return hashlib.sha256(seed + salt).digest()

def encrypt(plaintext, key):
    ciphertext = bytearray()
    counter = 0
    for i in range(0, len(plaintext), 32):
        keystream = hashlib.sha256(key + struct.pack('<Q', counter)).digest()
        block_size = min(32, len(plaintext) - i)
        for j in range(block_size):
            ciphertext.append(plaintext[i + j] ^ keystream[j])
        counter += 1
    return bytes(ciphertext)

# Encrypt with Python
key = derive_key(74768361)
ciphertext = encrypt(b"Hello, World!", key)

# Save to file
with open("test.enc", "wb") as f:
    f.write(ciphertext)
```

```cpp
// C++: Decrypt
uint8_t key[32];
AP_Crypto_Simple::derive_key_from_leigh_key(74768361, key);

// Read ciphertext from file
uint8_t ciphertext[256];
size_t len = read_file("test.enc", ciphertext, sizeof(ciphertext));

// Decrypt
uint8_t plaintext[256];
int plaintext_len = AP_Crypto_Simple::decrypt_simple(
    key, ciphertext, len, plaintext, sizeof(plaintext));

// Verify: plaintext should equal "Hello, World!"
```

## Troubleshooting

### Build Errors

**Error**: `undefined reference to 'sha256_init'`
- **Solution**: Ensure `sha256.cpp` is compiled and linked. Check that `AP_Crypto` is in the library list.

**Error**: `'AP_CRYPTO_ENABLED' is not defined`
- **Solution**: Include `AP_Crypto_config.h` in files using AP_Crypto.

### Runtime Errors

**Decryption fails with correct key**:
- Verify byte order matches (little-endian)
- Check that same key derivation algorithm is used
- Ensure file wasn't corrupted during transfer

**Key derivation produces different keys**:
- Verify both implementations use SHA-256
- Check that salt matches exactly: `"LEIGH_KEY_SALT_1"`
- Ensure INT32 is packed as little-endian in both

### Performance Issues

**Encryption is too slow**:
- Use Simple XOR instead of Fernet-compatible for better performance
- Use streaming encryption for large files
- Consider encrypting only sensitive portions of files

## Related Documentation

- [COMPATIBILITY.md](COMPATIBILITY.md) - Detailed compatibility information
- [SIMPLE_XOR_ENCRYPTION.md](SIMPLE_XOR_ENCRYPTION.md) - Simple XOR encryption details
- [STREAMING_ENCRYPTION_GUIDE.md](STREAMING_ENCRYPTION_GUIDE.md) - Streaming encryption guide
- [LEIGH_CRYPT_LEVEL_SETTINGS.md](LEIGH_CRYPT_LEVEL_SETTINGS.md) - Parameter settings guide
- [LUA_ENCRYPTION_COMPLETE.md](LUA_ENCRYPTION_COMPLETE.md) - Lua script encryption

## License

This library is part of ArduPilot and is licensed under the GNU General Public License v3.0.

## Version History

- **v1.0** (Current): Initial release
  - Fernet-compatible encryption (ChaCha20-Poly1305)
  - Simple XOR encryption (SHA-256 based)
  - Python compatibility
  - Streaming support
  - Parameter-based key management

## Contributing

When contributing to AP_Crypto:

1. Maintain Python compatibility for `AP_Crypto_Simple`
2. Test encryption/decryption with both C++ and Python
3. Update this documentation for any API changes
4. Follow ArduPilot coding standards
5. Add unit tests for new functionality

## Support

For issues or questions:
- Check existing documentation files in `libraries/AP_Crypto/`
- Review compatibility documentation
- Test with Python reference implementation
- Check build system integration

---

**Last Updated**: 2024-12-15  
**Maintainer**: ArduPilot Development Team

