# Encryption Key Storage

This document describes the key storage and retrieval system for AP_Crypto encryption keys.

## Overview

The AP_Crypto library provides secure key storage and retrieval using ArduPilot's StorageManager. Keys are stored persistently in non-volatile storage and can be retrieved across reboots.

## Key Storage Location

Keys are stored in the `StorageKeys` area of StorageManager:
- **Storage Type**: `StorageManager::StorageKeys`
- **Size**: 64 bytes (32 bytes for key + header)
- **Format**: Magic number + version + 32-byte key

## API Functions

### Store Key

```cpp
bool AP_Crypto::store_key(const uint8_t key[32]);
```

Stores a 32-byte encryption key in persistent storage.

**Parameters:**
- `key`: 32-byte encryption key (raw bytes)

**Returns:**
- `true`: Key stored successfully
- `false`: Storage failed (e.g., insufficient space)

**Example:**
```cpp
uint8_t my_key[32] = { /* your key bytes */ };
if (AP_Crypto::store_key(my_key)) {
    // Key stored successfully
}
```

### Retrieve Key

```cpp
bool AP_Crypto::retrieve_key(uint8_t key[32]);
```

Retrieves the stored encryption key from persistent storage.

**Parameters:**
- `key`: Output buffer for 32-byte key

**Returns:**
- `true`: Key retrieved successfully
- `false`: No key stored or retrieval failed

**Example:**
```cpp
uint8_t key[32];
if (AP_Crypto::retrieve_key(key)) {
    // Use key for encryption/decryption
}
```

### Check for Stored Key

```cpp
bool AP_Crypto::has_stored_key(void);
```

Checks if a key is currently stored in persistent storage.

**Returns:**
- `true`: Key exists in storage
- `false`: No key stored

**Example:**
```cpp
if (AP_Crypto::has_stored_key()) {
    // Key is available
}
```

### Generate and Store Key

```cpp
bool AP_Crypto::generate_and_store_key(uint8_t key[32] = nullptr);
```

Generates a new random encryption key and stores it in persistent storage.

**Parameters:**
- `key`: Optional output buffer for generated key (can be `nullptr`)

**Returns:**
- `true`: Key generated and stored successfully
- `false`: Generation or storage failed

**Example:**
```cpp
uint8_t new_key[32];
if (AP_Crypto::generate_and_store_key(new_key)) {
    // New key generated and stored
    // new_key contains the generated key
}
```

### Derive Key from Board ID

```cpp
bool AP_Crypto::derive_key_from_board_id(uint8_t key[32]);
```

Derives a deterministic encryption key from the board's unique ID. This ensures each device has a unique key while maintaining consistency across reboots.

**Parameters:**
- `key`: Output buffer for 32-byte derived key

**Returns:**
- `true`: Key derived successfully
- `false`: Board ID not available

**Example:**
```cpp
uint8_t key[32];
if (AP_Crypto::derive_key_from_board_id(key)) {
    // Key derived from board ID
    // Store it for future use
    AP_Crypto::store_key(key);
}
```

## Key Retrieval Priority

The key retrieval functions (`log_get_default_key()` and `lua_get_default_key()`) use the following priority:

1. **Stored Key**: Retrieve key from persistent storage
2. **Board ID Derivation**: Derive key from board ID (if no stored key)
3. **Fallback**: Use hardcoded default key (PoC only, not secure)

This ensures:
- Keys persist across reboots
- Each device can have a unique key
- Backward compatibility with PoC implementation

## Storage Format

The key is stored with the following structure:

```cpp
struct crypto_key_header {
    uint32_t magic;      // 0x43525950 ("CRYP" in ASCII)
    uint32_t version;    // 1 (for future compatibility)
    uint8_t key[32];     // The actual 32-byte encryption key
};
```

**Total Size**: 40 bytes (4 + 4 + 32)

## Security Considerations

### Current Implementation

- **Storage Location**: Uses StorageManager's `StorageKeys` area
- **Persistence**: Keys survive reboots and firmware updates
- **Uniqueness**: Can derive unique keys per device from board ID
- **Fallback**: Hardcoded key for PoC compatibility (not secure)

### Production Recommendations

1. **Key Rotation**: Implement key rotation mechanism
2. **Secure Storage**: Consider hardware security modules for sensitive deployments
3. **Key Backup**: Provide mechanism to backup/restore keys
4. **Access Control**: Restrict key access to authorized operations only

## Usage Examples

### Initial Setup (First Boot)

```cpp
// Generate and store a new key on first boot
if (!AP_Crypto::has_stored_key()) {
    uint8_t key[32];
    if (AP_Crypto::generate_and_store_key(key)) {
        // Key generated and stored
    } else if (AP_Crypto::derive_key_from_board_id(key)) {
        // Fallback: derive from board ID
        AP_Crypto::store_key(key);
    }
}
```

### Using Stored Key

```cpp
// Retrieve key for encryption/decryption
uint8_t key[32];
if (AP_Crypto::retrieve_key(key)) {
    // Use key for encryption/decryption
    char key_b64[45];
    // Convert to base64url for AP_Crypto functions
    // ...
}
```

### Key Rotation

```cpp
// Generate new key and replace old one
uint8_t new_key[32];
if (AP_Crypto::generate_and_store_key(new_key)) {
    // Old key replaced with new key
    // Note: Files encrypted with old key will need re-encryption
}
```

## Migration from Hardcoded Keys

The system automatically migrates from hardcoded keys:

1. **First Use**: If no stored key exists, derives from board ID
2. **Storage**: Stores derived key for future use
3. **Consistency**: Same key used across reboots
4. **Backward Compatibility**: Falls back to hardcoded key if storage fails

## Troubleshooting

### Key Not Found

- Check that StorageManager is initialized
- Verify StorageKeys area is available (64 bytes minimum)
- Check storage access permissions

### Key Storage Fails

- Verify sufficient storage space
- Check StorageManager status
- Ensure storage is not read-only

### Key Retrieval Fails

- Verify key was previously stored
- Check storage integrity
- Ensure StorageManager is accessible

## Future Enhancements

- [ ] Key versioning (support multiple key versions)
- [ ] Key rotation with backward compatibility
- [ ] Hardware security module support
- [ ] Key backup/restore mechanism
- [ ] MAVLink commands for key management

