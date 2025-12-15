# Key Sources for Encrypted Log Files

## Overview

When a log file is encrypted, the key used comes from `log_get_default_key()` which follows this priority:

## Key Retrieval Priority

### 1. Stored Key (Secure Storage) - **HIGHEST PRIORITY**

**Source:** `AP_Crypto::retrieve_key()` from `StorageManager::StorageKeys` area

**How keys get stored:**
- **Direct storage:** `AP_Crypto::store_key(key)` - stores a 32-byte key directly
- **LEIGH_KEY parameter:** `AP_Crypto_Params::handle_key_set(value)` 
  - Derives key using: `BLAKE2b(LEIGH_KEY_INT32 + salt)` → 32-byte key
  - Then stores the derived key via `AP_Crypto::store_key()`
  - **Important:** The stored key is the DERIVED key, not the LEIGH_KEY value itself
- **Random generation:** `AP_Crypto::generate_and_store_key()` - generates random key and stores it

**Storage format:**
```
struct crypto_key_header {
    uint32_t magic;      // 0x43525950 ("CRYP" in ASCII)
    uint32_t version;    // 1
    uint8_t key[32];     // The actual 32-byte encryption key
};
```

**Total size:** 40 bytes

**⚠️ Cannot be determined from file alone** - requires access to firmware's secure storage

### 2. Default Key (Fallback) - **LOWEST PRIORITY**

**Source:** Hardcoded fallback if `retrieve_key()` returns false

**Key value:**
- ASCII: `"LEIGH AEROSPACE DEADBEEF_IS_COLD"`
- Base64URL: `TEVJR0ggQUVST1NQQUNFIERFQURCRUVGX0lTX0NPTEQ`
- Hex: `4c45494748204145524f53504143452044454144424545465f49535f434f4c44`

**Used when:**
- No key is stored in secure storage
- `retrieve_key()` returns false

## Board ID Derivation

**Status:** ❌ **DISABLED**

The `derive_key_from_board_id()` function always returns `false` and is not used.

## Implications for Decryption

If decryption fails with:
- Default key
- LEIGH_KEY-derived keys (including LEIGH_KEY=74768361)

Then the file was likely encrypted with:

1. **A key stored in secure storage** (most likely)
   - Could be from `AP_Crypto::store_key(key)`
   - Could be from `AP_Crypto_Params::handle_key_set(value)` (LEIGH_KEY-derived but stored)
   - Could be from `AP_Crypto::generate_and_store_key()` (random key)

2. **A different LEIGH_KEY value**
   - If LEIGH_KEY was set, the derived key is stored
   - The stored key persists even if LEIGH_KEY parameter is later changed
   - Need the exact LEIGH_KEY value that was used when the key was stored

## How to Determine the Key

### Option 1: Extract from Firmware Storage

If you have access to the firmware's `StorageManager::StorageKeys` area:
1. Read 40 bytes from `StorageKeys` area at offset 0
2. Verify magic is `0x43525950` ("CRYP")
3. Verify version is `1`
4. Extract the 32-byte key from bytes 8-39

### Option 2: Find the LEIGH_KEY Value

If the key was stored via `LEIGH_KEY` parameter:
1. Query the MAVLink parameter `LEIGH_KEY` (will return 0 if set, but check if key exists)
2. Check firmware logs/config for the LEIGH_KEY value that was used
3. Derive the key using: `BLAKE2b(LEIGH_KEY_INT32 + "LEIGH_KEY_SALT_1")`

### Option 3: Use Diagnostic Tool

Use `decrypt_diagnostic.py` to try multiple key sources:

```bash
# Try default key and LEIGH_KEY=74768361
python3 decrypt_diagnostic.py 00000002.tlog --leigh-key 74768361

# Try range of LEIGH_KEY values
python3 decrypt_diagnostic.py 00000002.tlog --leigh-key-range 74768350 74768370

# Try multiple specific values
python3 decrypt_diagnostic.py 00000002.tlog --leigh-keys 74768361 12345 98765
```

## Key Derivation from LEIGH_KEY

If LEIGH_KEY parameter was set, the key derivation process is:

```python
import struct
import hashlib

def derive_key_from_leigh_key(leigh_key_value):
    # Salt from AP_Crypto_Params.cpp
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes
    
    # Convert INT32 to little-endian 4-byte integer
    seed_bytes = struct.pack('<i', leigh_key_value)
    
    # Derive key using BLAKE2b
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    
    return key_bytes
```

**Important:** Once derived and stored, this key persists in secure storage even if LEIGH_KEY parameter is changed later.

## Summary

For `00000002.tlog` that fails to decrypt with default key and LEIGH_KEY=74768361:

**Most likely:** File was encrypted with a key stored in secure storage that cannot be determined from the file alone.

**To decrypt, you need:**
- Access to the firmware's `StorageManager::StorageKeys` area, OR
- The exact LEIGH_KEY value that was used when the key was stored, OR
- The exact 32-byte key that was stored





