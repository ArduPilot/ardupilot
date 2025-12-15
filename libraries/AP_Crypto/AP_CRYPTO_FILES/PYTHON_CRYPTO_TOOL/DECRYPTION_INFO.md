# Decryption Information for .tlog Files

## Salt Value

**Exact salt value used in C++ code (AP_Crypto_Params.cpp):**

```cpp
const uint8_t salt[16] = {
    0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
    0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31
};
```

**Salt (bytes):** `b'LEIGH_KEY_SALT_1'`  
**Salt (hex):** `4c454947485f4b45595f53414c545f31`  
**Salt (hex with spaces):** `4c 45 49 47 48 5f 4b 45 59 5f 53 41 4c 54 5f 31`  
**Salt (bytes array):** `[0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45, 0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31]`  
**Salt length:** 16 bytes  
**Salt (ASCII):** `LEIGH_KEY_SALT_1`

**Python representation:**
```python
salt = bytes([0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
              0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31])
# Or equivalently:
salt = b'LEIGH_KEY_SALT_1'
```

## Key Derivation Method

**Algorithm:** BLAKE2b  
**Input format:** `seed_bytes + salt`  
**Output:** 32-byte key

### Process:
1. Convert LEIGH_KEY (INT32) to little-endian 4-byte integer: `struct.pack('<i', leigh_key)`
2. Concatenate with salt: `seed_bytes + salt`
3. Compute BLAKE2b hash: `hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()`

### Example (LEIGH_KEY = 74768361):
```
LEIGH_KEY (decimal): 74768361
LEIGH_KEY (hex): 0x474dfe9
Seed bytes (little-endian): e9df7404
Salt (hex): 4c454947485f4b45595f53414c545f31
Derived key (hex): 3f99def3e4d65513093564e6e99a536b44b522c3806a1c3889c6e8a45dff09f6
```

## Python Code

```python
import struct
import hashlib

# Salt value (constant)
salt = b'LEIGH_KEY_SALT_1'  # Hex: 4c454947485f4b45595f53414c545f31

# Key derivation
leigh_key = 74768361  # Replace with your actual LEIGH_KEY value
seed_bytes = struct.pack('<i', leigh_key)  # Little-endian INT32
key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()

# key_bytes is now a 32-byte key ready for decryption
```

## File Format

**Format:** AP_Crypto streaming format  
**Structure:**
- [Version: 1 byte] - Always `0x80` for streaming format
- [Timestamp: 8 bytes] - Big-endian uint64
- [Nonce: 24 bytes] - Random nonce
- [Ciphertext: variable length] - Encrypted data
- [MAC: 16 bytes] - Poly1305 authentication tag at the end

## Decryption Process

1. **Parse file structure:**
   - Read version byte (should be `0x80`)
   - Read timestamp (8 bytes)
   - Read nonce (24 bytes)
   - Read MAC from last 16 bytes
   - Ciphertext is everything between nonce and MAC

2. **Derive encryption key:**
   - Use BLAKE2b with salt as described above

3. **Derive sub-key:**
   - `hchacha20(key, nonce[0:16])` → 32-byte sub-key

4. **Derive auth key:**
   - **Exact call:** `crypto_chacha20(auth_key, nullptr, 64, sub_key, nonce + 16)`
   - **Python equivalent:** `crypto_chacha20(auth_key, None, 64, sub_key, nonce[16:24])`
   - **Parameters:**
     - `auth_key` - Output buffer (64 bytes)
     - `nullptr/None` - Input buffer (null = encrypt zeros, generate keystream)
     - `64` - Size in bytes (64 bytes output)
     - `sub_key` - Encryption key (32-byte sub-key from hchacha20)
     - `nonce + 16` / `nonce[16:24]` - Nonce (last 8 bytes of 24-byte nonce)
   - **Result:** 64-byte ChaCha20 keystream, first 32 bytes used as Poly1305 auth key
   - **Counter:** Starts at 0 (default)

5. **Verify MAC:**
   - `Poly1305(auth_key[0:32], ciphertext)` → computed MAC
   - Compare with stored MAC (last 16 bytes of file)

6. **Decrypt ciphertext:**
   - `chacha20_ctr(sub_key, nonce[16:24], counter=1, ciphertext)` → plaintext

## Important Notes

- **Endianness:** LEIGH_KEY must be converted to little-endian INT32
- **Salt concatenation:** Always `seed_bytes + salt` (not `salt + seed_bytes`)
- **BLAKE2b digest size:** Must be exactly 32 bytes
- **File extension:** Encrypted files use `.tlog` extension, decrypted should be `.log`

## Troubleshooting

If decryption fails:
1. Verify LEIGH_KEY value is correct
2. Ensure salt is exactly `b'LEIGH_KEY_SALT_1'` (16 bytes)
3. Check that seed is converted as little-endian INT32
4. Verify file format matches expected structure (starts with `0x80`)
5. Check that MAC verification passes (if MAC fails, key is likely wrong)

## Diagnostic Tool

Use the diagnostic tool to try multiple key sources:

```bash
# Try default key and LEIGH_KEY=74768361
python3 decrypt_diagnostic.py 00000002.tlog --leigh-key 74768361

# Try default key only
python3 decrypt_diagnostic.py 00000002.tlog --default-key-only

# Try range of LEIGH_KEY values
python3 decrypt_diagnostic.py 00000002.tlog --leigh-key-range 74768350 74768370

# Try multiple specific LEIGH_KEY values
python3 decrypt_diagnostic.py 00000002.tlog --leigh-keys 74768361 12345 98765

# Show derived keys in hex
python3 decrypt_diagnostic.py 00000002.tlog --leigh-key 74768361 --show-keys
```

The diagnostic tool will:
1. Try the default key first
2. Try any provided LEIGH_KEY values
3. Report which key (if any) successfully decrypts the file
4. Provide suggestions if all attempts fail

