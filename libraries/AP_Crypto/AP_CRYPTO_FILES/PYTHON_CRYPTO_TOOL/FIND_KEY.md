# Finding the Encryption Key for 00000002.tlog

## Status

✅ **Decryption implementation is correct**  
❌ **Encryption key is unknown**

Both the default key and LEIGH_KEY=74768361 failed MAC verification, indicating the file was encrypted with a different key.

## Key Sources (in priority order)

### 1. Stored Key in Secure Storage (Most Likely)

The file was likely encrypted with a key stored in `StorageManager::StorageKeys` area.

**Location:** Firmware's non-volatile storage  
**Format:**
```
Offset 0-3:   Magic (0x43525950 = "CRYP")
Offset 4-7:   Version (1)
Offset 8-39:  32-byte encryption key
```

**Total size:** 40 bytes

**How to extract:**
1. Connect to the firmware via MAVLink or debugger
2. Read 40 bytes from `StorageManager::StorageKeys` area at offset 0
3. Verify magic is `0x43525950`
4. Extract bytes 8-39 as the 32-byte key
5. Convert to base64url for use with decryption tools

### 2. LEIGH_KEY Parameter Value

If the key was stored via `LEIGH_KEY` MAVLink parameter:

**Note:** The `LEIGH_KEY` parameter itself is hidden (returns 0), but you can:
- Check firmware logs/config for the value that was set
- Try a range of values using the diagnostic tool
- Check if there's a record of what value was used

**Key derivation:**
```python
import struct
import hashlib

def derive_key_from_leigh_key(leigh_key_value):
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes
    seed_bytes = struct.pack('<i', leigh_key_value)
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    return key_bytes
```

### 3. Default Key (Already Tried - Failed)

- ASCII: `"LEIGH AEROSPACE DEADBEEF_IS_COLD"`
- Base64URL: `TEVJR0ggQUVST1NQQUNFIERFQURCRUVGX0lTX0NPTEQ`

## Tools Available

### 1. Diagnostic Tool (Try Multiple Keys)

```bash
cd libraries/AP_Crypto/PTYHON_CRYPTO_TOOL
python3 decrypt_diagnostic.py ../00000002.tlog --leigh-key-range 0 100000000
```

This will try:
- Default key
- LEIGH_KEY values from 0 to 100,000,000

### 2. Direct Decryption (Once Key is Known)

```bash
# With base64url-encoded key
python3 decrypt_working.py ../00000002.tlog output.log --key <base64url_key>

# With LEIGH_KEY value
python3 decrypt_working.py ../00000002.tlog output.log --leigh-key <value>

# With default key
python3 decrypt_working.py ../00000002.tlog output.log --default-key
```

### 3. Using pymonocypher unlock (Simplest)

```python
import monocypher
import struct

# Read file
with open('00000002.tlog', 'rb') as f:
    data = f.read()

# Parse
nonce = data[9:33]
ciphertext = data[33:-16]
mac = data[-16:]

# Try with your key (32 bytes)
key = b'...'  # Your 32-byte key here
plaintext = monocypher.unlock(key, nonce, mac, ciphertext)

if plaintext:
    with open('00000002_decrypted.log', 'wb') as f:
        f.write(plaintext)
    print("Success!")
else:
    print("MAC verification failed - wrong key")
```

## Next Steps

1. **Extract key from firmware storage:**
   - Use MAVLink to read storage area
   - Use debugger to read memory
   - Check if firmware has a way to export the key

2. **Try wider LEIGH_KEY range:**
   ```bash
   python3 decrypt_diagnostic.py ../00000002.tlog --leigh-key-range 0 1000000000
   ```

3. **Check firmware logs/config:**
   - Look for LEIGH_KEY parameter value in logs
   - Check configuration files
   - Check if key was set via MAVLink and logged

4. **Contact firmware developer:**
   - Ask for the encryption key
   - Ask for the LEIGH_KEY value that was used
   - Ask how to extract the key from secure storage

## File Information

- **File:** `00000002.tlog`
- **Size:** 1,015,808 bytes
- **Format:** Encrypted ArduPilot log (binary streaming)
- **Version:** 0x80
- **Timestamp:** 621,126 ms
- **Nonce:** `57172e9296799f9dedcbaedbfcc45d519c5031ddb9b6d1f0`
- **MAC:** `a015eb9e83f5ff534d75a89cdad7480e`

## Verification

Once you have the key and decrypt successfully, verify the output:

```bash
# Check file type
file 00000002_decrypted.log

# Check first bytes (should be log format)
hexdump -C 00000002_decrypted.log | head -5

# Should see ArduPilot log format markers
```

A valid ArduPilot log file should start with log format definitions or recognizable binary structures.





