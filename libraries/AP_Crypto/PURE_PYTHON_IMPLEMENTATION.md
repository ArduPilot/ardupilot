# Pure Python Implementation for AP_Crypto

## Overview

A pure Python implementation of AP_Crypto encryption/decryption using Python's `cryptography` library instead of `pymonocypher`.

## Important Note

**The firmware (C++) implementation cannot use Python** - ArduPilot runs on embedded microcontrollers where Python cannot execute. The firmware will continue to use Monocypher (C library).

This pure Python implementation is for:
- **Host-side tools** (encryption/decryption scripts)
- **Development and testing**
- **Compatibility with Python's cryptography library**

## Implementation

### File Created

- `PTYHON_CRYPTO_TOOL/encrypt_decrypt_pure_python.py` - Pure Python encryption/decryption tool

### Dependencies

```bash
pip3 install cryptography
```

### Features

- Uses Python's `cryptography.hazmat.primitives.ciphers.aead.ChaCha20Poly1305`
- Implements XChaCha20-Poly1305 with 24-byte nonce (matching firmware)
- Uses HChaCha20 key derivation (first 16 bytes of nonce)
- Compatible with firmware's encryption format

### Usage

```bash
# Encrypt a log file
python3 encrypt_decrypt_pure_python.py encrypt input.log output.tlog --key KEY --binary

# Decrypt a log file
python3 encrypt_decrypt_pure_python.py decrypt input.tlog output.log --key KEY --binary

# Generate a key
python3 encrypt_decrypt_pure_python.py generate-key

# Use LEIGH_KEY
python3 encrypt_decrypt_pure_python.py decrypt input.tlog output.log --leigh-key 74768361 --binary
```

## Compatibility

### Nonce Format

The firmware uses:
- **24-byte nonce** for XChaCha20
- **HChaCha20** to derive sub-key from first 16 bytes
- **Last 8 bytes** used as nonce for ChaCha20Poly1305

Python's `ChaCha20Poly1305` uses:
- **12-byte nonce** for ChaCha20Poly1305

**Solution:** The pure Python implementation:
1. Derives sub-key using HChaCha20 (first 16 bytes of 24-byte nonce)
2. Pads the last 8 bytes to 12 bytes (adds 4 zero bytes)
3. Uses ChaCha20Poly1305 with the derived sub-key

### Key Derivation

Both implementations use the same key derivation:
- LEIGH_KEY → BLAKE2b(LEIGH_KEY + "LEIGH_KEY_SALT_1") → 32-byte key

## Limitations

1. **Streaming encryption:** Python's ChaCha20Poly1305 doesn't support incremental encryption like Monocypher's streaming API. The current implementation reads the entire file into memory.

2. **HChaCha20 implementation:** The current HChaCha20 implementation is simplified. For production use, consider using a proper XChaCha20-Poly1305 library.

3. **Firmware compatibility:** The firmware must continue using Monocypher (C library) as Python cannot run on embedded systems.

## Future Improvements

1. **Proper XChaCha20-Poly1305 library:** Use a library that natively supports 24-byte nonces
2. **Streaming support:** Implement chunked encryption/decryption for large files
3. **Better HChaCha20:** Use a proper HChaCha20 implementation

## Testing

Test the pure Python implementation:

```bash
# Test encryption
python3 encrypt_decrypt_pure_python.py encrypt test.log test_encrypted.tlog --default-key --binary

# Test decryption
python3 encrypt_decrypt_pure_python.py decrypt test_encrypted.tlog test_decrypted.log --default-key --binary

# Verify files match
diff test.log test_decrypted.log
```

## Comparison

| Feature | Monocypher (C++) | Pure Python |
|---------|------------------|-------------|
| Platform | Embedded firmware | Host tools |
| Library | pymonocypher | cryptography |
| Nonce size | 24 bytes (XChaCha20) | 12 bytes (padded to match) |
| Streaming | Yes (incremental) | No (full file) |
| HChaCha20 | Native | Simplified implementation |





