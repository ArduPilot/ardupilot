# Python Crypto Tool for AP_Crypto

Python tool for encrypting and decrypting files on the SD card using AP_Crypto compatible format.

## Overview

This tool provides command-line utilities to encrypt and decrypt files that are stored on the SD card, using the same encryption format as the AP_Crypto library in ArduPilot.

## Features

- **Encrypt files**: Convert plaintext files to encrypted format
- **Decrypt files**: Convert encrypted files back to plaintext
- **Key generation**: Generate new encryption keys
- **In-place encryption**: Optionally overwrite original files
- **AP_Crypto compatible**: Uses the same ChaCha20-Poly1305 encryption scheme

## Installation

### Prerequisites

```bash
pip3 install pymonocypher
```

## Usage

### Generate a Key

First, generate an encryption key:

```bash
python3 encrypt_decrypt_files.py generate-key
```

This will output a base64url-encoded 32-byte key like:
```
Generated key: dGVzdF9rZXlfZm9yX2VuY3J5cHRpb25fdGVzdA
```

**Save this key securely!** You'll need it to encrypt and decrypt files.

### Encrypt a File

Encrypt a file on the SD card:

```bash
python3 encrypt_decrypt_files.py encrypt input.txt output.enc --key YOUR_KEY
```

Or encrypt in-place (overwrites the original):

```bash
python3 encrypt_decrypt_files.py encrypt config.txt --in-place --key YOUR_KEY
```

### Decrypt a File

Decrypt an encrypted file:

```bash
python3 encrypt_decrypt_files.py decrypt input.enc output.txt --key YOUR_KEY
```

Or decrypt in-place:

```bash
python3 encrypt_decrypt_files.py decrypt config.enc --in-place --key YOUR_KEY
```

## Examples

### Encrypting Configuration Files

```bash
# Generate a key
KEY=$(python3 encrypt_decrypt_files.py generate-key | grep "Generated key:" | cut -d' ' -f3)

# Encrypt a configuration file
python3 encrypt_decrypt_files.py encrypt /path/to/sdcard/config.txt \
    /path/to/sdcard/config.enc --key "$KEY"

# Later, decrypt it
python3 encrypt_decrypt_files.py decrypt /path/to/sdcard/config.enc \
    /path/to/sdcard/config.txt --key "$KEY"
```

### Encrypting Multiple Files

```bash
KEY="your-key-here"

# Encrypt all .txt files
for file in /path/to/sdcard/*.txt; do
    python3 encrypt_decrypt_files.py encrypt "$file" "${file}.enc" --key "$KEY"
done

# Decrypt all .enc files
for file in /path/to/sdcard/*.enc; do
    python3 encrypt_decrypt_files.py decrypt "$file" "${file%.enc}" --key "$KEY"
done
```

### Working with SD Card Files

When the SD card is mounted on your system:

```bash
# Mount point might be /media/sdcard or /mnt/sdcard
SDCARD="/media/sdcard"

# Encrypt a file on SD card
python3 encrypt_decrypt_files.py encrypt \
    "$SDCARD/APM/scripts/my_script.lua" \
    "$SDCARD/APM/scripts/my_script.lua.enc" \
    --key "$KEY"

# Decrypt on SD card
python3 encrypt_decrypt_files.py decrypt \
    "$SDCARD/APM/scripts/my_script.lua.enc" \
    "$SDCARD/APM/scripts/my_script.lua" \
    --key "$KEY"
```

## File Format

Encrypted files are stored as base64url-encoded strings containing:

```
[Version: 1 byte] [Timestamp: 8 bytes] [Nonce: 24 bytes] [Ciphertext] [MAC: 16 bytes]
```

- **Version**: Always `0x80`
- **Timestamp**: 8-byte big-endian Unix timestamp (milliseconds)
- **Nonce**: 24-byte random nonce for ChaCha20-Poly1305
- **Ciphertext**: Encrypted data (same size as plaintext)
- **MAC**: 16-byte Poly1305 authentication tag

## Security Notes

1. **Key Storage**: Store keys securely. Consider:
   - Environment variables
   - Encrypted key files
   - Hardware security modules (for production)

2. **Key Sharing**: If files need to be decrypted on ArduPilot, use the same key that AP_Crypto uses (or derive from board ID).

3. **File Permissions**: Ensure encrypted files have appropriate permissions:
   ```bash
   chmod 600 encrypted_file.enc
   ```

4. **Backup**: Always keep backups of unencrypted files before encrypting in-place.

## Integration with ArduPilot

To use encrypted files with ArduPilot's AP_Crypto:

1. **Encrypt on host computer**:
   ```bash
   python3 encrypt_decrypt_files.py encrypt config.txt config.enc --key "$KEY"
   ```

2. **Copy to SD card**:
   ```bash
   cp config.enc /path/to/sdcard/
   ```

3. **Decrypt in ArduPilot code**:
   ```cpp
   const char *key = "your-base64url-key";
   const char *ciphertext = "..."; // Read from file
   uint8_t plaintext[256];
   int len = AP_Crypto::decode(key, ciphertext, strlen(ciphertext),
                               plaintext, sizeof(plaintext));
   ```

## Compatibility

- **AP_Crypto**: Fully compatible (uses same encryption scheme)
- **Python Fernet**: Not compatible (uses AES-128-CBC, not ChaCha20-Poly1305)

## Troubleshooting

### "Invalid key format" error
- Ensure key is base64url-encoded (44 characters)
- Use `generate-key` command to create a valid key

### "Decryption/MAC verification failed"
- Key is incorrect
- File was corrupted
- File was encrypted with a different key

### "Invalid ciphertext format"
- File is not in AP_Crypto format
- File was corrupted during transfer
- File is not base64url-encoded

## Related

- [AP_Crypto Library Documentation](../README.md)
- [Lua File Encryption](../../AP_Scripting/LUA_ENCRYPTION_README.md)
- [Log File Encryption](../../AP_Logger/LOG_ENCRYPTION_README.md)

