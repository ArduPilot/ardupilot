#!/usr/bin/env python3
"""
Helper script to extract encryption key from firmware storage format.

This script helps convert a 40-byte storage block (from StorageManager::StorageKeys)
into a usable base64url-encoded key for decryption.
"""

import sys
import struct
import base64

def base64url_encode(data):
    """Encode data to base64url without padding."""
    encoded = base64.b64encode(data).decode('ascii')
    encoded = encoded.rstrip('=')
    encoded = encoded.replace('+', '-')
    encoded = encoded.replace('/', '_')
    return encoded

def extract_key_from_storage(storage_bytes):
    """
    Extract encryption key from 40-byte storage block.
    
    Format:
    - Bytes 0-3: Magic (0x43525950 = "CRYP")
    - Bytes 4-7: Version (1)
    - Bytes 8-39: 32-byte encryption key
    """
    if len(storage_bytes) < 40:
        print(f"Error: Storage block too small (got {len(storage_bytes)} bytes, need 40)")
        return None
    
    # Check magic
    magic = struct.unpack('<I', storage_bytes[0:4])[0]
    if magic != 0x43525950:
        print(f"Error: Invalid magic (got 0x{magic:08x}, expected 0x43525950)")
        print("  This doesn't appear to be a valid crypto key storage block")
        return None
    
    # Check version
    version = struct.unpack('<I', storage_bytes[4:8])[0]
    if version != 1:
        print(f"Warning: Unexpected version (got {version}, expected 1)")
    
    # Extract key
    key = storage_bytes[8:40]
    
    return key

def main():
    if len(sys.argv) < 2:
        print("Usage: extract_key_from_storage.py <storage_file>")
        print("")
        print("This script extracts a 32-byte encryption key from a 40-byte")
        print("storage block (from StorageManager::StorageKeys area).")
        print("")
        print("The storage file should be 40 bytes containing:")
        print("  - Magic: 0x43525950 ('CRYP')")
        print("  - Version: 1")
        print("  - Key: 32 bytes")
        print("")
        print("Output: Base64url-encoded key suitable for decryption tools")
        sys.exit(1)
    
    storage_file = sys.argv[1]
    
    # Read storage block
    try:
        with open(storage_file, 'rb') as f:
            storage_bytes = f.read()
    except IOError as e:
        print(f"Error reading storage file: {e}")
        sys.exit(1)
    
    # Extract key
    key = extract_key_from_storage(storage_bytes)
    if key is None:
        sys.exit(1)
    
    # Output key in multiple formats
    print("Extracted Encryption Key:")
    print("=" * 70)
    print(f"Key (hex): {key.hex()}")
    print(f"Key (base64url): {base64url_encode(key)}")
    print("")
    print("Use this key with decryption tools:")
    print(f"  python3 decrypt_working.py input.tlog output.log --key {base64url_encode(key)}")
    print("")
    print("Or in Python:")
    print(f"  key = bytes.fromhex('{key.hex()}')")
    print(f"  # or")
    print(f"  key_b64 = '{base64url_encode(key)}'")

if __name__ == '__main__':
    main()





