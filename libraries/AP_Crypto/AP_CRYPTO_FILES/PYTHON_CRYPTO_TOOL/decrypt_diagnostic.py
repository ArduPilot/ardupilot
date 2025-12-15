#!/usr/bin/env python3
"""
Diagnostic tool to decrypt log files by trying multiple key sources.

This tool will attempt decryption using:
1. LEIGH_KEY value (if provided)
2. Default key
3. Multiple LEIGH_KEY values (if range provided)
"""

import sys
import struct
import hashlib
import base64
import argparse

try:
    from monocypher import (crypto_hchacha20, crypto_chacha20, crypto_chacha20_ctr,
                            crypto_poly1305_init, crypto_poly1305_update,
                            crypto_poly1305_final, crypto_verify16)
except ImportError:
    print("Error: monocypher Python library not found.")
    print("Install it with: pip3 install pymonocypher")
    sys.exit(1)

# Constants
FERNET_VERSION = 0x80
FERNET_HEADER_SIZE = 33  # 1 + 8 + 24
FERNET_MAC_SIZE = 16
FERNET_NONCE_SIZE = 24

def base64url_encode(data):
    """Encode data to base64url without padding."""
    encoded = base64.b64encode(data).decode('ascii')
    encoded = encoded.rstrip('=')
    encoded = encoded.replace('+', '-')
    encoded = encoded.replace('/', '_')
    return encoded

def base64url_decode(data):
    """Decode base64url data."""
    data = data.replace('-', '+')
    data = data.replace('_', '/')
    padding = 4 - len(data) % 4
    if padding != 4:
        data += '=' * padding
    return base64.b64decode(data)

def derive_key_from_leigh_key(leigh_key_value):
    """Derive a 32-byte key from LEIGH_KEY INT32 value."""
    # Exact salt from AP_Crypto_Params.cpp
    salt = bytes([0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
                  0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31])
    
    # Convert INT32 to little-endian 4-byte integer (matches C++ on ARM)
    seed_bytes = struct.pack('<i', leigh_key_value)
    
    # Derive key using BLAKE2b: hash(seed_bytes + salt) -> 32 bytes
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    
    return key_bytes

def get_default_key():
    """Get the default hardcoded key."""
    default_key_str = "LEIGH AEROSPACE DEADBEEF_IS_COLD"
    key_bytes = default_key_str.encode('ascii')
    # Pad to 32 bytes if needed (should already be 32)
    if len(key_bytes) < 32:
        key_bytes = key_bytes + b'\x00' * (32 - len(key_bytes))
    elif len(key_bytes) > 32:
        key_bytes = key_bytes[:32]
    return key_bytes

def decrypt_with_key(key_bytes, data):
    """Attempt to decrypt file data with given key."""
    if len(data) < FERNET_HEADER_SIZE + FERNET_MAC_SIZE:
        return None, "File too small"
    
    # Parse header
    pos = 0
    version = data[pos]
    pos += 1
    
    if version != FERNET_VERSION:
        return None, f"Invalid version: 0x{version:02x}"
    
    # Read timestamp (8 bytes, big-endian) - skip it, not needed for decryption
    pos += 8
    
    # Read nonce (24 bytes)
    nonce = data[pos:pos+24]
    pos += 24
    
    # Extract MAC from end of file
    mac = data[-FERNET_MAC_SIZE:]
    
    # Extract ciphertext (everything between header and MAC)
    ciphertext = data[FERNET_HEADER_SIZE:-FERNET_MAC_SIZE]
    
    if len(ciphertext) == 0:
        return None, "No encrypted data"
    
    # Derive keys for decryption
    # 1. Derive sub-key using hchacha20
    sub_key = bytearray(32)
    crypto_hchacha20(sub_key, key_bytes, nonce)
    
    # 2. Derive auth key using chacha20
    auth_key = bytearray(64)
    crypto_chacha20(auth_key, None, 64, sub_key, nonce[16:24])
    
    # 3. Initialize Poly1305 for MAC verification
    poly1305_ctx = bytearray(256)
    crypto_poly1305_init(poly1305_ctx, auth_key)
    
    # 4. Update Poly1305 with ciphertext
    crypto_poly1305_update(poly1305_ctx, ciphertext, len(ciphertext))
    
    # 5. Finalize Poly1305 to get computed MAC
    computed_mac = bytearray(16)
    crypto_poly1305_final(poly1305_ctx, computed_mac)
    
    # 6. Verify MAC
    if crypto_verify16(mac, computed_mac) != 0:
        return None, "MAC verification failed"
    
    # 7. Decrypt ciphertext using ChaCha20-CTR
    plaintext = bytearray(len(ciphertext))
    counter = 1  # ChaCha20-CTR starts at 1
    crypto_chacha20_ctr(plaintext, ciphertext, len(ciphertext), sub_key, nonce[16:24], counter)
    
    return bytes(plaintext), None

def main():
    parser = argparse.ArgumentParser(
        description='Diagnostic tool to decrypt log files by trying multiple key sources',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Try default key and LEIGH_KEY=74768361
  python3 decrypt_diagnostic.py 00000002.tlog --leigh-key 74768361
  
  # Try default key only
  python3 decrypt_diagnostic.py 00000002.tlog --default-key-only
  
  # Try range of LEIGH_KEY values
  python3 decrypt_diagnostic.py 00000002.tlog --leigh-key-range 74768350 74768370
  
  # Try specific LEIGH_KEY values
  python3 decrypt_diagnostic.py 00000002.tlog --leigh-keys 74768361 12345 98765
        """
    )
    parser.add_argument('input', help='Input encrypted .tlog file')
    parser.add_argument('output', nargs='?', help='Output decrypted file (default: input_decrypted.log)')
    parser.add_argument('--leigh-key', type=int, help='LEIGH_KEY value to try')
    parser.add_argument('--leigh-keys', type=int, nargs='+', help='Multiple LEIGH_KEY values to try')
    parser.add_argument('--leigh-key-range', type=int, nargs=2, metavar=('START', 'END'),
                       help='Range of LEIGH_KEY values to try (inclusive)')
    parser.add_argument('--default-key-only', action='store_true',
                       help='Only try default key (skip LEIGH_KEY derivation)')
    parser.add_argument('--show-keys', action='store_true',
                       help='Show derived keys in hex format')
    
    args = parser.parse_args()
    
    # Read input file
    try:
        with open(args.input, 'rb') as f:
            data = f.read()
    except IOError as e:
        print(f"Error reading input file: {e}")
        return 1
    
    # Determine output filename
    if args.output:
        output_file = args.output
    else:
        if args.input.endswith('.tlog'):
            output_file = args.input[:-5] + '_decrypted.log'
        else:
            output_file = args.input + '_decrypted.log'
    
    # Build list of keys to try
    keys_to_try = []
    
    # 1. Default key (always try first)
    if not args.default_key_only:
        default_key = get_default_key()
        keys_to_try.append(("default key", default_key))
        if args.show_keys:
            print(f"Default key (hex): {default_key.hex()}")
    
    # 2. LEIGH_KEY values
    leigh_keys = []
    if args.leigh_key:
        leigh_keys.append(args.leigh_key)
    if args.leigh_keys:
        leigh_keys.extend(args.leigh_keys)
    if args.leigh_key_range:
        start, end = args.leigh_key_range
        leigh_keys.extend(range(start, end + 1))
    
    for leigh_key_val in leigh_keys:
        key_bytes = derive_key_from_leigh_key(leigh_key_val)
        keys_to_try.append((f"LEIGH_KEY={leigh_key_val}", key_bytes))
        if args.show_keys:
            print(f"LEIGH_KEY={leigh_key_val} derived key (hex): {key_bytes.hex()}")
    
    if not keys_to_try:
        print("Error: No keys to try. Use --default-key-only or provide --leigh-key")
        return 1
    
    # Try each key
    print(f"Attempting to decrypt {args.input}...")
    print(f"Trying {len(keys_to_try)} key(s)...")
    print()
    
    for key_name, key_bytes in keys_to_try:
        print(f"Trying {key_name}...", end=' ')
        plaintext, error = decrypt_with_key(key_bytes, data)
        
        if plaintext is not None:
            # Success!
            try:
                with open(output_file, 'wb') as f:
                    f.write(plaintext)
            except IOError as e:
                print(f"\nError writing output file: {e}")
                return 1
            
            print("✓ SUCCESS!")
            print(f"  Decrypted file: {output_file}")
            print(f"  Key used: {key_name}")
            print(f"  Encrypted size: {len(data)} bytes")
            print(f"  Decrypted size: {len(plaintext)} bytes")
            return 0
        else:
            print(f"✗ Failed: {error}")
    
    print()
    print("All decryption attempts failed.")
    print()
    print("All decryption attempts failed.")
    print()
    print("Key Retrieval Priority (from log_get_default_key):")
    print("  1. AP_Crypto::retrieve_key() - from secure storage (StorageManager::StorageKeys)")
    print("  2. Fallback to default key: 'LEIGH AEROSPACE DEADBEEF_IS_COLD'")
    print()
    print("Note: Board ID derivation is DISABLED (derive_key_from_board_id always returns false)")
    print()
    print("Possible reasons for failure:")
    print("  1. File was encrypted with a key stored in secure storage")
    print("     - Keys stored via AP_Crypto::store_key() cannot be determined from file")
    print("     - Keys stored via AP_Crypto::generate_and_store_key() are random")
    print("     - Keys stored via LEIGH_KEY parameter are derived but stored")
    print("  2. File was encrypted with a different LEIGH_KEY value")
    print("  3. File is corrupted or not encrypted")
    print()
    print("If file was encrypted with stored key:")
    print("  - The key is in StorageManager::StorageKeys area (40 bytes)")
    print("  - Structure: [magic: 4 bytes] [version: 4 bytes] [key: 32 bytes]")
    print("  - Magic: 0x43525950 ('CRYP' in ASCII)")
    print("  - Cannot be extracted without access to firmware's storage")
    print()
    print("Suggestions:")
    print("  - Try a wider range of LEIGH_KEY values with --leigh-key-range")
    print("  - Check if LEIGH_KEY parameter was set in the firmware")
    print("  - If key was stored in secure storage, extract from firmware's storage")
    print("  - Verify the file is actually encrypted (should start with 0x80)")
    print("  - Check firmware logs/config to see if generate_and_store_key() was called")
    
    return 1

if __name__ == '__main__':
    sys.exit(main())

