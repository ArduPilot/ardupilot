#!/usr/bin/env python3
"""
View stored encryption key from firmware storage dump.

This script reads a 40-byte storage block (from StorageManager::StorageKeys)
and displays the stored encryption key in various formats.
"""

import sys
import struct
import base64
import argparse

def base64url_encode(data):
    """Encode data to base64url without padding."""
    encoded = base64.b64encode(data).decode('ascii')
    encoded = encoded.rstrip('=')
    encoded = encoded.replace('+', '-')
    encoded = encoded.replace('/', '_')
    return encoded

def view_stored_key(storage_bytes):
    """
    View encryption key from 40-byte storage block.
    
    Format:
    - Bytes 0-3: Magic (0x43525950 = "CRYP")
    - Bytes 4-7: Version (1)
    - Bytes 8-39: 32-byte encryption key
    """
    if len(storage_bytes) < 40:
        print(f"Error: Storage block too small (got {len(storage_bytes)} bytes, need 40)")
        return False
    
    # Parse header
    magic = struct.unpack('<I', storage_bytes[0:4])[0]
    version = struct.unpack('<I', storage_bytes[4:8])[0]
    key = storage_bytes[8:40]
    
    print("=" * 70)
    print("STORED ENCRYPTION KEY")
    print("=" * 70)
    print()
    
    # Check magic
    if magic == 0x43525950:
        print("✓ Valid key storage block found")
        magic_str = "".join(chr((magic >> (i * 8)) & 0xFF) for i in range(4))
        print(f"  Magic: 0x{magic:08x} ('{magic_str}')")
    else:
        print(f"⚠ Warning: Invalid magic (got 0x{magic:08x}, expected 0x43525950)")
        print("  This may not be a valid crypto key storage block")
    
    print(f"  Version: {version}")
    if version != 1:
        print(f"  ⚠ Warning: Unexpected version (expected 1)")
    
    print()
    print("ENCRYPTION KEY:")
    print("-" * 70)
    print(f"  Key (hex): {key.hex()}")
    print(f"  Key (base64url): {base64url_encode(key)}")
    print()
    print("KEY FORMATS FOR USE:")
    print("-" * 70)
    print(f"  For decrypt_working.py:")
    print(f"    python3 decrypt_working.py input.tlog output.log --key {base64url_encode(key)}")
    print()
    print(f"  For Python script:")
    print(f"    key_b64 = '{base64url_encode(key)}'")
    print(f"    # or")
    print(f"    key = bytes.fromhex('{key.hex()}')")
    print()
    print(f"  For monocypher.unlock():")
    print(f"    key = bytes.fromhex('{key.hex()}')")
    print()
    
    return True

def main():
    parser = argparse.ArgumentParser(
        description='View stored encryption key from firmware storage dump',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This script reads a 40-byte storage block from StorageManager::StorageKeys area
and displays the stored encryption key.

Storage Format:
  Offset 0-3:   Magic: 0x43525950 ("CRYP")
  Offset 4-7:   Version: 1
  Offset 8-39:  32-byte encryption key

To extract the storage block from firmware:
  1. Use debugger to read 40 bytes from StorageManager::StorageKeys area at offset 0
  2. Save to a file: storage_block.bin
  3. Run: python3 view_stored_key.py storage_block.bin

Example:
  python3 view_stored_key.py storage_block.bin
        """
    )
    parser.add_argument('storage_file', help='40-byte storage block file (from StorageManager::StorageKeys)')
    parser.add_argument('--hex', action='store_true', help='Show key in hex format only')
    parser.add_argument('--base64url', action='store_true', help='Show key in base64url format only')
    
    args = parser.parse_args()
    
    # Read storage block
    try:
        with open(args.storage_file, 'rb') as f:
            storage_bytes = f.read()
    except IOError as e:
        print(f"Error reading storage file: {e}")
        sys.exit(1)
    
    if args.hex:
        if len(storage_bytes) >= 40:
            key = storage_bytes[8:40]
            print(key.hex())
        else:
            print("Error: File too small", file=sys.stderr)
            sys.exit(1)
    elif args.base64url:
        if len(storage_bytes) >= 40:
            key = storage_bytes[8:40]
            print(base64url_encode(key))
        else:
            print("Error: File too small", file=sys.stderr)
            sys.exit(1)
    else:
        if not view_stored_key(storage_bytes):
            sys.exit(1)

if __name__ == '__main__':
    main()





