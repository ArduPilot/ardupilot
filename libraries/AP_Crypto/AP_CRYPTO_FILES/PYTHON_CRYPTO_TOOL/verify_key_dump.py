#!/usr/bin/env python3
"""
Verify if a key.bin file is a valid crypto key storage block.
"""

import sys

def verify_key_dump(file_path):
    """Verify if the file is a valid crypto key storage block."""
    try:
        with open(file_path, 'rb') as f:
            data = f.read()
    except IOError as e:
        print(f"Error reading file: {e}")
        return False
    
    print(f"File size: {len(data)} bytes")
    
    if len(data) != 40:
        print(f"❌ ERROR: File must be exactly 40 bytes, got {len(data)} bytes")
        return False
    
    # Check magic
    magic = int.from_bytes(data[0:4], 'little')
    expected_magic = 0x43525950  # "CRYP"
    
    print(f"Magic: 0x{magic:08X} (expected: 0x{expected_magic:08X})")
    
    if magic != expected_magic:
        print(f"❌ ERROR: Invalid magic! Expected 'CRYP' (0x43525950), got 0x{magic:08X}")
        print(f"   This is NOT a valid crypto key storage block.")
        print(f"   You may have dumped the wrong memory address.")
        return False
    
    # Check version
    version = int.from_bytes(data[4:8], 'little')
    print(f"Version: {version} (expected: 1)")
    
    if version != 1:
        print(f"⚠ WARNING: Unexpected version (expected 1)")
    
    # Extract key
    key = data[8:40]
    print(f"\n✅ VALID crypto key storage block!")
    print(f"\nKey (hex): {key.hex()}")
    
    # Check if key looks valid (not all zeros, not repeating pattern)
    if key == b'\x00' * 32:
        print(f"⚠ WARNING: Key is all zeros - may not be initialized")
    elif len(set(key)) < 4:
        print(f"⚠ WARNING: Key has very low entropy - may be uninitialized")
    
    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 verify_key_dump.py <key.bin>")
        sys.exit(1)
    
    if verify_key_dump(sys.argv[1]):
        print("\n✅ File is valid! You can now extract the key:")
        print(f"   python3 view_stored_key.py {sys.argv[1]}")
        sys.exit(0)
    else:
        print("\n❌ File is NOT valid. Please dump again from the correct address.")
        sys.exit(1)





